#include "car.h"
#include "main.h"
#include "button.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include "line_calibration.h"
#include "line_sensor.h"
#include "line_follow.h"

#define CAR_DEFAULT_TARGET_CPS LINE_FOLLOW_BASE_CPS
#define HISTORY_LENGTH 101U
#define RECENT_WINDOW_MS 1000U

_Static_assert(sizeof(MotorTestResult) == 64U, "Diagnostic record layout changed");
volatile MotorTestResult motor_test_result = {
    .magic = 0x43313042U, .version = 3U, .target_cps = CAR_DEFAULT_TARGET_CPS
};

typedef struct
{
    int64_t left;
    int64_t right;
    uint32_t time_ms;
} Car_Sample;

static Button_State user_button;
static Car_Sample history[HISTORY_LENGTH];
static uint32_t history_next;
static uint32_t history_count;
static uint32_t last_sample_count;
static uint32_t run_start_ms;
static uint32_t key_press_ms;
static uint8_t pending_short_press;

static uint8_t ReadUserButton(void)
{
    /* C10B USER: PA0, external 10k pulldown, pressed connects to 3V3. */
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_SET;
}

static void RecordSample(uint32_t now_ms)
{
    uint32_t index = history_next;
    uint32_t latest = (history_next + HISTORY_LENGTH - 1U) % HISTORY_LENGTH;
    if (history_count != 0U && history[latest].time_ms == now_ms)
    {
        index = latest;
    }
    else
    {
        history_next = (history_next + 1U) % HISTORY_LENGTH;
        if (history_count < HISTORY_LENGTH) history_count++;
    }
    history[index].left = encoder_data.left_total;
    history[index].right = encoder_data.right_total;
    history[index].time_ms = now_ms;
    last_sample_count = encoder_data.sample_count;
}

static void FinishRun(uint32_t now_ms)
{
    motor_test_result.left_last_pwm = speed_control.left_pwm;
    motor_test_result.right_last_pwm = speed_control.right_pwm;
    SpeedControl_Stop();
    Encoder_Sample(now_ms);
    RecordSample(now_ms);

    uint32_t elapsed_ms = now_ms - run_start_ms;
    motor_test_result.elapsed_ms = elapsed_ms;
    motor_test_result.left_counts = (int32_t)encoder_data.left_total;
    motor_test_result.right_counts = (int32_t)encoder_data.right_total;
    if (elapsed_ms != 0U)
    {
        motor_test_result.left_mean_cps = (int32_t)((float)encoder_data.left_total * 1000.0f / (float)elapsed_ms);
        motor_test_result.right_mean_cps = (int32_t)((float)encoder_data.right_total * 1000.0f / (float)elapsed_ms);
    }
    /* Find a snapshot approximately one second before stop, or the start
     * for a shorter run. Uses actual timestamps, including millis wrap. */
    uint32_t oldest = (history_next + HISTORY_LENGTH - 1U) % HISTORY_LENGTH;
    for (uint32_t offset = 0U; offset < history_count; offset++)
    {
        oldest = (history_next + HISTORY_LENGTH - 1U - offset) % HISTORY_LENGTH;
        if ((uint32_t)(now_ms - history[oldest].time_ms) >= RECENT_WINDOW_MS) break;
    }
    uint32_t window_ms = now_ms - history[oldest].time_ms;
    motor_test_result.cruise_ms = window_ms;
    if (window_ms != 0U)
    {
        motor_test_result.left_cruise_cps = (int32_t)((float)(encoder_data.left_total - history[oldest].left) * 1000.0f / (float)window_ms);
        motor_test_result.right_cruise_cps = (int32_t)((float)(encoder_data.right_total - history[oldest].right) * 1000.0f / (float)window_ms);
    }
    motor_test_result.samples = encoder_data.sample_count;
    motor_test_result.fault = speed_control.fault;
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    /* Publish final state last, after all diagnostic fields are stored. */
    motor_test_result.state = speed_control.fault == SPEED_FAULT_NONE && line_follow.state != LF_FAULT ? CAR_STOPPED : CAR_FAULT;
}

static void StartRun(uint32_t now_ms)
{
    if (!LineFollow_Start(now_ms)) return;
    uint32_t target = LINE_FOLLOW_BASE_CPS;
    motor_test_result = (MotorTestResult){
        .magic = 0x43313042U, .version = 3U, .state = CAR_RUNNING,
        .target_cps = target
    };
    history_next = 0U;
    history_count = 0U;
    run_start_ms = now_ms;
    RecordSample(now_ms);
    if (speed_control.fault != SPEED_FAULT_NONE)
    {
        FinishRun(now_ms);
        return;
    }
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
}

void Car_Init(void)
{
    Motor_Init();
    Encoder_Init();
    SpeedControl_Stop();
    motor_test_result = (MotorTestResult){
        .magic = 0x43313042U, .version = 3U, .state = CAR_WAITING,
        .target_cps = CAR_DEFAULT_TARGET_CPS
    };
    LineFollow_Init();
    pending_short_press = 0U;
    Button_Init(&user_button, ReadUserButton(), HAL_GetTick());
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void Car_Update(uint32_t now_ms)
{
    uint8_t pressed = Button_Update(&user_button, ReadUserButton(), now_ms);
    Encoder_Update(now_ms);
    if (pressed)
    {
        pending_short_press = 0U;
        if (LineCalibration_IsActive()) LineCalibration_Cancel();
        else if (motor_test_result.state == CAR_RUNNING)
        {
            LineFollow_Stop(LF_STOP_USER, now_ms);
            FinishRun(now_ms);
        }
        else { pending_short_press = 1U; key_press_ms = now_ms; }
        return;
    }
    if (pending_short_press)
    {
        if (!user_button.stable)
        {
            pending_short_press = 0U;
            StartRun(now_ms); /* Short press starts on release, not press. */
            return;
        }
        if (user_button.candidate && (uint32_t)(now_ms - key_press_ms) >= 2000U)
        {
            pending_short_press = 0U;
            LineFollow_Stop(LF_STOP_CALIBRATING, now_ms);
            LineCalibration_Start(now_ms);
        }
    }
    if (LineCalibration_IsActive())
    {
        SpeedControl_Stop();
        /* White reference: steady. Sweep: five flashes per second. */
        uint8_t on = line_sensor.cal_state == LINE_CAL_WHITE || (now_ms / 100U) % 2U == 0U;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        return;
    }
    if (motor_test_result.state != CAR_RUNNING)
    {
        uint8_t on = line_sensor.cal_state == LINE_CAL_READY;
        if (line_sensor.cal_state == LINE_CAL_FAILED || line_sensor.cal_error == LINE_CAL_SAVE_FAILED ||
            line_follow.state == LF_FAULT || line_follow.state == LF_START_REJECTED)
            on = (now_ms / 500U) % 2U == 0U;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        return;
    }

    LineFollow_Update(now_ms);
    if (line_follow.state != LF_RUNNING)
    {
        FinishRun(now_ms);
    }
    else if (encoder_data.sample_count != last_sample_count)
    {
        RecordSample(now_ms);
    }
}
