#include "car.h"
#include "main.h"
#include "button.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include "line_calibration.h"
#include "line_sensor.h"
#include "line_follow.h"
#include "task2.h"
#include "task_key.h"
#include <limits.h>

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
static TaskKey task_key;

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
    if (Task2_DistanceMm() == INT32_MIN) {
        SpeedControl_Stop();
        line_follow.state=LF_START_REJECTED;
        line_follow.stop_reason=LF_STOP_DISTANCE_CAL;
        line_follow.start_denied++;
        return;
    }
    if (!LineFollow_Configure(Task2_TargetCps(), 1U) || !LineFollow_Start(now_ms)) return;
    LapCounter_Init(&task_laps, task_target_laps, now_ms, 0);
    uint32_t target = Task2_TargetCps();
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
    Task2_Init(HAL_GetTick());
    task_key = (TaskKey){0};
    Button_Init(&user_button, ReadUserButton(), HAL_GetTick());
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void Car_Update(uint32_t now_ms)
{
    uint8_t pressed = Button_Update(&user_button, ReadUserButton(), now_ms);
    Encoder_Update(now_ms);
    if (motor_test_result.state != CAR_RUNNING) LineFollow_Update(now_ms);
    uint32_t event = TaskKey_Update(&task_key, pressed, user_button.stable, user_button.candidate,
        motor_test_result.state == CAR_RUNNING, LineCalibration_IsActive(), now_ms);
    task_key_hint = task_key.tracking ? (task_key.held_ms >= 6000U ? 2U : (task_key.held_ms >= 2000U ? 1U : 0U)) : 0U;
    if (event == TASK_KEY_STOP) {
        LineFollow_Stop(LF_STOP_USER, now_ms); FinishRun(now_ms); return;
    }
    if (event == TASK_KEY_CANCEL_CAL) { LineCalibration_Cancel(); return; }
    if (event == TASK_KEY_START) { StartRun(now_ms); return; }
    if (event == TASK_KEY_SPEED) task_speed_level = (task_speed_level+1U)%TASK_SPEED_LEVEL_COUNT;
    if (event == TASK_KEY_LAPS) task_target_laps = Task2_NextLapSetting();
    if (event == TASK_KEY_CALIBRATE) {
        LineFollow_Stop(LF_STOP_CALIBRATING, now_ms); LineCalibration_Start(now_ms);
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
        if (task_key_hint) on = (now_ms / (task_key_hint == 2U ? 100U : 250U)) % 2U == 0U;
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
        return;
    }

    /* Only the last lap slows down. Interpolate over the final 600..150 mm
     * instead of stepping 2400->1200 at 350 mm: the measured speed loop
     * undershot badly after that step. Keep the final 150 mm at <=1200
     * for the existing 24 mm coast correction; steering/PI stay unchanged. */
    int32_t distance_mm = Task2_DistanceMm();
    uint32_t cruise = Task2_TargetCps();
    if (distance_mm != INT32_MIN && cruise > 1200U) {
        int64_t remaining = (int64_t)task_laps.target_distance_mm-distance_mm;
        if (remaining <= 150) cruise = 1200U;
        else if (remaining < 600)
            cruise = 1200U + (uint32_t)((int64_t)(cruise-1200U)*(remaining-150)/450);
    }
    LineFollow_SetCruiseLimit(cruise);
    LineFollow_Update(now_ms);
    /* Safety faults from line following take priority. Otherwise evaluate
     * distance every loop, independent of new frames or transverse stripes. */
    if (line_follow.state == LF_RUNNING &&
        LapCounter_Update(&task_laps, now_ms, Task2_DistanceMm())) {
        LineFollow_Stop(LF_STOP_LAPS_DONE, now_ms);
    }
    if (line_follow.state != LF_RUNNING)
    {
        FinishRun(now_ms);
    }
    else if (encoder_data.sample_count != last_sample_count)
    {
        RecordSample(now_ms);
    }
}
