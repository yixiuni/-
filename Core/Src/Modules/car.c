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
#include "position_stop.h"
#include "odometry_config.h"

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
PositionStop position_stop;
uint32_t landmark_start_ready;
static uint32_t start_seen_ms, start_seen_sequence, start_seen_frames;
static uint8_t start_seen;
/* Diagnostic-only, captured before LapCounter_Update. No output/control writes.
 * Keep the most recent expected-return window even after a missing-line fault. */
#define LANDMARK_CAPTURE_CAPACITY 256U
typedef struct {
    uint32_t time_ms;
    int32_t distance_mm;
    uint32_t sensor_sequence, flags;
    uint16_t raw[8], strength[8];
} LandmarkCaptureFrame;
typedef struct {
    uint32_t magic, version, sequence_begin, capacity;
    uint32_t count, next, total, run_start_ms;
    int32_t expected_mm;
    uint32_t return_index, active, last_sensor_sequence;
    uint32_t skipped_samples, polarity, calibrated, sequence_end;
    uint32_t threshold[8], white[8];
    LandmarkCaptureFrame frames[LANDMARK_CAPTURE_CAPACITY];
} LandmarkCapture;
_Static_assert(sizeof(LandmarkCaptureFrame)==48U, "Capture frame layout");
_Static_assert(sizeof(LandmarkCapture)==0x3080U, "Capture dump size");
volatile LandmarkCapture landmark_capture;
static void ResetLandmarkCapture(uint32_t now_ms)
{
    uint32_t seq=(landmark_capture.sequence_begin+2U)&~1U;
    landmark_capture.sequence_begin=seq-1U;
    landmark_capture.magic=0x4C544731U;landmark_capture.version=1U;
    landmark_capture.capacity=LANDMARK_CAPTURE_CAPACITY;
    landmark_capture.count=landmark_capture.next=landmark_capture.total=0U;
    landmark_capture.run_start_ms=now_ms;landmark_capture.expected_mm=0;
    landmark_capture.return_index=landmark_capture.active=0U;
    landmark_capture.last_sensor_sequence=0U;landmark_capture.skipped_samples=0U;
    landmark_capture.polarity=line_sensor.cal_polarity;
    landmark_capture.calibrated=line_sensor.cal_valid;
    for (uint32_t i=0;i<8U;i++) {
        landmark_capture.threshold[i]=line_sensor.threshold[i];
        landmark_capture.white[i]=line_sensor.white[i];
    }
    landmark_capture.sequence_end=seq;landmark_capture.sequence_begin=seq;
}
static void RecordLandmarkCapture(uint32_t now_ms, int32_t distance_mm)
{
    if (distance_mm==INT32_MIN || !line_sensor.cal_valid || !line_sensor.sample_sequence ||
        (uint32_t)(now_ms-line_sensor.last_frame_ms)>150U ||
        line_sensor.sample_sequence==landmark_capture.last_sensor_sequence) return;
    int64_t expected=(int64_t)task_laps.center_mm+TRACK_LAP_LENGTH_MM;
    uint32_t next_return=task_laps.marker_count+1U;
    uint8_t start=next_return<=task_laps.target && next_return!=landmark_capture.return_index &&
        distance_mm>=expected-500 && distance_mm<=expected+500;
    if (!start && !landmark_capture.active) return;
    uint32_t seq=(landmark_capture.sequence_begin+2U)&~1U;
    landmark_capture.sequence_begin=seq-1U;
    if (start) {
        landmark_capture.count=landmark_capture.next=landmark_capture.total=0U;
        landmark_capture.skipped_samples=0U;landmark_capture.last_sensor_sequence=0U;
        landmark_capture.expected_mm=(int32_t)expected;
        landmark_capture.return_index=next_return;landmark_capture.active=1U;
    }
    if ((int64_t)distance_mm>landmark_capture.expected_mm+500LL) {
        landmark_capture.active=0U;
    } else {
        uint32_t current=line_sensor.sample_sequence;
        if (landmark_capture.total) {
            uint32_t delta=current-landmark_capture.last_sensor_sequence;
            if (delta>1U) landmark_capture.skipped_samples+=delta-1U;
        }
        uint32_t index=landmark_capture.next;
        landmark_capture.frames[index].time_ms=now_ms-landmark_capture.run_start_ms;
        landmark_capture.frames[index].distance_mm=distance_mm;
        landmark_capture.frames[index].sensor_sequence=current;
        landmark_capture.frames[index].flags=(line_sensor.mask&0xFFU) |
            ((task_laps.state&0xFU)<<8U) | ((task_laps.marker_count&0xFU)<<12U) |
            ((task_laps.candidate!=0U)<<16U) | ((task_laps.armed!=0U)<<17U) |
            ((task_laps.candidate && task_laps.candidate_edge)!=0U)<<18U |
            ((task_laps.candidate_frames&0xFFU)<<20U);
        for (uint32_t i=0;i<8U;i++) {
            landmark_capture.frames[index].raw[i]=(uint16_t)line_sensor.analog[i];
            landmark_capture.frames[index].strength[i]=(uint16_t)line_sensor.strength[i];
        }
        landmark_capture.last_sensor_sequence=current;
        landmark_capture.next=(index+1U)%LANDMARK_CAPTURE_CAPACITY;
        if (landmark_capture.count<LANDMARK_CAPTURE_CAPACITY) landmark_capture.count++;
        landmark_capture.total++;
    }
    landmark_capture.sequence_end=seq;landmark_capture.sequence_begin=seq;
}

static float WheelAbs(float x) { return x<0.0f ? -x : x; }
static void ObserveStart(uint32_t now_ms)
{
    uint8_t valid=line_sensor.cal_valid && line_sensor.sample_sequence &&
        (uint32_t)(now_ms-line_sensor.last_frame_ms)<=150U && LapCounter_IsStartMarker(line_sensor.mask) &&
        WheelAbs(encoder_data.left_cps)<60.0f && WheelAbs(encoder_data.right_cps)<60.0f && !LineCalibration_IsActive();
    if (!valid) { start_seen=0U;start_seen_frames=landmark_start_ready=0U;return; }
    if (!start_seen) { start_seen=1U;start_seen_ms=now_ms;start_seen_sequence=line_sensor.sample_sequence;start_seen_frames=1U; }
    else if (start_seen_sequence!=line_sensor.sample_sequence) { start_seen_sequence=line_sensor.sample_sequence;start_seen_frames++; }
    landmark_start_ready=start_seen_frames>=3U && (uint32_t)(now_ms-start_seen_ms)>=200U;
}

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
    /* SpeedControl_Stop above disables PI and releases drive first.
     * Hold electrical brake until the next explicit start/stop operation. */
    if (line_follow.stop_reason==LF_STOP_LAPS_DONE) Motor_Brake();
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
    if (!landmark_start_ready) {
        SpeedControl_Stop();line_follow.state=LF_START_REJECTED;
        line_follow.stop_reason=!line_sensor.cal_valid ? LF_STOP_NOT_CALIBRATED :
            (!line_sensor.sample_sequence || (uint32_t)(now_ms-line_sensor.last_frame_ms)>150U ? LF_STOP_SENSOR_STALE : LF_STOP_NEED_MARKER);
        line_follow.start_denied++;return;
    }
    if (!LineFollow_Configure(Task2_TargetCps(), 1U)) return;
    LineFollow_SetCruiseLimit(LANDMARK_START_CPS);
    if (!LineFollow_Start(now_ms)) return;
    SpeedControl_SetLaunchSync(1U);
    position_stop=(PositionStop){0};start_seen=0U;landmark_start_ready=0U;
    LapCounter_Init(&task_laps, task_target_laps, now_ms, 0);
    ResetLandmarkCapture(now_ms);
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

void Car_Activate(uint32_t now)
{
    Encoder_Reset(now);
    SpeedControl_Stop();
    motor_test_result = (MotorTestResult){
        .magic = 0x43313042U, .version = 3U, .state = CAR_WAITING,
        .target_cps = CAR_DEFAULT_TARGET_CPS
    };
    LineFollow_Init();
    Task2_Init(HAL_GetTick());
    task_key = (TaskKey){0};
    position_stop=(PositionStop){0};landmark_start_ready=start_seen_frames=0U;start_seen=0U;
    Button_Init(&user_button, ReadUserButton(), HAL_GetTick());
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
}

void Car_Update(uint32_t now_ms)
{
    uint8_t pressed = Button_Update(&user_button, ReadUserButton(), now_ms);
    Encoder_Update(now_ms);
    if (motor_test_result.state!=CAR_RUNNING) ObserveStart(now_ms);
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

    int32_t distance_mm=Task2_DistanceMm();
    uint8_t fresh=line_sensor.cal_valid && line_sensor.sample_sequence && (uint32_t)(now_ms-line_sensor.last_frame_ms)<=150U;
    RecordLandmarkCapture(now_ms,distance_mm);
    if (fresh) LapCounter_UpdateAnalog(&task_laps,now_ms,distance_mm,line_sensor.sample_sequence,line_sensor.mask,line_sensor.strength);
    /* Synchronize initial wheel travel only over the starting stripe.
     * Disable before line steering; never re-enable at later landmarks. */
    if (!(task_laps.state==LAP_DEPART && task_laps.marker_count==0U && LapCounter_IsMarker(line_sensor.mask)))
        SpeedControl_SetLaunchSync(0U);
    if (task_laps.state==LAP_FAULT) {
        LineFollow_Stop(task_laps.fault==LAP_FAULT_STUCK_MARKER ? LF_STOP_MARKER_TIMEOUT : LF_STOP_MARKER_MISSING,now_ms);
    } else if (task_laps.state==LAP_POSITION) {
        /* Confirmed stripe +160 mm is the brake trigger, not a crawl target.
         * Keep sensor safety active; invalid/stale data cannot complete a lap. */
        if (fresh && distance_mm!=INT32_MIN && distance_mm>=task_laps.stop_target_mm) {
            LapCounter_StopReached(&task_laps,now_ms);
            LineFollow_Stop(LF_STOP_LAPS_DONE,now_ms);
        } else {
            LineFollow_SetCruiseLimit(Task2_TargetCps());
            LineFollow_Update(now_ms);
        }
    } else {
        LineFollow_SetCruiseLimit(LapCounter_CruiseCps(&task_laps,Task2_TargetCps(),distance_mm));
        LineFollow_Update(now_ms);
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

void Car_Init(void)
{
    Motor_Init();Encoder_Init();Car_Activate(HAL_GetTick());
}
uint8_t Car_RemoteStart(uint32_t now)
{
    if (motor_test_result.state==CAR_RUNNING || LineCalibration_IsActive() || ReadUserButton()) return 0U;
    task_key=(TaskKey){0};Button_Init(&user_button,ReadUserButton(),now);
    ObserveStart(now);LineFollow_Update(now);StartRun(now);
    return motor_test_result.state==CAR_RUNNING;
}
void Car_RemoteStop(uint32_t now)
{
    if (motor_test_result.state==CAR_RUNNING) { LineFollow_Stop(LF_STOP_USER,now);FinishRun(now); }
    if (LineCalibration_IsActive()) LineCalibration_Cancel();
    SpeedControl_Stop();Motor_Brake();
    task_key=(TaskKey){0};Button_Init(&user_button,ReadUserButton(),now);task_key_hint=0U;
}
uint8_t Car_KeyPending(void)
{
    return ReadUserButton() || user_button.stable || user_button.candidate || task_key.tracking || task_key.waiting;
}
