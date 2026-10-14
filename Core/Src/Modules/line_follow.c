#include "line_follow.h"
#include "line_sensor.h"
#include "line_calibration.h"
#include "speed_control.h"

/* Conservative first drive: single black line on white, gentle curves.
 * Crossings / disconnected black groups are deliberately ambiguous.
 * Tune on the real course before adding intersection behavior or speed. */
#define SENSOR_MAX_AGE_MS 150U
#define CONTROL_PERIOD_MS 20U
#define CORNER_BASE_CPS 900.0f
#define MIN_WHEEL_CPS 450.0f
#define MAX_STEERING_CPS 650.0f
#define POSITION_KP 0.18f
#define POSITION_KD 0.006f
#define DERIVATIVE_FILTER_S 0.060f

volatile LineFollow_Data line_follow;
_Static_assert(sizeof(LineFollow_Data) == 96U, "Line-follow diagnostic layout");
static uint32_t start_ms, last_control_ms, last_sequence;
static float last_position, derivative;

static float Limit(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

static void CaptureSpeed(void)
{
    line_follow.left_measured_cps = (int32_t)speed_control.left_cps;
    line_follow.right_measured_cps = (int32_t)speed_control.right_cps;
    line_follow.left_pwm = speed_control.left_pwm;
    line_follow.right_pwm = speed_control.right_pwm;
    line_follow.speed_fault = speed_control.fault;
}

/* Read only foreground-published fields. Recompute age from the frame time
 * instead of trusting a state flag cached during an earlier main iteration. */
static uint32_t ReadPosition(uint32_t now_ms, float *position)
{
    line_follow.mask = line_sensor.mask & 255U;
    line_follow.active_channels = 0U;
    uint32_t age = now_ms - line_sensor.last_frame_ms;
    line_follow.frame_age_ms = line_sensor.sample_sequence ? (age > 0x7FFFFFFFU ? 0U : age) : UINT32_MAX;
    if (LineCalibration_IsActive()) return LF_STOP_CALIBRATING;
    if (!line_sensor.cal_valid) return LF_STOP_NOT_CALIBRATED;
    if (line_follow.frame_age_ms > SENSOR_MAX_AGE_MS) return LF_STOP_SENSOR_STALE;
    uint32_t groups = 0U, previous = 0U, total = 0U;
    int32_t weighted = 0;
    for (uint32_t i = 0U; i < 8U; i++) {
        uint32_t on = (line_follow.mask >> i) & 1U;
        if (on) {
            line_follow.active_channels++;
            if (!previous) groups++;
            uint32_t strength = line_sensor.strength[i];
            if (strength > 1000U) strength = 1000U;
            uint32_t weight = strength > 400U ? strength - 400U : 1U;
            weighted += ((int32_t)i * 1000 - 3500) * (int32_t)weight;
            total += weight;
        }
        previous = on;
    }
    if (!total) return LF_STOP_NO_LINE;
    if (groups != 1U || line_follow.active_channels > 4U) return LF_STOP_AMBIGUOUS;
    *position = (float)weighted / (float)total;
    return LF_STOP_NONE;
}

static void Targets(float position, float rate, float *left, float *right)
{
    float magnitude = position < 0.0f ? -position : position;
    float base = Limit((float)LINE_FOLLOW_BASE_CPS - magnitude * 0.10f,
                       CORNER_BASE_CPS, (float)LINE_FOLLOW_BASE_CPS);
    float cap = Limit(base - MIN_WHEEL_CPS, 0.0f, MAX_STEERING_CPS);
    float turn = Limit(POSITION_KP * position + POSITION_KD * rate, -cap, cap);
    /* Black on right => left wheel faster, right wheel slower => right turn. */
    *left = base + turn;
    *right = base - turn;
    line_follow.position_milli = (int32_t)position;
    line_follow.steering_cps = (int32_t)turn;
    line_follow.base_cps = (uint32_t)base;
    line_follow.left_target_cps = (uint32_t)*left;
    line_follow.right_target_cps = (uint32_t)*right;
}

void LineFollow_Init(void)
{
    line_follow = (LineFollow_Data){ .magic = 0x4C465731U, .version = 1U,
        .base_cps = LINE_FOLLOW_BASE_CPS, .frame_age_ms = UINT32_MAX };
    start_ms = last_control_ms = last_sequence = 0U;
    last_position = derivative = 0.0f;
}

void LineFollow_Stop(uint32_t reason, uint32_t now_ms)
{
    if (line_follow.state == LF_RUNNING) line_follow.elapsed_ms = now_ms - start_ms;
    SpeedControl_Stop();
    line_follow.uptime_ms = now_ms;
    line_follow.stop_reason = reason;
    line_follow.state = reason == LF_STOP_USER || reason == LF_STOP_CALIBRATING ? LF_STOPPED : LF_FAULT;
    if (reason == LF_STOP_USER) line_follow.user_stops++;
    else if (reason != LF_STOP_CALIBRATING) line_follow.automatic_stops++;
    CaptureSpeed();
}

uint8_t LineFollow_Start(uint32_t now_ms)
{
    SpeedControl_Stop();
    float position = 0.0f;
    uint32_t reason = ReadPosition(now_ms, &position);
    line_follow.uptime_ms = now_ms;
    line_follow.elapsed_ms = line_follow.updates = 0U;
    if (reason != LF_STOP_NONE) {
        line_follow.state = LF_START_REJECTED;
        line_follow.stop_reason = reason;
        line_follow.start_denied++;
        CaptureSpeed();
        return 0U;
    }
    derivative = 0.0f;
    last_position = position;
    start_ms = last_control_ms = now_ms;
    last_sequence = line_sensor.sample_sequence;
    line_follow.sensor_sequence = last_sequence;
    float left = 0.0f, right = 0.0f;
    Targets(position, 0.0f, &left, &right);
    SpeedControl_StartDifferential(left, right, now_ms);
    if (speed_control.fault != SPEED_FAULT_NONE) {
        LineFollow_Stop(LF_STOP_SPEED_FAULT, now_ms);
        return 0U;
    }
    line_follow.state = LF_RUNNING;
    line_follow.stop_reason = LF_STOP_NONE;
    CaptureSpeed();
    return 1U;
}

void LineFollow_Update(uint32_t now_ms)
{
    line_follow.uptime_ms = now_ms;
    if (line_follow.state != LF_RUNNING) return;
    line_follow.elapsed_ms = now_ms - start_ms;
    float position = 0.0f;
    uint32_t reason = ReadPosition(now_ms, &position);
    if (reason != LF_STOP_NONE) { LineFollow_Stop(reason, now_ms); return; }
    uint32_t elapsed = now_ms - last_control_ms;
    if (line_sensor.sample_sequence != last_sequence && elapsed >= CONTROL_PERIOD_MS) {
        float dt = (float)elapsed * 0.001f;
        float raw_rate = (position - last_position) / dt;
        derivative += dt / (DERIVATIVE_FILTER_S + dt) * (raw_rate - derivative);
        float left = 0.0f, right = 0.0f;
        Targets(position, derivative, &left, &right);
        SpeedControl_SetDifferentialTargets(left, right);
        last_position = position;
        last_control_ms = now_ms;
        last_sequence = line_sensor.sample_sequence;
        line_follow.sensor_sequence = last_sequence;
        line_follow.updates++;
    }
    SpeedControl_Update(now_ms);
    CaptureSpeed();
    if (speed_control.fault != SPEED_FAULT_NONE) LineFollow_Stop(LF_STOP_SPEED_FAULT, now_ms);
}
