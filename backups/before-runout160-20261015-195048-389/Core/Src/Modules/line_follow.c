#include "line_follow.h"
#include "line_sensor.h"
#include "line_calibration.h"
#include "speed_control.h"
#include "lap_counter.h"

/* Single black line on white, gentle curves. While tracking, a clearly
 * nearest group can reject a distant extra detection. No arbitrary choice
 * between similarly plausible branches. Uncertainty is time-bounded. */
#define SENSOR_MAX_AGE_MS 150U
#define CONTROL_PERIOD_MS 50U
#define CORNER_BASE_CPS 300.0f
#define MIN_WHEEL_CPS 150.0f
#define MAX_STEERING_CPS 650.0f
#define POSITION_KP 0.18f
#define POSITION_KD 0.006f
#define DERIVATIVE_FILTER_S 0.060f
#define UNCERTAIN_CONFIRM_MS 100U
#define HISTORY_DISTANCE_MAX 1000.0f
#define HISTORY_MARGIN_MIN 1500.0f

volatile LineFollow_Data line_follow;
_Static_assert(sizeof(LineFollow_Data) == 128U, "Line-follow diagnostic layout");
static uint32_t start_ms, last_control_ms, last_sequence;
static float last_position, derivative;
static uint32_t pending_since_ms, last_uncertain_sequence, last_group_sequence;
static float recovery_left, recovery_right;
static uint32_t configured_cps = LINE_FOLLOW_BASE_CPS;
static uint32_t marker_since;
static uint32_t cruise_limit_cps = LINE_FOLLOW_BASE_CPS;
static uint8_t marker_enabled, marker_active, marker_exit_pending;

static float Limit(float value, float lo, float hi)
{
    return value < lo ? lo : (value > hi ? hi : value);
}

volatile LineFollow_Trace line_follow_trace;
_Static_assert(sizeof(LineFollow_TraceEntry)==32U,"Trace entry ABI");
_Static_assert(sizeof(LineFollow_Trace)==16448U,"Trace buffer ABI");

static int16_t TraceSigned(float value)
{
    return (int16_t)Limit(value,-32768.0f,32767.0f);
}
static void TraceBegin(uint32_t now_ms)
{
    line_follow_trace.header[10]++;
    line_follow_trace.header[4]=line_follow_trace.header[5]=line_follow_trace.header[6]=0U;
    line_follow_trace.header[7]=now_ms; line_follow_trace.header[8]=0U;
    line_follow_trace.header[9]=1U; line_follow_trace.header[11]=0U;
    line_follow_trace.header[13]=configured_cps;
    line_follow_trace.header[14]=now_ms-20U;
    line_follow_trace.header[10]++;
}
static void TraceRecord(void)
{
    if (!line_follow_trace.header[9]) return;
    uint32_t now=line_follow.uptime_ms;
    uint8_t running=line_follow.state==LF_RUNNING;
    if (running && (uint32_t)(now-line_follow_trace.header[14])<20U) return;
    LineFollow_TraceEntry e={0};
    e.time_ms=now-line_follow_trace.header[7];
    e.position=TraceSigned((float)line_follow.position_milli);
    e.steering=TraceSigned((float)line_follow.steering_cps);
    e.left_target=(int16_t)line_follow.left_target_cps;
    e.right_target=(int16_t)line_follow.right_target_cps;
    e.left_measured=TraceSigned(speed_control.left_cps);
    e.right_measured=TraceSigned(speed_control.right_cps);
    e.left_pwm=(int8_t)Limit((float)speed_control.left_pwm,-100,100);
    e.right_pwm=(int8_t)Limit((float)speed_control.right_pwm,-100,100);
    e.mask=(uint8_t)line_sensor.mask; e.selected=(uint8_t)line_follow.selected_mask;
    e.age_ms=(uint16_t)(line_follow.frame_age_ms>65535U ? 65535U : line_follow.frame_age_ms);
    e.state=(uint8_t)line_follow.state; e.reason=(uint8_t)line_follow.stop_reason;
    e.s4=(uint16_t)line_sensor.strength[3]; e.s5=(uint16_t)line_sensor.strength[4];
    e.derivative=TraceSigned(derivative);
    e.pending=(uint8_t)line_follow.pending_reason; e.fault=(uint8_t)speed_control.fault;
    line_follow_trace.header[10]++;
    uint32_t index=line_follow_trace.header[4];
    line_follow_trace.entries[index]=e;
    line_follow_trace.header[4]=(index+1U)%512U;
    if (line_follow_trace.header[5]<512U) line_follow_trace.header[5]++;
    line_follow_trace.header[6]++; line_follow_trace.header[14]=now;
    if (!running) {
        line_follow_trace.header[8]=now; line_follow_trace.header[9]=0U;
        line_follow_trace.header[11]=line_follow.stop_reason;
    }
    line_follow_trace.header[10]++;
}

static void CaptureSpeed(void)
{
    line_follow.left_measured_cps = (int32_t)speed_control.left_cps;
    line_follow.right_measured_cps = (int32_t)speed_control.right_cps;
    line_follow.left_pwm = speed_control.left_pwm;
    line_follow.right_pwm = speed_control.right_pwm;
    line_follow.speed_fault = speed_control.fault;
    TraceRecord();
}

/* Read only foreground-published fields. Recompute age from the frame time
 * instead of trusting a state flag cached during an earlier main iteration. */
static uint32_t ReadPosition(uint32_t now_ms, float *position, uint8_t use_history)
{
    line_follow.mask = line_sensor.mask & 255U;
    line_follow.active_channels = 0U;
    line_follow.selected_mask = 0U;
    uint32_t age = now_ms - line_sensor.last_frame_ms;
    line_follow.frame_age_ms = line_sensor.sample_sequence ? (age > 0x7FFFFFFFU ? 0U : age) : UINT32_MAX;
    if (LineCalibration_IsActive()) return LF_STOP_CALIBRATING;
    if (!line_sensor.cal_valid) return LF_STOP_NOT_CALIBRATED;
    if (line_follow.frame_age_ms > SENSOR_MAX_AGE_MS) return LF_STOP_SENSOR_STALE;
    if (marker_enabled && LapCounter_IsMarker(line_follow.mask)) {
        if (!marker_active) { marker_since = now_ms; marker_active = 1U; }
        if (use_history && (uint32_t)(now_ms-marker_since) >= 2000U) return LF_STOP_MARKER_TIMEOUT;
        /* Count all active probes, including a laterally shifted stripe. */
        for (uint32_t i = 0U; i < 8U; i++)
            line_follow.active_channels += (line_follow.mask >> i) & 1U;
        line_follow.selected_mask = line_follow.mask;
        /* A transverse stripe has no lateral position. Hold the last
         * direction, bounded to one probe spacing; start straight. */
        *position = use_history ? Limit(last_position,-1000.0f,1000.0f) : 0.0f;
        return LF_STOP_NONE;
    }
    if (marker_active) marker_exit_pending=1U;
    marker_active = 0U;
    /* A stripe has no measured lateral position. Do not differentiate
     * across its exit; normal proportional steering resumes immediately. */
    uint32_t groups = 0U, previous = 0U, total = 0U;
    int32_t weighted = 0;
    uint32_t group_mask[4] = {0}, group_weight[4] = {0};
    int32_t group_position[4] = {0};
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
            group_mask[groups - 1U] |= 1U << i;
            group_weight[groups - 1U] += weight;
            group_position[groups - 1U] += ((int32_t)i * 1000 - 3500) * (int32_t)weight;
        }
        previous = on;
    }
    if (!total) return LF_STOP_NO_LINE;
    if (line_follow.active_channels > 4U) return LF_STOP_AMBIGUOUS;
    if (groups == 1U) {
        *position = (float)weighted / (float)total;
        line_follow.selected_mask = line_follow.mask;
        return LF_STOP_NONE;
    }
    /* Start remains strict: there is no preceding line position to trust.
     * During a run, accept only two groups with a CLEAR continuity winner.
     * Example: main S5/S6 plus distant S8 must not pull the car to S8. */
    if (!use_history || groups != 2U) return LF_STOP_AMBIGUOUS;
    float p0 = (float)group_position[0] / (float)group_weight[0];
    float p1 = (float)group_position[1] / (float)group_weight[1];
    float d0 = p0 - last_position, d1 = p1 - last_position;
    if (d0 < 0.0f) d0 = -d0;
    if (d1 < 0.0f) d1 = -d1;
    uint32_t best = d0 < d1 ? 0U : 1U;
    float nearest = best == 0U ? d0 : d1;
    float other = best == 0U ? d1 : d0;
    if (nearest > HISTORY_DISTANCE_MAX || other - nearest < HISTORY_MARGIN_MIN) return LF_STOP_AMBIGUOUS;
    *position = best == 0U ? p0 : p1;
    line_follow.selected_mask = group_mask[best];
    if (line_sensor.sample_sequence != last_group_sequence) {
        line_follow.ignored_group_frames++;
        last_group_sequence = line_sensor.sample_sequence;
    }
    return LF_STOP_NONE;
}

static void Targets(float position, float rate, float *left, float *right)
{
    float magnitude = position < 0.0f ? -position : position;
    float cruise = (float)(cruise_limit_cps < configured_cps ? cruise_limit_cps : configured_cps);
    float floor = cruise < 900.0f ? CORNER_BASE_CPS : 900.0f;
    float base = Limit(cruise - magnitude * 0.10f, floor, cruise);
    /* The lap state machine controls stripe speed: initial departure and
     * final search are slow; intermediate stripe crossings stay continuous. */
    float wheel_floor = cruise < 900.0f ? MIN_WHEEL_CPS : 450.0f;
    float cap = Limit(base - wheel_floor, 0.0f, MAX_STEERING_CPS);
    float turn = Limit(POSITION_KP * position + POSITION_KD * rate, -cap, cap);
    /* Keep steering authority at the highest setting: reduce common speed
     * to leave outside-wheel headroom, rather than clipping turn to zero.
     * The speed-loop output limit remains unchanged until hardware tests. */
    float turn_magnitude = turn < 0.0f ? -turn : turn;
    if (base > (float)SPEED_CONTROL_MAX_CPS-turn_magnitude)
        base = (float)SPEED_CONTROL_MAX_CPS-turn_magnitude;
    /* Black on right => left wheel faster, right wheel slower => right turn. */
    *left = base + turn;
    *right = base - turn;
    line_follow.position_milli = (int32_t)position;
    line_follow.steering_cps = (int32_t)turn;
    line_follow.base_cps = (uint32_t)base;
    line_follow.left_target_cps = (int32_t)*left;
    line_follow.right_target_cps = (int32_t)*right;
}

void LineFollow_Init(void)
{
    for (uint32_t i=0U;i<16U;i++) line_follow_trace.header[i]=0U;
    line_follow_trace.header[0]=0x54524331U; line_follow_trace.header[1]=2U;
    line_follow_trace.header[2]=32U; line_follow_trace.header[3]=512U;
    line_follow_trace.header[12]=20U;
    line_follow = (LineFollow_Data){ .magic = 0x4C465731U, .version = 4U,
        .base_cps = LINE_FOLLOW_BASE_CPS, .frame_age_ms = UINT32_MAX };
    start_ms = last_control_ms = last_sequence = 0U;
    last_position = derivative = 0.0f;
    pending_since_ms = last_uncertain_sequence = last_group_sequence = 0U;
    recovery_left = recovery_right = 0.0f;
    configured_cps = cruise_limit_cps = LINE_FOLLOW_BASE_CPS; marker_enabled = marker_active = marker_exit_pending = 0U;
}

uint8_t LineFollow_Configure(uint32_t cps, uint8_t markers)
{
    if (line_follow.state == LF_RUNNING || cps < 900U || cps > LINE_FOLLOW_MAX_CPS) return 0U;
    configured_cps = cruise_limit_cps = cps; marker_enabled = markers != 0U; marker_active = 0U;
    return 1U;
}

void LineFollow_SetCruiseLimit(uint32_t cps)
{
    if (cps < 300U || cps > LINE_FOLLOW_MAX_CPS) return;
    cruise_limit_cps = cps < configured_cps ? cps : configured_cps;
}

void LineFollow_Stop(uint32_t reason, uint32_t now_ms)
{
    if (line_follow.state == LF_RUNNING) line_follow.elapsed_ms = now_ms - start_ms;
    SpeedControl_Stop();
    line_follow.uptime_ms = now_ms;
    line_follow.stop_reason = reason;
    if (reason == LF_STOP_USER || reason == LF_STOP_CALIBRATING || reason == LF_STOP_LAPS_DONE)
        line_follow.pending_reason = line_follow.pending_ms = 0U;
    line_follow.state = reason == LF_STOP_USER || reason == LF_STOP_CALIBRATING || reason == LF_STOP_LAPS_DONE ? LF_STOPPED : LF_FAULT;
    if (reason == LF_STOP_USER) line_follow.user_stops++;
    else if (reason != LF_STOP_CALIBRATING && reason != LF_STOP_LAPS_DONE) line_follow.automatic_stops++;
    CaptureSpeed();
}

uint8_t LineFollow_Start(uint32_t now_ms)
{
    SpeedControl_Stop();
    float position = 0.0f;
    line_follow.pending_reason = line_follow.pending_ms = 0U;
    marker_active = marker_exit_pending = 0U;
    uint32_t reason = ReadPosition(now_ms, &position, 0U);
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
    last_uncertain_sequence = last_group_sequence = last_sequence;
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
    TraceBegin(now_ms);
    CaptureSpeed();
    return 1U;
}

void LineFollow_Update(uint32_t now_ms)
{
    line_follow.uptime_ms = now_ms;
    if (line_follow.state != LF_RUNNING) return;
    line_follow.elapsed_ms = now_ms - start_ms;
    if (speed_control.fault != SPEED_FAULT_NONE) { LineFollow_Stop(LF_STOP_SPEED_FAULT, now_ms); return; }
    float position = 0.0f;
    uint32_t reason = ReadPosition(now_ms, &position, 1U);
    if (reason == LF_STOP_NO_LINE || reason == LF_STOP_AMBIGUOUS) {
        if (line_follow.pending_reason == LF_STOP_NONE) {
            pending_since_ms = now_ms;
            /* Scale ONCE per incident. Preserve turn direction and reduce
             * both targets; repeated main-loop polls must not compound it. */
            recovery_left = Limit((float)line_follow.left_target_cps * 0.75f, MIN_WHEEL_CPS, (float)line_follow.left_target_cps);
            recovery_right = Limit((float)line_follow.right_target_cps * 0.75f, MIN_WHEEL_CPS, (float)line_follow.right_target_cps);
        }
        line_follow.pending_reason = reason;
        line_follow.pending_ms = now_ms - pending_since_ms;
        line_follow.last_uncertain_mask = line_follow.mask;
        line_follow.last_uncertain_reason = reason;
        if (line_sensor.sample_sequence != last_uncertain_sequence) {
            line_follow.uncertain_frames++;
            last_uncertain_sequence = line_sensor.sample_sequence;
        }
        /* Switching between no-line and ambiguous must not reset the timer. */
        if (line_follow.pending_ms >= UNCERTAIN_CONFIRM_MS) { LineFollow_Stop(reason, now_ms); return; }
        line_follow.left_target_cps = (int32_t)recovery_left;
        line_follow.right_target_cps = (int32_t)recovery_right;
        line_follow.base_cps = (uint32_t)((recovery_left + recovery_right) * 0.5f);
        line_follow.steering_cps = (int32_t)((recovery_left - recovery_right) * 0.5f);
        SpeedControl_SetDifferentialTargets(recovery_left, recovery_right);
        SpeedControl_Update(now_ms);
        CaptureSpeed();
        if (speed_control.fault != SPEED_FAULT_NONE) LineFollow_Stop(LF_STOP_SPEED_FAULT, now_ms);
        return;
    }
    if (reason != LF_STOP_NONE) { LineFollow_Stop(reason, now_ms); return; }
    if (line_follow.pending_reason != LF_STOP_NONE) {
        line_follow.recovered_transients++;
        line_follow.pending_reason = line_follow.pending_ms = 0U;
        /* Do not differentiate across an interval of unknown position. */
        last_position = position;
        derivative = 0.0f;
        last_control_ms = now_ms - CONTROL_PERIOD_MS;
    }
    uint32_t elapsed = now_ms - last_control_ms;
    if (line_sensor.sample_sequence != last_sequence && elapsed >= CONTROL_PERIOD_MS) {
        float dt = (float)elapsed * 0.001f;
        if (marker_exit_pending) {
            derivative=0.0f;last_position=position;marker_exit_pending=0U;
        }
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

void LineFollow_UpdateParking(uint32_t now_ms, float left, float right)
{
    if (line_follow.state!=LF_RUNNING) return;
    line_follow.uptime_ms=now_ms;line_follow.elapsed_ms=now_ms-start_ms;
    line_follow.frame_age_ms=line_sensor.sample_sequence ? now_ms-line_sensor.last_frame_ms : UINT32_MAX;
    if (!line_sensor.cal_valid) { LineFollow_Stop(LF_STOP_NOT_CALIBRATED,now_ms);return; }
    if (line_follow.frame_age_ms>SENSOR_MAX_AGE_MS) { LineFollow_Stop(LF_STOP_SENSOR_STALE,now_ms);return; }
    line_follow.mask=line_sensor.mask;line_follow.pending_reason=line_follow.pending_ms=0U;
    line_follow.left_target_cps=(int32_t)left;line_follow.right_target_cps=(int32_t)right;
    line_follow.steering_cps=(int32_t)((left-right)*0.5f);
    line_follow.base_cps=(uint32_t)((left+right)>=0.0f ? (left+right)*0.5f : -(left+right)*0.5f);
    SpeedControl_SetParkingTargets(left,right);SpeedControl_Update(now_ms);CaptureSpeed();
    if (speed_control.fault!=SPEED_FAULT_NONE) LineFollow_Stop(LF_STOP_SPEED_FAULT,now_ms);
}
