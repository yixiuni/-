#include "speed_control.h"
#include "encoder.h"
#include "motor.h"

/* Initial tuning; validate on the actual car before increasing speed.
 * Feed-forward estimates use the measured 50% test: 3346 / 2727.5 cps.
 * PI feedback corrects the error of this approximate linear estimate. */
#define LEFT_FEED_FORWARD  (50.0f / 3346.0f)
#define RIGHT_FEED_FORWARD (50.0f / 2727.5f)
#define SPEED_KP           0.010f
#define SPEED_KI           0.030f
#define PWM_LIMIT         65.0f
#define INTEGRAL_LIMIT    20.0f
#define TARGET_LIMIT_CPS  3000.0f
#define TARGET_RAMP_CPS_S 5000.0f
#define FILTER_TIME_S     0.030f
/* Outer correction for startup wheel travel imbalance, in cps/count.
 * Keep it small relative to the speed loops; it is not a heading sensor. */
#define COUNT_SYNC_KP      1.5f
#define COUNT_SYNC_MAX_CPS 100.0f
#define COUNT_SYNC_MAX_FRACTION 0.05f
#define SAMPLE_TIMEOUT_MS 100U
#define START_GRACE_MS    700U
#define NO_FEEDBACK_MS    250U

volatile Speed_ControlData speed_control;
static float left_integral;
static float right_integral;
static uint32_t start_ms;
static uint32_t last_sample_ms;
static uint32_t last_sample_count;
static uint32_t left_motion_ms;
static uint32_t right_motion_ms;
static uint32_t left_reverse_counts;
static uint32_t right_reverse_counts;

static float Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static int32_t ControlWheel(float reference, float measured, float dt,
                            float feed_forward, float *integral)
{
    float error = reference - measured;
    float candidate = Clamp(*integral + SPEED_KI * error * dt,
                            -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
    float base = feed_forward * reference + SPEED_KP * error;
    float proposed = base + candidate;
    /* Do not integrate further into output saturation; allow unwinding. */
    if ((proposed >= 0.0f && proposed <= PWM_LIMIT) ||
        (proposed > PWM_LIMIT && error < 0.0f) ||
        (proposed < 0.0f && error > 0.0f))
    {
        *integral = candidate;
    }
    return (int32_t)(Clamp(base + *integral, 0.0f, PWM_LIMIT) + 0.5f);
}

void SpeedControl_Stop(void)
{
    Motor_Stop();
    speed_control.running = 0U;
    speed_control.left_pwm = 0;
    speed_control.right_pwm = 0;
    left_integral = 0.0f;
    right_integral = 0.0f;
}

static void StopWithFault(Speed_Fault fault)
{
    speed_control.fault = (uint32_t)fault;
    SpeedControl_Stop();
}

void SpeedControl_Start(float target_cps, uint32_t now_ms)
{
    SpeedControl_Stop();
    speed_control = (Speed_ControlData){0};
    /* This comparison also rejects NaN and infinity. */
    if (!(target_cps > 0.0f && target_cps <= TARGET_LIMIT_CPS))
    {
        StopWithFault(SPEED_FAULT_BAD_TARGET);
        return;
    }
    Encoder_Reset(now_ms);
    start_ms = now_ms;
    last_sample_ms = now_ms;
    last_sample_count = encoder_data.sample_count;
    left_motion_ms = now_ms;
    right_motion_ms = now_ms;
    left_reverse_counts = 0U;
    right_reverse_counts = 0U;
    speed_control.requested_cps = target_cps;
    speed_control.running = 1U;
}

static uint8_t ValidTargets(float left, float right)
{
    return left > 0.0f && left <= TARGET_LIMIT_CPS && right > 0.0f && right <= TARGET_LIMIT_CPS;
}

void SpeedControl_StartDifferential(float left_cps, float right_cps, uint32_t now_ms)
{
    if (!ValidTargets(left_cps, right_cps)) { StopWithFault(SPEED_FAULT_BAD_TARGET); return; }
    SpeedControl_Start((left_cps + right_cps) * 0.5f, now_ms);
    if (!speed_control.running) return;
    speed_control.differential_mode = 1U;
    speed_control.requested_left_cps = left_cps;
    speed_control.requested_right_cps = right_cps;
}

void SpeedControl_SetDifferentialTargets(float left_cps, float right_cps)
{
    if (!speed_control.running || !speed_control.differential_mode || !ValidTargets(left_cps, right_cps))
    {
        StopWithFault(SPEED_FAULT_BAD_TARGET);
        return;
    }
    speed_control.requested_left_cps = left_cps;
    speed_control.requested_right_cps = right_cps;
    speed_control.requested_cps = (left_cps + right_cps) * 0.5f;
}

static float Approach(float value, float target, float step)
{
    return value + Clamp(target - value, -step, step);
}

void SpeedControl_Update(uint32_t now_ms)
{
    if (!speed_control.running) return;
    if ((uint32_t)(now_ms - last_sample_ms) > SAMPLE_TIMEOUT_MS)
    {
        StopWithFault(SPEED_FAULT_SAMPLE_TIMEOUT);
        return;
    }
    if (encoder_data.sample_count == last_sample_count) return;
    if (encoder_data.sample_ms == 0U ||
        encoder_data.sample_ms > SAMPLE_TIMEOUT_MS ||
        (uint32_t)(encoder_data.sample_count - last_sample_count) != 1U)
    {
        StopWithFault(SPEED_FAULT_SAMPLE_TIMEOUT);
        return;
    }
    last_sample_count = encoder_data.sample_count;
    last_sample_ms = now_ms;

    int32_t left_delta = encoder_data.left_delta;
    int32_t right_delta = encoder_data.right_delta;
    if (left_delta > 0) left_motion_ms = now_ms;
    if (right_delta > 0) right_motion_ms = now_ms;
    left_reverse_counts = left_delta < 0 ? left_reverse_counts + (uint32_t)(-left_delta) : 0U;
    right_reverse_counts = right_delta < 0 ? right_reverse_counts + (uint32_t)(-right_delta) : 0U;
    if (left_reverse_counts >= 8U || right_reverse_counts >= 8U)
    {
        StopWithFault(SPEED_FAULT_REVERSED_FEEDBACK);
        return;
    }
    if ((uint32_t)(now_ms - start_ms) >= START_GRACE_MS)
    {
        if ((uint32_t)(now_ms - left_motion_ms) >= NO_FEEDBACK_MS)
        {
            StopWithFault(SPEED_FAULT_LEFT_NO_FEEDBACK);
            return;
        }
        if ((uint32_t)(now_ms - right_motion_ms) >= NO_FEEDBACK_MS)
        {
            StopWithFault(SPEED_FAULT_RIGHT_NO_FEEDBACK);
            return;
        }
    }

    float dt = (float)encoder_data.sample_ms * 0.001f;
    float alpha = dt / (FILTER_TIME_S + dt);
    speed_control.left_cps += alpha * (encoder_data.left_cps - speed_control.left_cps);
    speed_control.right_cps += alpha * (encoder_data.right_cps - speed_control.right_cps);

    if (speed_control.differential_mode)
    {
        float step = TARGET_RAMP_CPS_S * dt;
        speed_control.left_reference_cps = Approach(speed_control.left_reference_cps, speed_control.requested_left_cps, step);
        speed_control.right_reference_cps = Approach(speed_control.right_reference_cps, speed_control.requested_right_cps, step);
        speed_control.reference_cps = (speed_control.left_reference_cps + speed_control.right_reference_cps) * 0.5f;
        /* Intentional turns accumulate unequal travel. Never correct that
         * difference toward zero, even after the line becomes centered. */
        speed_control.sync_error_counts = speed_control.sync_trim_cps = 0.0f;
    }
    else
    {
        speed_control.reference_cps = Clamp(speed_control.reference_cps + TARGET_RAMP_CPS_S * dt,
                                            0.0f, speed_control.requested_cps);
        /* Matching instantaneous speeds alone preserves a startup heading error.
         * Encoder_Reset in Start gives both totals the same starting baseline.
         * A leading left wheel gets a lower target; a leading right wheel gets
         * a lower right target. Opposite trims preserve the mean reference.
         * During the ramp the 5% limit also keeps both targets nonnegative. */
        float trim_limit = Clamp(COUNT_SYNC_MAX_FRACTION * speed_control.reference_cps,
                                 0.0f, COUNT_SYNC_MAX_CPS);
        speed_control.sync_error_counts = (float)(encoder_data.left_total - encoder_data.right_total);
        speed_control.sync_trim_cps = Clamp(COUNT_SYNC_KP * speed_control.sync_error_counts,
                                            -trim_limit, trim_limit);
        speed_control.left_reference_cps = speed_control.reference_cps - speed_control.sync_trim_cps;
        speed_control.right_reference_cps = speed_control.reference_cps + speed_control.sync_trim_cps;
    }

    speed_control.left_pwm = ControlWheel(speed_control.left_reference_cps, speed_control.left_cps,
                                          dt, LEFT_FEED_FORWARD, &left_integral);
    speed_control.right_pwm = ControlWheel(speed_control.right_reference_cps, speed_control.right_cps,
                                           dt, RIGHT_FEED_FORWARD, &right_integral);
    Motor_SetOutput((int16_t)speed_control.left_pwm, (int16_t)speed_control.right_pwm);
}
