#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>
#define SPEED_CONTROL_MAX_CPS 3200U

typedef enum
{
    SPEED_FAULT_NONE = 0,
    SPEED_FAULT_LEFT_NO_FEEDBACK = 1,
    SPEED_FAULT_RIGHT_NO_FEEDBACK = 2,
    SPEED_FAULT_REVERSED_FEEDBACK = 3,
    SPEED_FAULT_SAMPLE_TIMEOUT = 4,
    SPEED_FAULT_BAD_TARGET = 5
} Speed_Fault;

typedef struct
{
    uint32_t running;
    uint32_t fault;
    float requested_cps;
    float reference_cps;
    float left_reference_cps;
    float right_reference_cps;
    float sync_error_counts; /* Left total minus right total, since start */
    float sync_trim_cps;     /* Positive: slow left and speed up right */
    float left_cps;
    float right_cps;
    int32_t left_pwm;
    int32_t right_pwm;
    uint32_t differential_mode;
    float requested_left_cps, requested_right_cps;
} Speed_ControlData;

extern volatile Speed_ControlData speed_control;

/* Forward-only straight test with cumulative wheel-count synchronization.
 * Units: normalized encoder counts/second.
 * Equal wheel counts assume identical encoder resolution and gear ratios.
 * This does not measure heading or compensate wheel diameter/slip.
 * Motor_Init and Encoder_Init must already have run. */
void SpeedControl_Start(float target_cps, uint32_t now_ms);
/* Differential forward drive: independent positive targets <=SPEED_CONTROL_MAX_CPS.
 * Disables equal-count synchronization; keeps PI/feed-forward/fault checks.
 * Changing targets does NOT reset encoder totals, filters, or integrators. */
void SpeedControl_StartDifferential(float left_cps, float right_cps, uint32_t now_ms);
void SpeedControl_SetDifferentialTargets(float left_cps, float right_cps);
/* Call after Encoder_Update, frequently from the same main-loop context. */
void SpeedControl_Update(uint32_t now_ms);
/* Stops immediately, clears integral, retains fault and last feedback. */
void SpeedControl_Stop(void);
/* Explicit bounded final-position mode; signed targets, including zero.
 * Normal line following remains positive-only. No encoder reset here. */
void SpeedControl_EnterParking(void);
void SpeedControl_SetParkingTargets(float left_cps, float right_cps);

#endif
