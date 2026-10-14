#ifndef CAR_H
#define CAR_H
#include <stdint.h>

typedef enum
{
    CAR_WAITING = 0,
    CAR_RUNNING = 1,
    CAR_STOPPED = 2,
    CAR_FAULT = 3
} Car_State;

/* Fixed 64-byte layout used by scripts/read_speed_test.ps1 (version 3).
 * Only inspect the numeric results after stopping; they are frozen then. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t state;
    uint32_t fault;
    uint32_t target_cps;
    uint32_t elapsed_ms;
    int32_t left_counts;
    int32_t right_counts;
    int32_t left_mean_cps;
    int32_t right_mean_cps;
    int32_t left_cruise_cps;
    int32_t right_cruise_cps;
    int32_t left_last_pwm;
    int32_t right_last_pwm;
    uint32_t cruise_ms;
    uint32_t samples;
} MotorTestResult;

extern volatile MotorTestResult motor_test_result;
/* After GPIO/TIM initialization. Initializes motor, encoder and button. */
void Car_Init(void);
/* Poll continuously from the main loop. No blocking delays. */
void Car_Update(uint32_t now_ms);
#endif
