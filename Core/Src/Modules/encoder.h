#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_SAMPLE_PERIOD_MS 10U

/* Raw counters are unchanged; deltas, totals and cps are normalized so
 * forward is positive on BOTH wheels (left +1, right -1, measured on car).
 * Counts/revolution still need calibration; cps is NOT cm/s or wheel RPM. */
typedef struct
{
    uint16_t left_raw;
    uint16_t right_raw;
    int32_t left_delta;
    int32_t right_delta;
    int64_t left_total;
    int64_t right_total;
    float left_cps;
    float right_cps;
    uint32_t sample_ms;
    uint32_t sample_count;
} Encoder_Data;

/* Inspect while halted in the debugger. Updated only by the main loop. */
extern volatile Encoder_Data encoder_data;

/* Call after MX_TIM4_Init and MX_TIM8_Init. */
void Encoder_Init(void);
/* Reset software totals/baselines without stopping hardware counters. */
void Encoder_Reset(uint32_t now_ms);
/* Call frequently from main; samples when at least 10 ms has elapsed. */
void Encoder_Update(uint32_t now_ms);
/* Force a sample for a measurement boundary; ignores zero elapsed time.
 * Net movement must remain strictly below 32768 counts between samples.
 * After a long debug halt, stop the motor and reset the baseline. */
void Encoder_Sample(uint32_t now_ms);

#endif
