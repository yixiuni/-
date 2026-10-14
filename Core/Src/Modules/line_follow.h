#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H
#include <stdint.h>

#define LINE_FOLLOW_BASE_CPS 1200U
enum { LF_IDLE, LF_RUNNING, LF_STOPPED, LF_FAULT, LF_START_REJECTED };
enum { LF_STOP_NONE, LF_STOP_USER, LF_STOP_NOT_CALIBRATED, LF_STOP_SENSOR_STALE,
       LF_STOP_NO_LINE, LF_STOP_AMBIGUOUS, LF_STOP_SPEED_FAULT, LF_STOP_CALIBRATING };

/* 24 words, 96 bytes. Units are encoder counts/sec, not mm/sec.
 * Confirmed installation: S1 vehicle left, S8 vehicle right.
 * Position -3500=leftmost, +3500=rightmost. Positive steering turns right.
 * Stop diagnostics retain the last position and requested wheel targets;
 * state, actual PWM and speed fault determine whether motors are running. */
typedef struct {
    uint32_t magic, version, state, stop_reason;
    uint32_t mask, active_channels;
    int32_t position_milli, steering_cps;
    uint32_t base_cps, left_target_cps, right_target_cps, frame_age_ms;
    uint32_t updates, uptime_ms, elapsed_ms, sensor_sequence;
    int32_t left_measured_cps, right_measured_cps, left_pwm, right_pwm;
    uint32_t speed_fault, start_denied, user_stops, automatic_stops;
} LineFollow_Data;
extern volatile LineFollow_Data line_follow;

void LineFollow_Init(void);
uint8_t LineFollow_Start(uint32_t now_ms);
void LineFollow_Stop(uint32_t reason, uint32_t now_ms);
void LineFollow_Update(uint32_t now_ms);
#endif
