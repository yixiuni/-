#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H
#include <stdint.h>

#define LINE_FOLLOW_BASE_CPS 1200U
#define LINE_FOLLOW_MAX_CPS 2400U
enum { LF_IDLE, LF_RUNNING, LF_STOPPED, LF_FAULT, LF_START_REJECTED };
enum { LF_STOP_NONE, LF_STOP_USER, LF_STOP_NOT_CALIBRATED, LF_STOP_SENSOR_STALE,
       LF_STOP_NO_LINE, LF_STOP_AMBIGUOUS, LF_STOP_SPEED_FAULT, LF_STOP_CALIBRATING,
       LF_STOP_LAPS_DONE, LF_STOP_MARKER_TIMEOUT, LF_STOP_NEED_MARKER, LF_STOP_DISTANCE_CAL };

/* 32 words, 128 bytes. Units are encoder counts/sec, not mm/sec.
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
    uint32_t selected_mask, ignored_group_frames, uncertain_frames, pending_reason;
    uint32_t pending_ms, recovered_transients, last_uncertain_mask, last_uncertain_reason;
} LineFollow_Data;
extern volatile LineFollow_Data line_follow;
/* Bounded RAM recorder, no UART/flash writes. Last ~10 seconds at20ms;
 * freezes on stop and survives until next successful start or reset.
 * Header: magic,version,stride,capacity,next,count,total,start,stop,active,
 * sequence,reason,period,target,last_sample,reserved. Sequence stays even
 * outside mutation; the host only accepts a frozen matching snapshot. */
typedef struct {
    uint32_t time_ms;
    int16_t position,steering;
    uint16_t left_target,right_target;
    int16_t left_measured,right_measured;
    uint8_t left_pwm,right_pwm,mask,selected;
    uint16_t age_ms;
    uint8_t state,reason;
    uint16_t s4,s5;
    int16_t derivative;
    uint8_t pending,fault;
} LineFollow_TraceEntry;
typedef struct {
    uint32_t header[16];
    LineFollow_TraceEntry entries[512];
} LineFollow_Trace;
extern volatile LineFollow_Trace line_follow_trace;

void LineFollow_Init(void);
uint8_t LineFollow_Configure(uint32_t cps, uint8_t markers);
void LineFollow_SetCruiseLimit(uint32_t cps);
uint8_t LineFollow_Start(uint32_t now_ms);
void LineFollow_Stop(uint32_t reason, uint32_t now_ms);
void LineFollow_Update(uint32_t now_ms);
#endif
