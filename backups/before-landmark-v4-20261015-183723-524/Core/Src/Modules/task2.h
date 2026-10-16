#ifndef TASK2_H
#define TASK2_H
#include <stdint.h>
#include "lap_counter.h"
#define TASK_SPEED_LEVEL_COUNT 6U
#define TASK_DEFAULT_SPEED_LEVEL 1U /* validated 1200 cps baseline */
extern LapCounter task_laps;
extern uint32_t task_speed_level, task_target_laps, task_key_hint;
/* V3 keeps 32 words. Word 10 is selected laps, not marker status.
 * target_laps/target_distance_mm describe the active or last run;
 * selected_laps describes the next start. */
typedef struct {
    uint32_t magic, version, sequence_begin, state;
    uint32_t stop_reason, speed_level, target_cps, target_laps;
    uint32_t completed_laps, lap_armed, selected_laps, elapsed_ms;
    int32_t mean_counts, distance_mm, actual_cps, speed_mm_s;
    int32_t left_counts, right_counts;
    uint32_t cal_valid, mask, frame_age_ms, odometry_valid;
    uint32_t oled_frames, oled_max_us, key_hint, uptime_ms;
    int32_t left_cps, right_cps;
    uint32_t lap_length_mm, target_distance_mm, last_lap_ms, sequence_end;
} Task2_Data;
extern volatile Task2_Data task2_data;
uint32_t Task2_TargetCps(void);
uint32_t Task2_NextLapSetting(void);
int32_t Task2_DistanceMm(void);
void Task2_Init(uint32_t now_ms);
void Task2_Update(uint32_t now_ms);
#endif
