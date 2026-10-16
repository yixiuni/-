#ifndef LAP_COUNTER_H
#define LAP_COUNTER_H
#include <stdint.h>
#define TASK_LAPS_MIN 1U
#define TASK_LAPS_MAX 5U
typedef struct {
    uint32_t target, completed, armed;
    uint32_t lap_length_mm, target_distance_mm;
    uint32_t last_lap_ms, last_event_ms;
    int32_t origin_mm;
} LapCounter;
/* Marker detection remains available to line steering only. */
uint8_t LapCounter_IsMarker(uint32_t mask);
void LapCounter_Init(LapCounter *lap, uint32_t target, uint32_t now_ms, int32_t distance_mm);
/* Encoder distance only: no mask, sensor-frame, wall-clock or stripe gate.
 * Final completion issues STOP early by measured coast distance.
 * Intermediate laps cross the full geometric length; completion latches. */
uint8_t LapCounter_Update(LapCounter *lap, uint32_t now_ms, int32_t distance_mm);
#endif
