#include "lap_counter.h"
#include "odometry_config.h"
#include <limits.h>
uint8_t LapCounter_IsMarker(uint32_t mask) { return (mask & 0x7EU) == 0x7EU; }
void LapCounter_Init(LapCounter *lap, uint32_t target, uint32_t now_ms, int32_t distance_mm)
{
    uint8_t valid = target >= TASK_LAPS_MIN && target <= TASK_LAPS_MAX && distance_mm != INT32_MIN &&
                    (int64_t)TRACK_LAP_LENGTH_MM-TRACK_STOP_COAST_MM+TRACK_FINISH_TRIM_MM > 0;
    *lap = (LapCounter){ .target=target, .armed=valid,
        .lap_length_mm=TRACK_LAP_LENGTH_MM,
        .target_distance_mm=valid ? target*TRACK_LAP_LENGTH_MM : 0U,
        .last_event_ms=now_ms, .origin_mm=distance_mm };
}
uint8_t LapCounter_Update(LapCounter *lap, uint32_t now_ms, int32_t distance_mm)
{
    if (lap->target && lap->completed >= lap->target) return 1U;
    if (!lap->armed || distance_mm == INT32_MIN) return 0U;
    int64_t progress=(int64_t)distance_mm-lap->origin_mm;
    if (progress < 0) return 0U;
    if (progress >= (int64_t)lap->target_distance_mm-TRACK_STOP_COAST_MM+TRACK_FINISH_TRIM_MM) {
        lap->completed=lap->target;
        lap->last_lap_ms=now_ms-lap->last_event_ms;
        lap->last_event_ms=now_ms;
        lap->armed=0U;
        return 1U;
    }
    uint32_t completed=(uint32_t)(progress/lap->lap_length_mm);
    /* A positive finish trim may extend past the nominal last lap.
     * Do not publish/latch completion until the adjusted stop threshold. */
    if (completed >= lap->target) completed=lap->target-1U;
    if (completed > lap->completed) {
        lap->completed=completed;
        lap->last_lap_ms=now_ms-lap->last_event_ms;
        lap->last_event_ms=now_ms;
    }
    return 0U;
}
