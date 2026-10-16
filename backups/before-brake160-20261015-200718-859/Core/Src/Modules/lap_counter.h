#ifndef LAP_COUNTER_H
#define LAP_COUNTER_H
#include <stdint.h>
#define TASK_LAPS_MIN 1U
#define TASK_LAPS_MAX 5U
#define LANDMARK_WIDTH_MM 50
#define LANDMARK_START_CPS 1200U
#define LANDMARK_SEARCH_CPS 1200U
/* Distance from the final CONFIRMED return, not the starting stripe. */
#define LANDMARK_RUNOUT_MM 160
#define LANDMARK_APPROACH_START_MM 750
#define LANDMARK_APPROACH_SLOW_MM 325
enum { LAP_IDLE, LAP_DEPART, LAP_CRUISE, LAP_APPROACH, LAP_POSITION, LAP_COMPLETE, LAP_FAULT };
enum { LAP_FAULT_NONE, LAP_FAULT_MISSING, LAP_FAULT_STUCK_MARKER, LAP_FAULT_BAD_DISTANCE,
       LAP_FAULT_BAD_INTERVAL };
typedef struct {
    uint32_t target, completed, armed;
    uint32_t lap_length_mm, target_distance_mm, last_lap_ms, last_event_ms;
    int32_t origin_mm;
    uint32_t state, fault, marker_count, rejected_markers, cruise_cps;
    int32_t center_mm, previous_center_mm, correction_mm, stop_target_mm;
    uint32_t last_sequence, candidate, candidate_frames, candidate_ms;
    int32_t candidate_mm;
    uint32_t clear_active, clear_ms;
    int32_t clear_mm;
    uint32_t pending_center, started_ms;
    int32_t blocked_mm;
} LapCounter;
/* Parked start stays S3..S6. Running landmark detection tolerates one
 * channel of lateral shift; distance/debounce checks still qualify laps. */
uint8_t LapCounter_IsStartMarker(uint32_t mask);
uint8_t LapCounter_IsMarker(uint32_t mask);
void LapCounter_Init(LapCounter *lap, uint32_t target, uint32_t now_ms, int32_t distance_mm);
/* Called with a valid calibrated fresh sample; repeated sequences cannot
 * confirm a landmark. Encoders bound plausibility, never create a lap. */
void LapCounter_Update(LapCounter *lap, uint32_t now_ms, int32_t distance_mm,
    uint32_t sequence, uint32_t mask);
uint32_t LapCounter_CruiseCps(const LapCounter *lap, uint32_t selected_cps, int32_t distance_mm);
void LapCounter_PositionDone(LapCounter *lap, uint32_t now_ms);
#endif
