#include "lap_counter.h"
#include "odometry_config.h"
#include <limits.h>
uint8_t LapCounter_IsStartMarker(uint32_t mask) { return (mask & 0x3CU)==0x3CU; }
uint8_t LapCounter_IsMarker(uint32_t mask)
{
    /* Four contiguous probes: S2..S5, S3..S6, or S4..S7. Do not
     * accept an ordinary two/three-probe line or disconnected bits. */
    return (mask & 0x1EU)==0x1EU || (mask & 0x3CU)==0x3CU || (mask & 0x78U)==0x78U;
}
static void FailLap(LapCounter *lap, uint32_t fault)
{
    lap->state=LAP_FAULT;lap->fault=fault;lap->armed=0U;
}
void LapCounter_Init(LapCounter *lap, uint32_t target, uint32_t now_ms, int32_t distance_mm)
{
    *lap=(LapCounter){.target=target,.lap_length_mm=TRACK_LAP_LENGTH_MM,
        .target_distance_mm=target*TRACK_LAP_LENGTH_MM,.last_event_ms=now_ms,
        .origin_mm=distance_mm,.state=LAP_DEPART,.center_mm=distance_mm,.previous_center_mm=distance_mm,
        .last_sequence=UINT32_MAX,.started_ms=now_ms,.blocked_mm=distance_mm};
    if (target<TASK_LAPS_MIN || target>TASK_LAPS_MAX || distance_mm==INT32_MIN)
        FailLap(lap,LAP_FAULT_BAD_DISTANCE);
}
void LapCounter_Update(LapCounter *lap, uint32_t now_ms, int32_t distance_mm,
    uint32_t sequence, uint32_t mask)
{
    if (lap->state==LAP_IDLE || lap->state==LAP_COMPLETE || lap->state==LAP_FAULT || lap->state==LAP_POSITION) return;
    if (distance_mm==INT32_MIN) { FailLap(lap,LAP_FAULT_BAD_DISTANCE);return; }
    int64_t travel=(int64_t)distance_mm-lap->center_mm;
    if (travel < -50 || travel > (int64_t)TRACK_LAP_LENGTH_MM+800) {
        FailLap(lap,LAP_FAULT_MISSING);return;
    }
    /* A confirmed landmark gives its center in raw encoder coordinates.
     * Count only when the probe reaches that center, then rearm after exit. */
    if (lap->pending_center && distance_mm>=lap->stop_target_mm) {
        lap->pending_center=0U;lap->completed++;
        lap->last_lap_ms=now_ms-lap->last_event_ms;lap->last_event_ms=now_ms;
    }
    if (lap->state==LAP_CRUISE && lap->completed+1U==lap->target &&
        travel>=(int64_t)TRACK_LAP_LENGTH_MM-1500) lap->state=LAP_APPROACH;
    if (sequence==lap->last_sequence) return;
    lap->last_sequence=sequence;
    uint8_t marker=LapCounter_IsMarker(mask);
    if (!lap->armed) {
        if (marker) {
            lap->clear_active=0U;
            if (distance_mm-lap->blocked_mm>LANDMARK_WIDTH_MM+150) FailLap(lap,LAP_FAULT_STUCK_MARKER);
            return;
        }
        if (!lap->clear_active) { lap->clear_active=1U;lap->clear_ms=now_ms;lap->clear_mm=distance_mm; }
        if ((uint32_t)(now_ms-lap->clear_ms)>=20U && distance_mm-lap->clear_mm>=8 &&
            distance_mm-lap->center_mm>=LANDMARK_WIDTH_MM/2+10) {
            lap->armed=1U;lap->candidate=0U;lap->state=LAP_CRUISE;
        }
        return;
    }
    if (!marker) { lap->candidate=0U;return; }
    if (!lap->candidate) {
        lap->candidate=1U;lap->candidate_frames=1U;lap->candidate_ms=now_ms;lap->candidate_mm=distance_mm;
        return;
    }
    lap->candidate_frames++;
    if (lap->candidate_frames<3U || (uint32_t)(now_ms-lap->candidate_ms)<10U ||
        distance_mm-lap->candidate_mm<5) return;
    int64_t interval=(int64_t)lap->candidate_mm+LANDMARK_WIDTH_MM/2-lap->center_mm;
    if (interval<(int64_t)TRACK_LAP_LENGTH_MM-500) {
        /* Ignore an early broad reading once, then require a clear exit.
         * Do not reset the last accepted center or mark another lap. */
        lap->rejected_markers++;lap->candidate=0U;lap->armed=0U;
        lap->clear_active=0U;lap->blocked_mm=lap->candidate_mm;
        return;
    }
    if (interval>(int64_t)TRACK_LAP_LENGTH_MM+800) { FailLap(lap,LAP_FAULT_BAD_INTERVAL);return; }
    lap->marker_count++;lap->previous_center_mm=lap->center_mm;
    lap->stop_target_mm=lap->candidate_mm+LANDMARK_WIDTH_MM/2;
    lap->correction_mm=lap->stop_target_mm-lap->center_mm-(int32_t)TRACK_LAP_LENGTH_MM;
    lap->center_mm=lap->stop_target_mm;
    lap->blocked_mm=lap->candidate_mm;
    lap->candidate=0U;lap->armed=0U;lap->clear_active=0U;
    if (lap->marker_count==lap->target) {
        /* Preserve the measured stripe center for odometry correction.
         * Final stopping is 150 mm beyond this confirmation snapshot. */
        lap->stop_target_mm=distance_mm+LANDMARK_RUNOUT_MM;
        lap->state=LAP_POSITION;
    }
    else { lap->state=LAP_DEPART;lap->pending_center=1U; }
}
uint32_t LapCounter_CruiseCps(const LapCounter *lap, uint32_t selected_cps, int32_t distance_mm)
{
    if (lap->state==LAP_DEPART && lap->marker_count==0U) return LANDMARK_START_CPS;
    if (lap->state==LAP_POSITION) {
        int64_t runout=(int64_t)lap->stop_target_mm-distance_mm;
        if (runout<=25) return 600U;
        if (runout>=LANDMARK_RUNOUT_MM) return LANDMARK_SEARCH_CPS;
        return 600U+(uint32_t)((LANDMARK_SEARCH_CPS-600U)*(runout-25)/(LANDMARK_RUNOUT_MM-25));
    }
    if (lap->state!=LAP_APPROACH) return selected_cps;
    int64_t remaining=(int64_t)lap->center_mm+TRACK_LAP_LENGTH_MM-distance_mm;
    /* Reach search speed before the earliest accepted final landmark.
     * The empirical encoder scale only predicts approach, never completion. */
    if (remaining<=650) return LANDMARK_SEARCH_CPS;
    if (remaining>=1500) return selected_cps;
    return LANDMARK_SEARCH_CPS+(uint32_t)((selected_cps-LANDMARK_SEARCH_CPS)*(remaining-650)/850);
}
void LapCounter_PositionDone(LapCounter *lap, uint32_t now_ms)
{
    if (lap->state!=LAP_POSITION || lap->marker_count!=lap->target) return;
    lap->completed=lap->target;lap->last_lap_ms=now_ms-lap->last_event_ms;
    lap->last_event_ms=now_ms;lap->state=LAP_COMPLETE;lap->armed=0U;
}
