#include "lap_counter.h"
#include "odometry_config.h"
#include <limits.h>
uint8_t LapCounter_IsStartMarker(uint32_t mask) { return (mask & 0x3CU)==0x3CU; }
uint8_t LapCounter_IsMarker(uint32_t mask)
{
    /* Any four adjacent probes, including the two edges. */
    mask &= 0xFFU;
    return (mask & (mask>>1U) & (mask>>2U) & (mask>>3U))!=0U;
}
static uint8_t MarkerTail(uint32_t mask, uint32_t seed)
{
    /* A three-probe tail only extends an already observed four-probe
     * stripe. Require three adjacent probes shared with that seed. */
    uint32_t shared=mask & seed & 0xFFU;
    return (shared & (shared>>1U) & (shared>>2U))!=0U;
}

static uint8_t StrongEdge(uint32_t mask, uint32_t strong_mask)
{
    /* Only the OUTERMOST three, all independently >=65% black strength.
     * Neither a central three-probe line nor hysteresis bits are enough. */
    mask &= 0xFFU;
    return (mask==0x07U || mask==0xE0U) && (strong_mask & mask)==mask;
}
static uint8_t EdgeInReturnWindow(const LapCounter *lap, int32_t distance_mm,
    uint32_t mask, uint32_t strong_mask)
{
    int64_t center=(int64_t)distance_mm+LANDMARK_WIDTH_MM/2;
    int64_t expected=(int64_t)lap->center_mm+TRACK_LAP_LENGTH_MM;
    return center>=expected-LANDMARK_EDGE_WINDOW_MM && center<=expected+LANDMARK_EDGE_WINDOW_MM &&
        StrongEdge(mask,strong_mask);
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
static void UpdateWithEvidence(LapCounter *lap, uint32_t now_ms, int32_t distance_mm,
    uint32_t sequence, uint32_t mask, uint32_t strong_mask)
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
    if (sequence==lap->last_sequence) return;
    lap->last_sequence=sequence;
    uint8_t marker=LapCounter_IsMarker(mask);
    uint8_t edge=EdgeInReturnWindow(lap,distance_mm,mask,strong_mask);
    if (!lap->armed) {
        if (marker || (lap->last_marker_edge && StrongEdge(mask,strong_mask))) {
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
    if (lap->candidate && ((uint32_t)(now_ms-lap->candidate_ms)>80U ||
        distance_mm-lap->candidate_mm>60 || distance_mm<lap->candidate_mm ||
        !MarkerTail(mask,lap->candidate_seed_mask) ||
        (lap->candidate_edge && (!edge || mask!=lap->candidate_seed_mask)))) lap->candidate=0U;
    if (!lap->candidate) {
        if (!marker && !edge) return;
        lap->candidate=1U;lap->candidate_frames=1U;lap->candidate_ms=now_ms;lap->candidate_mm=distance_mm;
        lap->candidate_seed_mask=mask;lap->candidate_edge=!marker;
        return;
    }
    lap->candidate_frames++;
    if (lap->candidate_frames<3U || (uint32_t)(now_ms-lap->candidate_ms)<20U ||
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
    lap->last_marker_edge=lap->candidate_edge;
    lap->marker_count++;lap->previous_center_mm=lap->center_mm;
    lap->stop_target_mm=lap->candidate_mm+LANDMARK_WIDTH_MM/2;
    lap->correction_mm=lap->stop_target_mm-lap->center_mm-(int32_t)TRACK_LAP_LENGTH_MM;
    lap->center_mm=lap->stop_target_mm;
    lap->blocked_mm=lap->candidate_mm;
    lap->candidate=0U;lap->armed=0U;lap->clear_active=0U;
    if (lap->marker_count==lap->target) {
        /* Preserve the measured stripe center for odometry correction.
         * Final stopping is LANDMARK_RUNOUT_MM beyond this confirmation snapshot. */
        lap->stop_target_mm=distance_mm+LANDMARK_RUNOUT_MM;
        lap->state=LAP_POSITION;
    }
    else { lap->state=LAP_DEPART;lap->pending_center=1U; }
}
void LapCounter_Update(LapCounter *lap, uint32_t now_ms, int32_t distance_mm,
    uint32_t sequence, uint32_t mask)
{
    /* Mask-only callers cannot synthesize the analog-only edge fallback. */
    UpdateWithEvidence(lap,now_ms,distance_mm,sequence,mask,0U);
}
void LapCounter_UpdateAnalog(LapCounter *lap, uint32_t now_ms, int32_t distance_mm,
    uint32_t sequence, uint32_t mask, const volatile uint32_t strength[8])
{
    uint32_t strong_mask=0U;
    for (uint32_t i=0U;i<8U;i++) if (strength[i]>=LANDMARK_EDGE_STRENGTH) strong_mask|=1U<<i;
    UpdateWithEvidence(lap,now_ms,distance_mm,sequence,mask,strong_mask);
}
uint32_t LapCounter_CruiseCps(const LapCounter *lap, uint32_t selected_cps, int32_t distance_mm)
{
    (void)distance_mm;
    /* Initial launch limit remains; no predicted-finish or runout taper. */
    if (lap->state==LAP_DEPART && lap->marker_count==0U) return LANDMARK_START_CPS;
    return selected_cps;
}
void LapCounter_StopReached(LapCounter *lap, uint32_t now_ms)
{
    if (lap->state!=LAP_POSITION || lap->marker_count!=lap->target) return;
    lap->completed=lap->target;lap->last_lap_ms=now_ms-lap->last_event_ms;
    lap->last_event_ms=now_ms;lap->state=LAP_COMPLETE;lap->armed=0U;
}
