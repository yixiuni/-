#ifndef POSITION_STOP_H
#define POSITION_STOP_H
#include <stdint.h>
enum { POSITION_STOP_IDLE, POSITION_STOP_MOVING, POSITION_STOP_SETTLING,
       POSITION_STOP_DONE, POSITION_STOP_FAULT };
typedef struct {
    uint32_t state, started_ms, settled_ms, settling;
    float left_target_mm, right_target_mm, left_error_mm, right_error_mm;
    float left_cps, right_cps;
} PositionStop;
void PositionStop_Begin(PositionStop *p, uint32_t now_ms,
    float left_mm, float right_mm, float remaining_mm);
void PositionStop_Update(PositionStop *p, uint32_t now_ms,
    float left_mm, float right_mm, float left_actual_cps, float right_actual_cps);
#endif
