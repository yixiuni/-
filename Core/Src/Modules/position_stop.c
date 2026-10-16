#include "position_stop.h"
#include "odometry_config.h"
static float Abs(float x) { return x < 0.0f ? -x : x; }
static float PositionSpeed(float error_mm, float measured_cps, float counts_per_mm)
{
    if (Abs(error_mm) <= 2.0f) return 0.0f;
    /* Position outer loop -> signed wheel-speed inner loop. Reverse is
     * confined to final positioning. Completion uses position AND speed. */
    float direction = error_mm > 0.0f ? 1.0f : -1.0f;
    float speed = 4.0f*Abs(error_mm)*counts_per_mm - 0.2f*direction*measured_cps;
    if (speed < 120.0f) speed=120.0f;
    if (speed > 450.0f) speed=450.0f;
    return direction*speed;
}
void PositionStop_Begin(PositionStop *p, uint32_t now_ms,
    float left_mm, float right_mm, float remaining_mm)
{
    *p=(PositionStop){.state=POSITION_STOP_MOVING,.started_ms=now_ms,
        .left_target_mm=left_mm+remaining_mm,.right_target_mm=right_mm+remaining_mm};
    if (!(remaining_mm >= -10.0f && remaining_mm <= 30.0f)) p->state=POSITION_STOP_FAULT;
}
void PositionStop_Update(PositionStop *p, uint32_t now_ms,
    float left_mm, float right_mm, float left_actual_cps, float right_actual_cps)
{
    p->left_cps=p->right_cps=0.0f;
    if (p->state==POSITION_STOP_IDLE || p->state==POSITION_STOP_DONE || p->state==POSITION_STOP_FAULT) return;
    p->left_error_mm=p->left_target_mm-left_mm;
    p->right_error_mm=p->right_target_mm-right_mm;
    if (!(Abs(p->left_error_mm)<=35.0f && Abs(p->right_error_mm)<=35.0f) ||
        (uint32_t)(now_ms-p->started_ms)>10000U) { p->state=POSITION_STOP_FAULT;return; }
    uint8_t in_position=Abs(p->left_error_mm)<=2.0f && Abs(p->right_error_mm)<=2.0f;
    uint8_t stationary=Abs(left_actual_cps)<=60.0f && Abs(right_actual_cps)<=60.0f;
    if (in_position && stationary) {
        if (!p->settling) { p->settling=1U;p->settled_ms=now_ms; }
        p->state=(uint32_t)(now_ms-p->settled_ms)>=200U ? POSITION_STOP_DONE : POSITION_STOP_SETTLING;
        return;
    }
    p->settling=0U;p->state=POSITION_STOP_MOVING;
    p->left_cps=PositionSpeed(p->left_error_mm,left_actual_cps,ODOMETRY_LEFT_COUNTS_PER_MM);
    p->right_cps=PositionSpeed(p->right_error_mm,right_actual_cps,ODOMETRY_RIGHT_COUNTS_PER_MM);
}
