#include "mode2_control.h"
#include "imu_heading.h"
static float Abs(float v) { return v<0.0f ? -v : v; }
static float Clamp(float v,float lo,float hi) { return v<lo ? lo : (v>hi ? hi : v); }
uint8_t Mode2Control_Active(const Mode2Control *m) { return m->state>=M2_CALIBRATING && m->state<=M2_FINISH_SETTLE; }
uint8_t Mode2Control_Configure(Mode2Control *m,float angle,float distance_mm)
{
    if (Mode2Control_Active(m) || !(angle>=-180.0f && angle<=180.0f && distance_mm>=100.0f && distance_mm<=2000.0f)) return 0U;
    m->angle=angle;m->length_mm=distance_mm;return 1U;
}
void Mode2Control_Fail(Mode2Control *m,uint32_t fault,uint32_t now)
{
    m->state=M2_FAULT;m->fault=fault;m->ended=now;m->left_target=m->right_target=0.0f;
}
void Mode2Control_Stop(Mode2Control *m,uint32_t now)
{
    m->state=M2_STOPPED;m->ended=now;m->left_target=m->right_target=0.0f;
}
void Mode2Control_Begin(Mode2Control *m,const Mode2Input *in)
{
    float angle=m->angle,length=m->length_mm;
    *m=(Mode2Control){0};m->angle=angle;m->length_mm=length;
    m->started=m->phase_ms=in->now;m->cal_left=in->left_mm;m->cal_right=in->right_mm;
    if (!Mode2Control_Configure(m,angle,length)) { Mode2Control_Fail(m,M2_CONFIG,in->now);return; }
    m->state=M2_CALIBRATING;
}
static void EnterDrive(Mode2Control *m,const Mode2Input *in)
{
    m->drive_left=in->left_mm;m->drive_right=in->right_mm;
    m->distance_mm=0.0f;m->remaining_mm=m->length_mm;
    m->state=M2_DRIVE;m->phase_ms=m->drive_started=in->now;
}
void Mode2Control_Update(Mode2Control *m,const Mode2Input *in)
{
    m->left_target=m->right_target=0.0f;
    if (!Mode2Control_Active(m)) return;
    if (in->device_fault || in->gyro_failure || in->gyro_state==GYRO_TEST_FAILED) { Mode2Control_Fail(m,M2_SENSOR,in->now);return; }
    if (in->speed_fault) { Mode2Control_Fail(m,M2_SPEED_FAULT,in->now);return; }
    if (m->state==M2_CALIBRATING) {
        if (Abs(in->left_mm-m->cal_left)>5.0f || Abs(in->right_mm-m->cal_right)>5.0f) { Mode2Control_Fail(m,M2_MOVED_DURING_CAL,in->now);return; }
        if ((uint32_t)(in->now-m->started)>17000U) { Mode2Control_Fail(m,M2_CAL_TIMEOUT,in->now);return; }
        if (in->gyro_state!=GYRO_TEST_READOUT || !in->sensor_fresh) return;
        m->yaw_origin=m->turn_origin=in->yaw;m->turn_started=m->phase_ms=in->now;
        m->turn_left=in->left_mm;m->turn_right=in->right_mm;
        if (Abs(m->angle)<=0.1f) EnterDrive(m,in);else m->state=M2_TURN;
    }
    if (!in->sensor_fresh || in->gyro_state!=GYRO_TEST_READOUT) { Mode2Control_Fail(m,M2_SENSOR,in->now);return; }
    m->heading_error=m->yaw_origin+m->angle-in->yaw;
    if (m->state==M2_TURN || m->state==M2_TURN_SETTLE) {
        if (Abs((in->left_mm-m->cal_left+in->right_mm-m->cal_right)*0.5f)>80.0f) { Mode2Control_Fail(m,M2_TURN_TRANSLATION,in->now);return; }
        if (Abs(in->left_mm-m->cal_left)+Abs(in->right_mm-m->cal_right)>1200.0f) { Mode2Control_Fail(m,M2_GYRO_RESPONSE,in->now);return; }
        if ((uint32_t)(in->now-m->turn_started)>25000U) { Mode2Control_Fail(m,M2_TURN_TIMEOUT,in->now);return; }
        if (m->state==M2_TURN) {
            if ((uint32_t)(in->now-m->phase_ms)>500U && Abs(in->left_mm-m->turn_left)+Abs(in->right_mm-m->turn_right)>40.0f && Abs(in->yaw-m->turn_origin)<1.0f) {
                Mode2Control_Fail(m,M2_GYRO_RESPONSE,in->now);return;
            }
            float direction=m->yaw_origin+m->angle-m->turn_origin;
            if ((uint32_t)(in->now-m->phase_ms)>300U && direction*(in->yaw-m->turn_origin)< -Abs(direction)*5.0f) {
                Mode2Control_Fail(m,M2_TURN_DIRECTION,in->now);return;
            }
            if (Abs(m->heading_error)<=2.0f) { m->state=M2_TURN_SETTLE;m->phase_ms=in->now;return; }
            float desired=Clamp(m->heading_error*2.0f,-25.0f,25.0f);
            float command=Clamp(10.0f*desired+8.0f*(desired-in->rate),-450.0f,450.0f);
            /* Do not reverse merely to damp: brake/settle close to the target. */
            if (m->heading_error>0.0f) command=Clamp(command,120.0f,450.0f);
            else command=Clamp(command,-450.0f,-120.0f);
            m->left_target=-command;m->right_target=command;return;
        }
        if ((uint32_t)(in->now-m->phase_ms)<500U) return;
        if (Abs(m->heading_error)<=3.0f && Abs(in->rate)<=3.0f && Abs(in->left_cps)<60.0f && Abs(in->right_cps)<60.0f) EnterDrive(m,in);
        else if (m->turn_corrections<3U) {
            m->turn_corrections++;m->turn_origin=in->yaw;m->turn_left=in->left_mm;m->turn_right=in->right_mm;m->state=M2_TURN;m->phase_ms=in->now;return;
        } else { Mode2Control_Fail(m,M2_TURN_TIMEOUT,in->now);return; }
    }
    m->distance_mm=((in->left_mm-m->drive_left)+(in->right_mm-m->drive_right))*0.5f;
    m->remaining_mm=m->length_mm-m->distance_mm;
    if ((uint32_t)(in->now-m->drive_started)>30000U) { Mode2Control_Fail(m,M2_DRIVE_TIMEOUT,in->now);return; }
    if (Abs(m->heading_error)>15.0f) { Mode2Control_Fail(m,M2_HEADING,in->now);return; }
    if (m->distance_mm< -30.0f || m->remaining_mm< -80.0f) { Mode2Control_Fail(m,M2_POSITION,in->now);return; }
    if (m->state==M2_DRIVE && m->remaining_mm<=150.0f) { m->state=M2_PARK;m->phase_ms=in->now; }
    if (m->state==M2_DRIVE) {
        float base=Clamp(m->remaining_mm*5.0f,750.0f,1200.0f);
        float trim=Clamp(10.0f*m->heading_error-3.0f*in->rate,-250.0f,250.0f);
        m->left_target=base-trim;m->right_target=base+trim;return;
    }
    if (m->state==M2_PARK) {
        if (Abs(m->remaining_mm)<=4.0f) { m->state=M2_FINISH_SETTLE;m->phase_ms=in->now;return; }
        float base=Clamp(m->remaining_mm*6.0f,-450.0f,450.0f);
        /* Keep rolling to the position threshold instead of crawling at
         * 120 cps. Heading trim cannot reduce either wheel below 250 cps. */
        if (base>0.0f && base<300.0f) base=300.0f;
        if (base<0.0f && base> -300.0f) base=-300.0f;
        float limit=Clamp(Abs(base)-250.0f,0.0f,120.0f);
        float trim=Clamp(8.0f*m->heading_error-2.0f*in->rate,-limit,limit);
        m->left_target=Clamp(base-trim,-450.0f,450.0f);
        m->right_target=Clamp(base+trim,-450.0f,450.0f);return;
    }
    if (m->state==M2_FINISH_SETTLE && (uint32_t)(in->now-m->phase_ms)>=500U) {
        if (Abs(m->remaining_mm)<=8.0f && Abs(m->heading_error)<=5.0f && Abs(in->rate)<=3.0f && Abs(in->left_cps)<60.0f && Abs(in->right_cps)<60.0f) {
            m->state=M2_DONE;m->ended=in->now;
        } else if (m->park_corrections<2U && Abs(m->heading_error)<=5.0f) {
            m->park_corrections++;m->state=M2_PARK;m->phase_ms=in->now;
        } else Mode2Control_Fail(m,M2_POSITION,in->now);
    }
}
