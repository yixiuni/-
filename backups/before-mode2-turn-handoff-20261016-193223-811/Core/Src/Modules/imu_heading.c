#include "imu_heading.h"
/* MPU6050: gyro FS_SEL=0 (131 LSB/dps), accelerometer AFS_SEL=0.
 * Relative yaw around sensor Z only; module must lie flat on the car.
 * No accelerometer-derived heading or fictitious absolute yaw correction. */
static int32_t AbsRaw(int16_t value) { return value<0 ? -(int32_t)value : value; }
void ImuHeading_Reset(ImuHeading *h) { *h=(ImuHeading){0}; }
static void RestartWindow(ImuHeading *h, uint32_t reason)
{
    h->samples=0U;h->mean_z=h->m2_z=0.0f;h->restarts++;h->reject_reason=reason;
}
uint32_t ImuHeading_Sample(ImuHeading *h, const int16_t accel[3], const int16_t gyro[3], uint32_t now_ms)
{
    uint32_t dt=h->have_time ? now_ms-h->last_ms : 0U;
    if (h->have_time && !dt) return h->ready ? HEADING_READY : HEADING_CALIBRATING;
    h->last_ms=now_ms;h->have_time=1U;
    if (h->ready) {
        if (dt>30U) { h->ready=0U;return HEADING_GAP; }
        for (uint32_t i=0;i<3;i++) if (AbsRaw(gyro[i])>=32700) { h->ready=0U;return HEADING_SATURATED; }
        h->rate_dps=((float)gyro[2]-h->bias_raw)/131.0f;
        h->yaw_deg+=(h->previous_rate+h->rate_dps)*0.0005f*(float)dt;
        h->previous_rate=h->rate_dps;
        return HEADING_READY;
    }
    if (dt>30U && h->samples) RestartWindow(h,3U);
    uint8_t moving=0U;
    for (uint32_t i=0;i<3;i++) if (AbsRaw(gyro[i])>655) moving=1U;
    if (moving) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
    if (AbsRaw(accel[0])>4000 || AbsRaw(accel[1])>4000 || AbsRaw(accel[2])<14000 || AbsRaw(accel[2])>18500) {
        RestartWindow(h,2U);return HEADING_CALIBRATING;
    }
    if (!h->samples) {
        h->first_ms=now_ms;
        for (uint32_t i=0;i<3;i++) h->low[i]=h->high[i]=(float)gyro[i];
    }
    for (uint32_t i=0;i<3;i++) {
        if ((float)gyro[i]<h->low[i]) h->low[i]=(float)gyro[i];
        if ((float)gyro[i]>h->high[i]) h->high[i]=(float)gyro[i];
        if (h->high[i]-h->low[i]>196.5f) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
    }
    h->samples++;
    float delta=(float)gyro[2]-h->mean_z;
    h->mean_z+=delta/(float)h->samples;
    h->m2_z+=delta*((float)gyro[2]-h->mean_z);
    if (h->samples>=200U && (uint32_t)(now_ms-h->first_ms)>=1900U) {
        if (h->m2_z/(float)h->samples>1544.49f) { RestartWindow(h,1U);return HEADING_CALIBRATING; }
        h->bias_raw=h->mean_z;h->yaw_deg=0.0f;
        h->rate_dps=((float)gyro[2]-h->bias_raw)/131.0f;
        h->previous_rate=h->rate_dps;h->ready=1U;h->reject_reason=0U;
    }
    return h->ready ? HEADING_READY : HEADING_CALIBRATING;
}
void ImuHeading_Zero(ImuHeading *h) { if (h->ready) h->yaw_deg=0.0f; }

void GyroCheck_Reset(GyroCheck *h) { *h=(GyroCheck){0}; }
void GyroCheck_Begin(GyroCheck *h,uint32_t now_ms)
{
    GyroCheck_Reset(h);h->state=GYRO_TEST_SETTLE;h->start_ms=h->last_ms=now_ms;
}
static void GyroCheck_Fail(GyroCheck *h,uint32_t reason)
{
    h->state=GYRO_TEST_FAILED;h->reason=reason;
}
static void GyroCheck_Restart(GyroCheck *h)
{
    h->samples=0U;h->mean=h->m2=0.0f;h->restarts++;
}
void GyroCheck_Tick(GyroCheck *h,uint32_t now_ms,uint8_t device_fault)
{
    if (h->state==GYRO_TEST_WAIT || h->state==GYRO_TEST_FAILED) return;
    if (device_fault) GyroCheck_Fail(h,GYRO_TEST_DEVICE);
    else if ((uint32_t)(now_ms-h->last_ms)>50U) GyroCheck_Fail(h,GYRO_TEST_GAP);
    else if (h->state!=GYRO_TEST_READOUT && (uint32_t)(now_ms-h->start_ms)>15000U) GyroCheck_Fail(h,GYRO_TEST_UNSTABLE);
}
void GyroCheck_Sample(GyroCheck *h,const int16_t gyro[3],uint32_t now_ms)
{
    if (h->state==GYRO_TEST_WAIT || h->state==GYRO_TEST_FAILED) return;
    uint32_t dt=h->have_time ? now_ms-h->last_ms : 0U;
    if (h->have_time && !dt) return;
    h->last_ms=now_ms;h->have_time=1U;
    if (dt>30U) { GyroCheck_Fail(h,GYRO_TEST_GAP);return; }
    for (uint32_t i=0;i<3U;i++) if (AbsRaw(gyro[i])>=32700) { GyroCheck_Fail(h,GYRO_TEST_RAIL);return; }
    if (h->state==GYRO_TEST_READOUT) {
        h->rate=((float)gyro[2]-h->bias)/131.0f;
        h->yaw+=(h->previous_rate+h->rate)*0.0005f*(float)dt;h->previous_rate=h->rate;
        if (h->yaw>36000.0f || h->yaw< -36000.0f) GyroCheck_Fail(h,GYRO_TEST_RANGE);
        return;
    }
    if ((uint32_t)(now_ms-h->start_ms)>15000U) { GyroCheck_Fail(h,GYRO_TEST_UNSTABLE);return; }
    if (h->state==GYRO_TEST_SETTLE) {
        if ((uint32_t)(now_ms-h->start_ms)<1000U) return;
        h->state=GYRO_TEST_CAL;
    }
    if (!h->samples) {
        h->first_ms=now_ms;
        for (uint32_t i=0;i<3U;i++) h->low[i]=h->high[i]=(float)gyro[i];
    }
    for (uint32_t i=0;i<3U;i++) {
        if ((float)gyro[i]<h->low[i]) h->low[i]=(float)gyro[i];
        if ((float)gyro[i]>h->high[i]) h->high[i]=(float)gyro[i];
        if (h->high[i]-h->low[i]>196.5f) { GyroCheck_Restart(h);return; }
    }
    h->samples++;float delta=(float)gyro[2]-h->mean;
    h->mean+=delta/(float)h->samples;h->m2+=delta*((float)gyro[2]-h->mean);
    if (h->samples>=400U && (uint32_t)(now_ms-h->first_ms)>=3900U) {
        if (h->m2/(float)h->samples>1544.49f) { GyroCheck_Restart(h);return; }
        h->bias=h->mean;h->rate=((float)gyro[2]-h->bias)/131.0f;
        h->previous_rate=h->rate;h->yaw=0.0f;h->zero_ms=now_ms;h->state=GYRO_TEST_READOUT;
    }
}
void GyroCheck_Zero(GyroCheck *h,uint32_t now_ms)
{
    if (h->state==GYRO_TEST_READOUT) { h->yaw=0.0f;h->zero_ms=now_ms;h->zero_count++; }
}
