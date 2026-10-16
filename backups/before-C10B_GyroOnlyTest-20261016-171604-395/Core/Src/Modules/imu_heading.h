#ifndef IMU_HEADING_H
#define IMU_HEADING_H
#include <stdint.h>
enum { HEADING_CALIBRATING, HEADING_READY, HEADING_GAP, HEADING_SATURATED };
typedef struct {
    uint32_t samples, restarts, last_ms, first_ms, have_time, ready, reject_reason;
    float mean_z, m2_z, low[3], high[3], bias_raw, yaw_deg, rate_dps, previous_rate;
} ImuHeading;
void ImuHeading_Reset(ImuHeading *h);
uint32_t ImuHeading_Sample(ImuHeading *h, const int16_t accel[3], const int16_t gyro[3], uint32_t now_ms);
void ImuHeading_Zero(ImuHeading *h);
#endif
