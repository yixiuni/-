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
/* Independent manual experiment. GYRO_TEST_READOUT never certifies a sensor
 * or grants permission to a motor controller. Operator must ensure stillness. */
enum { GYRO_TEST_WAIT, GYRO_TEST_SETTLE, GYRO_TEST_CAL, GYRO_TEST_READOUT, GYRO_TEST_FAILED };
enum { GYRO_TEST_OK, GYRO_TEST_UNSTABLE, GYRO_TEST_GAP, GYRO_TEST_RAIL, GYRO_TEST_DEVICE, GYRO_TEST_RANGE };
typedef struct {
    uint32_t state, reason, start_ms, last_ms, have_time, first_ms;
    uint32_t samples, restarts, zero_count, zero_ms;
    float mean, m2, low[3], high[3], bias, rate, previous_rate, yaw;
} GyroCheck;
void GyroCheck_Reset(GyroCheck *h);
void GyroCheck_Begin(GyroCheck *h,uint32_t now_ms);
void GyroCheck_Sample(GyroCheck *h,const int16_t gyro[3],uint32_t now_ms);
void GyroCheck_Tick(GyroCheck *h,uint32_t now_ms,uint8_t device_fault);
void GyroCheck_Zero(GyroCheck *h,uint32_t now_ms);
#endif
