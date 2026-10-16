#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H
#include <stdint.h>
enum { IMU_BOOT, IMU_RESET_WAIT, IMU_SETTLE, IMU_CALIBRATING, IMU_READY, IMU_FAULT, IMU_SIGNAL_RESET_WAIT };
enum { IMU_OK, IMU_NO_ACK, IMU_BAD_ID, IMU_BUS_ERROR, IMU_CONFIG_ERROR,
       IMU_SAMPLE_GAP, IMU_GYRO_SATURATED, IMU_CLOCK_ERROR };
extern volatile uint32_t imu_diag[32];
extern volatile uint32_t imu_raw_diag[64];
void Imu_Init(uint32_t now_ms);
void Imu_Update(uint32_t now_ms);
void Imu_Calibrate(uint32_t now_ms);
void Imu_Zero(void);
#endif
