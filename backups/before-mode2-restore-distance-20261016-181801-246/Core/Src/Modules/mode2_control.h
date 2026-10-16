#ifndef MODE2_CONTROL_H
#define MODE2_CONTROL_H
#include <stdint.h>
enum { M2_IDLE, M2_CALIBRATING, M2_TURN, M2_TURN_SETTLE, M2_DRIVE,
       M2_PARK, M2_FINISH_SETTLE, M2_DONE, M2_STOPPED, M2_FAULT };
enum { M2_OK, M2_SENSOR, M2_CAL_TIMEOUT, M2_SPEED_FAULT, M2_TURN_TIMEOUT,
       M2_TURN_DIRECTION, M2_HEADING, M2_DRIVE_TIMEOUT, M2_POSITION,
       M2_CONFIG, M2_MOVED_DURING_CAL, M2_TURN_TRANSLATION, M2_GYRO_RESPONSE };
typedef struct {
    uint32_t now, gyro_state, gyro_failure, sensor_fresh, device_fault, speed_fault;
    uint32_t gyro_age_ms, gyro_last_ms, accel_mask;
    float gyro_bias_dps;
    /* left/right_mm use legacy odometry; straight output is ground-adjusted. */
    float yaw, rate, left_mm, right_mm, left_cps, right_cps;
} Mode2Input;
typedef struct {
    uint32_t state, fault, started, phase_ms, turn_started, drive_started, ended;
    uint32_t turn_corrections, park_corrections;
    float angle, length_mm, yaw_origin, turn_origin, cal_left, cal_right;
    float drive_left, drive_right, distance_mm, remaining_mm, heading_error;
    float turn_left, turn_right;
    float left_target, right_target;
} Mode2Control;
uint8_t Mode2Control_Active(const Mode2Control *m);
uint8_t Mode2Control_Configure(Mode2Control *m,float angle,float distance_mm);
void Mode2Control_Begin(Mode2Control *m,const Mode2Input *in);
void Mode2Control_Stop(Mode2Control *m,uint32_t now);
void Mode2Control_Fail(Mode2Control *m,uint32_t fault,uint32_t now);
void Mode2Control_Update(Mode2Control *m,const Mode2Input *in);
#endif
