#include "mode2.h"
#include "mode2_control.h"
#include "imu_mpu6050.h"
#include "imu_heading.h"
#include "main.h"
#include "button.h"
#include "encoder.h"
#include "speed_control.h"
#include "motor.h"
#include "oled.h"
#include "task2.h"
#include "odometry_config.h"
#include "mode2_distance_config.h"
volatile uint32_t mode2_diag[64];
static Mode2Control control;
static Button_State key;
static uint32_t press_ms,last_draw,last_publish,last_encoder_sample,actuator_mode,angle_index;
static uint8_t tracking,consumed;
static uint32_t record_frozen,phase_before_update,max_gyro_age;
/* These totals survive SpeedControl_Start's internal Encoder_Reset. */
static int64_t left_counts,right_counts;
static uint8_t Pressed(void) { return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin)==GPIO_PIN_SET; }
static Mode2Input Input(uint32_t now)
{
    ImuGyroSample gyro={0};Imu_GetGyroSample(now,&gyro);
    Mode2Input in={0};in.now=now;in.gyro_state=gyro.state;in.gyro_failure=gyro.reason;
    in.gyro_age_ms=gyro.age_ms;in.gyro_last_ms=gyro.last_ms;
    in.sensor_fresh=in.gyro_state==GYRO_TEST_READOUT && gyro.age_ms<=30U;
    in.device_fault=gyro.device_fault;in.speed_fault=speed_control.fault;
    in.yaw=gyro.yaw;in.rate=gyro.rate;in.gyro_bias_dps=gyro.bias_dps;in.accel_mask=gyro.accel_mask;
    in.left_mm=(float)left_counts/ODOMETRY_LEFT_COUNTS_PER_MM;
    in.right_mm=(float)right_counts/ODOMETRY_RIGHT_COUNTS_PER_MM;
    in.left_cps=encoder_data.left_cps;in.right_cps=encoder_data.right_cps;return in;
}
static void Brake(void) { SpeedControl_Stop();Motor_Brake();actuator_mode=0U; }
static void Apply(uint32_t now)
{
    uint32_t wanted=control.state==M2_DRIVE ? 2U : ((control.state==M2_TURN || control.state==M2_PARK) ? 1U : 0U);
    if (!wanted) { if (actuator_mode || speed_control.running) Brake();return; }
    if (actuator_mode!=wanted) {
        if (actuator_mode==2U && wanted==1U) SpeedControl_EnterParking();
        else {
            SpeedControl_StartDifferential(wanted==2U ? control.left_target : 450.0f,
                                           wanted==2U ? control.right_target : 450.0f,now);
            last_encoder_sample=encoder_data.sample_count;
            if (wanted==1U && speed_control.running) {
                if (control.state==M2_TURN) SpeedControl_EnterTurn();
                else SpeedControl_EnterParking();
            }
        }
        actuator_mode=wanted;
    }
    if (wanted==1U) SpeedControl_SetParkingTargets(control.left_target,control.right_target);
    else SpeedControl_SetDifferentialTargets(control.left_target,control.right_target);
    SpeedControl_Update(now);
    if (speed_control.fault) { Mode2Control_Fail(&control,M2_SPEED_FAULT,now);Brake(); }
}
static void MirrorStoppedGuard(uint32_t now)
{
    /* Existing flash scripts read task2_data. Never leave PARKED there while
     * this independent mode is running or has a pending automatic start. */
    uint32_t seq=(task2_data.sequence_begin+2U)&~1U;task2_data.sequence_begin=seq-1U;
    task2_data.state=Mode2Control_Active(&control) ? 1U : (control.state==M2_DONE ? 2U : (control.state==M2_FAULT ? 3U : 0U));
    task2_data.stop_reason=control.fault;task2_data.uptime_ms=now;
    task2_data.sequence_end=seq;task2_data.sequence_begin=seq;
}
static void Publish(uint32_t now,const Mode2Input *in)
{
    if (record_frozen) return;
    uint32_t terminal=control.state==M2_DONE || control.state==M2_STOPPED || control.state==M2_FAULT;
    uint32_t seq=(mode2_diag[2]+2U)&~1U;mode2_diag[2]=seq-1U;
    mode2_diag[0]=0x4D324431U;mode2_diag[1]=4U;mode2_diag[3]=control.state;mode2_diag[4]=control.fault;
    mode2_diag[5]=(uint32_t)(int32_t)(control.angle*1000.0f);mode2_diag[6]=(uint32_t)control.length_mm;
    mode2_diag[7]=(uint32_t)(int32_t)((in->yaw-control.yaw_origin)*1000.0f);
    mode2_diag[8]=(uint32_t)(int32_t)(in->rate*1000.0f);
    mode2_diag[9]=(uint32_t)(int32_t)control.distance_mm;mode2_diag[10]=(uint32_t)(int32_t)control.remaining_mm;
    mode2_diag[11]=(uint32_t)(int32_t)control.left_target;mode2_diag[12]=(uint32_t)(int32_t)control.right_target;
    mode2_diag[13]=(uint32_t)(int32_t)in->left_cps;mode2_diag[14]=(uint32_t)(int32_t)in->right_cps;
    mode2_diag[15]=(uint32_t)speed_control.left_pwm;mode2_diag[16]=(uint32_t)speed_control.right_pwm;
    mode2_diag[17]=now;mode2_diag[18]=control.state==M2_IDLE ? 0U : (Mode2Control_Active(&control) ? now : control.ended)-control.started;
    mode2_diag[19]=now-control.phase_ms;mode2_diag[20]=control.turn_corrections;mode2_diag[21]=control.park_corrections;
    mode2_diag[22]=in->gyro_state;mode2_diag[23]=in->gyro_failure;mode2_diag[24]=(uint32_t)(int32_t)(in->gyro_bias_dps*1000.0f);
    mode2_diag[25]=in->accel_mask;mode2_diag[26]=speed_control.fault;
    mode2_diag[27]=(uint32_t)(int32_t)left_counts;mode2_diag[28]=(uint32_t)(int32_t)right_counts;
    mode2_diag[29]=in->gyro_age_ms;mode2_diag[30]=speed_control.running;
    mode2_diag[32]=terminal;mode2_diag[33]=now;mode2_diag[34]=phase_before_update;
    mode2_diag[35]=in->device_fault;mode2_diag[36]=in->gyro_last_ms;
    mode2_diag[37]=in->sensor_fresh;mode2_diag[38]=max_gyro_age;
    for (uint32_t i=39U;i<63U;i++) mode2_diag[i]=0U;
    if (control.fault==M2_SPEED_FAULT) {
        mode2_diag[39]=speed_fault_data.mode;mode2_diag[40]=speed_fault_data.sample_ms;
        mode2_diag[41]=(uint32_t)speed_fault_data.left_pwm;mode2_diag[42]=(uint32_t)speed_fault_data.right_pwm;
        mode2_diag[43]=(uint32_t)speed_fault_data.left_reference;mode2_diag[44]=(uint32_t)speed_fault_data.right_reference;
        mode2_diag[45]=(uint32_t)speed_fault_data.left_cps;mode2_diag[46]=(uint32_t)speed_fault_data.right_cps;
        mode2_diag[47]=speed_fault_data.left_idle_ms;mode2_diag[48]=speed_fault_data.right_idle_ms;
        mode2_diag[49]=(uint32_t)speed_fault_data.left_integral_milli;mode2_diag[50]=(uint32_t)speed_fault_data.right_integral_milli;
        mode2_diag[51]=speed_fault_data.fault;
    }
    mode2_diag[52]=(uint32_t)(MODE2_STRAIGHT_DISTANCE_SCALE*1000000.0f+0.5f);
    mode2_diag[53]=MODE2_DISTANCE_PROVISIONAL;
    mode2_diag[54]=(uint32_t)(ODOMETRY_LEFT_COUNTS_PER_MM/MODE2_STRAIGHT_DISTANCE_SCALE*1000000.0f+0.5f);
    mode2_diag[55]=(uint32_t)(ODOMETRY_RIGHT_COUNTS_PER_MM/MODE2_STRAIGHT_DISTANCE_SCALE*1000000.0f+0.5f);
    if (control.drive_started) {
        mode2_diag[56]=(uint32_t)(int32_t)((in->left_mm-control.drive_left)*ODOMETRY_LEFT_COUNTS_PER_MM);
        mode2_diag[57]=(uint32_t)(int32_t)((in->right_mm-control.drive_right)*ODOMETRY_RIGHT_COUNTS_PER_MM);
    }
    mode2_diag[31]=seq;mode2_diag[63]=seq;mode2_diag[2]=seq;last_publish=now;
    record_frozen=terminal;
}
static void SignedDecimal(uint32_t row,uint32_t col,int32_t milli)
{
    uint32_t a=milli<0 ? (uint32_t)(-(int64_t)milli) : (uint32_t)milli;
    OLED_Text(row,col,milli<0 ? "-" : "+");OLED_Number(row,col+1U,(int32_t)(a/1000U));
    uint32_t digits=1U;for (uint32_t n=a/1000U;n>=10U;n/=10U) digits++;
    OLED_Text(row,col+1U+digits,".");OLED_Number(row,col+2U+digits,(int32_t)((a/100U)%10U));
}
static void Draw(const Mode2Input *in)
{
    static const char *const names[]={"PARKED","CAL KEEP STILL","TURNING","TURN SETTLE","STRAIGHT","POSITION STOP","STOP SETTLE","DONE","STOPPED","FAULT"};
    OLED_Clear();OLED_Text(0,0,"MODE2");OLED_Text(0,6,names[control.state]);
    OLED_Text(1,0,"TARGET DEG:");OLED_Number(1,12,(int32_t)control.angle);
    OLED_Text(2,0,"YAW DEG:");SignedDecimal(2,9,(int32_t)((in->yaw-control.yaw_origin)*1000.0f));
    OLED_Text(3,0,"MM:");OLED_Number(3,4,(int32_t)control.distance_mm);OLED_Text(3,10,"/");OLED_Number(3,12,(int32_t)control.length_mm);
    OLED_Text(4,0,"MM/S:");OLED_Number(4,6,(int32_t)((in->left_cps/ODOMETRY_LEFT_COUNTS_PER_MM+in->right_cps/ODOMETRY_RIGHT_COUNTS_PER_MM)*0.5f*MODE2_STRAIGHT_DISTANCE_SCALE));
    if (control.state==M2_CALIBRATING) { OLED_Text(5,0,"CAL:");OLED_Number(5,5,(int32_t)imu_gyro_test[5]);OLED_Text(5,10,"/400"); }
    else { OLED_Text(5,0,"F:");OLED_Number(5,3,(int32_t)control.fault);OLED_Text(5,7,"S:");OLED_Number(5,10,(int32_t)speed_control.fault); }
    OLED_Text(6,0,"CLICK RUN/STOP HOLD A");
    OLED_Text(7,0,imu_gyro_test[18] ? "GYRO ONLY / ACC BAD" : "MODE2 LOW SPEED TEST");OLED_Present();
}
uint8_t Mode2_SetTargets(float angle,float distance)
{
    if (!Mode2Control_Configure(&control,angle,distance)) return 0U;
    control.state=M2_IDLE;control.fault=0U;control.distance_mm=0.0f;control.remaining_mm=distance;record_frozen=0U;return 1U;
}
void Mode2_Init(uint32_t now)
{
    control=(Mode2Control){0};(void)Mode2_SetTargets(90.0f,1000.0f);
    Button_Init(&key,Pressed(),now);tracking=consumed=0U;angle_index=0U;
    actuator_mode=0U;left_counts=right_counts=0;last_encoder_sample=encoder_data.sample_count;
    record_frozen=phase_before_update=max_gyro_age=0U;
    last_draw=last_publish=now-200U;SpeedControl_Stop();Imu_Init(now);
    Mode2Input in=Input(now);MirrorStoppedGuard(now);Publish(now,&in);
}
void Mode2_Update(uint32_t now)
{
    phase_before_update=control.state;
    Imu_Update(now);Encoder_Update(now);
    if (encoder_data.sample_count!=last_encoder_sample) {
        left_counts+=encoder_data.left_delta;right_counts+=encoder_data.right_delta;last_encoder_sample=encoder_data.sample_count;
    }
    if (Button_Update(&key,Pressed(),now)) {
        tracking=1U;press_ms=now;consumed=Mode2Control_Active(&control);
        if (consumed) { Mode2Control_Stop(&control,now);Brake(); }
    }
    if (tracking && !key.stable) {
        tracking=0U;
        if (!consumed) {
            if ((uint32_t)(now-press_ms)>=2000U) {
                static const float angles[]={90.0f,-90.0f,0.0f,45.0f,180.0f};
                angle_index=(angle_index+1U)%5U;(void)Mode2_SetTargets(angles[angle_index],control.length_mm);
            } else {
                Brake();speed_control.fault=0U;
                record_frozen=max_gyro_age=0U;
                Imu_GyroTestBegin(now);
                Mode2Input start=Input(now);Mode2Control_Begin(&control,&start);
                if (start.gyro_state!=GYRO_TEST_SETTLE) Mode2Control_Fail(&control,M2_SENSOR,now);
            }
        }
    }
    Mode2Input in=Input(now);
    if (Mode2Control_Active(&control) && in.gyro_state==GYRO_TEST_READOUT && in.gyro_age_ms>max_gyro_age) max_gyro_age=in.gyro_age_ms;
    Mode2Control_Update(&control,&in);
    MirrorStoppedGuard(now);Apply(now);MirrorStoppedGuard(now);
    if (control.state!=phase_before_update || control.state>=M2_DONE || (uint32_t)(now-last_publish)>=20U) Publish(now,&in);
    if ((uint32_t)(now-last_draw)>=200U && OLED_CanDraw()) { Draw(&in);last_draw=now; }
    OLED_Update(now);
}
