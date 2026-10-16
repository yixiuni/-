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
volatile uint32_t mode2_diag[32];
static Mode2Control control;
static Button_State key;
static uint32_t press_ms,last_draw,last_publish,last_encoder_sample,actuator_mode,angle_index;
static uint8_t tracking,consumed;
/* These totals survive SpeedControl_Start's internal Encoder_Reset. */
static int64_t left_counts,right_counts;
static uint8_t Pressed(void) { return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin)==GPIO_PIN_SET; }
static Mode2Input Input(uint32_t now)
{
    Mode2Input in={0};in.now=now;in.gyro_state=imu_gyro_test[3];in.gyro_failure=imu_gyro_test[4];
    in.sensor_fresh=in.gyro_state==GYRO_TEST_READOUT && (uint32_t)(now-imu_gyro_test[8])<=30U;
    in.device_fault=imu_diag[4];in.speed_fault=speed_control.fault;
    in.yaw=(float)(int32_t)imu_gyro_test[12]*0.001f;in.rate=(float)(int32_t)imu_gyro_test[11]*0.001f;
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
            if (wanted==1U && speed_control.running) SpeedControl_EnterParking();
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
    uint32_t seq=(mode2_diag[2]+2U)&~1U;mode2_diag[2]=seq-1U;
    mode2_diag[0]=0x4D324431U;mode2_diag[1]=1U;mode2_diag[3]=control.state;mode2_diag[4]=control.fault;
    mode2_diag[5]=(uint32_t)(int32_t)(control.angle*1000.0f);mode2_diag[6]=(uint32_t)control.length_mm;
    mode2_diag[7]=(uint32_t)(int32_t)((in->yaw-control.yaw_origin)*1000.0f);
    mode2_diag[8]=(uint32_t)(int32_t)(in->rate*1000.0f);
    mode2_diag[9]=(uint32_t)(int32_t)control.distance_mm;mode2_diag[10]=(uint32_t)(int32_t)control.remaining_mm;
    mode2_diag[11]=(uint32_t)(int32_t)control.left_target;mode2_diag[12]=(uint32_t)(int32_t)control.right_target;
    mode2_diag[13]=(uint32_t)(int32_t)in->left_cps;mode2_diag[14]=(uint32_t)(int32_t)in->right_cps;
    mode2_diag[15]=(uint32_t)speed_control.left_pwm;mode2_diag[16]=(uint32_t)speed_control.right_pwm;
    mode2_diag[17]=now;mode2_diag[18]=control.state==M2_IDLE ? 0U : (Mode2Control_Active(&control) ? now : control.ended)-control.started;
    mode2_diag[19]=now-control.phase_ms;mode2_diag[20]=control.turn_corrections;mode2_diag[21]=control.park_corrections;
    mode2_diag[22]=in->gyro_state;mode2_diag[23]=in->gyro_failure;mode2_diag[24]=imu_gyro_test[10];
    mode2_diag[25]=imu_gyro_test[18];mode2_diag[26]=speed_control.fault;
    mode2_diag[27]=(uint32_t)(int32_t)left_counts;mode2_diag[28]=(uint32_t)(int32_t)right_counts;
    mode2_diag[29]=imu_gyro_test[9];mode2_diag[30]=speed_control.running;
    mode2_diag[31]=seq;mode2_diag[2]=seq;last_publish=now;
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
    OLED_Text(4,0,"MM/S:");OLED_Number(4,6,(int32_t)((in->left_cps/ODOMETRY_LEFT_COUNTS_PER_MM+in->right_cps/ODOMETRY_RIGHT_COUNTS_PER_MM)*0.5f));
    if (control.state==M2_CALIBRATING) { OLED_Text(5,0,"CAL:");OLED_Number(5,5,(int32_t)imu_gyro_test[5]);OLED_Text(5,10,"/400"); }
    else { OLED_Text(5,0,"F:");OLED_Number(5,3,(int32_t)control.fault);OLED_Text(5,7,"S:");OLED_Number(5,10,(int32_t)speed_control.fault); }
    OLED_Text(6,0,"CLICK RUN/STOP HOLD A");
    OLED_Text(7,0,imu_gyro_test[18] ? "GYRO ONLY / ACC BAD" : "MODE2 LOW SPEED TEST");OLED_Present();
}
uint8_t Mode2_SetTargets(float angle,float distance)
{
    if (!Mode2Control_Configure(&control,angle,distance)) return 0U;
    control.state=M2_IDLE;control.fault=0U;control.distance_mm=0.0f;control.remaining_mm=distance;return 1U;
}
void Mode2_Init(uint32_t now)
{
    control=(Mode2Control){0};(void)Mode2_SetTargets(90.0f,1000.0f);
    Button_Init(&key,Pressed(),now);tracking=consumed=0U;angle_index=0U;
    actuator_mode=0U;left_counts=right_counts=0;last_encoder_sample=encoder_data.sample_count;
    last_draw=last_publish=now-200U;SpeedControl_Stop();Imu_Init(now);
    Mode2Input in=Input(now);MirrorStoppedGuard(now);Publish(now,&in);
}
void Mode2_Update(uint32_t now)
{
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
                Imu_GyroTestBegin(now);
                Mode2Input start=Input(now);Mode2Control_Begin(&control,&start);
                if (imu_gyro_test[3]!=GYRO_TEST_SETTLE) Mode2Control_Fail(&control,M2_SENSOR,now);
            }
        }
    }
    Mode2Input in=Input(now);Mode2Control_Update(&control,&in);
    MirrorStoppedGuard(now);Apply(now);MirrorStoppedGuard(now);
    if ((uint32_t)(now-last_publish)>=20U) Publish(now,&in);
    if ((uint32_t)(now-last_draw)>=200U && OLED_CanDraw()) { Draw(&in);last_draw=now; }
    OLED_Update(now);
}
