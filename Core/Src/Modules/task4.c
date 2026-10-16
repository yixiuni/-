#include "task4.h"
#include "bt_uart.h"
#include "bt_protocol.h"
#include "car.h"
#include "task2.h"
#include "mode2.h"
#include "line_calibration.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "encoder.h"
#include "speed_control.h"
#include "imu_mpu6050.h"
#include "mode2_distance_config.h"
#include "odometry_config.h"
#include <stdio.h>
#include <limits.h>
volatile uint32_t task4_diag[16];
static BtParser parser;
static uint32_t selected_mode,session,last_id,last_ping,last_tx,last_rx,rx_error_seen;
static uint32_t remote_owned,link_lost,rejected,accepted;
static uint32_t saved_level,saved_laps;
static float saved_angle,saved_distance;
static const char *last_reply;
static uint8_t Active(void)
{
    return selected_mode==1U ? motor_test_result.state==CAR_RUNNING || LineCalibration_IsActive() : Mode2Control_Active(Mode2_GetControl());
}
static uint8_t KeyPending(void) { return selected_mode==1U ? Car_KeyPending() : Mode2_KeyPending(); }
static void Stop(uint32_t now)
{
    if(selected_mode==1U)Car_RemoteStop(now);else Mode2_RemoteStop(now);
    remote_owned=0U;
}
static void Select(uint32_t mode,uint32_t now)
{
    if(mode==selected_mode)return;
    if(selected_mode==1U) { saved_level=task_speed_level;saved_laps=task_target_laps; }
    else { saved_angle=Mode2_GetControl()->angle;saved_distance=Mode2_GetControl()->length_mm; }
    Stop(now);selected_mode=mode;
    if(mode==1U) { Car_Activate(now);task_speed_level=saved_level;task_target_laps=saved_laps; }
    else { Mode2_Init(now);(void)Mode2_SetTargets(saved_angle,saved_distance); }
}
static void Reply(const BtCommand *c,const char *status)
{
    char line[80];int n=snprintf(line,sizeof(line),"A,%lu,%lu,%s\n",(unsigned long)c->session,(unsigned long)c->id,status);
    if(n>0 && (unsigned)n<sizeof(line))(void)BtUart_Send(line);
}
static void Execute(const BtCommand *c,uint32_t now)
{
    /* STOP is idempotent and does not require the current session. */
    if(c->kind==BT_STOP) {
        Stop(now);
        if(c->session==session && c->id>last_id) {last_id=c->id;last_reply="OK";}
        Reply(c,"OK");return;
    }
    if(c->kind==BT_HELLO) {
        if(Active() || KeyPending()) {Reply(c,"BUSY");return;}
        /* Same-session stale HELLO must not reopen already consumed IDs. */
        if(c->session==session && c->id<=last_id) {Reply(c,"STALE");return;}
        session=c->session;last_id=c->id;last_ping=now;last_reply="OK";Reply(c,"OK");return;
    }
    if(!session || c->session!=session) {Reply(c,"SESSION");return;}
    if(c->id<=last_id) {Reply(c,c->id==last_id ? last_reply : "STALE");return;}
    last_id=c->id;last_reply="OK";
    if(c->kind==BT_PING)last_ping=now;
    else if(Active() || KeyPending())last_reply="BUSY";
    else if(c->kind==BT_MODE) {
        if(c->a!=1 && c->a!=2)last_reply="RANGE";else Select((uint32_t)c->a,now);
    } else if(c->kind==BT_SET1) {
        if(selected_mode!=1U)last_reply="MODE";
        else if((c->a!=2400 && c->a!=2800 && c->a!=3200) || c->b<1 || c->b>5)last_reply="RANGE";
        else {task_speed_level=(uint32_t)(c->a-2400)/400U;task_target_laps=(uint32_t)c->b;}
    } else if(c->kind==BT_SET2) {
        if(selected_mode!=2U)last_reply="MODE";
        else if(c->a< -180000 || c->a>180000 || c->b<100 || c->b>2000)last_reply="RANGE";
        else if(!Mode2_SetTargets((float)c->a/1000.0f,(float)c->b))last_reply="RANGE";
    } else if(c->kind==BT_START) {
        if((uint32_t)(now-last_ping)>1500U)last_reply="HEARTBEAT";
        else {
            uint8_t ok=selected_mode==1U ? Car_RemoteStart(now) : Mode2_RemoteStart(now);
            if(ok){remote_owned=1U;link_lost=0U;}else last_reply="START_REJECTED";
        }
    }
    if(last_reply[0]=='O')accepted++;else rejected++;
    Reply(c,last_reply);
}
static int32_t SafeInt(float n)
{
    if(!(n>=-2147480000.0f && n<=2147480000.0f))return INT32_MIN;
    return (int32_t)n;
}
static void Telemetry(uint32_t now)
{
    uint32_t state,fault,target_speed,laps,completed,elapsed,angle_valid=0;
    int32_t distance,speed,angle=0,rate=0,target_angle=0,target_distance=0;
    uint32_t phase=0;
    if(selected_mode==1U) {
        state=task2_data.state;fault=task2_data.stop_reason;target_speed=Task2_TargetCps();
        laps=task_target_laps;completed=task_laps.completed;elapsed=task2_data.elapsed_ms;
        distance=task2_data.distance_mm;speed=task2_data.speed_mm_s;phase=task_laps.state;
    } else {
        const Mode2Control *m=Mode2_GetControl();ImuGyroSample gyro={0};Imu_GetGyroSample(now,&gyro);
        state=m->state;fault=m->fault;target_speed=0;laps=completed=0;
        elapsed=m->state==M2_IDLE ? 0U : (Mode2Control_Active(m) ? now : m->ended)-m->started;
        distance=SafeInt(m->distance_mm);
        speed=SafeInt((encoder_data.left_cps/ODOMETRY_LEFT_COUNTS_PER_MM+encoder_data.right_cps/ODOMETRY_RIGHT_COUNTS_PER_MM)*0.5f*MODE2_STRAIGHT_DISTANCE_SCALE);
        angle=SafeInt((gyro.yaw-m->yaw_origin)*1000.0f);rate=SafeInt(gyro.rate*1000.0f);
        angle_valid=gyro.state==3U && !gyro.reason && !gyro.device_fault && gyro.age_ms<=30U;
        target_angle=SafeInt(m->angle*1000.0f);target_distance=SafeInt(m->length_mm);phase=m->state;
    }
    /* 2 Hz, bounded integer CSV: comfortably below 9600-baud capacity. */
    char line[384];int n=snprintf(line,sizeof(line),
        "T,1,%lu,%lu,%lu,%lu,%ld,%ld,%lu,%lu,%lu,%ld,%ld,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)now,(unsigned long)selected_mode,(unsigned long)state,(unsigned long)fault,
        (long)speed,(long)distance,(unsigned long)target_speed,(unsigned long)laps,(unsigned long)completed,
        (long)angle,(long)rate,(unsigned long)angle_valid,(long)target_angle,(long)target_distance,
        (long)SafeInt(encoder_data.left_cps),(long)SafeInt(encoder_data.right_cps),
        (long)speed_control.left_pwm,(long)speed_control.right_pwm,
        (unsigned long)elapsed,(unsigned long)line_sensor.mask,(unsigned long)line_sensor.cal_valid,
        (unsigned long)remote_owned,(unsigned long)link_lost,(unsigned long)speed_control.fault,
        (unsigned long)phase,(unsigned long)bt_uart_errors,(unsigned long)bt_rx_overflows);
    if(n>0 && (unsigned)n<sizeof(line))(void)BtUart_Send(line);
}
void Task4_Init(uint32_t now)
{
    /* main has already called Car_Init and LineSensor_Init. Boot parked. */
    selected_mode=1U;session=last_id=remote_owned=link_lost=accepted=rejected=0U;
    saved_level=0;saved_laps=1;saved_angle=90;saved_distance=1000;
    parser=(BtParser){0};last_reply="OK";last_tx=now;last_rx=now;last_ping=now;rx_error_seen=0;
    BtUart_Init();
}
void Task4_Update(uint32_t now)
{
    if(remote_owned && (uint32_t)(now-last_ping)>2000U) {
        Stop(now);link_lost=1U;session=0U;last_id=0U;
        /* Buffered commands from the lost session cannot restart motion. */
    }
    uint32_t errors=BtUart_Errors();uint8_t byte=0;
    if(errors!=rx_error_seen) {
        rx_error_seen=errors;BtProtocol_Invalidate(&parser);
        for(uint32_t i=0;i<256U && BtUart_Read(&byte);i++) { /* discard damaged batch */ }
    }
    if(parser.length && (uint32_t)(now-last_rx)>500U)BtProtocol_Invalidate(&parser);
    for(uint32_t i=0;i<64U && BtUart_Read(&byte);i++) {
        last_rx=now;BtCommand c={0};int result=BtProtocol_Feed(&parser,byte,&c);
        if(result==1)Execute(&c,now);else if(result<0)rejected++;
    }
    if(selected_mode==1U){Car_Update(now);Task2_Update(now);}else Mode2_Update(now);
    if(!Active())remote_owned=0U;
    if((uint32_t)(now-last_tx)>=500U) {last_tx=now;Telemetry(now);}
    uint32_t seq=(task4_diag[2]+2U)&~1U;task4_diag[2]=seq-1U;
    task4_diag[0]=0x42543431U;task4_diag[1]=1;task4_diag[3]=selected_mode;task4_diag[4]=session;
    task4_diag[5]=last_id;task4_diag[6]=remote_owned;task4_diag[7]=link_lost;
    task4_diag[8]=bt_rx_bytes;task4_diag[9]=bt_tx_bytes;task4_diag[10]=bt_uart_errors;
    task4_diag[11]=bt_rx_overflows;task4_diag[12]=accepted;task4_diag[13]=rejected;task4_diag[14]=now;
    task4_diag[15]=seq;task4_diag[2]=seq;
}
