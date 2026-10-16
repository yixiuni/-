#include "task2.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "line_calibration.h"
#include "encoder.h"
#include "oled.h"
#include "odometry_config.h"
#include <limits.h>

LapCounter task_laps;
uint32_t task_speed_level, task_target_laps, task_key_hint;
volatile Task2_Data task2_data;
static uint32_t last_publish, last_draw;
_Static_assert(sizeof(Task2_Data)==128U,"Task2 diagnostic size");
uint32_t Task2_TargetCps(void)
{
    return task_speed_level<TASK_SPEED_LEVEL_COUNT ? 900U+task_speed_level*300U : 1200U;
}
uint32_t Task2_NextLapSetting(void)
{
    return task_target_laps >= TASK_LAPS_MAX || task_target_laps < TASK_LAPS_MIN ?
           TASK_LAPS_MIN : task_target_laps+1U;
}
void Task2_Init(uint32_t now_ms)
{
    task_speed_level=TASK_DEFAULT_SPEED_LEVEL; task_target_laps=1U; task_key_hint=0U;
    task_laps=(LapCounter){0};
    task2_data=(Task2_Data){.magic=0x54324331U,.version=3U};
    last_publish=now_ms-100U; last_draw=now_ms-200U;
    OLED_Init(now_ms);
}
static int32_t Bounded(float v)
{
    if (v > 2147480000.0f) return INT32_MAX;
    if (v < -2147480000.0f) return INT32_MIN;
    return (int32_t)v;
}
int32_t Task2_DistanceMm(void)
{
    float lc=ODOMETRY_LEFT_COUNTS_PER_MM, rc=ODOMETRY_RIGHT_COUNTS_PER_MM;
    if (!(lc>=0.1f && lc<=1000.0f && rc>=0.1f && rc<=1000.0f)) return INT32_MIN;
    return Bounded(((float)encoder_data.left_total/lc+(float)encoder_data.right_total/rc)*0.5f);
}
static void Publish(uint32_t now_ms)
{
    Task2_Data d={0};
    d.magic=0x54324331U; d.version=3U;
    d.sequence_begin=task2_data.sequence_begin+2U;
    d.sequence_end=d.sequence_begin;
    d.state=0U;
    if (line_follow.state==LF_RUNNING) d.state=1U;
    else if (line_follow.stop_reason==LF_STOP_LAPS_DONE) d.state=2U;
    else if (line_follow.state==LF_FAULT) d.state=3U;
    else if (line_follow.state==LF_START_REJECTED) d.state=4U;
    if (LineCalibration_IsActive()) d.state=5U;
    d.stop_reason=line_follow.stop_reason;
    d.speed_level=task_speed_level; d.target_cps=Task2_TargetCps();
    /* Preserve active/finished run target when the next run is adjusted. */
    d.selected_laps=task_target_laps;
    d.target_laps=task_laps.target ? task_laps.target : task_target_laps;
    d.completed_laps=task_laps.completed;
    d.lap_armed=line_follow.state==LF_RUNNING && task_laps.armed;
    d.elapsed_ms=line_follow.elapsed_ms;
    d.left_counts=Bounded((float)encoder_data.left_total);
    d.right_counts=Bounded((float)encoder_data.right_total);
    d.mean_counts=Bounded(((float)encoder_data.left_total+(float)encoder_data.right_total)*0.5f);
    d.left_cps=Bounded(encoder_data.left_cps);d.right_cps=Bounded(encoder_data.right_cps);
    d.actual_cps=Bounded((encoder_data.left_cps+encoder_data.right_cps)*0.5f);
    float lc=ODOMETRY_LEFT_COUNTS_PER_MM, rc=ODOMETRY_RIGHT_COUNTS_PER_MM;
    d.odometry_valid=lc>=0.1f && lc<=1000.0f && rc>=0.1f && rc<=1000.0f;
    d.distance_mm=INT32_MIN; d.speed_mm_s=INT32_MIN;
    if (d.odometry_valid) {
        d.distance_mm=Task2_DistanceMm();
        d.speed_mm_s=Bounded((encoder_data.left_cps/lc+encoder_data.right_cps/rc)*0.5f);
    }
    d.cal_valid=line_sensor.cal_valid; d.mask=line_sensor.mask;
    d.frame_age_ms=line_sensor.sample_sequence ? now_ms-line_sensor.last_frame_ms : UINT32_MAX;
    d.oled_frames=oled_frames; d.oled_max_us=oled_max_update_us;
    d.key_hint=task_key_hint; d.uptime_ms=now_ms;
    d.lap_length_mm=TRACK_LAP_LENGTH_MM;
    d.target_distance_mm=d.target_laps*TRACK_LAP_LENGTH_MM;
    d.last_lap_ms=task_laps.last_lap_ms;
    /* Publish payload while odd; readers require equal EVEN sequence ends. */
    task2_data.sequence_begin=d.sequence_begin-1U;
    d.sequence_begin-=1U;
    task2_data=d;
    task2_data.sequence_begin=d.sequence_end;
}
static void Draw(void)
{
    static const char *const states[]={"PARKED","RUNNING","DONE","FAULT","START REJECTED","LEARNING"};
    static const char *const reasons[]={"READY","USER STOP","LEARN BLACK/WHITE","SENSOR TIMEOUT",
        "LINE LOST","UNCLEAR LINE","MOTOR FAULT","LEARNING","LAPS COMPLETE","STRIPE TOO WIDE","ALIGN START STRIPE","CALIBRATE DISTANCE"};
    OLED_Clear();OLED_Text(0,0,"MODE1");OLED_Text(0,6,states[task2_data.state]);
    if (task2_data.state==1U) {
        OLED_Text(1,0,"LAPS:");OLED_Number(1,6,(int32_t)task2_data.completed_laps);
        OLED_Text(1,7,"/");OLED_Number(1,8,(int32_t)task2_data.target_laps);
    } else {
        OLED_Text(1,0,"SET LAPS:");OLED_Number(1,10,(int32_t)task2_data.selected_laps);
    }
    if (task2_data.odometry_valid) {
        OLED_Text(2,0,"V MM/S:");OLED_Number(2,8,task2_data.speed_mm_s);
        OLED_Text(3,0,"D MM:");OLED_Number(3,6,task2_data.distance_mm);
    } else {
        OLED_Text(2,0,"V CPS:");OLED_Number(2,7,task2_data.actual_cps);
        OLED_Text(3,0,"D CNT:");OLED_Number(3,7,task2_data.mean_counts);
    }
    if (task2_data.state==1U && task2_data.odometry_valid) {
        int32_t remaining=(int32_t)task2_data.target_distance_mm-task2_data.distance_mm;
        OLED_Text(4,0,"LEFT MM:");OLED_Number(4,9,remaining>0 ? remaining : 0);
    } else {
        OLED_Text(4,0,"GOAL MM:");
        OLED_Number(4,9,(int32_t)(task2_data.selected_laps*TRACK_LAP_LENGTH_MM));
    }
    if (task2_data.state==2U) {
        OLED_Text(5,0,"DONE LAPS:");OLED_Number(5,11,(int32_t)task2_data.completed_laps);
        OLED_Text(5,12,"/");OLED_Number(5,13,(int32_t)task2_data.target_laps);
    } else {
        OLED_Text(5,0,!line_sensor.cal_valid ? "LEARN BLACK/WHITE" :
            (task2_data.stop_reason<=LF_STOP_DISTANCE_CAL ? reasons[task2_data.stop_reason] : "UNKNOWN"));
    }
    if (LineCalibration_IsActive()) {
        OLED_Text(6,0,line_sensor.cal_state==LINE_CAL_WHITE ? "HOLD STILL ON WHITE" : "SWEEP BLACK / WHITE");
        OLED_Text(7,0,"REMAIN MS:");OLED_Number(7,11,(int32_t)line_sensor.cal_remaining_ms);
    } else {
        if (task_key_hint==2U) OLED_Text(6,0,"RELEASE: CALIBRATE");
        else if (task_key_hint==1U) {
            OLED_Text(6,0,"RELEASE SET LAPS:");
            OLED_Number(6,17,(int32_t)Task2_NextLapSetting());
        } else OLED_Text(6,0,task2_data.state==1U ? "CLICK TO STOP" : "CLICK START HOLD:LAPS");
        OLED_Text(7,0,"SET CPS:");OLED_Number(7,9,(int32_t)Task2_TargetCps());
    }
    OLED_Present();
}
void Task2_Update(uint32_t now_ms)
{
    if ((uint32_t)(now_ms-last_publish)>=100U) { Publish(now_ms);last_publish=now_ms; }
    if ((uint32_t)(now_ms-last_draw)>=200U && OLED_CanDraw()) { Draw();last_draw=now_ms; }
    OLED_Update(now_ms);
}
