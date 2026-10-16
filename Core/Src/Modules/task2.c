#include "task2.h"
#include "line_follow.h"
#include "line_sensor.h"
#include "line_calibration.h"
#include "encoder.h"
#include "oled.h"
#include "odometry_config.h"
#include <limits.h>
#include "position_stop.h"

LapCounter task_laps;
uint32_t task_speed_level, task_target_laps, task_key_hint;
volatile Task2_Data task2_data;
extern PositionStop position_stop;
extern uint32_t landmark_start_ready;
volatile uint32_t landmark_diag[40];
static int32_t CorrectedDistance(void)
{
    int32_t raw=Task2_DistanceMm();
    if (raw==INT32_MIN || !task_laps.target) return raw;
    return raw-task_laps.center_mm+(int32_t)(task_laps.marker_count*TRACK_LAP_LENGTH_MM);
}
static uint32_t last_publish, last_draw;
_Static_assert(sizeof(Task2_Data)==128U,"Task2 diagnostic size");
uint32_t Task2_TargetCps(void)
{
    return task_speed_level<TASK_SPEED_LEVEL_COUNT ? 2400U+task_speed_level*400U : 2400U;
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
    task2_data=(Task2_Data){.magic=0x54324331U,.version=4U};
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
    d.magic=0x54324331U; d.version=4U;
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
        d.distance_mm=CorrectedDistance();
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
    uint32_t seq=landmark_diag[2]+2U;
    landmark_diag[2]=seq-1U;
    landmark_diag[0]=0x4C415031U;landmark_diag[1]=4U;
    landmark_diag[3]=task_laps.state;landmark_diag[4]=task_laps.fault;
    landmark_diag[5]=task_laps.marker_count;landmark_diag[6]=task_laps.rejected_markers;
    landmark_diag[7]=landmark_start_ready;landmark_diag[8]=LANDMARK_WIDTH_MM;
    landmark_diag[9]=(uint32_t)Task2_DistanceMm();landmark_diag[10]=(uint32_t)CorrectedDistance();
    landmark_diag[11]=(uint32_t)task_laps.center_mm;landmark_diag[12]=(uint32_t)task_laps.previous_center_mm;
    landmark_diag[13]=(uint32_t)task_laps.correction_mm;landmark_diag[14]=(uint32_t)task_laps.stop_target_mm;
    landmark_diag[15]=task_laps.armed;landmark_diag[16]=task_laps.pending_center;
    landmark_diag[17]=task_laps.candidate_frames;landmark_diag[18]=position_stop.state;
    landmark_diag[19]=(uint32_t)(int32_t)(position_stop.left_error_mm*1000.0f);
    landmark_diag[20]=(uint32_t)(int32_t)(position_stop.right_error_mm*1000.0f);
    landmark_diag[21]=(uint32_t)(int32_t)position_stop.left_cps;
    landmark_diag[22]=(uint32_t)(int32_t)position_stop.right_cps;
    landmark_diag[23]=0U; /* No position-settle tolerance in immediate brake mode. */landmark_diag[24]=now_ms;landmark_diag[25]=line_sensor.sample_sequence;
    landmark_diag[26]=LapCounter_CruiseCps(&task_laps,Task2_TargetCps(),Task2_DistanceMm());
    landmark_diag[27]=Task2_TargetCps();landmark_diag[28]=task_laps.completed;landmark_diag[29]=task_laps.target;
    landmark_diag[30]=d.elapsed_ms;landmark_diag[31]=task_laps.last_lap_ms;
    landmark_diag[32]=task_laps.candidate_ms;landmark_diag[33]=position_stop.started_ms;
    landmark_diag[34]=(uint32_t)d.left_cps;landmark_diag[35]=(uint32_t)d.right_cps;
    landmark_diag[36]=d.mask;landmark_diag[37]=d.frame_age_ms;landmark_diag[38]=d.stop_reason;
    landmark_diag[39]=seq;landmark_diag[2]=seq;
}
static void Draw(void)
{
    static const char *const states[]={"PARKED","RUNNING","DONE","FAULT","START REJECTED","LEARNING"};
    static const char *const reasons[]={"READY","USER STOP","LEARN BLACK/WHITE","SENSOR TIMEOUT",
        "LINE LOST","UNCLEAR LINE","MOTOR FAULT","LEARNING","LAPS COMPLETE","STRIPE TOO WIDE","ALIGN START STRIPE","CALIBRATE DISTANCE","LANDMARK MISSING","POSITION FAILED"};
    OLED_Clear();OLED_Text(0,0,"MODE1");OLED_Text(0,6,states[task2_data.state]);
    if (task2_data.state==3U) {
        OLED_Text(0,13,"F");OLED_Number(0,14,(int32_t)task2_data.stop_reason);
        if (task2_data.stop_reason==LF_STOP_SPEED_FAULT) {
            OLED_Text(0,17,"M");OLED_Number(0,18,(int32_t)line_follow.speed_fault);
        }
    }
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
    if (task2_data.state==1U) {
        static const char *const phases[]={"WAIT","LEAVE STRIPE","TRACK","APPROACH","POSITION","DONE","FAULT"};
        OLED_Text(4,0,"PHASE:");
        OLED_Text(4,7,position_stop.state==POSITION_STOP_SETTLING ? "SETTLING" :
            (task_laps.state==LAP_POSITION && position_stop.state==POSITION_STOP_IDLE ? "AFTER LINE" :
            (task_laps.state<=LAP_FAULT ? phases[task_laps.state] : "UNKNOWN")));
    } else {
        OLED_Text(4,0,"BRAKE: LINE +16CM");
    }
    if (task2_data.state==2U) {
        OLED_Text(5,0,"DONE LAPS:");OLED_Number(5,11,(int32_t)task2_data.completed_laps);
        OLED_Text(5,12,"/");OLED_Number(5,13,(int32_t)task2_data.target_laps);
    } else {
        OLED_Text(5,0,!line_sensor.cal_valid ? "LEARN BLACK/WHITE" :
            (task2_data.stop_reason==LF_STOP_NONE && task2_data.state==0U && !landmark_start_ready ? "ALIGN STRIPE CENTER" : (task2_data.stop_reason<=LF_STOP_POSITION ? reasons[task2_data.stop_reason] : "UNKNOWN")));
    }
    if (task2_data.state==3U && task2_data.stop_reason==LF_STOP_SPEED_FAULT) {
        static const char *const motor_faults[]={"MOTOR FAULT","LEFT NO FEEDBACK","RIGHT NO FEEDBACK",
            "ENCODER REVERSED","ENCODER TIMEOUT","BAD SPEED TARGET"};
        OLED_Text(5,0,line_follow.speed_fault<6U ? motor_faults[line_follow.speed_fault] : "UNKNOWN MOTOR FAULT");
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
        if (task2_data.state==1U) {
            OLED_Text(7,0,"SET:");OLED_Number(7,4,(int32_t)Task2_TargetCps());
            OLED_Text(7,9,"CMD:");OLED_Number(7,13,(line_follow.left_target_cps+line_follow.right_target_cps)/2);
        } else { OLED_Text(7,0,"SET CPS:");OLED_Number(7,9,(int32_t)Task2_TargetCps()); }
    }
    OLED_Present();
}
void Task2_Update(uint32_t now_ms)
{
    if ((uint32_t)(now_ms-last_publish)>=100U) { Publish(now_ms);last_publish=now_ms; }
    if ((uint32_t)(now_ms-last_draw)>=200U && OLED_CanDraw()) { Draw();last_draw=now_ms; }
    OLED_Update(now_ms);
}
