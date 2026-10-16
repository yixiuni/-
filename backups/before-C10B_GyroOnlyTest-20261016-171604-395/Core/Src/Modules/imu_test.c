#include "imu_test.h"
#include "imu_mpu6050.h"
#include "main.h"
#include "button.h"
#include "oled.h"
/* Stationary angle verification firmware. This module never starts motors. */
static Button_State key;
static uint32_t pressed_ms,last_draw;
static uint8_t tracking;
static uint8_t Pressed(void) { return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port,USER_KEY_Pin)==GPIO_PIN_SET; }
static void Decimal(uint32_t row,uint32_t column,int32_t milli)
{
    uint32_t magnitude=milli<0 ? (uint32_t)(-(int64_t)milli) : (uint32_t)milli;
    OLED_Text(row,column,milli<0 ? "-" : "+");OLED_Number(row,column+1U,(int32_t)(magnitude/1000U));
    uint32_t digits=1U;for (uint32_t n=magnitude/1000U;n>=10U;n/=10U) digits++;
    OLED_Text(row,column+1U+digits,".");OLED_Number(row,column+2U+digits,(int32_t)((magnitude/100U)%10U));
}
static void Draw(void)
{
    static const char *const states[]={"STARTING","RESET WAIT","SETTLING","KEEP FLAT + STILL","READY","FAULT","SIGNAL RESET"};
    static const char *const faults[]={"NONE","NO MPU ACK","WRONG DEVICE ID","I2C ERROR","CONFIG MISMATCH","SAMPLE GAP","TURN TOO FAST","CLOCK ERROR","I2C BUS HELD LOW"};
    OLED_Clear();OLED_Text(0,0,"MODE2 ANGLE CHECK");
    OLED_Text(1,0,imu_diag[3]<=IMU_SIGNAL_RESET_WAIT ? states[imu_diag[3]] : "UNKNOWN");
    OLED_Text(2,0,"YAW DEG:");Decimal(2,9,(int32_t)imu_diag[18]);
    OLED_Text(3,0,"RATE D/S:");Decimal(3,10,(int32_t)imu_diag[19]);
    if (imu_diag[3]==IMU_FAULT) {
        OLED_Text(4,0,imu_diag[4]<=IMU_BUS_STUCK ? faults[imu_diag[4]] : "UNKNOWN FAULT");
        OLED_Text(5,0,"WHO DEC:");OLED_Number(5,9,(int32_t)imu_diag[6]);
    } else {
        OLED_Text(4,0,"CAL:");OLED_Number(4,5,(int32_t)imu_diag[20]);OLED_Text(4,9,"/200");
        OLED_Text(5,0,"BIAS D/S:");Decimal(5,10,(int32_t)imu_diag[17]);
    }
    if (imu_diag[3]==IMU_CALIBRATING) {
        uint8_t accel_rail=0U;
        for (uint32_t i=14U;i<17U;i++) {
            int32_t raw=(int32_t)imu_diag[i];
            if (raw==-32768 || raw==32767) accel_rail=1U;
        }
        if (accel_rail) OLED_Text(1,0,"ACCEL AT LIMIT    ");
        else if (imu_diag[30]==1U) OLED_Text(1,0,"GYRO CAL REJECTED ");
        else if (imu_diag[30]==2U) OLED_Text(1,0,"CHECK FLAT/ACCEL  ");
    }
    OLED_Text(6,0,"CLICK:ZERO HOLD:CAL");
    OLED_Text(7,0,imu_diag[3]==IMU_READY ? "TURN CAR BY HAND" : "KEEP CAR STILL");
    OLED_Present();
}
void ImuTest_Init(uint32_t now_ms)
{
    Button_Init(&key,Pressed(),now_ms);tracking=0U;last_draw=now_ms-200U;Imu_Init(now_ms);
}
void ImuTest_Update(uint32_t now_ms)
{
    if (Button_Update(&key,Pressed(),now_ms)) { tracking=1U;pressed_ms=now_ms; }
    if (tracking && !key.stable) {
        tracking=0U;
        if ((uint32_t)(now_ms-pressed_ms)>=2000U) Imu_Calibrate(now_ms);else Imu_Zero();
    }
    Imu_Update(now_ms);
    if ((uint32_t)(now_ms-last_draw)>=200U && OLED_CanDraw()) { Draw();last_draw=now_ms; }
    OLED_Update(now_ms);
}
