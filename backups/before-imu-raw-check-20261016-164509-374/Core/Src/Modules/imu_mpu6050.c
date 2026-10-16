#include "imu_mpu6050.h"
#include "imu_heading.h"
#include "main.h"
/* C10B H6, schematic sheet 3: pin 3 PB14/SCL, pin 4 PB15/SDA.
 * Open-drain software I2C. PB9/INT is unused; poll DATA_RDY.
 * Register reference: InvenSense RM-MPU-6000A-00, revision 4.2. */
#define SCL GPIO_PIN_14
#define SDA GPIO_PIN_15
volatile uint32_t imu_diag[32];
static ImuHeading heading;
static uint32_t state, fault, phase_ms, poll_ms, publish_ms, last_frame_ms;
static uint32_t samples, errors, misses, max_update_us, zero_count;
static uint32_t transaction_cycles, bus_fault;
static uint8_t address, identity, configured;
static int16_t accel[3], gyro[3];
static uint32_t Cycles(void) { return DWT->CYCCNT; }
static void Delay(void) { uint32_t n=Cycles();for (uint32_t guard=0;guard<10000U && (uint32_t)(Cycles()-n)<SystemCoreClock/333333U;guard++) __NOP(); }
static void Low(uint32_t pin) { GPIOB->BRR=pin; }
static void Release(uint32_t pin) { GPIOB->BSRR=pin; }
static uint8_t HighClock(void)
{
    Release(SCL);
    uint32_t begin=Cycles();
    for (uint32_t guard=0;guard<20000U;guard++) {
        if ((uint32_t)(Cycles()-transaction_cycles)>SystemCoreClock/250U) { bus_fault=2U;return 0U; }
        if (GPIOB->IDR&SCL) return 1U;
        if ((uint32_t)(Cycles()-begin)>SystemCoreClock/5000U) break;
    }
    bus_fault=1U;return 0U;
}
static void Stop(void)
{
    Low(SCL);Low(SDA);Delay();(void)HighClock();Delay();Release(SDA);Delay();
    Release(SCL|SDA);
}
static uint8_t Start(void)
{
    Release(SDA);Delay();if (!HighClock()) return 0U;Delay();
    if (!(GPIOB->IDR&SDA)) { bus_fault=3U;return 0U; }
    Low(SDA);Delay();Low(SCL);return 1U;
}
static uint8_t WriteByte(uint8_t value)
{
    for (uint32_t i=0;i<8U;i++) {
        if (value&0x80U) Release(SDA);else Low(SDA);
        Delay();if (!HighClock()) return 0U;Delay();Low(SCL);value<<=1;
    }
    Release(SDA);Delay();if (!HighClock()) return 0U;Delay();
    uint8_t ack=(GPIOB->IDR&SDA)==0U;Low(SCL);
    if (!ack) bus_fault=4U;
    return ack;
}
static uint8_t ReadByte(uint8_t *value,uint8_t ack)
{
    *value=0;Release(SDA);
    for (uint32_t i=0;i<8U;i++) {
        Delay();if (!HighClock()) return 0U;Delay();
        *value=(uint8_t)((*value<<1)|((GPIOB->IDR&SDA)!=0U));Low(SCL);
    }
    if (ack) Low(SDA);else Release(SDA);
    Delay();if (!HighClock()) return 0U;Delay();Low(SCL);Release(SDA);return 1U;
}
static uint8_t ReadRegs(uint8_t device,uint8_t reg,uint8_t *data,uint32_t size)
{
    transaction_cycles=Cycles();bus_fault=0U;
    uint8_t ok=Start() && WriteByte((uint8_t)(device<<1)) && WriteByte(reg) && Start() && WriteByte((uint8_t)((device<<1)|1U));
    for (uint32_t i=0;ok && i<size;i++) ok=ReadByte(&data[i],i+1U<size);
    Stop();return ok && !bus_fault;
}
static uint8_t WriteReg(uint8_t reg,uint8_t value)
{
    transaction_cycles=Cycles();bus_fault=0U;
    uint8_t ok=Start() && WriteByte((uint8_t)(address<<1)) && WriteByte(reg) && WriteByte(value);
    Stop();return ok && !bus_fault;
}
static uint8_t InitBus(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    uint32_t before=Cycles();for (volatile uint32_t i=0;i<100U;i++) __NOP();
    if (Cycles()==before) return 0U;
    Release(SCL|SDA);
    GPIO_InitTypeDef pins={0};pins.Pin=SCL|SDA;pins.Mode=GPIO_MODE_OUTPUT_OD;
    pins.Pull=GPIO_NOPULL;pins.Speed=GPIO_SPEED_FREQ_LOW;HAL_GPIO_Init(GPIOB,&pins);
    transaction_cycles=Cycles();bus_fault=0U;
    for (uint32_t i=0;i<9U && !(GPIOB->IDR&SDA);i++) { Low(SCL);Delay();if (!HighClock()) break;Delay(); }
    Stop();return 1U;
}
static void Fail(uint32_t reason) { fault=reason;state=IMU_FAULT;heading.ready=0U; }
static int16_t BigEndian(const uint8_t *data)
{
    uint32_t value=((uint32_t)data[0]<<8U)|data[1];
    return (int16_t)(value>=32768U ? (int32_t)value-65536 : (int32_t)value);
}
static void Publish(uint32_t now_ms)
{
    uint32_t seq=(imu_diag[2]+2U)&~1U;imu_diag[2]=seq-1U;
    imu_diag[0]=0x494D5531U;imu_diag[1]=1U;imu_diag[3]=state;imu_diag[4]=fault;
    imu_diag[5]=address;imu_diag[6]=identity;imu_diag[7]=samples;imu_diag[8]=last_frame_ms;
    imu_diag[9]=now_ms;imu_diag[10]=samples ? now_ms-last_frame_ms : UINT32_MAX;
    for (uint32_t i=0;i<3;i++) { imu_diag[11+i]=(uint32_t)(int32_t)gyro[i];imu_diag[14+i]=(uint32_t)(int32_t)accel[i]; }
    imu_diag[17]=(uint32_t)(int32_t)(heading.bias_raw*1000.0f/131.0f);
    imu_diag[18]=(uint32_t)(int32_t)(heading.yaw_deg*1000.0f);imu_diag[19]=(uint32_t)(int32_t)(heading.rate_dps*1000.0f);
    imu_diag[20]=heading.samples;imu_diag[21]=heading.restarts;imu_diag[22]=errors;
    imu_diag[23]=max_update_us;imu_diag[24]=bus_fault;imu_diag[25]=misses;
    imu_diag[26]=configured==9U ? 0U : UINT32_MAX;imu_diag[27]=configured==9U ? 9U : UINT32_MAX;
    imu_diag[28]=configured==9U ? 3U : UINT32_MAX;imu_diag[29]=zero_count;imu_diag[30]=heading.reject_reason;
    imu_diag[31]=seq;imu_diag[2]=seq;publish_ms=now_ms;
}
void Imu_Init(uint32_t now_ms)
{
    ImuHeading_Reset(&heading);state=IMU_BOOT;fault=IMU_OK;phase_ms=poll_ms=now_ms;
    samples=errors=misses=max_update_us=zero_count=0U;address=identity=configured=0U;last_frame_ms=0U;
    if (!InitBus()) Fail(IMU_CLOCK_ERROR);
    Publish(now_ms);
}
void Imu_Calibrate(uint32_t now_ms)
{
    if (state==IMU_FAULT) { Imu_Init(now_ms);return; }
    if (state!=IMU_CALIBRATING && state!=IMU_READY && state!=IMU_SETTLE) return;
    ImuHeading_Reset(&heading);state=IMU_SETTLE;phase_ms=now_ms;Publish(now_ms);
}
void Imu_Zero(void) { if (state==IMU_READY) { ImuHeading_Zero(&heading);zero_count++; } }
static void Advance(uint32_t now_ms)
{
    if (state==IMU_FAULT) return;
    if (state==IMU_BOOT) {
        if ((uint32_t)(now_ms-phase_ms)<200U) return;
        uint8_t response=0U;
        for (uint8_t a=0x68U;a<=0x69U;a++) {
            if (ReadRegs(a,0x75U,&identity,1U)) { address=a;response=1U;if (identity==0x68U) break; }
        }
        if (!response) { errors++;Fail(IMU_NO_ACK);return; }
        if (identity!=0x68U) { Fail(IMU_BAD_ID);return; }
        if (!WriteReg(0x6BU,0x80U)) { errors++;Fail(IMU_BUS_ERROR);return; }
        state=IMU_RESET_WAIT;phase_ms=now_ms;return;
    }
    if (state==IMU_RESET_WAIT) {
        if ((uint32_t)(now_ms-phase_ms)<100U) return;
        /* Configure and verify one register per main iteration. */
        static const uint8_t regs[]={0x6B,0x6C,0x19,0x1A,0x1B,0x1C,0x23,0x6A,0x38};
        static const uint8_t vals[]={0x01,0x00,0x09,0x03,0x00,0x00,0x00,0x00,0x01};
        if (configured<sizeof(regs)) {
            uint8_t value=0U;
            if (!WriteReg(regs[configured],vals[configured]) || !ReadRegs(address,regs[configured],&value,1U)) { errors++;Fail(IMU_BUS_ERROR);return; }
            if (value!=vals[configured]) { Fail(IMU_CONFIG_ERROR);return; }
            configured++;return;
        }
        state=IMU_SETTLE;phase_ms=now_ms;return;
    }
    if (state==IMU_SETTLE) {
        if ((uint32_t)(now_ms-phase_ms)<200U) return;
        state=IMU_CALIBRATING;poll_ms=now_ms-5U;last_frame_ms=now_ms;
    }
    if ((uint32_t)(now_ms-poll_ms)<5U) return;
    poll_ms=now_ms;
    uint8_t data[15]={0};
    if (!ReadRegs(address,0x3AU,data,sizeof(data))) { errors++;Fail(IMU_BUS_ERROR);return; }
    if (!(data[0]&1U)) {
        misses++;if ((uint32_t)(now_ms-last_frame_ms)>50U) Fail(IMU_SAMPLE_GAP);
        return;
    }
    for (uint32_t i=0;i<3;i++) { accel[i]=BigEndian(&data[1+i*2]);gyro[i]=BigEndian(&data[9+i*2]); }
    samples++;last_frame_ms=now_ms;
    uint32_t result=ImuHeading_Sample(&heading,accel,gyro,now_ms);
    if (result==HEADING_GAP) Fail(IMU_SAMPLE_GAP);
    else if (result==HEADING_SATURATED) Fail(IMU_GYRO_SATURATED);
    else state=heading.ready ? IMU_READY : IMU_CALIBRATING;
}
void Imu_Update(uint32_t now_ms)
{
    uint32_t begin=Cycles();Advance(now_ms);
    uint32_t us=(uint32_t)(Cycles()-begin)/(SystemCoreClock/1000000U);
    if (us>max_update_us) max_update_us=us;
    if ((uint32_t)(now_ms-publish_ms)>=20U) Publish(now_ms);
}
