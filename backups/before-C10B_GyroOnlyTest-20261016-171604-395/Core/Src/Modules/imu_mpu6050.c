#include "imu_mpu6050.h"
#include "imu_heading.h"
#include "main.h"
/* C10B H6, schematic sheet 3: pin 3 PB14/SCL, pin 4 PB15/SDA.
 * Open-drain software I2C. PB9/INT is unused; poll DATA_RDY.
 * Register reference: InvenSense RM-MPU-6000A-00, revision 4.2. */
#define SCL GPIO_PIN_14
#define SDA GPIO_PIN_15
volatile uint32_t imu_diag[32];
volatile uint32_t imu_raw_diag[64];
static ImuHeading heading;
static uint32_t state, fault, phase_ms, poll_ms, publish_ms, last_frame_ms;
static uint32_t samples, errors, misses, max_update_us, zero_count;
static uint32_t transaction_cycles, bus_fault;
static uint8_t address, identity, configured;
static int16_t accel[3], gyro[3];
static uint8_t slow_bus, signal_reset_done, config_readback[9];
static uint8_t normal_bytes[15], slow_bytes[14], ax_bytes[2], gz_bytes[2];
static uint32_t cross_stage, cross_count, cross_errors, cross_begin_ms;
static uint32_t slow_frame_ms, ax_frame_ms, gz_frame_ms;
static uint32_t boot_attempts, boot_failures, bus_clear_count, clear_clocks, recovery_ok;
static uint32_t Cycles(void) { return DWT->CYCCNT; }
static uint32_t PinLevels(void) { uint32_t pins=GPIOB->IDR;return ((pins&SCL)!=0U)|(((pins&SDA)!=0U)<<1U); }
static void Delay(void) { uint32_t n=Cycles();uint32_t ticks=SystemCoreClock/(slow_bus ? 166666U : 333333U);for (uint32_t guard=0;guard<10000U && (uint32_t)(Cycles()-n)<ticks;guard++) __NOP(); }
static void Low(uint32_t pin) { GPIOB->BRR=pin; }
static void Release(uint32_t pin) { GPIOB->BSRR=pin; }
static uint8_t HighClock(void)
{
    Release(SCL);
    uint32_t begin=Cycles();
    for (uint32_t guard=0;guard<20000U;guard++) {
        if ((uint32_t)(Cycles()-transaction_cycles)>SystemCoreClock/(slow_bus ? 125U : 250U)) { bus_fault=2U;return 0U; }
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
static uint8_t RecoverBus(void)
{
    /* NXP UM10204 bus clear: SDA is released for all nine clocks.
     * A temporary high data bit must not terminate the clock sequence. */
    transaction_cycles=Cycles();bus_fault=0U;slow_bus=1U;recovery_ok=0U;
    Release(SDA);
    uint8_t ok=HighClock();
    if (ok) Delay();
    if (ok && !(PinLevels()&2U)) {
        bus_clear_count++;
        for (uint32_t i=0;ok && i<9U;i++) {
            Low(SCL);Delay();ok=HighClock();
            if (ok) { Delay();clear_clocks++; }
        }
        if (ok) Stop();
    }
    if (!bus_fault && PinLevels()!=3U) bus_fault=(PinLevels()&1U) ? 3U : 1U;
    recovery_ok=ok && !bus_fault && PinLevels()==3U;
    Release(SCL|SDA);slow_bus=0U;return (uint8_t)recovery_ok;
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
    /* Bus recovery runs after the initial power-up wait, before each probe. */
    bus_fault=0U;return 1U;
}
static void Fail(uint32_t reason) { fault=reason;state=IMU_FAULT;heading.ready=0U; }
static void BootFailure(uint32_t reason,uint32_t now_ms)
{
    errors++;boot_failures++;phase_ms=now_ms;
    if (boot_attempts>=3U) Fail(reason);
}
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
    seq=(imu_raw_diag[2]+2U)&~1U;imu_raw_diag[2]=seq-1U;
    imu_raw_diag[0]=0x49525731U;imu_raw_diag[1]=2U;imu_raw_diag[3]=cross_count;
    imu_raw_diag[4]=now_ms;imu_raw_diag[5]=last_frame_ms;imu_raw_diag[6]=slow_frame_ms;
    imu_raw_diag[7]=ax_frame_ms;imu_raw_diag[8]=gz_frame_ms;imu_raw_diag[9]=cross_errors;
    imu_raw_diag[10]=signal_reset_done;
    for (uint32_t i=0;i<15U;i++) imu_raw_diag[11+i]=normal_bytes[i];
    for (uint32_t i=0;i<14U;i++) imu_raw_diag[26+i]=slow_bytes[i];
    for (uint32_t i=0;i<2U;i++) { imu_raw_diag[40+i]=ax_bytes[i];imu_raw_diag[42+i]=gz_bytes[i]; }
    for (uint32_t i=0;i<9U;i++) imu_raw_diag[44+i]=config_readback[i];
    imu_raw_diag[53]=cross_stage;imu_raw_diag[54]=configured;
    imu_raw_diag[55]=6U;imu_raw_diag[56]=3U;
    imu_raw_diag[57]=boot_attempts;imu_raw_diag[58]=bus_clear_count;imu_raw_diag[59]=clear_clocks;
    imu_raw_diag[60]=PinLevels();imu_raw_diag[61]=recovery_ok;imu_raw_diag[62]=boot_failures;
    imu_raw_diag[63]=seq;imu_raw_diag[2]=seq;
}
void Imu_Init(uint32_t now_ms)
{
    ImuHeading_Reset(&heading);state=IMU_BOOT;fault=IMU_OK;phase_ms=poll_ms=now_ms;
    samples=errors=misses=max_update_us=zero_count=0U;address=identity=configured=0U;last_frame_ms=0U;
    slow_bus=signal_reset_done=0U;cross_stage=cross_count=cross_errors=0U;cross_begin_ms=now_ms;
    slow_frame_ms=ax_frame_ms=gz_frame_ms=0U;
    boot_attempts=boot_failures=bus_clear_count=clear_clocks=recovery_ok=0U;
    for (uint32_t i=0;i<3U;i++) { accel[i]=0;gyro[i]=0; }
    for (uint32_t i=0;i<9U;i++) config_readback[i]=0U;
    for (uint32_t i=0;i<15U;i++) normal_bytes[i]=0U;
    for (uint32_t i=0;i<14U;i++) slow_bytes[i]=0U;
    for (uint32_t i=0;i<2U;i++) { ax_bytes[i]=0U;gz_bytes[i]=0U; }
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
        boot_attempts++;
        if (!RecoverBus()) { BootFailure(IMU_BUS_STUCK,now_ms);return; }
        uint8_t response=0U;
        for (uint8_t a=0x68U;a<=0x69U;a++) {
            if (ReadRegs(a,0x75U,&identity,1U)) { address=a;response=1U;if (identity==0x68U) break; }
        }
        if (!response) { BootFailure(IMU_NO_ACK,now_ms);return; }
        if (identity!=0x68U) { Fail(IMU_BAD_ID);return; }
        if (!WriteReg(0x6BU,0x80U)) { BootFailure(IMU_BUS_ERROR,now_ms);return; }
        state=IMU_RESET_WAIT;phase_ms=now_ms;return;
    }
    if (state==IMU_RESET_WAIT) {
        if ((uint32_t)(now_ms-phase_ms)<100U) return;
        if (!signal_reset_done) {
            /* RM-MPU-6000A-00 section 4.26: reset analog/digital paths.
             * This write-only, self-clearing register is not readback-tested. */
            if (!WriteReg(0x68U,0x07U)) { errors++;Fail(IMU_BUS_ERROR);return; }
            signal_reset_done=1U;phase_ms=now_ms;state=IMU_SIGNAL_RESET_WAIT;return;
        }
        /* Configure and verify one register per main iteration. */
        static const uint8_t regs[]={0x6B,0x6C,0x19,0x1A,0x1B,0x1C,0x23,0x6A,0x38};
        static const uint8_t vals[]={0x01,0x00,0x09,0x03,0x00,0x00,0x00,0x00,0x01};
        if (configured<sizeof(regs)) {
            uint8_t value=0U;
            if (!WriteReg(regs[configured],vals[configured]) || !ReadRegs(address,regs[configured],&value,1U)) { errors++;Fail(IMU_BUS_ERROR);return; }
            if (value!=vals[configured]) { Fail(IMU_CONFIG_ERROR);return; }
            config_readback[configured]=value;
            configured++;return;
        }
        state=IMU_SETTLE;phase_ms=now_ms;return;
    }
    if (state==IMU_SIGNAL_RESET_WAIT) {
        if ((uint32_t)(now_ms-phase_ms)<100U) return;
        state=IMU_RESET_WAIT;phase_ms=now_ms-100U;return;
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
    for (uint32_t i=0;i<15U;i++) normal_bytes[i]=data[i];
    samples++;last_frame_ms=now_ms;
    uint32_t result=ImuHeading_Sample(&heading,accel,gyro,now_ms);
    if (result==HEADING_GAP) Fail(IMU_SAMPLE_GAP);
    else if (result==HEADING_SATURATED) Fail(IMU_GYRO_SATURATED);
    else state=heading.ready ? IMU_READY : IMU_CALIBRATING;
}
static void CrossRead(uint32_t now_ms)
{
    if ((state!=IMU_CALIBRATING && state!=IMU_READY) || !samples) return;
    if (!cross_stage) {
        if ((uint32_t)(now_ms-cross_begin_ms)<250U) return;
        cross_begin_ms=now_ms;cross_stage=1U;
    }
    /* One extra transaction per loop. Separate start addresses/lengths and
     * slower clock exercise the transport without changing the calibration.
     * These are different measurement instants, not identical-frame checks. */
    uint8_t bytes[14]={0};uint8_t ok=0U;slow_bus=1U;
    if (cross_stage==1U) ok=ReadRegs(address,0x3BU,bytes,14U);
    else if (cross_stage==2U) ok=ReadRegs(address,0x3BU,bytes,2U);
    else ok=ReadRegs(address,0x47U,bytes,2U);
    slow_bus=0U;
    if (!ok) { cross_errors++;cross_stage=0U;return; }
    if (cross_stage==1U) {
        for (uint32_t i=0;i<14U;i++) slow_bytes[i]=bytes[i];
        slow_frame_ms=now_ms;cross_stage=2U;
    } else if (cross_stage==2U) {
        ax_bytes[0]=bytes[0];ax_bytes[1]=bytes[1];ax_frame_ms=now_ms;cross_stage=3U;
    } else {
        gz_bytes[0]=bytes[0];gz_bytes[1]=bytes[1];gz_frame_ms=now_ms;cross_stage=0U;cross_count++;
    }
}
void Imu_Update(uint32_t now_ms)
{
    uint32_t begin=Cycles();Advance(now_ms);CrossRead(now_ms);
    uint32_t us=(uint32_t)(Cycles()-begin)/(SystemCoreClock/1000000U);
    if (us>max_update_us) max_update_us=us;
    if ((uint32_t)(now_ms-publish_ms)>=20U) Publish(now_ms);
}
