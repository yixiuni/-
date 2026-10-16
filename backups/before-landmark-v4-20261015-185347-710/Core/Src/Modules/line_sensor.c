#include "line_sensor.h"
#include "line_calibration.h"
#include "usart.h"

/* Hiwonder manual mode: 00 once, then 02 analog requests.
 * Reply: 55 AA 02 10, eight little-endian uint16 values, checksum.
 * Checksum = low byte of ~(command + length + all payload bytes).
 * https://docs.hiwonder.com/projects/8-ch-Line-Follower/en/latest/docs/2.Quick_Start.html
 * UART4 is owned here; do not also call HAL_UART_Receive[_IT]. */
#define RX_ERROR_BITS (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)
#define BOOT_LISTEN_MS 1000U
#define COMMAND_PERIOD_MS 10U
#define RESPONSE_TIMEOUT_MS 250U
#define BYTE_TIMEOUT_MS 30U
#define FRESH_MS 500U

volatile LineSensor_Data line_sensor;
_Static_assert(sizeof(LineSensor_Data) == 464U, "V5 diagnostic layout");
static uint32_t boot_ms, previous_update_ms, consumed_sequence;
static volatile uint32_t pending_request, received_sequence, received_ms;
static volatile uint16_t received_values[8];
/* Parser belongs solely to ISR, except reset while UART4 IRQ is disabled. */
static uint8_t packet[21];
static uint32_t used, previous_byte_ms;

void LineSensor_Init(uint32_t now_ms)
{
    LineSensor_Data empty = {0};
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    line_sensor = empty;
    line_sensor.magic = 0x4C4E4631U;
    line_sensor.version = 5U;
    line_sensor.age_ms = line_sensor.frame_age_ms = UINT32_MAX;
    line_sensor.last_tx_byte = UINT32_MAX;
    boot_ms = previous_update_ms = now_ms;
    consumed_sequence = received_sequence = pending_request = used = 0U;
    received_ms = previous_byte_ms = now_ms;
    LineCalibration_Init();
    UART4->CR3 |= USART_CR3_EIE;
    UART4->CR1 |= USART_CR1_RXNEIE | USART_CR1_PEIE;
    HAL_NVIC_SetPriority(UART4_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}

static void ParseByte(uint8_t byte, uint32_t now_ms)
{
    if (used && (uint32_t)(now_ms - previous_byte_ms) > BYTE_TIMEOUT_MS) {
        used = 0U;
        line_sensor.frame_timeouts++;
    }
    previous_byte_ms = now_ms;
    /* Sliding header search also recovers if corrupt data contains 55 AA. */
    packet[used++] = byte;
    while (used) {
        uint8_t invalid = packet[0] != 0x55U;
        if (used >= 2U && packet[1] != 0xAAU) invalid = 1U;
        if (used >= 3U && packet[2] != 0x02U) invalid = 1U;
        if (used >= 4U && packet[3] != 16U) invalid = 1U;
        if (!invalid && used < sizeof(packet)) return;
        if (!invalid) {
            uint8_t sum = 0U;
            for (uint32_t i = 2U; i < 20U; i++) sum = (uint8_t)(sum + packet[i]);
            if (packet[20] == (uint8_t)(sum ^ 0xFFU)) {
                for (uint32_t i = 0U; i < 8U; i++)
                    received_values[i] = (uint16_t)(packet[4U + 2U*i] | ((uint16_t)packet[5U + 2U*i] << 8));
                received_ms = now_ms;
                received_sequence++;
                line_sensor.valid_frames++;
                pending_request = 0U;
                used = 0U;
                return;
            }
            line_sensor.checksum_errors++;
        }
        line_sensor.frame_errors++;
        used--;
        for (uint32_t i = 0U; i < used; i++) packet[i] = packet[i + 1U];
    }
}

static void CaptureByte(uint32_t sr, uint8_t byte, uint32_t now_ms)
{
    line_sensor.last_rx_sr = sr;
    if (sr & USART_SR_RXNE) {
        line_sensor.rx_bytes++;
        if (line_sensor.tx_commands == 0U) line_sensor.rx_before_first_tx++;
        if (line_sensor.first_count < 16U) {
            line_sensor.first_bytes[line_sensor.first_count] = byte;
            line_sensor.first_count++;
        }
        line_sensor.recent_bytes[line_sensor.recent_next] = byte;
        line_sensor.recent_next = (line_sensor.recent_next + 1U) % 16U;
        if (line_sensor.last_count < 16U) line_sensor.last_count++;
    }
    if (sr & RX_ERROR_BITS) {
        line_sensor.uart_errors++;
        if (sr & USART_SR_ORE) line_sensor.overrun_errors++;
        if (sr & USART_SR_FE) line_sensor.framing_errors++;
        if (sr & USART_SR_NE) line_sensor.noise_errors++;
        if (sr & USART_SR_PE) line_sensor.parity_errors++;
        used = 0U;
        return;
    }
    if (sr & USART_SR_RXNE) {
        line_sensor.last_rx_ms = now_ms;
        line_sensor.good_bytes++;
        ParseByte(byte, now_ms);
    }
}

void LineSensor_IRQHandler(void)
{
    uint32_t sr = UART4->SR;
    if (sr & (USART_SR_RXNE | RX_ERROR_BITS)) {
        uint8_t byte = (uint8_t)UART4->DR;
        CaptureByte(sr, byte, HAL_GetTick());
    }
}

static uint32_t Age(uint32_t now_ms, uint32_t then_ms)
{
    uint32_t age = now_ms - then_ms;
    return age > 0x7FFFFFFFU ? 0U : age;
}

void LineSensor_Update(uint32_t now_ms)
{
    uint32_t gap = now_ms - previous_update_ms;
    previous_update_ms = now_ms;
    if (gap > line_sensor.max_main_gap_ms) line_sensor.max_main_gap_ms = gap;
    line_sensor.now_ms = now_ms;
    uint16_t values[8] = {0};
    uint32_t sequence = 0U, frame_ms = 0U;
    /* Eight halfwords only: never disable interrupts across calibration or
     * flash operations. The foreground consumes complete frames atomically. */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    sequence = received_sequence;
    frame_ms = received_ms;
    if (sequence != consumed_sequence)
        for (uint32_t i = 0U; i < 8U; i++) values[i] = received_values[i];
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    line_sensor.frame_age_ms = line_sensor.valid_frames ? Age(now_ms, frame_ms) : UINT32_MAX;
    if (sequence != consumed_sequence) {
        consumed_sequence = sequence;
        line_sensor.last_frame_ms = frame_ms;
        for (uint32_t i = 0U; i < 8U; i++) line_sensor.analog[i] = values[i];
        LineCalibration_OnFrame(values, frame_ms);
        line_sensor.sample_sequence++;
    }
    LineCalibration_Update(now_ms);
    /* Saving can block flash reads while motors are stopped. Start the next
     * request from the actual current tick, not a pre-erase timestamp. */
    now_ms = HAL_GetTick();
    line_sensor.now_ms = now_ms;
    line_sensor.frame_age_ms = line_sensor.valid_frames ? Age(now_ms, line_sensor.last_frame_ms) : UINT32_MAX;
    line_sensor.age_ms = line_sensor.good_bytes ? Age(now_ms, line_sensor.last_rx_ms) : UINT32_MAX;
    if (line_sensor.valid_frames) line_sensor.state = line_sensor.frame_age_ms <= FRESH_MS ? 2U : 3U;
    else line_sensor.state = line_sensor.uart_errors && !line_sensor.good_bytes ? 4U : (line_sensor.tx_commands ? 1U : 0U);
    if (line_sensor.frame_age_ms > FRESH_MS) line_sensor.mask = 0U;

    if ((uint32_t)(now_ms - boot_ms) < BOOT_LISTEN_MS) return;
    if (line_sensor.tx_commands && (uint32_t)(now_ms - line_sensor.last_tx_ms) < COMMAND_PERIOD_MS) return;
    if (pending_request && (uint32_t)(now_ms - line_sensor.last_tx_ms) < RESPONSE_TIMEOUT_MS) return;
    if (!(UART4->SR & USART_SR_TXE)) return;
    /* Exclude just UART4 while changing pending state and writing DR. */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
    if (pending_request) {
        line_sensor.no_rx_windows++;
        used = 0U;
    }
    uint8_t command = line_sensor.tx_commands == 0U ? 0x00U : 0x02U;
    pending_request = command == 0x02U;
    line_sensor.last_tx_ms = now_ms;
    line_sensor.last_tx_byte = command;
    line_sensor.tx_commands++;
    UART4->DR = command;
    HAL_NVIC_EnableIRQ(UART4_IRQn);
}
