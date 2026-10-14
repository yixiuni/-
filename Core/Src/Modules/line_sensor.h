#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H
#include <stdint.h>

/* V5: framed analog UART + host-side learning. 116 words / 464 bytes.
 * mask is SOFTWARE black detection (1=black), usable only with cal_valid
 * and fresh frames. Numbering S1..S8 is not vehicle left-to-right order.
 * Debugger reads are live snapshots. ISR owns RX counters; main publishes
 * complete analog samples and all calibration fields.
 */
typedef struct {
    uint32_t magic, version, state, mask;
    uint32_t rx_bytes, good_bytes, tx_commands, no_rx_windows;
    uint32_t uart_errors, overrun_errors, framing_errors, noise_errors;
    uint32_t parity_errors, last_rx_ms, age_ms, now_ms;
    uint32_t last_tx_ms, last_rx_sr, max_main_gap_ms, rx_before_first_tx;
    uint32_t first_count, last_count, recent_next, last_tx_byte;
    uint8_t first_bytes[16], recent_bytes[16];
    uint32_t valid_frames, frame_errors, checksum_errors, frame_timeouts;
    uint32_t last_frame_ms, frame_age_ms, cal_state, cal_error;
    uint32_t cal_valid, cal_saved, cal_elapsed_ms, cal_samples;
    uint32_t cal_bad_channels, cal_polarity, cal_remaining_ms, sample_sequence;
    uint32_t analog[8], minimum[8], maximum[8], threshold[8], white[8], strength[8];
    uint32_t cal_white_samples, cal_sample_period_ms, cal_coverage_ms, reserved;
    uint32_t baseline[8], baseline_spread[8];
} LineSensor_Data;

extern volatile LineSensor_Data line_sensor;
void LineSensor_Init(uint32_t now_ms);
void LineSensor_Update(uint32_t now_ms);
void LineSensor_IRQHandler(void);
#endif
