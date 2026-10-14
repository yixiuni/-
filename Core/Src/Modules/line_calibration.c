#include "line_calibration.h"
#include "line_sensor.h"

#define WHITE_MS 1000U
#define SWEEP_MS 10000U
#define CAPACITY 256U
#define MIN_SAMPLES 40U
#define MIN_SPAN 100U
#define SAMPLE_PERIOD_MS 50U
#define MIN_COVERAGE_MS 9000U

static LineCalibration_Profile active;
static uint16_t samples[CAPACITY][8], ordered[CAPACITY];
static uint32_t white_median[8], white_spread[8];
static uint32_t start_ms, phase_ms, white_count, count;
static uint32_t last_sample_ms, first_sweep_ms;

static void SortChannel(uint32_t channel, uint32_t length)
{
    for (uint32_t j = 0U; j < length; j++) {
        uint16_t v = samples[j][channel];
        uint32_t k = j;
        while (k > 0U && ordered[k - 1U] > v) {
            ordered[k] = ordered[k - 1U];
            k--;
        }
        ordered[k] = v;
    }
}

static void PublishProfile(void)
{
    line_sensor.cal_polarity = active.polarity;
    for (uint32_t i = 0U; i < 8U; i++) {
        line_sensor.minimum[i] = active.low[i];
        line_sensor.maximum[i] = active.high[i];
        line_sensor.white[i] = active.white[i];
        line_sensor.threshold[i] = active.low[i] + (active.high[i] - active.low[i]) / 2U;
    }
}

void LineCalibration_Init(void)
{
    active = (LineCalibration_Profile){0};
    line_sensor.cal_sample_period_ms = SAMPLE_PERIOD_MS;
    line_sensor.cal_valid = LineCalibration_Load(&active);
    line_sensor.cal_saved = line_sensor.cal_valid;
    line_sensor.cal_state = line_sensor.cal_valid ? LINE_CAL_READY : LINE_CAL_NONE;
    if (line_sensor.cal_valid) PublishProfile();
}

uint8_t LineCalibration_IsActive(void)
{
    return line_sensor.cal_state == LINE_CAL_WHITE || line_sensor.cal_state == LINE_CAL_SWEEP;
}

void LineCalibration_Start(uint32_t now_ms)
{
    /* Caller must have stopped the motors. Keep the previous good profile
     * until every channel passes; a failed relearn cannot overwrite it. */
    start_ms = phase_ms = now_ms;
    last_sample_ms = now_ms - SAMPLE_PERIOD_MS;
    first_sweep_ms = now_ms;
    white_count = count = 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        white_median[i] = white_spread[i] = 0U;
        line_sensor.baseline[i] = line_sensor.baseline_spread[i] = 0U;
    }
    line_sensor.cal_samples = line_sensor.cal_elapsed_ms = 0U;
    line_sensor.cal_white_samples = line_sensor.cal_coverage_ms = 0U;
    line_sensor.cal_bad_channels = line_sensor.cal_error = 0U;
    line_sensor.cal_remaining_ms = WHITE_MS + SWEEP_MS;
    line_sensor.cal_state = LINE_CAL_WHITE;
    line_sensor.mask = 0U;
}

void LineCalibration_Cancel(void)
{
    if (!LineCalibration_IsActive()) return;
    line_sensor.cal_state = LINE_CAL_FAILED;
    line_sensor.cal_error = LINE_CAL_CANCELLED;
    line_sensor.cal_remaining_ms = 0U;
}

static void Classify(const uint16_t values[8])
{
    if (!line_sensor.cal_valid || LineCalibration_IsActive()) {
        line_sensor.mask = 0U;
        for (uint32_t i = 0U; i < 8U; i++) line_sensor.strength[i] = 0U;
        return;
    }
    uint32_t mask = line_sensor.mask;
    for (uint32_t i = 0U; i < 8U; i++) {
        uint32_t v = values[i], lo = active.low[i], hi = active.high[i];
        uint32_t strength = v <= lo ? 0U : (v >= hi ? 1000U : (v - lo) * 1000U / (hi - lo));
        if (!(active.polarity & (1U << i))) strength = 1000U - strength;
        line_sensor.strength[i] = strength;
        /* 40/60 percent hysteresis suppresses chatter at the boundary. */
        if (strength >= 600U) mask |= 1U << i;
        else if (strength <= 400U) mask &= ~(1U << i);
    }
    line_sensor.mask = mask;
}

void LineCalibration_OnFrame(const uint16_t values[8], uint32_t now_ms)
{
    /* Module may stream hundreds of frames/second. Space LEARNING samples
     * by time so the fixed buffer covers the full sweep, irrespective of RX
     * rate. Never duplicate/catch up old frames after a communication gap.
     * Live classification still consumes every complete received frame. */
    if ((uint32_t)(now_ms - phase_ms) <= 0x7FFFFFFFU &&
        (uint32_t)(now_ms - last_sample_ms) >= SAMPLE_PERIOD_MS) {
        if (line_sensor.cal_state == LINE_CAL_WHITE && (uint32_t)(now_ms - phase_ms) < WHITE_MS) {
            if (white_count < CAPACITY) {
                for (uint32_t i = 0U; i < 8U; i++) samples[white_count][i] = values[i];
                white_count++;
                line_sensor.cal_white_samples = white_count;
                last_sample_ms = now_ms;
            }
        } else if (line_sensor.cal_state == LINE_CAL_SWEEP &&
                   (uint32_t)(now_ms - phase_ms) < SWEEP_MS && count < CAPACITY) {
            for (uint32_t i = 0U; i < 8U; i++) samples[count][i] = values[i];
            if (count == 0U) first_sweep_ms = now_ms;
            count++;
            last_sample_ms = now_ms;
            line_sensor.cal_samples = count;
            line_sensor.cal_coverage_ms = now_ms - first_sweep_ms;
        }
    }
    Classify(values);
}

static void Fail(uint32_t error, uint32_t channels)
{
    line_sensor.cal_error = error;
    line_sensor.cal_bad_channels = channels;
    line_sensor.cal_remaining_ms = 0U;
    line_sensor.cal_state = LINE_CAL_FAILED;
}

static void Finish(void)
{
    LineCalibration_Profile candidate = {0};
    candidate.version = 1U;
    uint32_t bad = 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        SortChannel(i, count);
        /* Reject isolated reflections/spikes: use 5th/95th percentiles.
         * Each probe needs roughly half a second on both surfaces. */
        uint32_t lo = ordered[count / 20U];
        uint32_t hi = ordered[count - 1U - count / 20U];
        uint32_t w = white_median[i];
        uint32_t span = hi - lo;
        uint32_t noise = white_spread[i];
        uint32_t required = w / 20U;
        if (required < MIN_SPAN) required = MIN_SPAN;
        if (required < noise * 6U) required = noise * 6U;
        candidate.low[i] = lo;
        candidate.high[i] = hi;
        candidate.white[i] = w;
        if (span < required) bad |= 1U << i;
        else if (w <= lo + span / 4U) candidate.polarity |= 1U << i;
        else if (w < hi - span / 4U) bad |= 1U << i;
    }
    if (bad) { Fail(LINE_CAL_LOW_CONTRAST, bad); return; }
    active = candidate;
    PublishProfile();
    line_sensor.cal_valid = 1U;
    line_sensor.mask = 0U;
    line_sensor.cal_saved = LineCalibration_Save(&active);
    line_sensor.cal_error = line_sensor.cal_saved ? LINE_CAL_OK : LINE_CAL_SAVE_FAILED;
    line_sensor.cal_state = LINE_CAL_READY;
    line_sensor.cal_remaining_ms = 0U;
}

void LineCalibration_Update(uint32_t now_ms)
{
    if (!LineCalibration_IsActive()) return;
    line_sensor.cal_elapsed_ms = now_ms - start_ms;
    uint32_t elapsed = now_ms - phase_ms;
    if (line_sensor.cal_state == LINE_CAL_WHITE) {
        line_sensor.cal_remaining_ms = elapsed < WHITE_MS ? WHITE_MS - elapsed + SWEEP_MS : SWEEP_MS;
        if (elapsed < WHITE_MS) return;
        if (white_count < 5U || line_sensor.frame_age_ms > 500U) { Fail(LINE_CAL_NO_DATA, 255U); return; }
        uint32_t bad = 0U;
        for (uint32_t i = 0U; i < 8U; i++) {
            SortChannel(i, white_count);
            white_median[i] = ordered[white_count / 2U];
            /* Trim 10 percent at each end: one transient sample must not
             * abort learning or inflate the later contrast requirement. */
            white_spread[i] = ordered[white_count - 1U - white_count / 10U] - ordered[white_count / 10U];
            line_sensor.baseline[i] = white_median[i];
            line_sensor.baseline_spread[i] = white_spread[i];
            uint32_t allowed = white_median[i] / 10U;
            if (allowed < 100U) allowed = 100U;
            if (white_spread[i] > allowed) bad |= 1U << i;
        }
        if (bad) { Fail(LINE_CAL_WHITE_UNSTABLE, bad); return; }
        phase_ms = now_ms;
        last_sample_ms = now_ms - SAMPLE_PERIOD_MS;
        line_sensor.cal_state = LINE_CAL_SWEEP;
    } else {
        line_sensor.cal_remaining_ms = elapsed < SWEEP_MS ? SWEEP_MS - elapsed : 0U;
        if (elapsed < SWEEP_MS) return;
        if (count < MIN_SAMPLES || line_sensor.cal_coverage_ms < MIN_COVERAGE_MS ||
            line_sensor.frame_age_ms > 500U) { Fail(LINE_CAL_NO_DATA, 255U); return; }
        Finish();
    }
}
