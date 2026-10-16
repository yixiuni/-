#include "task_key.h"
/* Short press: start after the double-click window; running press: STOP now.
 * Two short clicks <=400ms: speed. Hold 2..6s/release: laps.
 * Hold >=6s/release: calibration. Nothing starts while a key is held. */
uint32_t TaskKey_Update(TaskKey *key, uint8_t press_event, uint8_t stable,
    uint8_t candidate, uint8_t running, uint8_t calibrating, uint32_t now_ms)
{
    if (running || calibrating) {
        *key = (TaskKey){0};
        if (press_event) return running ? TASK_KEY_STOP : TASK_KEY_CANCEL_CAL;
        return TASK_KEY_NONE;
    }
    if (press_event) {
        key->second = key->waiting && (uint32_t)(now_ms - key->release_ms) <= 400U;
        key->waiting = 0U;
        key->tracking = 1U;
        key->press_ms = now_ms;
    }
    if (key->tracking) {
        key->held_ms = now_ms - key->press_ms;
        if (!stable) {
            key->tracking = 0U;
            if (key->held_ms >= 6000U) { key->second = 0U; return TASK_KEY_CALIBRATE; }
            if (key->held_ms >= 2000U) { key->second = 0U; return TASK_KEY_LAPS; }
            if (key->second) { key->second = 0U; return TASK_KEY_SPEED; }
            key->waiting = 1U;
            key->release_ms = now_ms;
        }
    }
    /* A physical press being debounced also inhibits a pending start. */
    if (key->waiting && !stable && !candidate && (uint32_t)(now_ms - key->release_ms) > 400U) {
        key->waiting = 0U;
        return TASK_KEY_START;
    }
    return TASK_KEY_NONE;
}
