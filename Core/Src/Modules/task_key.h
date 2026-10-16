#ifndef TASK_KEY_H
#define TASK_KEY_H
#include <stdint.h>
enum { TASK_KEY_NONE, TASK_KEY_START, TASK_KEY_STOP, TASK_KEY_SPEED,
       TASK_KEY_LAPS, TASK_KEY_CALIBRATE, TASK_KEY_CANCEL_CAL };
typedef struct {
    uint32_t press_ms, release_ms, held_ms;
    uint8_t tracking, waiting, second;
} TaskKey;
uint32_t TaskKey_Update(TaskKey *key, uint8_t press_event, uint8_t stable,
    uint8_t candidate, uint8_t running, uint8_t calibrating, uint32_t now_ms);
#endif
