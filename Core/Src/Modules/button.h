#ifndef BUTTON_H
#define BUTTON_H
#include <stdint.h>

#define BUTTON_DEBOUNCE_MS 30U
typedef struct
{
    uint32_t changed_ms;
    uint8_t candidate;
    uint8_t stable;
    uint8_t armed;
} Button_State;

/* pressed=1 means physically pressed, independent of GPIO polarity.
 * Require a stable release after boot before accepting the first press. */
void Button_Init(Button_State *button, uint8_t pressed, uint32_t now_ms);
/* Returns 1 once per debounced press. Holding a key does not repeat. */
uint8_t Button_Update(Button_State *button, uint8_t pressed, uint32_t now_ms);
#endif
