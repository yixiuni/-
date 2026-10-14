#include "button.h"

void Button_Init(Button_State *button, uint8_t pressed, uint32_t now_ms)
{
    button->candidate = pressed != 0U;
    button->stable = button->candidate;
    button->changed_ms = now_ms;
    button->armed = 0U;
}

uint8_t Button_Update(Button_State *button, uint8_t pressed, uint32_t now_ms)
{
    pressed = pressed != 0U;
    if (pressed != button->candidate)
    {
        button->candidate = pressed;
        button->changed_ms = now_ms;
    }
    if ((uint32_t)(now_ms - button->changed_ms) < BUTTON_DEBOUNCE_MS)
    {
        return 0U;
    }
    if (!button->candidate)
    {
        button->stable = 0U;
        button->armed = 1U;
        return 0U;
    }
    if (!button->stable)
    {
        button->stable = 1U;
        if (button->armed)
        {
            button->armed = 0U;
            return 1U;
        }
    }
    return 0U;
}
