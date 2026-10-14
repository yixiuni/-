#include "encoder.h"
#include "tim.h"

volatile Encoder_Data encoder_data;
static uint16_t previous_left;
static uint16_t previous_right;
static uint32_t previous_ms;

static int32_t Encoder_Difference(uint16_t current, uint16_t previous)
{
    /* Explicit wrap correction avoids implementation-defined signed casts. */
    int32_t difference = (int32_t)current - (int32_t)previous;
    if (difference > 32767)
    {
        difference -= 65536;
    }
    else if (difference < -32768)
    {
        difference += 65536;
    }
    return difference;
}

void Encoder_Reset(uint32_t now_ms)
{
    previous_left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    previous_right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    previous_ms = now_ms;
    encoder_data = (Encoder_Data){0};
    encoder_data.left_raw = previous_left;
    encoder_data.right_raw = previous_right;
}

void Encoder_Init(void)
{
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    if (HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim4, TIM_CHANNEL_ALL);
        Error_Handler();
        return;
    }
    Encoder_Reset(HAL_GetTick());
}

void Encoder_Sample(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - previous_ms;
    if (elapsed_ms == 0U)
    {
        return;
    }

    uint16_t left = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint16_t right = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);
    int32_t left_delta = Encoder_Difference(left, previous_left);
    /* Forward test confirmed left positive, right negative raw counts. */
    int32_t right_delta = -Encoder_Difference(right, previous_right);
    previous_left = left;
    previous_right = right;
    previous_ms = now_ms;

    encoder_data.left_raw = left;
    encoder_data.right_raw = right;
    encoder_data.left_delta = left_delta;
    encoder_data.right_delta = right_delta;
    encoder_data.left_total += left_delta;
    encoder_data.right_total += right_delta;
    encoder_data.left_cps = (float)left_delta * 1000.0f / (float)elapsed_ms;
    encoder_data.right_cps = (float)right_delta * 1000.0f / (float)elapsed_ms;
    encoder_data.sample_ms = elapsed_ms;
    encoder_data.sample_count++;
}

void Encoder_Update(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - previous_ms) >= ENCODER_SAMPLE_PERIOD_MS)
    {
        Encoder_Sample(now_ms);
    }
}
