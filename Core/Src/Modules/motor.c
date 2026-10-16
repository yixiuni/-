#include "motor.h"
#include "tim.h"

/* 实际前进方向在架空测试后校准 */
#define LEFT_DIRECTION   1
#define RIGHT_DIRECTION  -1

static void Motor_SetOne(uint32_t channel1,
                         uint32_t channel2,
                         int16_t output)
{
    if (output > 100)  output = 100;
    if (output < -100) output = -100;

    uint32_t magnitude = (output < 0)
                         ? (uint32_t)(-output)
                         : (uint32_t)output;

    uint32_t pulse =
        ((__HAL_TIM_GET_AUTORELOAD(&htim3) + 1U) * magnitude) / 100U;

    if (output >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, channel2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, channel1, pulse);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, channel1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, channel2, pulse);
    }
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void Motor_Brake(void)
{
    /* C10B schematic U5/U6: AT8236. Datasheet H-bridge truth table:
     * IN1=IN2=1 -> OUT1=OUT2=L (brake); 0/0 would coast.
     * PWM1 active-high with CCR=ARR+1 keeps both inputs high. */
    uint32_t high=__HAL_TIM_GET_AUTORELOAD(&htim3)+1U;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,high);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,high);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,high);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,high);
}

void Motor_Init(void)
{
    Motor_Stop();

    const uint32_t channels[] = {
        TIM_CHANNEL_1, TIM_CHANNEL_2,
        TIM_CHANNEL_3, TIM_CHANNEL_4
    };

    for (uint32_t i = 0; i < 4; i++)
    {
        if (HAL_TIM_PWM_Start(&htim3, channels[i]) != HAL_OK)
        {
            Motor_Stop();
            Error_Handler();
        }
    }
}

void Motor_SetOutput(int16_t left, int16_t right)
{
    /* 先限幅，再应用方向，避免负数取反溢出 */
    if (left > 100)   left = 100;
    if (left < -100)  left = -100;
    if (right > 100)  right = 100;
    if (right < -100) right = -100;

    Motor_SetOne(TIM_CHANNEL_1, TIM_CHANNEL_2,
                 left * LEFT_DIRECTION);
    Motor_SetOne(TIM_CHANNEL_3, TIM_CHANNEL_4,
                 right * RIGHT_DIRECTION);
}