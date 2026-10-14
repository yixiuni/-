#include "usart.h"

UART_HandleTypeDef huart4;

void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) { Error_Handler(); }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == UART4)
    {
        GPIO_InitTypeDef pins = {0};
        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        /* CN6: PC10 TX -> module RX; PC11 RX <- module TX. */
        pins.Pin = GPIO_PIN_10;
        pins.Mode = GPIO_MODE_AF_PP;
        pins.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOC, &pins);
        pins.Pin = GPIO_PIN_11;
        pins.Mode = GPIO_MODE_INPUT;
        pins.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &pins);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == UART4)
    {
        __HAL_RCC_UART4_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
    }
}
