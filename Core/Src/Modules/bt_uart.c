#include "bt_uart.h"
#include "main.h"
#include <string.h>
/* H4: PB10 TX -> module RXD, PB11 RX <- module TXD. Dedicated USART3. */
static volatile uint8_t rx[256],tx[1024];
static volatile uint32_t rx_head,rx_tail,tx_head,tx_tail;
volatile uint32_t bt_rx_bytes,bt_tx_bytes,bt_uart_errors,bt_rx_overflows,bt_tx_dropped;
void BtUart_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();__HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitTypeDef pin={0};pin.Pin=GPIO_PIN_10;pin.Mode=GPIO_MODE_AF_PP;
    pin.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB,&pin);
    pin.Pin=GPIO_PIN_11;pin.Mode=GPIO_MODE_INPUT;pin.Pull=GPIO_NOPULL;HAL_GPIO_Init(GPIOB,&pin);
    USART3->CR1=0;USART3->CR2=0;USART3->CR3=USART_CR3_EIE;
    USART3->BRR=(HAL_RCC_GetPCLK1Freq()+BT_UART_BAUD/2U)/BT_UART_BAUD;
    rx_head=rx_tail=tx_head=tx_tail=0;
    bt_rx_bytes=bt_tx_bytes=bt_uart_errors=bt_rx_overflows=bt_tx_dropped=0;
    (void)USART3->SR;(void)USART3->DR;
    USART3->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE|USART_CR1_PEIE;
    HAL_NVIC_SetPriority(USART3_IRQn,6U,0U);HAL_NVIC_EnableIRQ(USART3_IRQn);
}
void USART3_IRQHandler(void)
{
    uint32_t sr=USART3->SR;
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        uint8_t b=(uint8_t)USART3->DR;
        if(sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE))bt_uart_errors++;
        else if(sr&USART_SR_RXNE) {
            bt_rx_bytes++;
            uint32_t next=(rx_head+1U)&255U;
            if(next==rx_tail)bt_rx_overflows++;
            else {rx[rx_head]=b;__DMB();rx_head=next;}
        }
    }
    if((sr&USART_SR_TXE) && (USART3->CR1&USART_CR1_TXEIE)) {
        if(tx_tail==tx_head)USART3->CR1&=~USART_CR1_TXEIE;
        else {USART3->DR=tx[tx_tail];tx_tail=(tx_tail+1U)&1023U;bt_tx_bytes++;}
    }
}
uint8_t BtUart_Read(uint8_t *byte)
{
    if(rx_tail==rx_head)return 0;
    *byte=rx[rx_tail];__DMB();rx_tail=(rx_tail+1U)&255U;return 1;
}
uint8_t BtUart_Send(const char *text)
{
    uint32_t n=(uint32_t)strlen(text),head=tx_head;
    if(n>1023U-((head-tx_tail)&1023U)) {bt_tx_dropped++;return 0;}
    /* Publish only a whole line. IRQ consumes committed bytes only. */
    for(uint32_t i=0;i<n;i++) {tx[head]=(uint8_t)text[i];head=(head+1U)&1023U;}
    __DMB();tx_head=head;
    HAL_NVIC_DisableIRQ(USART3_IRQn);USART3->CR1|=USART_CR1_TXEIE;HAL_NVIC_EnableIRQ(USART3_IRQn);
    return 1;
}
uint32_t BtUart_Errors(void) { return bt_uart_errors+bt_rx_overflows; }
