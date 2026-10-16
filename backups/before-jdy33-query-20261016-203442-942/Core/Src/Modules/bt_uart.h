#ifndef BT_UART_H
#define BT_UART_H
#include <stdint.h>
#define BT_UART_BAUD 9600U
extern volatile uint32_t bt_rx_bytes,bt_tx_bytes,bt_uart_errors,bt_rx_overflows,bt_tx_dropped;
void BtUart_Init(void);
uint8_t BtUart_Read(uint8_t *byte);
uint8_t BtUart_Send(const char *text);
uint32_t BtUart_Errors(void);
#endif
