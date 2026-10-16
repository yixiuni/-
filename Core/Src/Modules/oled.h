#ifndef OLED_H
#define OLED_H
#include <stdint.h>
/* Proposed direct H1 mapping by confirmed OLED pin order:
 * GND->G, VCC->3V3, SCL->PC14, SDA->PB5, RST->PB4, D/C->PB3.
 * SSD1306 128x64 SPI is the initial test profile, not an identified chip.
 * There is no readback/ACK: completed transfers do NOT prove a display. */
extern volatile uint32_t oled_frames, oled_max_update_us;
extern volatile uint32_t oled_diag[16]; /* Driver V2, 64-byte debug record */
void OLED_Init(uint32_t now_ms);
void OLED_Update(uint32_t now_ms);
uint8_t OLED_CanDraw(void);
void OLED_Clear(void);
void OLED_Text(uint32_t row, uint32_t col, const char *text);
void OLED_Number(uint32_t row, uint32_t col, int32_t value);
void OLED_Present(void);
#endif
