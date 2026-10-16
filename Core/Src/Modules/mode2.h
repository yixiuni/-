#ifndef MODE2_H
#define MODE2_H
#include <stdint.h>
extern volatile uint32_t mode2_diag[96];
void Mode2_Init(uint32_t now_ms);
void Mode2_Update(uint32_t now_ms);
uint8_t Mode2_SetTargets(float angle_degrees,float distance_mm);
#include "mode2_control.h"
const Mode2Control *Mode2_GetControl(void);
uint8_t Mode2_KeyPending(void);
uint8_t Mode2_RemoteStart(uint32_t now);
void Mode2_RemoteStop(uint32_t now);
#endif
