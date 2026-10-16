#ifndef MODE2_H
#define MODE2_H
#include <stdint.h>
extern volatile uint32_t mode2_diag[64];
void Mode2_Init(uint32_t now_ms);
void Mode2_Update(uint32_t now_ms);
uint8_t Mode2_SetTargets(float angle_degrees,float distance_mm);
#endif
