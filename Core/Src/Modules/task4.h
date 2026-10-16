#ifndef TASK4_H
#define TASK4_H
#include <stdint.h>
extern volatile uint32_t task4_diag[16];
void Task4_Init(uint32_t now);
void Task4_Update(uint32_t now);
#endif
