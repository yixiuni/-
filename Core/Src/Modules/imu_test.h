#ifndef IMU_TEST_H
#define IMU_TEST_H
#include <stdint.h>
void ImuTest_Init(uint32_t now_ms);
void ImuTest_Update(uint32_t now_ms);
#endif
