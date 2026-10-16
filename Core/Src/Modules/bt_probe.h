#ifndef BT_PROBE_H
#define BT_PROBE_H
#include <stdint.h>
#define BT_PROBE_WORDS 912U
extern volatile uint32_t bt_probe_diag[BT_PROBE_WORDS];
void BtProbe_Init(uint32_t now);
void BtProbe_Update(uint32_t now);
#endif
