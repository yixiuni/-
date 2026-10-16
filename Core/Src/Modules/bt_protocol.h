#ifndef BT_PROTOCOL_H
#define BT_PROTOCOL_H
#include <stdint.h>
enum { BT_HELLO, BT_PING, BT_STOP, BT_START, BT_MODE, BT_SET1, BT_SET2 };
typedef struct { uint32_t session,id,kind;int32_t a,b; } BtCommand;
typedef struct { char line[96];uint32_t length;uint8_t discard; } BtParser;
/* Returns 1 for a complete valid command, -1 for a rejected line, 0 otherwise. */
int BtProtocol_Feed(BtParser *p,uint8_t byte,BtCommand *out);
void BtProtocol_Invalidate(BtParser *p);
#endif
