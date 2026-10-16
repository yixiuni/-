#include "bt_protocol.h"
#include <string.h>
void BtProtocol_Invalidate(BtParser *p) { p->length=0;p->discard=1; }
static int Number(const char **s,int32_t *v)
{
    while (**s==' ') (*s)++;
    int negative=**s=='-';if(negative)(*s)++;
    if(**s<'0' || **s>'9')return 0;
    uint32_t n=0;
    while(**s>='0' && **s<='9') {
        uint32_t digit=(uint32_t)(**s-'0');
        if(n>(2147483647U-digit)/10U)return 0;
        n=n*10U+digit;(*s)++;
    }
    if(**s && **s!=' ')return 0;
    *v=negative?-(int32_t)n:(int32_t)n;return 1;
}
static int Parse(const char *s,BtCommand *c)
{
    if(*s++!='@')return 0;
    int32_t session=0,id=0;if(!Number(&s,&session)||!Number(&s,&id)||session<=0||id<=0)return 0;
    while(*s==' ')s++;
    char name[8]={0};uint32_t n=0;
    while(*s && *s!=' ') { if(n>=sizeof(name)-1U)return 0;name[n++]=*s++; }
    static const char *const names[]={"HELLO","PING","STOP","START","MODE","SET1","SET2"};
    uint32_t kind=0;for(;kind<7U;kind++)if(!strcmp(name,names[kind]))break;
    if(kind==7U)return 0;
    *c=(BtCommand){.session=(uint32_t)session,.id=(uint32_t)id,.kind=kind};
    if(kind>=BT_MODE && !Number(&s,&c->a))return 0;
    if(kind>=BT_SET1 && !Number(&s,&c->b))return 0;
    while(*s==' ')s++;
    return *s==0;
}
int BtProtocol_Feed(BtParser *p,uint8_t byte,BtCommand *out)
{
    if(byte=='\n') {
        if(p->discard){p->discard=0;p->length=0;return -1;}
        if(p->length && p->line[p->length-1U]=='\r')p->length--;
        p->line[p->length]=0;int ok=Parse(p->line,out);p->length=0;return ok?1:-1;
    }
    if(p->discard)return 0;
    if((byte<32U && byte!='\r') || byte>126U || p->length>=sizeof(p->line)-1U) { BtProtocol_Invalidate(p);return 0; }
    p->line[p->length++]=(char)byte;return 0;
}
