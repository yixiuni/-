#include "bt_probe.h"
#include "bt_uart.h"
#include "speed_control.h"
#include "motor.h"
#include "oled.h"
#include <string.h>
/* This diagnostic dispatcher never runs Car_Update, Mode2_Update or Task4_Update.
 * Queries only: no rename, baud change AT command, reset, sleep or defaults. */
volatile uint32_t bt_probe_diag[BT_PROBE_WORDS];
static const uint32_t rates[]={9600U,115200U,38400U,19200U,57600U,4800U,2400U,128000U};
static const char *const queries[]={"AT+VERSION\r\n","AT+NAME\r\n","AT+NAMB\r\n","AT+BAUD\r\n","AT+STAT\r\n","AT+TYPE\r\n","AT+LADDR\r\n"};
enum { BOOT, GAP, REPLY, DONE, NO_REPLY, PORT_BUSY };
static uint32_t phase_ms,draw_ms,error_origin,query_index;
static char line[96];static uint32_t line_length;static uint8_t line_bad;
static uint32_t Row(void) { return 16U+bt_probe_diag[8]*56U; }
static uint8_t Terminal(void) { return bt_probe_diag[3]>=DONE; }
static void NewRow(uint32_t command,uint32_t now)
{
    uint32_t row=bt_probe_diag[7];
    if(row>=16U){bt_probe_diag[3]=PORT_BUSY;return;}
    bt_probe_diag[8]=row;bt_probe_diag[7]=row+1U;
    uint32_t p=Row();bt_probe_diag[p]=bt_probe_diag[5];bt_probe_diag[p+1U]=command;
    bt_probe_diag[p+7U]=now;error_origin=BtUart_Errors();line_length=line_bad=0;
    phase_ms=now;bt_probe_diag[3]=GAP;
}
static void Record(uint8_t b)
{
    uint32_t p=Row(),n=bt_probe_diag[p+3U];bt_probe_diag[p+4U]++;
    if(n<192U) {
        uint32_t index=p+8U+n/4U,shift=(n%4U)*8U;
        bt_probe_diag[index]=(bt_probe_diag[index]&~(0xFFU<<shift))|((uint32_t)b<<shift);
        bt_probe_diag[p+3U]=n+1U;
    } else bt_probe_diag[p+6U]|=4U;
    if(b=='\r' || b=='\n') {
        line[line_length]=0;
        /* Match a structured response, never the echoed AT+VERSION command.
         * Only accept after a query was queued on this baud. */
        if(!line_bad && bt_probe_diag[p+2U] && !strncmp(line,"+VERSION=",9U) && line_length>9U)
            bt_probe_diag[p+6U]|=2U;
        line_length=line_bad=0;
    } else if(b>=32U && b<=126U && line_length<sizeof(line)-1U)line[line_length++]=(char)b;
    else line_bad=1U;
}
static void Send(uint32_t now)
{
    uint32_t p=Row(),command=bt_probe_diag[p+1U];
    const char *text=command ? queries[command-1U] : "\r\nAT+VERSION\r\n";
    if(!BtUart_Send(text)){bt_probe_diag[3]=PORT_BUSY;return;}
    bt_probe_diag[p+2U]++;bt_probe_diag[9]++;phase_ms=now;bt_probe_diag[3]=REPLY;
}
static void Draw(uint32_t now)
{
    if((uint32_t)(now-draw_ms)>=200U && OLED_CanDraw()) {
        static const char *const names[]={"BOOT WAIT","QUERY WAIT","READING","COMPLETE","NO AT REPLY","UART BUSY"};
        OLED_Clear();OLED_Text(0,0,"JDY33 QUERY ONLY");OLED_Text(1,0,names[bt_probe_diag[3]]);
        OLED_Text(2,0,"BAUD:");OLED_Number(2,6,(int32_t)bt_probe_diag[5]);
        OLED_Text(3,0,"FOUND:");OLED_Number(3,7,(int32_t)bt_probe_diag[6]);
        OLED_Text(4,0,"RX:");OLED_Number(4,4,(int32_t)bt_rx_bytes);
        OLED_Text(5,0,"ERROR:");OLED_Number(5,7,(int32_t)bt_uart_errors);
        OLED_Text(6,0,"MOTORS DISABLED");OLED_Text(7,0,"READ VIA ST-LINK");OLED_Present();draw_ms=now;
    }
    OLED_Update(now);
}
void BtProbe_Init(uint32_t now)
{
    SpeedControl_Stop();Motor_Brake();BtUart_Init();
    for(uint32_t i=0;i<BT_PROBE_WORDS;i++)bt_probe_diag[i]=0;
    bt_probe_diag[0]=0x4A445933U;bt_probe_diag[1]=1U;
    phase_ms=draw_ms=now;query_index=0;line_length=line_bad=0;
}
void BtProbe_Update(uint32_t now)
{
    if(Terminal()){Draw(now);return;}
    uint32_t seq=(bt_probe_diag[2]+2U)&~1U;bt_probe_diag[2]=seq-1U;
    uint8_t b=0;
    for(uint32_t n=0;n<64U && BtUart_Read(&b);n++)if(bt_probe_diag[7])Record(b);
    if(bt_probe_diag[7])bt_probe_diag[Row()+5U]=BtUart_Errors()-error_origin;
    if(bt_probe_diag[3]==BOOT && (uint32_t)(now-phase_ms)>=1000U) {
        bt_probe_diag[5]=rates[0];
        if(BtUart_SetBaud(rates[0]))NewRow(0U,now);else bt_probe_diag[3]=PORT_BUSY;
    } else if(bt_probe_diag[3]==GAP && (uint32_t)(now-phase_ms)>=100U)Send(now);
    else if(bt_probe_diag[3]==REPLY && (uint32_t)(now-phase_ms)>=600U) {
        uint32_t p=Row(),command=bt_probe_diag[p+1U];
        if(!command && !(bt_probe_diag[p+6U]&2U) && bt_probe_diag[p+2U]<2U)Send(now);
        else {
            bt_probe_diag[p+6U]|=1U;
            if(!command) {
                if(bt_probe_diag[p+6U]&2U) {bt_probe_diag[6]=bt_probe_diag[5];query_index=0;NewRow(1U,now);}
                else if(++bt_probe_diag[4]<sizeof(rates)/sizeof(rates[0])) {
                    bt_probe_diag[5]=rates[bt_probe_diag[4]];
                    if(BtUart_SetBaud(bt_probe_diag[5]))NewRow(0U,now);else bt_probe_diag[3]=PORT_BUSY;
                } else bt_probe_diag[3]=NO_REPLY;
            } else if(++query_index<sizeof(queries)/sizeof(queries[0]))NewRow(query_index+1U,now);
            else bt_probe_diag[3]=DONE;
        }
    }
    bt_probe_diag[10]=bt_rx_bytes;bt_probe_diag[11]=bt_uart_errors;bt_probe_diag[12]=bt_rx_overflows;
    bt_probe_diag[13]=now;bt_probe_diag[14]=Terminal();bt_probe_diag[15]=seq;bt_probe_diag[2]=seq;
    Draw(now);
}
