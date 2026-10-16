#include "oled.h"
#include "main.h"
#include <string.h>
static uint8_t pixels[1024], sent_pixels[1024];
static uint8_t dirty[128];
enum { OLED_POWER_SETTLE, OLED_RESET_LOW, OLED_RESET_WAIT, OLED_COMMANDS,
       OLED_FIRST_FRAME, OLED_STATIC_TEST, OLED_READY };
volatile uint32_t oled_diag[16];
_Static_assert(sizeof(oled_diag)==64U,"OLED diagnostic layout");
static uint32_t init_ms, init_stage, transfer_at;
static uint8_t transfer_pending;
volatile uint32_t oled_frames, oled_max_update_us;

/* Original compact 5x7 glyph set: space, digits, A..Z, punctuation. */
static const uint8_t digits[10][5] = {
 {62,81,73,69,62},{0,66,127,64,0},{98,81,73,73,70},{34,65,73,73,54},
 {24,20,18,127,16},{39,69,69,69,57},{60,74,73,73,48},{1,113,9,5,3},
 {54,73,73,73,54},{6,73,73,41,30}};
static const uint8_t letters[26][5] = {
 {126,9,9,9,126},{127,73,73,73,54},{62,65,65,65,34},{127,65,65,34,28},
 {127,73,73,73,65},{127,9,9,9,1},{62,65,73,73,122},{127,8,8,8,127},
 {0,65,127,65,0},{32,64,65,63,1},{127,8,20,34,65},{127,64,64,64,64},
 {127,2,12,2,127},{127,4,8,16,127},{62,65,65,65,62},{127,9,9,9,6},
 {62,65,81,33,94},{127,9,25,41,70},{38,73,73,73,50},{1,1,127,1,1},
 {63,64,64,64,63},{31,32,64,32,31},{63,64,56,64,63},{99,20,8,20,99},
 {3,4,120,4,3},{97,81,73,69,67}};

static void HalfCycle(void)
{
    /* 72 NOP cycles >=1us at 72MHz. Finite loop; no timer dependency. */
    for (volatile uint32_t n = 0U; n < 72U; n++) __NOP();
}
static void Byte(uint8_t value, uint8_t data)
{
    if (data) GPIOB->BSRR = GPIO_PIN_3; else GPIOB->BRR = GPIO_PIN_3;
    for (uint32_t bit = 0U; bit < 8U; bit++) {
        GPIOC->BRR = GPIO_PIN_14;
        if (value & 0x80U) GPIOB->BSRR = GPIO_PIN_5; else GPIOB->BRR = GPIO_PIN_5;
        HalfCycle();
        GPIOC->BSRR = GPIO_PIN_14;
        HalfCycle();
        value <<= 1;
    }
    GPIOC->BRR = GPIO_PIN_14;
    oled_diag[5]++;
}
void OLED_Init(uint32_t now_ms)
{
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE(); __HAL_AFIO_REMAP_SWJ_NOJTAG(); /* retain SWD */
    /* PB3 is D/C. Disable external SWO ownership without disabling SWD.
     * Record the prior state rather than claiming it was the failure. */
    uint32_t trace_before = DBGMCU->CR;
    DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN;
    GPIOB->BRR = GPIO_PIN_3 | GPIO_PIN_5;
    GPIOB->BSRR = GPIO_PIN_4;
    GPIOC->BRR = GPIO_PIN_14;
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP; gpio.Pull = GPIO_NOPULL; gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_14; HAL_GPIO_Init(GPIOC, &gpio);
    init_stage = OLED_POWER_SETTLE; init_ms = now_ms; transfer_pending = 0U; transfer_at = 0U;
    for (uint32_t i=0U;i<16U;i++) oled_diag[i]=0U;
    oled_diag[0]=0x4F4C4432U;oled_diag[1]=2U;oled_diag[14]=trace_before;
    memset(sent_pixels,0,sizeof(sent_pixels));memset(dirty,0,sizeof(dirty));
    oled_frames = oled_max_update_us = 0U;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    OLED_Clear();
}
uint8_t OLED_CanDraw(void) { return init_stage == OLED_READY && !transfer_pending; }
void OLED_Clear(void) { if (!transfer_pending) memset(pixels, 0, sizeof(pixels)); }
void OLED_Text(uint32_t row, uint32_t col, const char *text)
{
    if (row >= 8U || transfer_pending) return;
    while (*text && col < 21U) {
        char c = *text++;
        for (uint32_t x = 0U; x < 5U; x++) {
            uint8_t v = 0U;
            if (c >= '0' && c <= '9') v = digits[c-'0'][x];
            else if (c >= 'A' && c <= 'Z') v = letters[c-'A'][x];
            else if (c == '-') v = 8U;
            else if (c == '.') v = x == 2U ? 64U : 0U;
            else if (c == ':') v = x == 2U ? 36U : 0U;
            else if (c == '/') v = (uint8_t)(64U >> x);
            pixels[row*128U + col*6U + x] = v;
        }
        col++;
    }
}
void OLED_Number(uint32_t row, uint32_t col, int32_t value)
{
    char out[12], reverse[10]; uint32_t n = 0U, k = 0U;
    uint32_t v = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    if (value < 0) out[k++] = '-';
    do { reverse[n++] = (char)('0' + v%10U); v /= 10U; } while (v);
    while (n) out[k++] = reverse[--n];
    out[k] = 0; OLED_Text(row, col, out);
}

static void QueueFrame(uint8_t force)
{
    transfer_at=0U;transfer_pending=0U;
    for(uint32_t chunk=0U;chunk<128U;chunk++) {
        dirty[chunk]=force;
        for(uint32_t n=0U;n<8U;n++)
            if(pixels[chunk*8U+n]!=sent_pixels[chunk*8U+n]) dirty[chunk]=1U;
        if(dirty[chunk]) transfer_pending=1U;
    }
}
void OLED_Present(void) { if(OLED_CanDraw()) QueueFrame(0U); }

static void StaticPattern(void)
{
    OLED_Clear();
    for(uint32_t x=0U;x<128U;x++) { pixels[x]=1U;pixels[7U*128U+x]=128U; }
    for(uint32_t page=0U;page<8U;page++) { pixels[page*128U]=255U;pixels[page*128U+127U]=255U; }
    OLED_Text(2U,4U,"OLED TEST V2");
    OLED_Text(4U,3U,"0123456789 ABC");
    OLED_Text(6U,3U,"STATIC 4 SEC");
    QueueFrame(1U);
}
static void SendChunk(void)
{
    while(transfer_at<1024U && !dirty[transfer_at/8U]) transfer_at+=8U;
    if(transfer_at==1024U) {transfer_pending=0U;return;}
    uint32_t column=transfer_at%128U;
    /* Explicit page AND column for every chunk; never depend on an address
     * pointer surviving an arbitrary gap between foreground updates. */
    Byte((uint8_t)(0xB0U+transfer_at/128U),0U);
    Byte((uint8_t)(column&15U),0U);Byte((uint8_t)(0x10U|(column>>4)),0U);
    for(uint32_t n=0U;n<8U;n++) {
        Byte(pixels[transfer_at],1U);sent_pixels[transfer_at]=pixels[transfer_at];transfer_at++;
    }
    if(transfer_at==1024U) { transfer_pending=0U;oled_frames++; }
    else {
        uint32_t next=transfer_at;
        while(next<1024U && !dirty[next/8U]) next+=8U;
        if(next==1024U) { transfer_pending=0U;oled_frames++; }
    }
}
void OLED_Update(uint32_t now_ms)
{
    /* SSD1306 test profile. Display OFF until the ENTIRE known frame is in
     * RAM. Do not blank/reinitialise the display on normal UI refresh. */
    static const uint8_t commands[]={0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0,0x40,
        0x8D,0x14,0x20,0x02,0xA1,0xC8,0xDA,0x12,0x81,0x7F,0xD9,0xF1,
        0xDB,0x40,0xA4,0xA6,0x2E};
    uint32_t begin=DWT->CYCCNT;
    switch(init_stage) {
    case OLED_POWER_SETTLE:
        if((uint32_t)(now_ms-init_ms)>=150U) {
            GPIOB->BRR=GPIO_PIN_4;init_stage=OLED_RESET_LOW;init_ms=now_ms;
        }
        break;
    case OLED_RESET_LOW:
        if((uint32_t)(now_ms-init_ms)>=20U) {
            GPIOB->BSRR=GPIO_PIN_4;init_stage=OLED_RESET_WAIT;init_ms=now_ms;
        }
        break;
    case OLED_RESET_WAIT:
        if((uint32_t)(now_ms-init_ms)>=100U) {init_stage=OLED_COMMANDS;transfer_at=0U;}
        break;
    case OLED_COMMANDS:
        Byte(commands[transfer_at++],0U);
        if(transfer_at==sizeof(commands)) {init_stage=OLED_FIRST_FRAME;StaticPattern();}
        break;
    case OLED_FIRST_FRAME:
        SendChunk();
        if(!transfer_pending) {Byte(0xAF,0U);init_stage=OLED_STATIC_TEST;init_ms=now_ms;}
        break;
    case OLED_STATIC_TEST:
        /* No display traffic at all during this interval. */
        if((uint32_t)(now_ms-init_ms)>=4000U) init_stage=OLED_READY;
        break;
    case OLED_READY:
        if(transfer_pending) SendChunk();
        break;
    default: break;
    }
    uint32_t us=(DWT->CYCCNT-begin)/(SystemCoreClock/1000000U);
    if(us>oled_max_update_us) oled_max_update_us=us;
    oled_diag[2]=init_stage;oled_diag[3]=now_ms;oled_diag[4]=oled_frames;oled_diag[6]=oled_max_update_us;
    oled_diag[7]=transfer_pending;oled_diag[8]=GPIOB->CRL;oled_diag[9]=GPIOB->ODR;
    oled_diag[10]=GPIOC->CRH;oled_diag[11]=GPIOC->ODR;oled_diag[12]=DBGMCU->CR;
    oled_diag[13]=RCC->BDCR;oled_diag[15]=transfer_at;
}
