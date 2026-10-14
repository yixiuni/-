#ifndef LINE_CALIBRATION_H
#define LINE_CALIBRATION_H
#include <stdint.h>

enum { LINE_CAL_NONE, LINE_CAL_WHITE, LINE_CAL_SWEEP, LINE_CAL_READY, LINE_CAL_FAILED };
enum { LINE_CAL_OK, LINE_CAL_NO_DATA, LINE_CAL_WHITE_UNSTABLE,
       LINE_CAL_LOW_CONTRAST, LINE_CAL_SAVE_FAILED, LINE_CAL_CANCELLED };
typedef struct {
    uint32_t version, polarity;
    uint32_t low[8], high[8], white[8];
} LineCalibration_Profile;

void LineCalibration_Init(void);
void LineCalibration_Start(uint32_t now_ms);
void LineCalibration_Cancel(void);
uint8_t LineCalibration_IsActive(void);
void LineCalibration_OnFrame(const uint16_t values[8], uint32_t now_ms);
void LineCalibration_Update(uint32_t now_ms);
uint8_t LineCalibration_Load(LineCalibration_Profile *profile);
uint8_t LineCalibration_Save(const LineCalibration_Profile *profile);
#endif
