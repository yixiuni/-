#include "line_calibration.h"
#include "stm32f1xx_hal.h"

/* STM32F103RCT6: reserve the final 2 KiB page, excluded from code by the
 * linker. Missing linker symbols after CubeMX regeneration fail the build
 * instead of allowing an erase of application code. Motor must be stopped.
 * Storage: 26 profile words, checksum, then commit marker written last. */
extern const uint8_t __line_cal_start__, __line_cal_end__;
#define STORE_MAGIC 0x43414C34U
#define PROFILE_WORDS 26U
#define RECORD_WORDS 28U
_Static_assert(sizeof(LineCalibration_Profile) == 104U, "Calibration profile format");
_Static_assert(FLASH_PAGE_SIZE == 2048U, "Requires high-density F103 flash pages");

static uint32_t ReadWord(uint32_t address)
{
    return *(const volatile uint32_t *)(uintptr_t)address;
}

static uint32_t Checksum(const uint32_t words[RECORD_WORDS])
{
    uint32_t hash = 2166136261U;
    for (uint32_t i = 0U; i < PROFILE_WORDS; i++)
        for (uint32_t shift = 0U; shift < 32U; shift += 8U)
            hash = (hash ^ ((words[i] >> shift) & 255U)) * 16777619U;
    return hash;
}

static uint8_t Valid(const LineCalibration_Profile *p)
{
    if (p->version != 1U || p->polarity > 255U) return 0U;
    for (uint32_t i = 0U; i < 8U; i++) {
        if (p->high[i] > UINT16_MAX || p->white[i] > UINT16_MAX ||
            p->low[i] >= p->high[i] || p->high[i] - p->low[i] < 100U) return 0U;
    }
    return 1U;
}

static uint8_t RegionValid(void)
{
    return (uintptr_t)&__line_cal_start__ == 0x0803F800U &&
           (uintptr_t)&__line_cal_end__ == 0x08040000U;
}

uint8_t LineCalibration_Load(LineCalibration_Profile *profile)
{
    if (!RegionValid()) return 0U;
    uint32_t words[RECORD_WORDS] = {0};
    for (uint32_t i = 0U; i < RECORD_WORDS; i++) words[i] = ReadWord(0x0803F800U + 4U*i);
    if (words[27] != STORE_MAGIC || words[26] != Checksum(words)) return 0U;
    LineCalibration_Profile candidate = {0};
    candidate.version = words[0];
    candidate.polarity = words[1];
    for (uint32_t i = 0U; i < 8U; i++) {
        candidate.low[i] = words[2U + i];
        candidate.high[i] = words[10U + i];
        candidate.white[i] = words[18U + i];
    }
    if (!Valid(&candidate)) return 0U;
    *profile = candidate;
    return 1U;
}

uint8_t LineCalibration_Save(const LineCalibration_Profile *profile)
{
    if (!RegionValid() || !Valid(profile)) return 0U;
    uint32_t words[RECORD_WORDS] = {0};
    words[0] = profile->version;
    words[1] = profile->polarity;
    for (uint32_t i = 0U; i < 8U; i++) {
        words[2U + i] = profile->low[i];
        words[10U + i] = profile->high[i];
        words[18U + i] = profile->white[i];
    }
    words[26] = Checksum(words);
    words[27] = STORE_MAGIC;
    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = 0x0803F800U;
    erase.NbPages = 1U;
    uint32_t page_error = 0U;
    if (HAL_FLASH_Unlock() != HAL_OK) return 0U;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &page_error);
    /* Commit halfwords are last. Power loss cannot validate a partial record. */
    for (uint32_t i = 0U; status == HAL_OK && i < RECORD_WORDS * 2U; i++) {
        uint16_t value = (uint16_t)(words[i / 2U] >> (16U * (i % 2U)));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, erase.PageAddress + 2U*i, value);
    }
    HAL_FLASH_Lock();
    if (status != HAL_OK) return 0U;
    for (uint32_t i = 0U; i < RECORD_WORDS; i++)
        if (ReadWord(erase.PageAddress + 4U*i) != words[i]) return 0U;
    return 1U;
}
