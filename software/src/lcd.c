#include "stm32l0xx_hal.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"

const uint8_t digits_map[] = {0b00111111, 0b00000110, 0b01011011, 0b01001111,
                              0b01100110, 0b01101101, 0b01111101, 0b0000111,
                              0b01111111, 0b01101111};

LCD_HandleTypeDef hlcd;

void lcd_init()
{
    // LCD pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
                          GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_LCD;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 |
                          GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
                          GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_LCD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 |
                          GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                          GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
                          GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_LCD;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // LCD clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LCD);

    hlcd.Instance = LCD;
    hlcd.Init.Prescaler = LCD_PRESCALER_1;
    hlcd.Init.Divider = LCD_DIVIDER_16;
    // hlcd.Init.Divider = LCD_DIVIDER_30;
    hlcd.Init.Duty = LCD_DUTY_1_4;
    hlcd.Init.Bias = LCD_BIAS_1_3;
    hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
    hlcd.Init.Contrast = LCD_CONTRASTLEVEL_4;
    hlcd.Init.DeadTime = LCD_DEADTIME_0;
    hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_7;
    hlcd.Init.HighDrive = LCD_HIGHDRIVE_0;
    hlcd.Init.BlinkMode = LCD_BLINKMODE_SEG0_COM0;
    hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV1024;
    hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
    while (HAL_LCD_Init(&hlcd) != HAL_OK) {
    }
}

void lcd_write_mask(uint8_t pos, uint8_t mask)
{
    uint8_t offset = 2 * pos;
    uint32_t clearMask = 0xffffffff ^
                         ((1 << (22 - offset)) | (1 << (23 - offset)));

    HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER0, clearMask,
                  ((mask & 0b00000001) << (22 - offset)) |
                      (mask & 0b00100000) << (18 - offset));
    HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER2, clearMask,
                  ((mask & 0b00000010) << (21 - offset)) |
                      (mask & 0b01000000) << (17 - offset));
    HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER4, clearMask,
                  ((mask & 0b00000100) << (20 - offset)) |
                      (mask & 0b00010000) << (19 - offset));
    HAL_LCD_Write(&hlcd, LCD_RAM_REGISTER6, 0xffffffff ^ (1 << (22 - offset)),
                  (mask & 0b00001000) << (19 - offset));
}

void lcd_write_digit(uint8_t pos, uint8_t digit)
{
    lcd_write_mask(pos, digits_map[digit]);
}

void lcd_update()
{
    HAL_LCD_UpdateDisplayRequest(&hlcd);
}