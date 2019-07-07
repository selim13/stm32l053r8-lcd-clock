
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_lpuart.h"
#include "stm32l0xx_hal.h"

LCD_HandleTypeDef hlcd;

uint8_t numMap[] = {0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b0000111, 0b01111111, 0b01101111};

void init()
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
    {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    LL_RCC_MSI_Enable();
    while (LL_RCC_MSI_IsReady() != 1)
    {
    }
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5); // 2.097 MHz

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);    // enable APB1
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG); // enable APB2

    LL_PWR_EnableBkUpAccess();
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
    LL_RCC_LSE_Enable();
    while (LL_RCC_LSE_IsReady() != 1)
    {
    }

    // LL_RCC_LSI_Enable();
    // while (LL_RCC_LSI_IsReady() != 1)
    // {
    // }

    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    LL_RCC_EnableRTC();

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_MSI)
    {
    }

    LL_Init1msTick(2097000);
    // LL_InitTick(2097000, 1);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();

    SystemCoreClock = 2097000;
}

void gpioInit()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
}

void lpUartInit()
{
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_LSE);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);
    while (LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_LPUART1) != 1)
    {
    }

    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_0);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_0);

    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};
    LPUART_InitStruct.BaudRate = 9600;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_Enable(LPUART1);
}

void lcdWriteMask(LCD_HandleTypeDef *hlcd, uint8_t pos, uint8_t mask)
{
    uint8_t offset = 2 * pos;
    uint32_t clearMask = 0xffffffff ^ ((1 << (22 - offset)) | (1 << (23 - offset)));

    HAL_LCD_Write(hlcd, LCD_RAM_REGISTER0, clearMask, ((mask & 0b00000001) << (22 - offset)) | (mask & 0b00100000) << (18 - offset));
    HAL_LCD_Write(hlcd, LCD_RAM_REGISTER2, clearMask, ((mask & 0b00000010) << (21 - offset)) | (mask & 0b01000000) << (17 - offset));
    HAL_LCD_Write(hlcd, LCD_RAM_REGISTER4, clearMask, ((mask & 0b00000100) << (20 - offset)) | (mask & 0b00010000) << (19 - offset));
    HAL_LCD_Write(hlcd, LCD_RAM_REGISTER6, 0xffffffff ^ (1 << (22 - offset)), (mask & 0b00001000) << (19 - offset));
}

void lcdInit()
{
    // LCD pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_LCD;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_LCD;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
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
    hlcd.Init.Contrast = LCD_CONTRASTLEVEL_7;
    hlcd.Init.DeadTime = LCD_DEADTIME_0;
    hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_7;
    hlcd.Init.HighDrive = LCD_HIGHDRIVE_0;
    hlcd.Init.BlinkMode = LCD_BLINKMODE_SEG0_COM0;
    hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV1024;
    hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
    while (HAL_LCD_Init(&hlcd) != HAL_OK)
    {
    }
}

int main(void)
{
    HAL_Init();
    init();
    gpioInit();
    lpUartInit();
    lcdInit();

    while (1)
    {
        uint32_t seconds = LL_RTC_TIME_GetSecond(RTC);
        uint32_t minutes = LL_RTC_TIME_GetMinute(RTC);
        uint32_t hours = LL_RTC_TIME_GetHour(RTC);

        lcdWriteMask(&hlcd, 0, numMap[hours >> 4 & 0b00001111]);
        lcdWriteMask(&hlcd, 1, numMap[hours & 0b00001111]);
        lcdWriteMask(&hlcd, 2, 0b01000000);
        lcdWriteMask(&hlcd, 3, numMap[minutes >> 4 & 0b00001111]);
        lcdWriteMask(&hlcd, 4, numMap[minutes & 0b00001111]);
        lcdWriteMask(&hlcd, 5, 0b01000000);
        lcdWriteMask(&hlcd, 6, numMap[seconds >> 4 & 0b00001111]);
        lcdWriteMask(&hlcd, 7, numMap[seconds & 0b00001111]);
        HAL_LCD_UpdateDisplayRequest(&hlcd);
        LL_LPUART_TransmitData8(LPUART1, 't');
        LL_mDelay(1000);
    }
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}