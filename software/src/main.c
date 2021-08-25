#include <stdio.h>
#include <stm32l0xx_ll_rcc.h>
#include <stm32l0xx_ll_pwr.h>
#include <stm32l0xx_ll_rtc.h>
#include <stm32l0xx_ll_system.h>
#include <stm32l0xx_ll_utils.h>
#include <stm32l0xx_ll_cortex.h>
#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_ll_lpuart.h>
#include <stm32l0xx_hal.h>

#include "printf_retarget.h"
#include "interrupt.h"
#include "uart.h"
#include "lcd.h"

void init()
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0) {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);

    LL_RCC_MSI_Enable();
    while (LL_RCC_MSI_IsReady() != 1) {
    }
    LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_5); // 2.097 MHz

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR); // enable APB1
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG); // enable APB2

    LL_PWR_EnableBkUpAccess();
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_HIGH);
    LL_RCC_LSE_Enable();
    while (LL_RCC_LSE_IsReady() != 1) {
    }

    // LL_RCC_LSI_Enable();
    // while (LL_RCC_LSI_IsReady() != 1) {
    // }

    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    LL_RCC_EnableRTC();

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_MSI) {
    }

    LL_Init1msTick(2097000);
    // LL_InitTick(2097000, 1);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SYSTICK_EnableIT();

    SystemCoreClock = 2097000;
}

void gpio_init()
{
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
}

enum ClockMode {
    ModeClock,
    ModeTemperature,
    ModeCalendar,
    ModeVCC,
    ModeSetClock,
    ModeSetCalendar
};

enum ClockMode clock_mode = ModeClock;

void display_clock()
{
    uint32_t seconds = LL_RTC_TIME_GetSecond(RTC);
    uint32_t minutes = LL_RTC_TIME_GetMinute(RTC);
    uint32_t hours = LL_RTC_TIME_GetHour(RTC);

    lcd_write_digit(0, hours >> 4 & 0b00001111);
    lcd_write_digit(1, hours & 0b00001111);
    lcd_write_mask(2, 0b01000000);
    lcd_write_digit(3, minutes >> 4 & 0b00001111);
    lcd_write_digit(4, minutes & 0b00001111);
    lcd_write_mask(5, 0b01000000);
    lcd_write_digit(6, seconds >> 4 & 0b00001111);
    lcd_write_digit(7, seconds & 0b00001111);
    lcd_update();
}

void display_calendar()
{
    uint32_t day = LL_RTC_DATE_GetDay(RTC);
    uint32_t month = LL_RTC_DATE_GetMonth(RTC);
    uint32_t year = LL_RTC_DATE_GetYear(RTC);

    lcd_write_digit(0, day >> 4 & 0b00001111);
    lcd_write_digit(1, day & 0b00001111);
    lcd_write_mask(2, 0b01000000);
    lcd_write_digit(3, month >> 4 & 0b00001111);
    lcd_write_digit(4, month & 0b00001111);
    lcd_write_mask(5, 0b01000000);
    lcd_write_digit(6, year >> 4 & 0b00001111);
    lcd_write_digit(7, year & 0b00001111);
    lcd_update();
}

int main(void)
{
    init();
    HAL_Init();
    gpio_init();
    uart_init();
    printf("Init LCD\n");
    lcd_init();

    printf("Main loop:\n");
    while (1) {
        uart_process_cmd();

        switch (clock_mode) {
        case ModeCalendar:
            display_calendar();
            break;
        case ModeClock:
        default:
            display_clock();
        }

        uint32_t day = LL_RTC_DATE_GetDay(RTC);
        uint32_t month = LL_RTC_DATE_GetMonth(RTC);
        uint32_t year = LL_RTC_DATE_GetYear(RTC);
        uint32_t seconds = LL_RTC_TIME_GetSecond(RTC);
        uint32_t minutes = LL_RTC_TIME_GetMinute(RTC);
        uint32_t hours = LL_RTC_TIME_GetHour(RTC);

        printf("%d%d%d%d-%d%d-%d%dT%d%d:%d%d:%d%d\n",
               (uint8_t)(year >> 12 & 0b00001111),
               (uint8_t)(year >> 8 & 0b00001111),
               (uint8_t)(year >> 4 & 0b00001111), (uint8_t)(year & 0b00001111),
               (uint8_t)(month >> 4 & 0b00001111),
               (uint8_t)(month & 0b00001111), (uint8_t)(day >> 4 & 0b00001111),
               (uint8_t)(day & 0b00001111), (uint8_t)(hours >> 4 & 0b00001111),
               (uint8_t)(hours & 0b00001111),
               (uint8_t)(minutes >> 4 & 0b00001111),
               (uint8_t)(minutes & 0b00001111),
               (uint8_t)(seconds >> 4 & 0b00001111),
               (uint8_t)(seconds & 0b00001111));

        LL_mDelay(1000);
    }
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {
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