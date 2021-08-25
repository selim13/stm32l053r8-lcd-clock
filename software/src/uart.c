#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stm32l0xx_ll_rcc.h>
#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_ll_lpuart.h>

const uint8_t rx_buffer_size = 255;
char rx_buffer[255];
uint8_t rx_idx = 0;
bool rx_cmd_received = false;

void uart_init()
{
    LL_RCC_SetLPUARTClockSource(LL_RCC_LPUART1_CLKSOURCE_PCLK1);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPUART1);
    while (LL_APB1_GRP1_IsEnabledClock(LL_APB1_GRP1_PERIPH_LPUART1) != 1) {
    }

    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_10, LL_GPIO_AF_0);
    LL_GPIO_SetAFPin_8_15(GPIOC, LL_GPIO_PIN_11, LL_GPIO_AF_0);

    // Enable interrupts
    NVIC_SetPriority(RNG_LPUART1_IRQn, 0);
    NVIC_EnableIRQ(RNG_LPUART1_IRQn);

    LL_LPUART_InitTypeDef LPUART_InitStruct = {0};
    // LPUART_InitStruct.BaudRate = 115200;
    LPUART_InitStruct.BaudRate = 9600;
    LPUART_InitStruct.DataWidth = LL_LPUART_DATAWIDTH_8B;
    LPUART_InitStruct.StopBits = LL_LPUART_STOPBITS_1;
    LPUART_InitStruct.Parity = LL_LPUART_PARITY_NONE;
    LPUART_InitStruct.TransferDirection = LL_LPUART_DIRECTION_TX_RX;
    LPUART_InitStruct.HardwareFlowControl = LL_LPUART_HWCONTROL_NONE;
    LL_LPUART_Init(LPUART1, &LPUART_InitStruct);
    LL_LPUART_EnableIT_RXNE(LPUART1);
    LL_LPUART_Enable(LPUART1);
}

void uart_write_byte(uint8_t byte)
{
    LL_LPUART_TransmitData8(LPUART1, byte);
    while (!LL_LPUART_IsActiveFlag_TC(LPUART1)) {
    }
}

void uart_interrupt_handle()
{
    if (rx_cmd_received) {
        return;
    }

    if (!LL_LPUART_IsActiveFlag_RXNE(LPUART1)) {
        return;
    }

    uint8_t byte = LL_LPUART_ReceiveData8(LPUART1);
    uart_write_byte(byte);

    char c = (char)byte;
    rx_buffer[rx_idx] = c;

    if (c == '\n') {
        rx_cmd_received = true;
        return;
    }

    rx_idx++;
    if (rx_idx > rx_buffer_size) {
        rx_idx = 0;
    }
}

bool prefix(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}

void uart_process_cmd()
{
    if (!rx_cmd_received) {
        return;
    }

    if (prefix("AT+DT?", rx_buffer)) {
        printf("YES\n");
    }

    rx_cmd_received = false;
    rx_idx = 0;
}