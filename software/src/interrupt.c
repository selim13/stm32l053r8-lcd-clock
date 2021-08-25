#include "uart.h"

void RNG_LPUART1_IRQHandler()
{
    uart_interrupt_handle();
}