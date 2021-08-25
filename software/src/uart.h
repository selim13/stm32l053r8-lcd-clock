#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void uart_init();
void uart_interrupt_handle();
void uart_process_cmd();

#ifdef __cplusplus
}
#endif