#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void lcd_init();
void lcd_write_mask(uint8_t pos, uint8_t mask);
void lcd_write_digit(uint8_t pos, uint8_t digit);
void lcd_update();

#ifdef __cplusplus
}
#endif