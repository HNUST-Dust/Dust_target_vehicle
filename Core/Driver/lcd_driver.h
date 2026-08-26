#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#include <stdint.h>

#define LCD_W  280
#define LCD_H  240

#define CLR_WHITE   0xFFFF
#define CLR_BLACK   0x0000
#define CLR_RED     0xF800
#define CLR_BRRED   0xFC07

/* 兼容旧宏 */
#define WHITE  CLR_WHITE
#define BLACK  CLR_BLACK
#define BRRED  CLR_BRRED

void lcd_init(void);
void lcd_clear(uint16_t color);
void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color);
void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t size);
void lcd_show_chinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t size);
void lcd_show_int(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t size);
void lcd_show_picture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pic);

#endif
