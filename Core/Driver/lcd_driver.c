#include "lcd_driver.h"
#include "lcdfont.h"
#include "main.h"
#include "spi.h"
#include "cmsis_os2.h"
#include <stdio.h>

#define USE_ANALOG_SPI 0
#define USE_HORIZONTAL 2

#if USE_ANALOG_SPI
#define LCD_SCLK_Clr() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_RESET)
#define LCD_SCLK_Set() HAL_GPIO_WritePin(LCD_SCK_GPIO_Port, LCD_SCK_Pin, GPIO_PIN_SET)
#define LCD_MOSI_Clr() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_SDA_Pin, GPIO_PIN_RESET)
#define LCD_MOSI_Set() HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_SDA_Pin, GPIO_PIN_SET)
#endif

#define LCD_RES_Clr()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET)
#define LCD_RES_Set()  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET)
#define LCD_DC_Clr()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET)
#define LCD_DC_Set()   HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET)
#define LCD_CS_Clr()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_CS_Set()   HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_BLK_Set()  HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, GPIO_PIN_SET)

static void LCD_Writ_Bus(uint8_t dat)
{
    LCD_CS_Clr();
#if USE_ANALOG_SPI
    for (uint8_t i = 0; i < 8; i++) {
        LCD_SCLK_Clr();
        if (dat & 0x80) LCD_MOSI_Set();
        else             LCD_MOSI_Clr();
        LCD_SCLK_Set();
        dat <<= 1;
    }
#else
    HAL_SPI_Transmit(&hspi1, &dat, 1, 0xffff);
#endif
    LCD_CS_Set();
}

static void LCD_WR_DATA8(uint8_t dat) { LCD_Writ_Bus(dat); }
static void LCD_WR_DATA(uint16_t dat) { LCD_Writ_Bus(dat >> 8); LCD_Writ_Bus(dat); }
static void LCD_WR_REG(uint8_t dat)   { LCD_DC_Clr(); LCD_Writ_Bus(dat); LCD_DC_Set(); }

static void LCD_Address_Set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    if (USE_HORIZONTAL == 0) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1); LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1 + 20); LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 1) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1); LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1 + 20); LCD_WR_DATA(y2 + 20);
        LCD_WR_REG(0x2c);
    } else if (USE_HORIZONTAL == 2) {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1 + 20); LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1); LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    } else {
        LCD_WR_REG(0x2a); LCD_WR_DATA(x1 + 20); LCD_WR_DATA(x2 + 20);
        LCD_WR_REG(0x2b); LCD_WR_DATA(y1); LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);
    }
}

void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend, uint16_t color)
{
    LCD_Address_Set(xsta, ysta, xend - 1, yend - 1);
    for (uint16_t i = ysta; i < yend; i++)
        for (uint16_t j = xsta; j < xend; j++)
            LCD_WR_DATA(color);
}

static void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    LCD_Address_Set(x, y, x, y);
    LCD_WR_DATA(color);
}

void lcd_clear(uint16_t color)
{
    lcd_fill(0, 0, LCD_W, LCD_H, color);
}

static void LCD_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t sizex = sizey / 2;
    uint8_t bytes = (sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey;
    num = num - ' ';
    uint16_t x0 = x;
    LCD_Address_Set(x, y, x + sizex - 1, y + sizey - 1);
    for (uint8_t i = 0; i < bytes; i++) {
        uint8_t temp;
        if (sizey == 12)      temp = ascii_1206[num][i];
        else if (sizey == 16) temp = ascii_1608[num][i];
        else if (sizey == 24) temp = ascii_2412[num][i];
        else if (sizey == 32) temp = ascii_3216[num][i];
        else return;
        for (uint8_t t = 0; t < 8; t++) {
            if (!mode) LCD_WR_DATA((temp & (0x01 << t)) ? fc : bc);
            else if (temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
            x++;
            if ((x - x0) == sizex) { x = x0; y++; break; }
        }
    }
}

void lcd_show_string(uint16_t x, uint16_t y, const uint8_t *p, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    while (*p) {
        LCD_ShowChar(x, y, *p, fc, bc, sizey, 0);
        x += sizey / 2;
        p++;
    }
}

static uint32_t mypow(uint8_t m, uint8_t n)
{
    uint32_t r = 1;
    while (n--) r *= m;
    return r;
}

void lcd_show_int(uint16_t x, uint16_t y, uint16_t num, uint8_t len, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    uint8_t sizex = sizey / 2;
    uint8_t enshow = 0;
    for (uint8_t t = 0; t < len; t++) {
        uint8_t temp = (num / mypow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) { LCD_ShowChar(x + t * sizex, y, ' ', fc, bc, sizey, 0); continue; }
            else enshow = 1;
        }
        LCD_ShowChar(x + t * sizex, y, temp + 48, fc, bc, sizey, 0);
    }
}

static void LCD_ShowChinese12x12(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t bytes = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t cnt = sizeof(tfont12) / sizeof(typFNT_GB12);
    uint16_t x0 = x;
    for (uint16_t k = 0; k < cnt; k++) {
        if (tfont12[k].Index[0] == s[0] && tfont12[k].Index[1] == s[1]) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (uint8_t i = 0; i < bytes; i++) {
                for (uint8_t j = 0; j < 8; j++) {
                    if (!mode) LCD_WR_DATA((tfont12[k].Msk[i] & (0x01 << j)) ? fc : bc);
                    else if (tfont12[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                    x++; if ((x - x0) == sizey) { x = x0; y++; break; }
                }
            }
            return;
        }
    }
}

static void LCD_ShowChinese16x16(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t bytes = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t cnt = sizeof(tfont16) / sizeof(typFNT_GB16);
    uint16_t x0 = x;
    for (uint16_t k = 0; k < cnt; k++) {
        if (tfont16[k].Index[0] == s[0] && tfont16[k].Index[1] == s[1]) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (uint8_t i = 0; i < bytes; i++) {
                for (uint8_t j = 0; j < 8; j++) {
                    if (!mode) LCD_WR_DATA((tfont16[k].Msk[i] & (0x01 << j)) ? fc : bc);
                    else if (tfont16[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                    x++; if ((x - x0) == sizey) { x = x0; y++; break; }
                }
            }
            return;
        }
    }
}

static void LCD_ShowChinese24x24(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t bytes = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t cnt = sizeof(tfont24) / sizeof(typFNT_GB24);
    uint16_t x0 = x;
    for (uint16_t k = 0; k < cnt; k++) {
        if (tfont24[k].Index[0] == s[0] && tfont24[k].Index[1] == s[1]) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (uint8_t i = 0; i < bytes; i++) {
                for (uint8_t j = 0; j < 8; j++) {
                    if (!mode) LCD_WR_DATA((tfont24[k].Msk[i] & (0x01 << j)) ? fc : bc);
                    else if (tfont24[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                    x++; if ((x - x0) == sizey) { x = x0; y++; break; }
                }
            }
            return;
        }
    }
}

static void LCD_ShowChinese32x32(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey, uint8_t mode)
{
    uint8_t bytes = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    uint16_t cnt = sizeof(tfont32) / sizeof(typFNT_GB32);
    uint16_t x0 = x;
    for (uint16_t k = 0; k < cnt; k++) {
        if (tfont32[k].Index[0] == s[0] && tfont32[k].Index[1] == s[1]) {
            LCD_Address_Set(x, y, x + sizey - 1, y + sizey - 1);
            for (uint8_t i = 0; i < bytes; i++) {
                for (uint8_t j = 0; j < 8; j++) {
                    if (!mode) LCD_WR_DATA((tfont32[k].Msk[i] & (0x01 << j)) ? fc : bc);
                    else if (tfont32[k].Msk[i] & (0x01 << j)) LCD_DrawPoint(x, y, fc);
                    x++; if ((x - x0) == sizey) { x = x0; y++; break; }
                }
            }
            return;
        }
    }
}

void lcd_show_chinese(uint16_t x, uint16_t y, uint8_t *s, uint16_t fc, uint16_t bc, uint8_t sizey)
{
    while (*s) {
        if (sizey == 12)      LCD_ShowChinese12x12(x, y, s, fc, bc, sizey, 0);
        else if (sizey == 16) LCD_ShowChinese16x16(x, y, s, fc, bc, sizey, 0);
        else if (sizey == 24) LCD_ShowChinese24x24(x, y, s, fc, bc, sizey, 0);
        else if (sizey == 32) LCD_ShowChinese32x32(x, y, s, fc, bc, sizey, 0);
        else return;
        s += 2;
        x += sizey;
    }
}

void lcd_init(void)
{
    LCD_RES_Clr();
    HAL_Delay(100);
    LCD_RES_Set();
    HAL_Delay(100);

    LCD_BLK_Set();
    HAL_Delay(100);

    LCD_WR_REG(0x11);
    HAL_Delay(120);

    LCD_WR_REG(0x36);
    if (USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);
    else if (USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if (USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else                          LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A); LCD_WR_DATA8(0x05);
    LCD_WR_REG(0xB2); LCD_WR_DATA8(0x0C); LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x33);
    LCD_WR_REG(0xB7); LCD_WR_DATA8(0x35);
    LCD_WR_REG(0xBB); LCD_WR_DATA8(0x32);
    LCD_WR_REG(0xC2); LCD_WR_DATA8(0x01);
    LCD_WR_REG(0xC3); LCD_WR_DATA8(0x15);
    LCD_WR_REG(0xC4); LCD_WR_DATA8(0x20);
    LCD_WR_REG(0xC6); LCD_WR_DATA8(0x0F);
    LCD_WR_REG(0xD0); LCD_WR_DATA8(0xA4); LCD_WR_DATA8(0xA1);
    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17); LCD_WR_DATA8(0x14); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x34);
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x0E);
    LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x09); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x33); LCD_WR_DATA8(0x48);
    LCD_WR_DATA8(0x17); LCD_WR_DATA8(0x14); LCD_WR_DATA8(0x15);
    LCD_WR_DATA8(0x31); LCD_WR_DATA8(0x34);
    LCD_WR_REG(0x21);
    LCD_WR_REG(0x29);
}

extern const unsigned char gImage_pic[];

void lcd_show_picture(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pic)
{
    uint32_t k = 0;
    LCD_Address_Set(x, y, x + w - 1, y + h - 1);
    for (uint16_t i = 0; i < w; i++) {
        for (uint16_t j = 0; j < h; j++) {
            LCD_WR_DATA8(pic[k * 2]);
            LCD_WR_DATA8(pic[k * 2 + 1]);
            k++;
        }
    }
}
