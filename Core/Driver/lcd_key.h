#ifndef LCD_KEY_H
#define LCD_KEY_H

#include <stdint.h>

/* 屏幕按键动作回调 */
typedef void (*lcd_key_cb_t)(void);

/* 注册短按/长按动作（task 初始化时调用）*/
void lcd_key_register_short_press(lcd_key_cb_t cb);
void lcd_key_register_long_press(lcd_key_cb_t cb);

/* 初始化：默认下降沿等待按下 */
void lcd_key_init(void);

/* EXTI 中断里调用（按下/释放边沿切换，记录时间戳）*/
void lcd_key_exti_isr(void);

/* task 周期调用（长按计时检测）*/
void lcd_key_scan(void);

#endif // LCD_KEY_H