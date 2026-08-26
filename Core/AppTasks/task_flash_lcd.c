#include "task_flash_lcd.h"
#include "spi.h"
#include "lcd_driver.h"
#include "lcd_key.h"
#include "bsp_fdcan.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* 短按动作：发 CAN 0x777 改灯色 */
static void key_short_action(void)
{
    uint8_t data = 0xCC;
    bsp_fdcan_send(0x777, &data, 1, 0);
}

/* 长按动作：清空击打数 */
static void key_long_action(void)
{
    BIG_BALL  = 000;
    SMALL_BALL = 000;
}

void task_flash_lcd_loop(void *argument)
{
    (void)argument;

    /* 提高自身优先级：CubeMX 里 Flash_Save_Task 是 osPriorityLow，
     * 会被其他 Normal/High 控制任务饿死，导致 LCD 不刷新 */
    osThreadSetPriority(osThreadGetId(), osPriorityNormal);

    /* 注册按键动作 + 初始化（短按/长按检测在 lcd_key.c）*/
    lcd_key_register_short_press(key_short_action);
    lcd_key_register_long_press(key_long_action);
    lcd_key_init();

    char display_buf1[6];
    char display_buf2[6];
    while(1) {
        osDelay(20);   /* 20ms 刷新一次 */

        /* 只刷新击打数数字（图片和中文标签上电 main.c 刷新）
         * 先清数字区域再显示，防止位数变化（如 100→0）时旧数字残留 */
        sprintf(display_buf1, "%d", BIG_BALL);
        lcd_fill(180, 100, 280, 124, CLR_BLACK);
        lcd_show_string(180, 100, (uint8_t *)display_buf1, CLR_BRRED, CLR_BLACK, 24);
        sprintf(display_buf2, "%d", SMALL_BALL);
        lcd_fill(180, 150, 280, 174, CLR_BLACK);
        lcd_show_string(180, 150, (uint8_t *)display_buf2, CLR_BRRED, CLR_BLACK, 24);

        lcd_key_scan();   /* 长按计时检测 */
    }
}