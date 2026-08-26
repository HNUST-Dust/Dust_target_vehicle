#include "lcd_key.h"
#include "main.h"

/* ── 按键：PA5 GPIO 输入 + 上拉，按下接地 = 低电平 ── */
#define KEY_PORT       GPIOA
#define KEY_PIN        GPIO_PIN_5
#define KEY_LONG_MS    3000
/* 消抖周期数：约 5ms（按主频 550MHz 估算，主频低则实际延时更长，更保险）*/
#define KEY_DEBOUNCE_CYCLES  2750000

static lcd_key_cb_t key_short_cb = 0;
static lcd_key_cb_t key_long_cb  = 0;
static uint32_t key_press_start = 0;
static uint8_t  key_long_triggered = 0;

void lcd_key_register_short_press(lcd_key_cb_t cb) { key_short_cb = cb; }
void lcd_key_register_long_press(lcd_key_cb_t cb)  { key_long_cb  = cb; }

/* 切换 EXTI 触发边沿（边沿切换：每次只使能一个边沿）*/
static void key_set_edge(uint32_t mode)
{
    GPIO_InitTypeDef init = {0};
    init.Pin   = KEY_PIN;
    init.Mode  = mode;
    init.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_PORT, &init);
}

void lcd_key_init(void)
{
    /* 使能 DWT 周期计数器（消抖用，不依赖 SysTick，中断里不会死锁）*/
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    key_set_edge(GPIO_MODE_IT_FALLING);   /* 默认下降沿，等按下 */
}

/* ══════════════════════════════════════════════════════════════
 *  EXTI 中断里调用：按下记录时间，释放判断短按/长按
 *  gpio.c 的 HAL_GPIO_EXTI_Callback 转发到这里
 * ══════════════════════════════════════════════════════════════ */
void lcd_key_exti_isr(void)
{
    /* 消抖：DWT 周期计数延时（硬件计数器，中断优先级高于 SysTick 也不死锁）*/
    DWT->CYCCNT = 0;
    while (DWT->CYCCNT < KEY_DEBOUNCE_CYCLES) { }

    if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET) {
        /* 确认按下（下降沿）：记录时间，切到上升沿等释放 */
        key_press_start = HAL_GetTick();
        key_set_edge(GPIO_MODE_IT_RISING);
    } else {
        /* 确认释放（上升沿）：不足 3s 且未触发长按 → 短按 */
        uint32_t hold_ms = HAL_GetTick() - key_press_start;
        if (hold_ms < KEY_LONG_MS && !key_long_triggered) {
            if (key_short_cb) key_short_cb();
        }
        key_long_triggered = 0;
        key_set_edge(GPIO_MODE_IT_FALLING);   /* 等下一次按下 */
    }
}

/* ══════════════════════════════════════════════════════════════
 *  task 周期调用：按住 ≥3s → 长按回调
 * ══════════════════════════════════════════════════════════════ */
void lcd_key_scan(void)
{
    if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_RESET) {
        if (!key_long_triggered &&
            (HAL_GetTick() - key_press_start) >= KEY_LONG_MS) {
            key_long_triggered = 1;
            if (key_long_cb) key_long_cb();
        }
    }
}