#include "task_ibus.h"
#include "task_zdt.h"
#include "cmsis_os2.h"

/* ── 全局遥控器通道值 ── */
volatile int16_t  remote_ch0, remote_ch1, remote_ch2, remote_ch3;
volatile int16_t  remote_s1, remote_s2;
volatile uint8_t  gimbal_enabled  = 0;
volatile uint8_t  chassis_enabled = 0;
volatile uint8_t  chassis_auto_mode = 0;
volatile float    chassis_vx      = 0.0f;
volatile float    chassis_vR      = 0.0f;

/* ── ZDT 灵敏度（编码器刻度域，= 0.3 脉冲 × 65536/3200）──
 *  手感等效原 0.3：摇杆推满 zdt_step=660×6.144≈4055 编码器/10ms */
#define ZDT_STEP_SCALE  6.144f

/* ── 外部 DMA 缓冲区（上电需清空） ── */
#include <string.h>
#include "usart.h"
#include "bsp_uart.h"

void task_ibus_recv(void *argument)
{
    (void)argument;

    uint32_t startup_t0 = osKernelGetTickCount();
    uint8_t  startup_cleared = 0;

    while (1)
    {
        Remote_t rm = Remote_DBUS_to_RC();

        /* ══════════════════════════════════════════════════════
         *  上电 1 秒保护（按系统时间计时，非按帧数）：
         *  DMA 初始搬运的垃圾数据会导致误动作，期间电机输出归零
         * ══════════════════════════════════════════════════════ */
        if ((osKernelGetTickCount() - startup_t0) < 1000)
        {
            /* 到 1 秒时清一次 DMA 垃圾并重启接收 */
            if (!startup_cleared)
            {
                startup_cleared = 1;
                memset((void *)usart_rx_buffer, 0, sizeof(usart_rx_buffer));
                HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart_rx_buffer, 32);
                __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
            }

            /* 所有输出强制归零 */
            gimbal_enabled  = 0;
            zdt_enabled     = 0;
            zdt_step        = 0;
            zdt_homing_req  = 0;
            chassis_auto_mode = 0;
            chassis_enabled = 0;
            chassis_vx      = 0.0f;
            chassis_vR      = 0.0f;

            /* 如果刚上电时 ch2 已经有值（摇杆没回中），说明用户操作了 → 允许 ZDT */
            if (rm.ch2 < -5 || rm.ch2 > 5) zdt_enabled = 1;

            osDelay(1);
            continue;
        }

        /* ══════════════════════════════════════════════════════
         *  正常模式（启动保护结束后）
         * ══════════════════════════════════════════════════════ */

        /* 写入全局通道值（裸数据，不做映射）*/
        remote_ch0 = rm.ch0;
        remote_ch1 = rm.ch1;
        remote_ch2 = rm.ch2;
        remote_ch3 = rm.ch3;
        remote_s1  = rm.s1;
        remote_s2  = rm.s2;
      
        /* ── s1 模式映射 ── */
        switch (remote_s1)
        {
        case 1:
            gimbal_enabled = 0;
            zdt_enabled    = 0;
            zdt_step       = 0;
            break;

        case 3:
            gimbal_enabled = 1;
            zdt_enabled    = 1;
            break;

        default:
            break;
        }

        /* ── ZDT 步进计算 ── */
        if (zdt_enabled) {
            int16_t ch2 = remote_ch2;
            if (ch2 > -5 && ch2 < 5) ch2 = 0;
            zdt_step = (int32_t)((float)ch2 * ZDT_STEP_SCALE);
        }

        /* ── s1=2 边沿 → ZDT 回零请求 ── */
        {
            static uint8_t last_s1 = 0;
            if (remote_s1 == 2 && last_s1 != 2)
                zdt_homing_req = 1;
            last_s1 = remote_s1;
        }

        /* ── s2 模式映射 ── */
        switch (remote_s2) {

            case 1:
            chassis_enabled = 1;
            chassis_auto_mode = 1;
            chassis_vx = 0.0f;
            chassis_vR = 0.0f;
            break;

        case 2:
            chassis_enabled = 1;
            chassis_auto_mode = 0;
            chassis_vx = (float)remote_ch1 / 660.0f;
            chassis_vR = (float)remote_ch0 / 660.0f;

            if (chassis_vx >-0.05f && chassis_vx < 0.05f) chassis_vx = 0.0f;
            if (chassis_vR >-0.05f && chassis_vR < 0.05f) chassis_vR = 0.0f;

            float vR_abs = (chassis_vR > 0.0f) ? chassis_vR : -chassis_vR;
            chassis_vR = chassis_vR * vR_abs;
            break;
    
        case 3:
            chassis_enabled = 0;
            chassis_vx = 0.0f;
            chassis_vR = 0.0f;
            break;
        }

        osDelay(1);
    }
}
