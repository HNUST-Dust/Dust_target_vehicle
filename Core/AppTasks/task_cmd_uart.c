#include "task_cmd_uart.h"
#include "cmd_protocol.h"
#include "usart.h"
#include "cmsis_os2.h"


/* ── 全局控制变量定义（给 cmd_protocol.h 里的 extern 提供实体）── */
volatile uint8_t  cmd_chassis_mode   = 0;
volatile float    cmd_chassis_dist_m  = 2.0f;    /* 默认 3 米 */
volatile float    cmd_chassis_speed   = 100.0f;   /* 默认 100 RPM */
volatile uint8_t  cmd_chassis_accel   = 0;
volatile uint8_t  cmd_gimbal_mode      = 0;
volatile uint8_t  cmd_gimbal_speed     = 0;
volatile uint8_t  cmd_gimbal_speed_max = 0;
volatile uint8_t  cmd_gimbal_accel     = 0;
volatile uint8_t  cmd_gimbal_run       = 0;
volatile uint8_t  cmd_armor_motor     = 0;
volatile uint8_t  cmd_armor_accel     = 0;
volatile uint32_t  cmd_last_rx_tick    = 0;   /* 上次接收的有效帧 */

/* ── 接收队列句柄 ── */
static osMessageQueueId_t cmd_uart_queue;

/* ── 中断接收缓冲区（HAL_UART_Receive_IT 每次写入 1 字节到这里）── */
static uint8_t cmd_uart_rx_byte;

/* ═══════════════════════════════════════════════════════════
 *  CMD_ParseFrame — 收到完整帧 → 更新控制变量
 *  f 指向 8 字节数组: [0xA5] [CMD] [D0] [D1] [D2] [D3] [D4] [XOR]
 * ═══════════════════════════════════════════════════════════ */
void CMD_ParseFrame(const uint8_t *f)
{
    uint8_t cmd = f [1];

    switch (cmd)
    {
        case CMD_CHASSIS_CFG:       /*底盘参数设置*/
            cmd_chassis_dist_m = (float)f[2] * 0.1f;  
            cmd_chassis_speed  = (float)f[3];     /*D1:输出轴速度rpm*/
            cmd_chassis_accel  = f[4];            /*D2:加速度*/
            break;

        case CMD_CHASSIS_GO:        /*底盘运动控制*/
            cmd_chassis_mode = f[2];   /*D0=0停/1动*/
            break;

        case CMD_GIMBAL_CFG:        /*云台参数设置*/
            cmd_gimbal_mode      = f[2];   /*D0:模式 1=固定速度 2=区间变速*/
            cmd_gimbal_speed     = f[3];   /*D1:固定速度值/区间最低*/
            cmd_gimbal_speed_max = f[4];   /*D2:区间最高速度*/
            cmd_gimbal_accel     = f[5];   /*D3:加速度*/
            break;

        case CMD_GIMBAL_GO:         /*云台运动控制*/
            cmd_gimbal_run = f[2];     /*D0=0停/1启动*/
            break;

        case CMD_ARMOR_CFG:         /*装甲板参数设置*/
            cmd_armor_motor = f[2];    /*D0:电机号*/
            cmd_armor_accel = f[3];    /*D1:加速度*/
            break;          

        case CMD_ALL_STOP:          /*紧急停止*/
            cmd_chassis_mode    = 0;
            cmd_gimbal_mode     = 0;
            cmd_gimbal_run      = 0;
            cmd_gimbal_speed    = 0;
            cmd_gimbal_speed_max = 0;
            cmd_armor_motor     = 0;
            break;

        default:
            break;
    }
}

/* ═══════════════════════════════════════════════════════════
 *  HAL_UART_RxCpltCallback — 中断回调，每收到 1 字节触发
 *
 *  注意：这个函数在中断里跑，不能做耗时操作！
 *  它只做：读字节 → 塞队列 → 再启动下一次接收
 * ═══════════════════════════════════════════════════════════ */
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
    if (huart->Instance != UART7)
        return;

    /* HAL 已经把收到的字节写进了 cmd_uart_rx_byte，直接塞队列 */
    osMessageQueuePut(cmd_uart_queue, &cmd_uart_rx_byte, 0, 0);

    /* 再启动下一次接收 */
    HAL_UART_Receive_IT(huart, &cmd_uart_rx_byte, 1);
 }

 /* ═══════════════════════════════════════════════════════════
 *  task_cmd_uart_init — 初始化队列 + 启动 UART7 中断接收
 *  在 freertos.c 里创建任务之前调用
 * ═══════════════════════════════════════════════════════════ */
 void task_cmd_uart_init(void)
 {
    /* 创建队列：64 槽，每槽 1 字节 */
    cmd_uart_queue = osMessageQueueNew(64, 1, NULL);

    /* 启动链式接收 */
    HAL_UART_Receive_IT(&huart7, &cmd_uart_rx_byte, 1);
 }

 /* ═══════════════════════════════════════════════════════════
 *  task_cmd_uart_loop — 帧拼装 + 校验 + 解析
 *
 *  从队列取字节 → 找帧头 0xA5 → 收满 8 字节 → XOR 校验 → 解析
 * ═══════════════════════════════════════════════════════════ */
 void task_cmd_uart_loop(void *argument)
 {
    (void) argument;

    uint8_t buf[8];
    uint8_t idx = 0;
    uint8_t sync = 0; /* 0=等待帧头, 1=收数据 */

    while(1)
    {
        uint8_t byte;
        if (osMessageQueueGet(cmd_uart_queue, &byte, NULL, osWaitForever)
                != osOK) {
            continue;
        }

        if (!sync) {
            /* 没找到帧头，跳过非 0xA5 的字节 */
            if (byte == CMD_FRAME_HEADER) {
                sync = 1;
                idx  = 0;
                buf[idx++] = byte;   /* buf[0] = 0xA5 */
            }
        } else {
            buf[idx++] = byte;

            if (idx == CMD_FRAME_SIZE) {
                /* 收满 8 字节，算 XOR 校验 */
                uint8_t xor = 0;
                for (int i = 0; i < 7; i++) xor ^= buf[i];

                if (xor == buf[7]) {
                    cmd_last_rx_tick = osKernelGetTickCount();
                    CMD_ParseFrame(buf);  /* 校验过 → 执行 */
                }
                /* 校验不过 → 悄悄丢掉 */
                sync = 0;
                idx  = 0;
            }
        }
    
    }
 }