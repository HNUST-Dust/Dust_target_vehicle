#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include "cmsis_os2.h"
#include <stdint.h>

/* ===========================================================
 *  统一 CAN 消息结构（接收用）
 *  中断回调只做"帧→结构体"的透传打包，不解码 ID
 *  解码逻辑放在 task 层，不同电机可以有不同的解码方式
 * ===========================================================
 *  frame_type : 0 = 标准帧, 1 = 扩展帧
 *  can_id     : 原始 CAN ID（标准帧 11 位 / 扩展帧 29 位）
 *  data[8]    : 8 字节数据
 * ===========================================================*/
typedef struct {
    uint8_t  frame_type;
    uint32_t can_id;
    uint8_t  data[8];
} CAN_RxMsg_t;

/* ── FreeRTOS 消息队列句柄（定义在 freertos.c） ── */
extern osMessageQueueId_t can_rx_queueHandleHandle;

/* ── FDCAN 发送接口 ── */

/**
 * bsp_fdcan_send - 发送 CAN 帧
 *
 * @id     : CAN ID
 * @data   : 数据缓冲区
 * @len    : 数据长度（1~8 字节）
 * @ext    : 0=标准帧, 1=扩展帧
 * @return : HAL_OK=成功，其他=HAL 错误码
 */
int bsp_fdcan_send(uint32_t id, const uint8_t *data, uint8_t len, uint8_t ext);
 void bsp_fdcan_init(void);
#endif // BSP_FDCAN_H