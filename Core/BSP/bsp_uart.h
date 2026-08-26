#ifndef BSP_UART_H
#define BSP_UART_H

#include "cmsis_os2.h"
#include <stdint.h>

/* UART DMA 接收缓冲区（全局持久内存） */
extern uint8_t usart_rx_buffer[32];

/* FreeRTOS 队列句柄（定义在 freertos.c） */
extern osMessageQueueId_t ibus_rx_queueHandle;

void bsp_usart_init(void);

#endif // BSP_UART_H
