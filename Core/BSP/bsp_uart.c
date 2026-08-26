#include "bsp_uart.h"
#include "usart.h"

/* ── DMA 接收缓冲区 ──
 *  ⚠ 必须放在 .dma_buffers 段（RAM_D2），DTCM 不能被 DMA 访问 */
uint8_t usart_rx_buffer[32] __attribute__((section(".dma_buffers")));

/* ══════════════════════════════════════════════════════════════
 *  UART 接收回调（空闲中断触发）
 *  收到完整一帧（18 字节）→ 投递到队列 → 重启接收
 * ══════════════════════════════════════════════════════════════*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5){

    if (Size == 18)
    {
        osMessageQueuePut(ibus_rx_queueHandle, usart_rx_buffer, 0, 0);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart_rx_buffer, 32);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

/* ══════════════════════════════════════════════════════════════
 *  UART 错误回调
 *  发生校验错/帧错后重启 DMA 接收，否则永久停收
 * ══════════════════════════════════════════════════════════════*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == UART5)
    {
        // 【关键修改】：发生溢出、帧错误等情况时，必须先终止当前的接收状态机
        HAL_UART_AbortReceive(huart);

        // 之后再重新开启 DMA 空闲接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart_rx_buffer, 32);
    }
}

void bsp_usart_init(void)
{
    /* 启动第一次 DMA 空闲中断接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart_rx_buffer, 32);
    __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
}