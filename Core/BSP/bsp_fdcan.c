#include "bsp_fdcan.h"
#include "fdcan.h"

/* ══════════════════════════════════════════════════════════════
 *  接收中断回调
 *  FDCAN 收到消息 → 打包成 CAN_RxMsg_t → 丢入统一队列
 * ══════════════════════════════════════════════════════════════*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    CAN_RxMsg_t rx_msg;
    FDCAN_RxHeaderTypeDef RxHeader;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, rx_msg.data) != HAL_OK)
        return;

    rx_msg.frame_type = (RxHeader.IdType == FDCAN_EXTENDED_ID) ? 1 : 0;
    rx_msg.can_id     = RxHeader.Identifier;

    osMessageQueuePut(can_rx_queueHandleHandle, &rx_msg, 0, 0);
}

/* ══════════════════════════════════════════════════════════════
 *  发送接口
 *  task 层直接调用，不需要关心 FDCAN 头配置细节
 *
 *  用法示例:
 *    uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
 *    bsp_fdcan_send(0x201, data, 8);            // 标准帧
 *    bsp_fdcan_send(0x18FF50F1, data, 8);        // 扩展帧
 * ══════════════════════════════════════════════════════════════*/
int bsp_fdcan_send(uint32_t id, const uint8_t *data, uint8_t len, uint8_t ext)
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier             = id;
    tx_header.IdType                 = ext ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    tx_header.TxFrameType            = FDCAN_DATA_FRAME;
    tx_header.DataLength             = len;
    tx_header.ErrorStateIndicator    = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch          = FDCAN_BRS_OFF;
    tx_header.FDFormat               = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl     = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker          = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data);
}

/* ══════════════════════════════════════════════════════════════
 *  配置过滤器 + 启动 FDCAN
 *  在 MX_FDCAN1_Init() 之后调用
 * ══════════════════════════════════════════════════════════════*/
void bsp_fdcan_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    /* 标准帧过滤器 — MASK=0 放行所有 */
    fdcan_filter.IdType = FDCAN_STANDARD_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    /* 扩展帧过滤器 — MASK=0 放行所有 */
    fdcan_filter.IdType = FDCAN_EXTENDED_ID;
    fdcan_filter.FilterIndex = 0;
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1 = 0x00;
    fdcan_filter.FilterID2 = 0x00;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);

    /* 未匹配的帧丢弃 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
        FDCAN_REJECT, FDCAN_REJECT,
        FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    /* 启动 FDCAN，使能接收中断 */
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
}
