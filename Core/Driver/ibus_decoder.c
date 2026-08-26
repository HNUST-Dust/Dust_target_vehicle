#include "ibus_decoder.h"

/* ── FreeRTOS 队列句柄（定义在 freertos.c） ── */
extern osMessageQueueId_t ibus_rx_queueHandle;

static Remote_t Remote;

Remote_t Remote_DBUS_to_RC(void)
{
    uint8_t rx_data[18]; 

    // 获取18bite从queue
    osMessageQueueGet(ibus_rx_queueHandle, rx_data, NULL, osWaitForever);
    
    // --- 手柄映射 ---
    // Range: -660 to 660. Middle position is 0.
    // Important: Make sure Remote.ch0 ~ ch3 are defined as int16_t!
    Remote.ch0 = ((int16_t)(((rx_data[0])      | (rx_data[1] << 8)) & 0x07FF)) - 1024;
    Remote.ch1 = ((int16_t)(((rx_data[1] >> 3) | (rx_data[2] << 5)) & 0x07FF)) - 1024;
    Remote.ch2 = ((int16_t)(((rx_data[2] >> 6) | (rx_data[3] << 2) | (rx_data[4] << 10)) & 0x07FF)) - 1024;
    Remote.ch3 = ((int16_t)(((rx_data[4] >> 1) | (rx_data[5] << 7)) & 0x07FF)) - 1024;

    // --- RC Switches ---
    // Values: 1 (Up), 3 (Middle), 2 (Down)
    Remote.s1 = ((rx_data[5] >> 4) & 0x000C) >> 2;
    Remote.s2 = ((rx_data[5] >> 4) & 0x0003);
    


    // --- 鼠标映射 ---
    // X, Y, Z axes (Speed/Distance)
    Remote.mouse.x = (int16_t)(rx_data[6]  | (rx_data[7] << 8));
    Remote.mouse.y = (int16_t)(rx_data[8]  | (rx_data[9] << 8));
    Remote.mouse.z = (int16_t)(rx_data[10] | (rx_data[11] << 8));
    
    // Left and Right buttons (0 = release, 1 = press)
    Remote.mouse.press_l = rx_data[12];
    Remote.mouse.press_r = rx_data[13];
    
    // --- Keyboard Data ---
    // Each bit represents one key (W, A, S, D, Q, E, Shift, Ctrl, etc.)
    Remote.key.v = (uint16_t)(rx_data[14] | (rx_data[15] << 8));
 
    return Remote;
}