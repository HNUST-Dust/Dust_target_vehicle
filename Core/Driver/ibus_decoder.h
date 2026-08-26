#ifndef IBUS_DECODER_H
#define IBUS_DECODER_H

#include "cmsis_os2.h"
#include <stdint.h>

/* ── DBUS 遥控器解码结构 ── */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
} Remote_mouse_t;

typedef struct {
    uint16_t v;
} Remote_key_t;

typedef struct {
    /* 摇杆通道（-660 ~ 660，中位 0） */
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;

    /* 开关（1=上, 3=中, 2=下） */
    uint8_t s1;
    uint8_t s2;

    /* 鼠标数据 */
    Remote_mouse_t mouse;

    /* 键盘数据（位掩码） */
    Remote_key_t key;
} Remote_t;

/* ── 解码函数 ── */
Remote_t Remote_DBUS_to_RC(void);

#endif // IBUS_DECODER_H