#include "task_zdt.h"
#include "motor_zdt.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>

/* ── ZDT 全局状态（编码器刻度域，65536/圈） ── */
volatile int32_t  zdt_step          = 0;
volatile uint8_t  zdt_enabled       = 0;
volatile int32_t  zdt_target[2]     = {0, 0};
volatile int32_t  zdt_real_pos[2]   = {0, 0};
volatile int32_t  zdt_abs_offset[2] = {0, 0};
volatile uint8_t  zdt_motor_pos_valid[2] = {0, 0};
volatile uint8_t  zdt_is_stalled[2] = {0, 0};
volatile uint8_t  zdt_calibrating[2]= {0, 0};
volatile uint8_t  zdt_homing_req    = 0;

/* ── 是否已建立绝对坐标（flash 恢复成功或已回零），未建立前禁止位置控制 ── */
static uint8_t zdt_anchored[2] = {0, 0};

/* ── 软限位（编码器刻度，= 56000 脉冲 × 65536/3200 = 17.5 圈）── */
#define ZDT_SOFT_LIMIT_TOP      1146880
#define ZDT_SOFT_LIMIT_BOTTOM   0

/* 同步模式：使用 ID=0x00 广播一发控制双电机 */
#define ZDT_SYNC_ID  0x00

/* ── 回零参数 ── */
#define ZDT_HOMING_SPEED   200      /* RPM，向底部回零（太快易误触发堵转保护）*/
#define ZDT_HOMING_TIMEOUT 600      /* 6s 超时保护（600 × 10ms）*/

/* ══════════════════════════════════════════════════════════════
 *  flash 掉电位置持久化（STM32H723 内部 flash 顶部空闲扇区）
 *  存的是 MCU 维护的绝对目标位置 zdt_target[]（编码器刻度）
 * ══════════════════════════════════════════════════════════════ */
#define ZDT_FLASH_ADDR   0x080E0000u
#define ZDT_FLASH_SECTOR FLASH_SECTOR_7
#define ZDT_FLASH_MAGIC  0x5A5AA5A5u

static void zdt_flash_write(void)
{
    uint32_t buf[8] = {0};                 /* 32B flash 字 */
    buf[0] = ZDT_FLASH_MAGIC;
    buf[1] = (uint32_t)zdt_target[0];
    buf[2] = (uint32_t)zdt_target[1];
    buf[7] = ZDT_FLASH_MAGIC ^ buf[1] ^ buf[2];   /* 校验 */

    FLASH_EraseInitTypeDef e = {0};
    uint32_t serr = 0;
    e.TypeErase    = FLASH_TYPEERASE_SECTORS;
    e.Banks        = FLASH_BANK_1;
    e.Sector       = ZDT_FLASH_SECTOR;
    e.NbSectors    = 1;
    e.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&e, &serr) == HAL_OK)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, ZDT_FLASH_ADDR, (uint32_t)buf);
    HAL_FLASH_Lock();
}

static int zdt_flash_read(int32_t out[2])
{
    const volatile uint32_t *p = (const volatile uint32_t *)ZDT_FLASH_ADDR;
    if (p[0] != ZDT_FLASH_MAGIC) return 0;
    if (p[7] != (ZDT_FLASH_MAGIC ^ p[1] ^ p[2])) return 0;
    out[0] = (int32_t)p[1];
    out[1] = (int32_t)p[2];
    /* 位置必须在软限位内才可信 */
    if (out[0] < ZDT_SOFT_LIMIT_BOTTOM || out[0] > ZDT_SOFT_LIMIT_TOP) return 0;
    if (out[1] < ZDT_SOFT_LIMIT_BOTTOM || out[1] > ZDT_SOFT_LIMIT_TOP) return 0;
    return 1;
}

/* 上电恢复：读 flash 目标 → 该物理位置就是驱动器上电归零点(offset)，免回零。
 * flash 无效则保持未锚定，需 s1=2 回零 */
static void zdt_boot_restore(void)
{
    int32_t saved[2] = {0, 0};
    if (zdt_flash_read(saved)) {
        for (uint8_t i = 0; i < 2; i++) {
            zdt_abs_offset[i] = saved[i];
            zdt_target[i]     = saved[i];
            zdt_real_pos[i]   = saved[i];
            zdt_anchored[i]   = 1;
        }
        printf("ZDT flash restore (no home): pos %ld %ld\r\n",
               (long)saved[0], (long)saved[1]);
    } else {
        zdt_anchored[0] = zdt_anchored[1] = 0;
        printf("ZDT no valid flash position → 需 s1=2 回零\r\n");
    }
}

/* ══════════════════════════════════════════════════════════════
 *  双电机同时回零（并行）：两个电机同时向底部速度模式撞底，
 *  各自独立检测堵转，撞底的先停，都完成后归零
 * ══════════════════════════════════════════════════════════════ */
static void zdt_homing_both(void)
{
    /* 手动回零：正方向（向上）撞限位作为零点 */
    motor_zdt_send_velocity(1, (int32_t)ZDT_HOMING_SPEED, 0);
    motor_zdt_send_velocity(2, (int32_t)ZDT_HOMING_SPEED, 0);

    uint32_t tick = 0;
    uint32_t timeout = 0;
    uint8_t done[2] = {0, 0};

    /* 等待两个电机都撞底堵转（或超时）*/
    while ((!done[0] || !done[1]) && timeout < ZDT_HOMING_TIMEOUT) {
        if ((tick++ % 5) == 0) {
            if (!done[0]) motor_zdt_send_request_status(1);
            if (!done[1]) motor_zdt_send_request_status(2);
        }
        osDelay(10);
        timeout++;

        for (uint8_t i = 0; i < 2; i++) {
            if (!done[i] && zdt_is_stalled[i]) {
                /* 该电机撞底：停止 + 释放 */
                motor_zdt_send_velocity(i + 1, 0, 0);
                motor_zdt_send_release_stall(i + 1);
                done[i] = 1;
            }
        }
    }

    /* 超时未完成的电机，强制停止 */
    for (uint8_t i = 0; i < 2; i++) {
        if (!done[i]) {
            motor_zdt_send_velocity(i + 1, 0, 0);
            motor_zdt_send_release_stall(i + 1);
        }
    }

    /* 等待堵转清除 + 统一收尾（不管撞底成功与否）*/
    for (uint8_t i = 0; i < 2; i++) {
        uint32_t wait = 0;
        while (zdt_is_stalled[i] && wait < 50) {
            osDelay(10);
            wait++;
            if ((wait % 2) == 0)
                motor_zdt_send_request_status(i + 1);
        }

        /* 统一：解除堵转 + 当前位置清零为原点 + 目标归零 */
        motor_zdt_send_release_stall(i + 1);
        osDelay(20);
        motor_zdt_send_zero_position(i + 1);
        osDelay(50);

        zdt_is_stalled[i]   = 0;
        zdt_calibrating[i]  = 0;
        zdt_abs_offset[i]   = 0;    /* 回零后驱动器原点 = 物理底部 */
        zdt_target[i]       = 0;
        zdt_real_pos[i]     = 0;
        zdt_anchored[i]     = 1;
        printf("ZDT %d homing done (zeroed)\r\n", i + 1);
    }

    zdt_flash_write();              /* 以 0 为新基准落盘 */
}

/* ══════════════════════════════════════════════════════════════
 *  步进电机控制 + 监控任务（10ms 周期）
 *  职责 1：上电自动回零（速度模式撞底）
 *  职责 2：摇杆积分 → 软限位 → 发绝对位置指令
 *  职责 3：轮询 0x36/0x3A → 堵转自动解除
 * ══════════════════════════════════════════════════════════════ */
void task_zdt_loop(void *argument)
{
    (void)argument;

    /* 上电使能双电机，等 1.5s 驱动器就绪后：读 flash 免回零恢复 */
    motor_zdt_send_enable(1, 1);
    motor_zdt_send_enable(2, 1);
    osDelay(1500);

    motor_zdt_send_zero_position(1);
    osDelay(20);
    motor_zdt_send_zero_position(2);

    zdt_boot_restore();   /* 不再上电撞底回零（伤电机）*/

    uint8_t stall_cnt = 0;
    uint8_t poll_phase = 0;     /* 轮流查询：0=电机1, 1=电机2 */
    uint32_t stall_release_t[2] = {0, 0};   /* 堵转释放冷却时间戳 */

    while (1)
    {
        /* ── 每 10 次循环（~100ms）轮流查询一个电机的位置+状态 ── */
        if (++stall_cnt >= 10) {
            stall_cnt = 0;
            uint8_t id = poll_phase ? 2 : 1;
            motor_zdt_send_request_status(id);
            motor_zdt_send_request_pos(id);
            poll_phase ^= 1;
        }

        /* ── 堵转检测与急救（回零期间不自动解除）──
         *  加 500ms 冷却：释放后不立即重复，防止"释放-旋转-再堵转"死循环 */
        for (uint8_t i = 0; i < 2; i++) {
            if (zdt_is_stalled[i] && !zdt_calibrating[i]) {
                uint32_t now = osKernelGetTickCount();
                if ((now - stall_release_t[i]) > 500) {
                    stall_release_t[i] = now;
                    uint8_t id = i + 1;
                    printf("ZDT %d stalled, releasing...\r\n", id);
                    motor_zdt_send_release_stall(id);
                    zdt_target[i] = zdt_real_pos[i];   /* 追平实际位置 */
                    zdt_is_stalled[i] = 0;
                }
            }
        }

        /* ── 停稳落盘：目标不变满 ~2s 存一次（防 flash 磨损）── */
        {
            static int32_t last_t = 0;
            static uint32_t cnt = 0;
            if (zdt_anchored[0]) {
                if (zdt_target[0] == last_t) {
                    if (cnt < 200u) cnt++;
                    if (cnt == 200u) {
                        zdt_flash_write();
                        printf("ZDT saved %ld\r\n", (long)zdt_target[0]);
                        cnt = 2000u;          /* 存后直到再变化才重存 */
                    }
                } else {
                    last_t = zdt_target[0];
                    cnt = 0;
                }
            }
        }

        /* ── 手动回零请求（task_ibus: s1=2 边沿触发）── */
        if (zdt_homing_req) {
            zdt_homing_req = 0;
            printf("ZDT manual homing...\r\n");
            zdt_homing_both();
        }

        /* ── ZDT 位置控制（已锚定才执行绝对定位）── */
        if (zdt_enabled && zdt_anchored[0] && zdt_anchored[1]
            && !zdt_calibrating[0] && !zdt_calibrating[1]) {
            if (zdt_step != 0) {
                zdt_target[0] += zdt_step;
                zdt_target[1]  = zdt_target[0];   /* 默认双电机联动 */

                if (zdt_target[0] > ZDT_SOFT_LIMIT_TOP) {
                    zdt_target[0] = ZDT_SOFT_LIMIT_TOP;
                    zdt_target[1] = ZDT_SOFT_LIMIT_TOP;
                }
                if (zdt_target[0] < ZDT_SOFT_LIMIT_BOTTOM) {
                    zdt_target[0] = ZDT_SOFT_LIMIT_BOTTOM;
                    zdt_target[1] = ZDT_SOFT_LIMIT_BOTTOM;
                }

                /* 发 目标-偏移：= 相对"上电点"的位移（例：上电1cm+step2cm
                 * → 绝对目标3cm → 发 3-1=2cm，不是绝对3cm）。驱动器内部上电归零，
                 * 这个值让它从上电点走出该位移 = 等效相对运动 */
                motor_zdt_send_position(1, zdt_target[0] - zdt_abs_offset[0]);
                motor_zdt_send_position(2, zdt_target[1] - zdt_abs_offset[1]);
            }
        }

        osDelay(10);
    }
}