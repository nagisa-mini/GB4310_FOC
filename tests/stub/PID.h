/*
 * 主机侧测试用 PID.h 桩
 *
 * 用途: 编译 tests/usb_cmd_test.c 时, 通过 -I tests/stub 优先于 -I BSP,
 *       覆盖真实 PID.h, 从而避免把 STM32 HAL / MT6826S 依赖带入主机测试。
 *       USB.c 使用 #include <PID.h> 正是为了让这里的桩生效。
 *
 * 注意: 本文件只声明 USB 命令模块用到的接口, 必须与 BSP/PID.h 保持一致。
 */

#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

#define PID_PI          3.14159265358979323846f
#define PID_2PI         6.28318530717958647692f

#define PID_MODE_STOP   0u
#define PID_MODE_ANGLE  1u
#define PID_MODE_SPEED  2u

typedef enum {
    PID_LOOP_ANGLE = 0,
    PID_LOOP_SPEED = 1
} PID_Loop_t;

typedef struct {
    float Angle_Ref;
    float Angle_Fdb;
    float Speed_Ref;
    float Speed_Fdb;
    float Curr_Q_Ref;
    uint8_t mode;
} PID_RUN_t;

extern PID_RUN_t PID_Run;

bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd);
bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd);
bool PID_SetMode(uint8_t mode);
void PID_ResetIntegralAll(void);

/* Flash 参数保存接口的测试桩 */
bool PID_Config_Save(void);
bool PID_Config_Load(void);
bool PID_Config_IsLoaded(void);

#endif /* PID_H */