#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * PID 三环闭环: 角度环 → 角速度环 → 电流环 d/q
 *
 * 级联关系 (给定与反馈逐级传递):
 *   角度给定(rad)   + 角度反馈(rad)   → 角速度给定(RPM)
 *   角速度给定(RPM) + 角速度反馈(RPM) → q 轴电流给定(A)
 *   d 轴电流给定(A) + d 轴电流反馈(A) → d 轴电压给定(V)  (电流模式可设 Id)
 *   q 轴电流给定(A) + q 轴电流反馈(A) → q 轴电压给定(V)
 *
 * 运行频率 (由调用位置决定, 各环使用独立周期):
 *   电流环   20 kHz (50 µs)  TIM1 中断; 非电流模式 d 轴给定固定为 0,
 *                            电流模式 d/q 给定分别由 Id/Curr 命令下发
 *   角速度环  1 kHz (1 ms)   TIM7 中断
 *   角度环   200 Hz (5 ms)   TIM7 中断, 每 5 拍执行一次
 *
 * 单位: 角度 rad (机械角), 角速度 RPM, 电流 A, 电压 V, 周期 s
 *
 * 状态封装: PID 控制器实例与三环运行状态均为模块私有, 外部只能通过
 *          本文件的函数读写 (给定/模式/增益/遥测), 不再直接访问全局变量。
 *============================================================================*/

/*==============================================================================
 * 周期、换算与数学常量
 *============================================================================*/
#define PID_CURR_TS         0.00005f              /* 电流环周期: 1/20kHz */
#define PID_SPEED_TS        0.001f                /* 角速度环周期: 1/1kHz */
#define PID_ANGLE_TS        0.005f                /* 角度环周期: 1/200Hz */
#define PID_ANGLE_DIV       5U                    /* 角度环在 TIM7 中的分频 */
#define PID_RAD2RPM         9.549296586f          /* rad/s → RPM (60/2π) */
#define PID_PI              3.14159265358979323846f /* π */
#define PID_2PI             6.28318530717958647692f /* 2π */

/*==============================================================================
 * 运行模式
 *============================================================================*/
#define PID_MODE_STOP       0u                    /* 停止: 复位各环, 输出零电压 */
#define PID_MODE_ANGLE      1u                    /* 角度环: 外环生成角速度给定 */
#define PID_MODE_SPEED      2u                    /* 角速度环: 直接使用 Speed 给定 */
#define PID_MODE_CURRENT    3u                    /* 电流环: 直接使用 Curr/Id 给定 (q/d 轴) */

/* 可在线整定的环 (电流环 d/q 共用同一组增益) */
typedef enum {
    PID_LOOP_ANGLE = 0,                           /* 角度环 */
    PID_LOOP_SPEED = 1,                           /* 角速度环 */
    PID_LOOP_CURRENT = 2                          /* 电流环 (同时写入 d/q) */
} PID_Loop_t;

/* 遥测通道: PID_GetTelemetry() 按此顺序写入 float 数组 */
typedef enum {
    PID_TELEM_ANGLE_REF = 0,                      /* 角度给定 (rad) */
    PID_TELEM_ANGLE_FDB,                          /* 角度反馈 (rad) */
    PID_TELEM_ANGLE_OUT,                          /* 角度环输出 (RPM) */
    PID_TELEM_SPEED_REF,                          /* 角速度给定 (RPM) */
    PID_TELEM_SPEED_FDB,                          /* 角速度反馈 (RPM) */
    PID_TELEM_SPEED_OUT,                          /* 角速度环输出 (A) */
    PID_TELEM_CURR_Q_REF,                         /* q 轴电流给定 (A) */
    PID_TELEM_CURR_Q_FDB,                         /* q 轴电流反馈 (A) */
    PID_TELEM_CURR_Q_OUT,                         /* 电流环 q 轴输出 (V) */
    PID_TELEM_CURR_D_REF,                         /* d 轴电流给定 (A) */
    PID_TELEM_CURR_D_FDB,                         /* d 轴电流反馈 (A) */
    PID_TELEM_CURR_D_OUT,                         /* 电流环 d 轴输出 (V) */
    PID_TELEM_COUNT
} PID_Telem_t;

/*==============================================================================
 * 初始化与运行状态接口
 *============================================================================*/
void    PID_3Loop_Init(void);                     /* 四环参数初始化, 状态清零, 默认 STOP */
uint8_t PID_GetMode(void);                        /* 当前运行模式 (PID_MODE_xxx) */
void    PID_SetAngleRef(float rad);               /* 写角度给定; 值变化时清四环积分 */
void    PID_SetSpeedRef(float rpm);               /* 写角速度给定; 值变化时清四环积分 */
bool    PID_SetCurrentRef(float amp);             /* 写 q 轴电流给定; 仅电流模式, 变化时清四环积分 */
bool    PID_SetCurrentDRef(float amp);            /* 写 d 轴电流给定; 仅电流模式, 变化时清四环积分 */
void    PID_GetTelemetry(float *values);          /* 取 PID_TELEM_COUNT 个遥测值 */

/*==============================================================================
 * 三环中断运算 (调用周期与上文频率一致)
 *============================================================================*/
void PID_Angle_Loop_Isr(void);                    /* 角度环:   200 Hz */
void PID_Speed_Loop_Isr(void);                    /* 角速度环:  1 kHz */
void PID_Current_Loop_Isr(float id_fdb, float iq_fdb,
                          float *vd_out, float *vq_out); /* 电流环: 20 kHz */

/*==============================================================================
 * 上位机在线整定接口
 *============================================================================*/
bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd);
bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd);
bool PID_SetMode(uint8_t mode);                   /* 切换模式; 变化时复位各环状态 */
void PID_ResetIntegralAll(void);                  /* 只清四个积分项 */

/*==============================================================================
 * 参数掉电保存 (Flash 最后一页, 角度环/角速度环/电流环共 9 个增益)
 *============================================================================*/
bool PID_Config_Save(void);                       /* 仅 STOP 模式允许写入 */
bool PID_Config_Load(void);                       /* 无有效记录时保留默认增益 */
bool PID_Config_IsLoaded(void);                   /* 本次上电/最近一次 load 是否成功 */

#endif /* PID_H */