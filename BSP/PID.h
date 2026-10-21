#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>
#include "MT6826S.h"

/*==============================================================================
 * PID 模块: 角度环 - 角速度环 - 电流环 三环闭环控制
 *
 * 三环运行频率 (由调用位置决定, 模块内部用各自的 Ts):
 *   电流环:   20 kHz, 50us, 在 TIM1 FOC 中断里 (htim1 分支), 与 PWM/采样同频
 *   角速度环:  1 kHz,  1ms, 在 TIM7 中断里, 每拍都算
 *   角度环:   200 Hz,  5ms, 在 TIM7 中断里, 5 拍算一次
 *
 * 数据流 (给定 → 反馈 → 输出逐级传递):
 *   角度给定(rad) + 角度反馈(rad)          → 角速度给定(RPM)
 *   角速度给定(RPM) + 角速度反馈(RPM) → q 轴电流给定(A)
 *   q 轴电流给定(A) + Idq 反馈(A)           → Udq 电压给定(V)
 *
 * 电流环按 d/q 双轴独立控制: d 轴给定恒为 0, q 轴给定来自角速度环。
 *
 * 单位约定:
 *   角度      rad (弧度, 机械角)
 *   角速度    RPM (转每分钟)
 *   电流      A (安培)
 *   电压      V (伏特)
 *   周期      s (秒)
 *============================================================================*/

/*==============================================================================
 * 三环运算周期 (s)
 *============================================================================*/
#define PID_CURR_TS     0.00005f    /* 电流环周期: 1/20kHz (与 TIM1 PWM 同频) */
#define PID_SPEED_TS    0.001f      /* 角速度环周期: 1/1kHz (TIM7) */
#define PID_ANGLE_TS    0.005f      /* 角度环周期: 1/200Hz (TIM7 每 5 拍) */

/* 角度环分频: TIM7 每 5 拍执行一次角度环 */
#define PID_ANGLE_DIV   5U

#define PID_RAD2MIN     60.0f       /* rad/s → rad/min 换算系数 */
#define PID_MIN2RAD     0.016666667f /* rad/min → rad/s 换算系数 (1/60) */
#define PID_RAD2RPM     9.549296586f /* rad/s → rpm 换算系数 (60/2π) */

/*==============================================================================
 * 数学常量
 *============================================================================*/
#define PID_PI          3.14159265358979323846f     /* π */
#define PID_2PI         6.28318530717958647692f     /* 2π */

/*==============================================================================
 * 运行模式与可在线整定的环
 *============================================================================*/
#define PID_MODE_STOP   0u      /* 停止: 电流环输出零电压 */
#define PID_MODE_ANGLE  1u      /* 角度环: 外环生成角速度给定 */
#define PID_MODE_SPEED  2u      /* 角速度环: 直接使用 Speed_Ref */

typedef enum {
    PID_LOOP_ANGLE = 0,         /* 角度环 */
    PID_LOOP_SPEED = 1          /* 角速度环 */
} PID_Loop_t;

/*==============================================================================
 * 类型定义
 *============================================================================*/
/**
 * @brief PID 控制器 (位置式)
 * @note  积分项与输出均带限幅, 无其他附加保护
 */
typedef struct {
    float Kp;           /* 比例系数 */
    float Ki;           /* 积分系数 */
    float Kd;           /* 微分系数 */

    float I_limit;      /* 积分限幅 (积分项绝对值上限, 单位与输出一致) */
    float Out_limit;    /* 输出限幅 (输出绝对值上限, 单位与输出一致) */

    float Ref;          /* 给定值 */
    float Fdb;          /* 反馈值 */
    float Err;          /* 本次误差 */
    float Err_last;     /* 上次误差 (用于微分) */

    float Integral;     /* 积分累加项 */
    float Out;          /* 输出 */
} PID_t;

typedef struct {
    float Angle_Ref;      /* 角度环给定 (rad) */
    float Angle_Fdb;      /* 角度环反馈 (rad) */
    float Speed_Ref;      /* 角速度环给定 (RPM) */
    float Speed_Fdb;      /* 角速度环反馈 (RPM) */
    float Curr_Q_Ref;     /* 电流环 q 轴给定 (A) */
    uint8_t mode;         /* 运行模式: 0=停止, 1=角度环, 2=角速度环 */
} PID_RUN_t;

extern PID_t PID_Angle;     /* 角度环 */
extern PID_t PID_Speed;     /* 角速度环 */
extern PID_t PID_Curr_D;    /* 电流环 d 轴 */
extern PID_t PID_Curr_Q;    /* 电流环 q 轴 */
extern PID_RUN_t PID_Run;       /* 三环运行状态 (给定/反馈/模式) */

/*==============================================================================
 * 函数声明
 *============================================================================*/
void  PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
               float I_limit, float Out_limit);             /* 初始化参数并清空状态 */
void  PID_Reset(PID_t *pid);                                /* 清空状态 (误差/积分/输出) */
float PID_Calc(PID_t *pid, float ref, float fdb, float ts); /* 单环运算, 返回输出 */
float PID_Angle_Err(float ref, float fdb);                  /* 角度误差 (−π, π], 跨零点取最短路径 */

/* 三环闭环 */
void  PID_3Loop_Init(void);                                 /* 三环参数初始化 (各环状态清零) */

void  PID_Angle_Loop_Isr(void);              /* 角度环:   TIM7 @200Hz */
void PID_Speed_Loop_Isr(void);                              /* 角速度环: TIM7 @1kHz, 输出=q轴电流给定 */
void  PID_Current_Loop_Isr(float id_fdb, float iq_fdb,
                           float *vd_out, float *vq_out);               /* 电流环:   TIM1 @20kHz, d/q 双轴 */

/*==============================================================================
 * 上位机在线整定接口
 *============================================================================*/
bool PID_SetGains(PID_Loop_t loop, float Kp, float Ki, float Kd); /* 写一组增益 (短临界区, 不改变积分状态) */
bool PID_GetGains(PID_Loop_t loop, float *Kp, float *Ki, float *Kd); /* 读一组增益 */
bool PID_SetMode(uint8_t mode);                                   /* 切换运行模式 (变化时复位各环状态) */
void PID_ResetIntegralAll(void);                                  /* 清空角度/角速度/电流 d/q 四个积分项 (短临界区) */

/*==============================================================================
 * 参数掉电保存 (Flash 最后一页)
 *   仅保存角度环/角速度环各 3 个增益; 模式与给定不保存。
 *============================================================================*/
bool PID_Config_Save(void);                                       /* 保存当前 6 个增益到 Flash; 仅 STOP 模式允许 */
bool PID_Config_Load(void);                                       /* 上电读取并应用; 无有效记录时保留默认值 */
bool PID_Config_IsLoaded(void);                                   /* 本次上电是否成功加载了保存的参数 */
#endif /* PID_H */
