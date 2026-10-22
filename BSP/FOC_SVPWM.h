#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "tim.h"
#include "cordic.h"
#include "stm32g4xx_ll_cordic.h"    /* LL CORDIC API: CSR 位宏 + 内联读写函数 */
#include "ADC_Sample.h"
#include "MT6826S.h"
#include "USB.h"
#include "Ringbuffer.h"
#include "PID.h"
#include "Fmt.h"

/*==============================================================================
 * FOC SVPWM 电压调制 (CORDIC 硬件加速)
 *
 *   TIM1 更新中断 (20 kHz) 内完成: 取电流/角度 → Clarke+Park → 电流环 →
 *   Park 逆变换 → 扇区判断 → 矢量作用时间 → 更新三相比较值。
 *   所有三角函数由 CORDIC 硬件完成, 避免软件 sinf/cosf 的高开销。
 *============================================================================*/

/*==============================================================================
 * 数学常量 (FPU 单精度)
 *============================================================================*/
#define SQRT_3              1.732050807f
#define SQRT_3_DIV_2        0.866025403f
#define INV_SQRT_3          0.577350269f
#define ONE_THIRD           0.333333333f

/* q1.31 定点数转换常量 (用于 CORDIC) */
#define Q31_SCALE           2147483648.0f       /* 2^31 */
#define Q31_INV_SCALE       4.656612873e-10f    /* 1/2^31 */

/* CORDIC 精度配置: 10 次迭代 (~0.1% 误差, 电机控制最佳平衡点)
 * 6 次=1.5% | 8 次=0.4% | 10 次=0.1% | 12 次=0.025% */
#define FOC_CORDIC_PRECISION    LL_CORDIC_PRECISION_10CYCLES

/*==============================================================================
 * 类型定义 (FPU 对齐优化)
 *============================================================================*/
typedef struct {
    float a;
    float b;
    float c;
} __attribute__((aligned(4))) abc_t;

typedef struct {
    float q;
    float d;
} __attribute__((aligned(4))) qd_t;

typedef struct {
    float alpha;
    float beta;
} __attribute__((aligned(4))) alphabeta_t;

/* FOC SVPWM 计算状态 (4 字节对齐, 便于 FPU 访问) */
typedef struct {
    uint8_t     sector;         /* 当前扇区 (1~6) */
    qd_t        Uqd;            /* d-q 轴电压指令 */
    qd_t        Iqd;            /* d-q 轴电流反馈 (Clarke+Park 结果) */
    abc_t       Uabc;           /* 三相输出电压 */
    abc_t       Iabc;           /* 三相电流 (原始+滤波) */
    alphabeta_t Ualpha_beta;    /* α-β 轴电压 */
    float       ElAngle;        /* 电角度 (rad) */
    float       power;          /* 母线电压 (1 kHz 更新) */
    uint32_t    Tpwm;           /* PWM 周期计数值 */
    bool        State;          /* FOC 运行状态 (true=运行) */
} __attribute__((aligned(4))) FOC_State_t;

extern FOC_State_t FOC_State;

/*==============================================================================
 * 接口函数
 *============================================================================*/
void FOC_Init(void);    /* 配置 CORDIC + 清零状态 + 启动 TIM1 更新中断 */
void FOC_Start(void);   /* 启动 TIM1 三相 PWM (含互补输出) */
void FOC_Stop(void);    /* 停止 TIM1 三相 PWM (含互补输出) */

#endif /* FOC_SVPWM_H */