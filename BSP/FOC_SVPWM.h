#ifndef FOC_SVPWM_H
#define FOC_SVPWM_H

#include <stdio.h>
#include <stdbool.h>
#include "tim.h"
#include "cordic.h"
#include "stm32g4xx_ll_cordic.h"    /* LL CORDIC API: CS寄存器位宏 + 内联读写函数 */
#include "ADC_Sample.h"
#include "MT6826S.h"
#include "USB.h"
#include "Ringbuffer.h"

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

/* CORDIC 精度配置: 10次迭代 (~0.1% 误差, 电机控制最佳平衡点)
 * 6次=1.5%误差 | 8次=0.4% | 10次=0.1% | 12次=0.025% */
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

/**
 * @brief FOC SVPWM 计算结构体 (4字节对齐, 优化FPU访问)
 */
typedef struct {
    uint8_t       sector;           /* 当前扇区 (1-6) */
    qd_t          Uqd;              /* D-Q轴电压指令 */
    qd_t          Iqd;              /* D-Q轴电流反馈 (Clarke+Park变换结果) */
    abc_t         Uabc;             /* 三相输出电压 */
    abc_t         Iabc;             /* 三相电流 (原始+滤波) */
    alphabeta_t   Ualpha_beta;      /* α-β轴电压 */
    float         ElAngle;          /* 电角度 (弧度) */
    float         power;            /* 母线电压 (1ms更新) */
    uint32_t      Tpwm;             /* PWM周期计数值 */
    bool          State;             /* FOC运行状态 (true=运行, false=停止) */
} __attribute__((aligned(4))) FOC_State_t;

extern FOC_State_t FOC_State;

/*==============================================================================
 * 函数声明
 *============================================================================*/
void FOC_Init(void);
void FOC_Start(void);
void FOC_Stop(void);

#endif /* _FOC_SVPWM_H */
