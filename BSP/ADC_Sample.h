#ifndef ADC_SAMPLE_H
#define ADC_SAMPLE_H

#include "adc.h"
#include "tim.h"
#include "dac.h"
#include "System.h"

/*==============================================================================
 * 硬件参数
 *============================================================================*/
#define MAX_Phase_Current    1.9f      /* 相电流量程 (A) */
#define ZERO_CALIBRATE_SIZE  250       /* 零点校准采样次数 */
#define VDDA_CAL_VOLTAGE     3.0f      /* VREFINT 校准条件: VDDA = 3.0V */
#define ADC_FULL_SCALE       4095.0f   /* 12 位 ADC 满量程码值 */
#define VBUS_DIVIDER         11.0f     /* 母线分压比 (R上 + R下) / R下 */
#define CURR_AMP_GAIN        20.0f     /* 电流采样运放增益 */
#define CURR_SHUNT           0.05f     /* 采样电阻 (Ω) */
#define INJ_TRIG_CC4         4245U     /* TIM1 CC4 比较值: 载波峰值处触发注入组 */

/*==============================================================================
 * 采样缓冲区索引 (注入组与规则组共用同一数组)
 *   [0] Ia  [1] Ic        —— 注入组, TIM1 CC4 触发
 *   [2] Vbus  [3] VREFINT ——  规则组, TIM7 TRGO + DMA
 *============================================================================*/
#define ADC_BUF_IA           0U
#define ADC_BUF_IC           1U
#define ADC_BUF_VBUS         2U
#define ADC_BUF_VREFINT      3U
#define ADC_BUF_SIZE         4U

/* ADC_GetData() 通道选择位 */
#define ADC_CMD_IA           0x01
#define ADC_CMD_IB           0x02
#define ADC_CMD_IC           0x04
#define ADC_CMD_VBUS         0x08
#define ADC_CMD_VUNIT        0x10

/*==============================================================================
 * 类型定义
 *============================================================================*/
/* 采样状态: 电流单位 A, 电压单位 V, Vunit 为单个 ADC 码对应的电压 */
typedef struct {
    uint8_t MODE;       /* 运行模式: 0=空闲, 1=零点校准, 2=正常采样 */
    float   Ia;         /* A 相电流 */
    float   Ib;         /* B 相电流 */
    float   Ic;         /* C 相电流 */
    float   Vbus;       /* 母线电压 (1ms 更新) */
    float   Vunit;      /* 单位 ADC 码电压, 由 VREFINT 实测反推 */
} ADC_State_t;

/*==============================================================================
 * 函数声明
 *============================================================================*/
void  ADC_Sample_Init(void);                        /* 初始化 ADC 各采样通道 */
void  ADC_CalibrateZero(void);                      /* 电流零点校准 (阻塞) */
void  ADC_Sample_Start(void);                       /* 开始采样 */
void  ADC_Sample_Stop(void);                        /* 停止采样 */
float ADC_GetData(uint8_t cmd);                     /* 读取单通道数据, cmd 见 ADC_CMD_xxx */
void  ADC_GetCurrents(float *Ia, float *Ib, float *Ic); /* 一次临界区内读取三相电流, 保证同帧一致 */

#endif /* ADC_SAMPLE_H */
