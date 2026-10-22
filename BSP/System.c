#include "System.h"

/*==============================================================================
 * 系统初始化
 *============================================================================*/

/**
 * @brief  初始化并启动全部 BSP 模块
 * @note   顺序与旧 main.c 的 USER CODE 2 一致:
 *         FOC_Init() 先配置 CORDIC 并清零状态, ADC/MT6826S 共用 TIM1 CC4
 *         注入触发, 最后启动三环 PID、加载 Flash 增益并使能 PWM 输出。
 */
void System_Init(void)
{
    FOC_Init();
    MP6539_Init();
    ADC_Sample_Init();
    ADC_Sample_Start();
    MT6826S_Init();
    MT6826S_Start();
    PID_3Loop_Init();
    PID_Config_Load();      /* 无有效记录时保留默认增益 */
    FOC_Start();
}