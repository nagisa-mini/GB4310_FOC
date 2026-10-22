#ifndef SYSTEM_H
#define SYSTEM_H

#include "FOC_SVPWM.h"
#include "MP6539.h"
#include "ADC_Sample.h"
#include "MT6826S.h"
#include "PID.h"

/*==============================================================================
 * 系统初始化
 *
 *   把 main.c 中 BSP 各模块的启动顺序收拢到 System_Init(), 与 CubeMX 生成的
 *   MX_xxx_Init() 外设初始化分开, main.c 只保留外设初始化 + 入口调用。
 *============================================================================*/

/**
 * @brief  初始化并启动全部 BSP 模块
 * @note   须在 CubeMX 生成的 MX_xxx_Init() 全部完成之后调用;
 *         顺序为 FOC → 栅极驱动 → ADC 采样 → 编码器 → 三环 PID,
 *         PID 增益随后从 Flash 加载, 无有效记录时保留默认值。
 */
void System_Init(void);

#endif /* SYSTEM_H */