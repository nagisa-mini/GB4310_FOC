#ifndef MP6539_H
#define MP6539_H

#include "gpio.h"
#include "dac.h"
#include "usbd_cdc_if.h"
#include "Ringbuffer.h"
#include "FOC_SVPWM.h"

/*==============================================================================
 * MP6539 三相栅极驱动: 使能控制与 DAC 输出
 *============================================================================*/

void MP6539_Init(void);     /* 释放 nSLEEP 使能, 启动 DAC 通道 1 */
void MP6539_Restart(void);  /* 重启 MP6539 */

#endif /* MP6539_H */