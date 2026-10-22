#include "MP6539.h"

/**
 * @brief  初始化 MP6539: 拉高 nSLEEP 使能驱动, 并启动 DAC 通道 1
 */
void MP6539_Init(void)
{
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}