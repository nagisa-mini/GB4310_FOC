#include "MP6539.h"

/**
 * @brief  初始化 MP6539: 拉高 nSLEEP 使能驱动, 并启动 DAC 通道 1
 */
void MP6539_Init(void)
{
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}

/**
 * @brief  重启 MP6539
 */
void MP6539_Restart(void)
{
    FOC_Stop();
    HAL_Delay(20);
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_RESET);
    HAL_Delay(20);
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
    FOC_Start();
}

/**
 * @brief  GPIO 外部中断回调函数
 * @note   处理 MP6539 故障中断
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == nFAULT_Pin)
    {
        /* MP6539 故障中断: 立即停止 FOC */
        FOC_Stop();
        const char *fault_msg = "达到阈值电流!\r\n";
        if (RingBuffer_GetFree(&USB_TxBuffer) > strlen(fault_msg))
        {
            RingBuffer_Write(&USB_TxBuffer, (const uint8_t *)fault_msg, strlen(fault_msg));
        }
    }
}