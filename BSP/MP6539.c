#include "MP6539.h"

void MP6539_Init(void)
{
    HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, GPIO_PIN_SET);
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
}