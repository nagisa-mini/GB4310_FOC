#ifndef MP6539_H
#define MP6539_H

#include "gpio.h"
#include "dac.h"

extern float MT6826S_Angle;
extern uint8_t MT6826S_Status;

void MP6539_Init(void);

#endif