#include <stm32f469xx.h>
#include "stm32f4xx.h"

void SysClockConfig (void);

void activateFPU(void);

void GPIO_Config (void);

void InterruptGPIO_Config (void);

void ITM_Init(void);
void SWV_SendString(const char *str);
