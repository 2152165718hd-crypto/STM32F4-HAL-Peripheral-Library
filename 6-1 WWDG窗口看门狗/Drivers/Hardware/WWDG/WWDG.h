#ifndef __WWDG_H
#define __WWDG_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#define PCLK1_FREQ   42000000

void WWDG_Init(uint16_t timeout_us, uint8_t rate);

#endif
