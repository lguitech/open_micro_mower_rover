
/*********************************************************************
 *
 *  This file is part of the [OPEN_MICRO_MOWER_ROVER] project.
 *  Licensed under the MIT License for non-commercial purposes.
 *  Author: Brook Li
 *  Email: lguitech@126.com
 *
 *  For more details, refer to the LICENSE file or contact [lguitech@126.com].
 *
 *  Commercial use requires a separate license.
 *
 *  This software is provided "as is", without warranty of any kind.
 *
 *********************************************************************/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define PPS_Pin GPIO_PIN_10
#define PPS_GPIO_Port GPIOE
#define RadioConfig_Pin GPIO_PIN_13
#define RadioConfig_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOC


#ifdef __cplusplus
}
#endif

#endif
