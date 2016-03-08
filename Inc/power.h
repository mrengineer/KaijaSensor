/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
	#include "stm32l1xx_hal.h"
	#include "stm32l1xx_hal_rcc.h"
	#include "mxconstants.h"
	#include "stm32l1xx_hal_adc.h"
	#include "stm32l1xx_hal_adc_ex.h"

/* Exported constants --------------------------------------------------------*/ 
void lowest_power(void);

/* Exported functions --------------------------------------------------------*/   
   
#ifdef __cplusplus
}
#endif

#endif /* __POWER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
