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

	// Set uC DC-DC module power ---------------------------------------------------------------------------------
#define UC_1_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_1_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET);	 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_2_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)
#define UC_2_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET); 		HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)

// Power DC-DC		--------------------------------------------------------------------------------------------
#define ENABLE_2_5V					HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_SET)
#define DISABLE_2_5V				HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_RESET)

// Power measurment condensator discharge control
//nDISCHARGE_Pin
#define CHARGE		HAL_GPIO_WritePin(nDISCHARGE_GPIO_Port, nDISCHARGE_Pin, GPIO_PIN_SET);  	 //Close discharge transistor
#define DISCHARGE	HAL_GPIO_WritePin(nDISCHARGE_GPIO_Port, nDISCHARGE_Pin, GPIO_PIN_RESET);   //OPEN discharge transistor
 
	 
	 
/* Exported constants --------------------------------------------------------*/ 
void lowest_power(void);

/* Exported functions --------------------------------------------------------*/   
   
#ifdef __cplusplus
}
#endif

#endif /* __POWER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
