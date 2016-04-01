#ifndef __DEFINES__H
#define __DEFINES__H

#define   SetBit(reg, bit)          reg |= (1<<bit)            
#define   ClearBit(reg, bit)       reg &= (~(1<<bit))
#define   InvBit(reg, bit)          reg ^= (1<<bit)
#define   BitIsSet(reg, bit)       ((reg & (1<<bit)) != 0)
#define   BitIsClear(reg, bit)    ((reg & (1<<bit)) == 0)

//Acc
#define ACC_ENABLE					HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET)
#define ACC_DISABLE					HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET)

// Magnetic sensors		----------------------------------------------------------------------------------------
// Power management
#define CLAMP_SENS_PWR_ON		HAL_GPIO_WritePin(HALL_CLAMP_PWR_GPIO_Port, HALL_CLAMP_PWR_Pin, GPIO_PIN_SET)
#define CLAMP_SENS_PWR_OFF	HAL_GPIO_WritePin(HALL_CLAMP_PWR_GPIO_Port, HALL_CLAMP_PWR_Pin, GPIO_PIN_RESET)
#define HALL_SENS_PWR_ON		HAL_GPIO_WritePin(HALL_SENS_PWR_GPIO_Port, HALL_SENS_PWR_Pin, GPIO_PIN_SET)
#define HALL_SENS_PWR_OFF		HAL_GPIO_WritePin(HALL_SENS_PWR_GPIO_Port, HALL_SENS_PWR_Pin, GPIO_PIN_RESET)

//Read sensors
#define IS_SENS_CLAMP_A_ON 	HAL_GPIO_ReadPin(SENS_CLAMP_A_GPIO_Port, SENS_CLAMP_A_Pin) 	== GPIO_PIN_RESET				//Sensor gives + when no magnetic field
#define IS_SENS_CLAMP_B_ON 	HAL_GPIO_ReadPin(SENS_CLAMP_B_GPIO_Port, SENS_CLAMP_B_Pin) 	== GPIO_PIN_RESET				//Sensor gives + when no magnetic field
#define IS_SENS_OPEN_ON		 	HAL_GPIO_ReadPin(SENS_OPEN_GPIO_Port, 		SENS_OPEN_Pin) 		== GPIO_PIN_RESET				//Sensor gives + when no magnetic field
#define IS_SENS_TAKEOFF_ON	HAL_GPIO_ReadPin(SENS_TAKEOFF_GPIO_Port, 	SENS_TAKEOFF_Pin) == GPIO_PIN_RESET				//Sensor gives + when no magnetic field

//Indictaors		---------------------------------------------------------------------------------------------
#define IND1_ON							HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_SET)
#define IND1_OFF						HAL_GPIO_WritePin(INDICATOR1_GPIO_Port, INDICATOR1_Pin, GPIO_PIN_RESET)

#define IND2_ON							HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_SET)
#define IND2_OFF						HAL_GPIO_WritePin(INDICATOR2_GPIO_Port, INDICATOR2_Pin, GPIO_PIN_RESET)

#define IND3_ON							HAL_GPIO_WritePin(INDICATOR3_GPIO_Port, INDICATOR3_Pin, GPIO_PIN_SET)   
#define IND3_OFF						HAL_GPIO_WritePin(INDICATOR3_GPIO_Port, INDICATOR3_Pin, GPIO_PIN_RESET)

#define IND4_ON							HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_SET)  
#define IND4_OFF						HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET)

//SD card	Do not forget enable 2.5 V for power supply and switch uC to higher voltage in order to meet IO signals of memory card!
#define SD_PWR_ON						HAL_GPIO_WritePin(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_RESET)
#define SD_PWR_OFF					HAL_GPIO_WritePin(SD_PWR_GPIO_Port, SD_PWR_Pin, GPIO_PIN_SET)

#endif /* __DEFINES_H */
