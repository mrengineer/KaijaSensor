/**
  ******************************************************************************
  * File Name          : power.c
  * Description        : Power management and battery control
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "power.h"
#include <stdio.h>

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

#define ADC_RESOLUTION 4096
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;

unsigned int adc_power;
unsigned char shunt;
unsigned int shunt_devider;
unsigned int gain = 100; //for MAX9938H, see datasheet

void power_read(void){
//	UC_2_8V;			    //max
	adc_power = 2900; //2.9V adc power supply
	
	shunt     = 68;  //150 Ohm
	shunt_devider = 1000;
	
	uint32_t adcResult = 0;
	unsigned long I, V = 0;
	
		//DISCHARGE;
		HAL_Delay(1);
		CHARGE;
    HAL_Delay(10);
		
		HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 50);
    adcResult = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

	/*
	Uizm = I/R * 100 = gain*I/shunt
	Uizm = adc_power * adcResult / ADC_RESOLUTION
	*/

	// Voltage on ADC pin
	V = (adcResult * adc_power)/ADC_RESOLUTION;
	
	// Current calculation
	I = (V * shunt_devider) /(gain * shunt);	//mA
	
		printf("ADC: %i=>%imV => %imA\r\n", adcResult, V, I);

	// init gpio before!


	//ENABLE_2_5V;	//DC-DC enable

}

/*
MCU in Low Power Mode sleep: 
32.5 kHz clock. active: GPIOx,  SYSTICK, RTC, WDT
WIFI OFF, SD card off, RF/BLE OFF, LEDs OFF
ACC - as was.
Wakeup on timer or IRQ 
Magnet sensors are off, checked on timer systick.
Shunt: 150 OHm
*/
void lowest_power (void){
	//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	    /* Disable all used wakeup sources: WKUP pin */
 //   HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	//	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
    
    /* Clear all related wakeup flags */
    /* Clear PWR wake up Flag */
    //__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
    /* Enable WKUP pin */
  //  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
  //  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
	
    /* Request to enter STANDBY mode */
		printf("lowest_power\r\n");
    HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
	
}
