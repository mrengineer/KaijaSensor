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
/*#define UC_1_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_1_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET);	 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_2_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)
#define UC_2_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET); 		HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)

// Power DC-DC		--------------------------------------------------------------------------------------------
#define ENABLE_2_5V					HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_SET)
#define DISABLE_2_5V				HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_RESET)*/

// Power measurment condensator discharge control
//nDISCHARGE_Pin
#define CHARGE		HAL_GPIO_WritePin(nDISCHARGE_GPIO_Port, nDISCHARGE_Pin, GPIO_PIN_SET);  	 //Close discharge transistor
#define DISCHARGE	HAL_GPIO_WritePin(nDISCHARGE_GPIO_Port, nDISCHARGE_Pin, GPIO_PIN_RESET);   //OPEN discharge transistor

#define ADC_RESOLUTION 4096
/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;

unsigned int adc_power;
unsigned int shunt;
unsigned int gain = 100; //for MAX9938H, see datasheet

void power_read(void){
//	UC_2_8V;			    //max
	adc_power = 2800; //2.8V adc power supply
	shunt     = 150;  //150 Ohm
	
	uint32_t adcResult = 0;
	unsigned int I = 0;
	
	
		DISCHARGE;
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
	
	I = (adcResult * adc_power)/ADC_RESOLUTION;
	
	//I = adc_power * shunt;
	//I = I / gain;
	//I = I * adcResult;
	//I = I / ADC_RESOLUTION;
	
		printf("ADC: %i=>%i\r\n", adcResult, I);

	// init gpio before!


	//ENABLE_2_5V;	//DC-DC enable

}
