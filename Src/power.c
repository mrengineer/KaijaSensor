/**
  ******************************************************************************
  * File Name          : power.c
  * Description        : Power management and battery control
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "power.h"
#include <stdio.h>


#define ADC_RESOLUTION 4096
#define gain 100 								//for MAX9938H, see datasheet

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;

unsigned int adc_power;
unsigned char shunt;
unsigned int shunt_devider;


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
	
	    /* Disable all used wakeup sources: WKUP pin */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
    
    /* Clear all related wakeup flags */
    /* Clear PWR wake up Flag */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    
    /* Enable WKUP pin */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2);
	
	
		/* Enable Ultra low power mode */
		HAL_PWREx_EnableUltraLowPower();

		/* Enable the fast wake up from Ultra low power mode */
		HAL_PWREx_EnableFastWakeUp();
		
		HAL_SuspendTick();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
