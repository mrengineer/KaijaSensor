/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>


// Set uC DC-DC module power ---------------------------------------------------------------------------------
#define UC_1_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_1_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET);	 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_RESET)
#define UC_2_8V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_RESET); 	HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)
#define UC_2_9V HAL_GPIO_WritePin(PWR_TO2_8AND2_9V_GPIO_Port, PWR_TO2_8AND2_9V_Pin, GPIO_PIN_SET); 		HAL_GPIO_WritePin(PWR_TO_2_8V_GPIO_Port, PWR_TO_2_8V_Pin, GPIO_PIN_SET)

// Power DC-DC		--------------------------------------------------------------------------------------------
#define ENABLE_2_5V					HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_SET)
#define DISABLE_2_5V				HAL_GPIO_WritePin(ENABLE_2_5V_GPIO_Port, ENABLE_2_5V_Pin, GPIO_PIN_RESET)

// Magnetic sensors		----------------------------------------------------------------------------------------
//Power management
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

const char wtext[] = "Hello World!";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten, bytesread; /* File write/read counts */
	char rtext[256]; /* File read buffer */
	
	//	HALL_SENS_PWR_ON;
	//	CLAMP_SENS_PWR_ON;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
 // MX_ADC_Init();
 // MX_SPI1_Init();
 // MX_SPI2_Init();
 // MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
	
	
	/*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit)
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  huart1.Instance        = USART1;

  huart1.Init.BaudRate   = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits   = UART_STOPBITS_1;
  huart1.Init.Parity     = UART_PARITY_ODD;
  huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  huart1.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
		while (1) {}
    /* Initialization Error */
    //Error_Handler();
  }

	
	
	//HALL_SENS_PWR_ON;
	//CLAMP_SENS_PWR_OFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	  /* USER CODE BEGIN 2 */
	//HALL_SENS_PWR_ON;
	//CLAMP_SENS_PWR_ON;
  /* USER CODE END 2 */
	
	// UC power change - fail
	// SENS on-of-sensing - OK @ 2.9V
	// LEDS ok @ 2.9V
	// power DC/DC domain 2.5V OK @ uC at 2.9V
	// D/C 1.8..2.9V ok
	
	// init gpio before!
	UC_2_8V;			//minimal power
	
	
goto skp;
	
	//ENABLE_2_5V;	//DC-DC enable
	SD_PWR_ON;		//Power to SD card
	//INIT SD and CARD after because no power to sd
	
	HAL_Delay(50);
  MX_SDIO_SD_Init();

  MX_FATFS_Init();

/*##-1- FatFS: Link the SD disk I/O driver ##########*/
 
 if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0){
		/* success: set the orange LED on */
		 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
		/*##-2- Register the file system object to the FatFs module 
	 
		NB! mout right now!
	 ###*/
		res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 1) ;
	 
		 if(res != FR_OK){
				 /* FatFs Initialization Error : set the red LED on */
				 IND1_ON;
				 while(1);
		 } else {
				/*##-3- Create a FAT file system (format) on the logical drive#*/
				 /* WARNING: Formatting the uSD card will delete all content on the device */
					res = f_mkfs((TCHAR const*)SD_Path, 0, 0);
					if(res != FR_OK){
						 /* FatFs Format Error : set the red LED on */
				//		 ORANGE_ON;
						IND1_ON;
						 while(1);
					} else {
						/*##-4- Create & Open a new text file object with write access#*/
						 if(f_open(&MyFile, "Hello.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
								 /* 'Hello.txt' file Open for write Error : set the red LED on */
								 IND2_ON;
								 while(1);
						 } else {
								 /*##-5- Write data to the text file ####################*/
								 res = f_write(&MyFile, wtext, sizeof(wtext), (void*)&byteswritten);
									 
								 if((byteswritten == 0) || (res != FR_OK)){
										 /* 'Hello.txt' file Write or EOF Error : set the red LED on */
										 IND3_ON;
										 while(1);
								 } else {
									 
										 /*##-6- Successful open/write : set the blue LED on */
										// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
										 f_close(&MyFile);
									 
										 /*##-7- Open the text file object with read access #*/
										 if(f_open(&MyFile, "Hello.txt", FA_READ) != FR_OK){
												 /* 'Hello.txt' file Open for read Error : set the red LED on */
												//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
												 while(1);
										 } else {
												 /*##-8- Read data from the text file #########*/
												 res = f_read(&MyFile, rtext, sizeof(wtext), &bytesread);
												 if((strcmp(rtext,wtext)!=0)|| (res != FR_OK)){
													 /* 'Hello.txt' file Read or EOF Error : set the red LED on */
													 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
													 while(1);
												 } else {
													 /* Successful read : set the green LED On */
													 //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
													 /*##-9- Close the open text file ################*/
													 f_close(&MyFile);
												 }
										 }
								 }
						 }
				 }
		 }
 }
 /*##-10- Unlink the micro SD disk I/O driver #########*/
 FATFS_UnLinkDriver(SD_Path);

 IND4_ON;
 
 skp:
	while (1){
			HAL_Delay(500);
			printf("Main\r\n");
			//HAL_UART_Transmit_IT(&huart1, "A", 1);
		
//		if (IS_SENS_CLAMP_A_ON) IND3_ON; else IND3_OFF;
//		if (IS_SENS_CLAMP_B_ON)	IND2_ON; else IND2_OFF;
//		if (IS_SENS_OPEN_ON) 		IND1_ON; else IND1_OFF;
//		if (IS_SENS_TAKEOFF_ON) IND4_ON; else IND4_OFF;

		//HAL_Delay(1000);
		
		//		printf("TEST\r\n");
		
			//HAL_GPIO_TogglePin(STATUS_GPIO_Port, STATUS_Pin);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  HAL_ADC_Init(&hadc);

    /**Configure the channel speed in Low mode 
    */
  __HAL_ADC_CHANNEL_SPEED_SLOW(ADC_CHANNEL_8);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 100;

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nDISCHARGE_Pin|PWR_TO_2_8V_Pin|ENABLE_2_5V_Pin|INDICATOR2_Pin 
                          |WIFI_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_Pin|HALL_CLAMP_PWR_Pin|RF_PWR_Pin|INDICATOR3_Pin 
                          |SD_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHUNT_0_06_DISABLE_Pin|HALL_SENS_PWR_Pin|RF_CE_Pin|PWR_TO2_8AND2_9V_Pin 
                          |INDICATOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nDISCHARGE_Pin PWR_TO_2_8V_Pin ENABLE_2_5V_Pin INDICATOR2_Pin 
                           WIFI_PWR_Pin */
  GPIO_InitStruct.Pin = nDISCHARGE_Pin|PWR_TO_2_8V_Pin|ENABLE_2_5V_Pin|INDICATOR2_Pin 
                          |WIFI_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_Pin HALL_CLAMP_PWR_Pin RF_PWR_Pin INDICATOR3_Pin 
                           SD_PWR_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin|HALL_CLAMP_PWR_Pin|RF_PWR_Pin|INDICATOR3_Pin 
                          |SD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_CLAMP_A_Pin */
  GPIO_InitStruct.Pin = SENS_CLAMP_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENS_CLAMP_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_TAKEOFF_Pin */
  GPIO_InitStruct.Pin = SENS_TAKEOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENS_TAKEOFF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUNT_0_06_DISABLE_Pin HALL_SENS_PWR_Pin RF_CE_Pin PWR_TO2_8AND2_9V_Pin 
                           INDICATOR1_Pin */
  GPIO_InitStruct.Pin = SHUNT_0_06_DISABLE_Pin|HALL_SENS_PWR_Pin|RF_CE_Pin|PWR_TO2_8AND2_9V_Pin 
                          |INDICATOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_CLAMP_B_Pin */
  GPIO_InitStruct.Pin = SENS_CLAMP_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SENS_CLAMP_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_IRQ_Pin */
  GPIO_InitStruct.Pin = RF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENS_OPEN_Pin */
  GPIO_InitStruct.Pin = SENS_OPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENS_OPEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
