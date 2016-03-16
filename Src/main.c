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
#include "lis3dh_driver.h"
#include "power.h"

//Acc
#define ACC_ENABLE					HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET)
#define ACC_DISABLE					HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET)

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

RTC_HandleTypeDef hrtc;

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

uint8_t resp;

/* Size of buffer */
#define BUFFERSIZE                       (COUNTOF(aTxBuffer) - 1)

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
	
/* Exported functions ------------------------------------------------------- */


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
static void MX_RTC_Init(void);
static void EXTI0_IRQHandler_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/*int fputc(int ch, FILE *f){
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}*/

// FOR DEBUG send to  KEIL
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {

while (ITM_Port32(0) == 0);
    ITM_Port8(0) = ch;
  }
  return(ch);
}	

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

	
  AxesRaw_t data;
	
	FRESULT res; /* FatFs function common result code */
	uint32_t byteswritten, bytesread; /* File write/read counts */
	char rtext[256]; /* File read buffer */
	
	//	HALL_SENS_PWR_ON;
	//	CLAMP_SENS_PWR_ON;
	
	//	1. SPI1 is for ACC 
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  //MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  //MX_FATFS_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  EXTI0_IRQHandler_Config();
		
	HAL_DBGMCU_EnableDBGStopMode();


	//HALL_SENS_PWR_ON;
	//CLAMP_SENS_PWR_OFF;
	
	
	// init gpio before!
	UC_2_8V;
	
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
				 printf ("Problem fmount\r\n");
				 while(1);
		 } else {
				/*##-3- Create a FAT file system (format) on the logical drive#*/
				 /* WARNING: Formatting the uSD card will delete all content on the device */
					res = f_mkfs((TCHAR const*)SD_Path, 0, 0);
					if(res != FR_OK){
						 /* FatFs Format Error : set the red LED on */
							printf ("Problem f_mkfs\r\n");
						 while(1);
					} else {
						/*##-4- Create & Open a new text file object with write access#*/
						 if(f_open(&MyFile, "Hello.txt", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK){
								 /* 'Hello.txt' file Open for write Error : set the red LED on */
								 printf ("Problem f_open\r\n");
								 while(1);
						 } else {
								 /*##-5- Write data to the text file ####################*/
								 res = f_write(&MyFile, wtext, sizeof(wtext), (void*)&byteswritten);
									 
								 if((byteswritten == 0) || (res != FR_OK)){
										 /* 'Hello.txt' file Write or EOF Error : set the red LED on */
										 printf ("Problem f_write\r\n");
										 while(1);
								 } else {
									 
										 /*##-6- Successful open/write : set the blue LED on */
										// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
										 f_close(&MyFile);
									 
										 /*##-7- Open the text file object with read access #*/
										 if(f_open(&MyFile, "Hello.txt", FA_READ) != FR_OK){
												 /* 'Hello.txt' file Open for read Error : set the red LED on */
												//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
													printf ("Problem f_open\r\n");
												 while(1);
										 } else {
												 /*##-8- Read data from the text file #########*/
												 res = f_read(&MyFile, rtext, sizeof(wtext), &bytesread);
												 if((strcmp(rtext,wtext)!=0)|| (res != FR_OK)){
													 /* 'Hello.txt' file Read or EOF Error : set the red LED on */
													 				 printf ("Problem f_read\r\n");
													 while(1);
												 } else {
													 printf ("FAT operation done OK!\r\n");
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

 skp:
 
	printf("FW start\r\n");

	LIS3DH_GetWHO_AM_I(&resp);
	printf("LIS3DH_GetWHO_AM_I=%i\r\n", resp);	

	
 
HAL_Delay(1);
  

  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);

LIS3DH_FIFOModeEnable(LIS3DH_FIFO_MODE);	//Enable store into FIFO
LIS3DH_SetInt1Duration(64);   									// len of irq pulse in n/odr
//LIS3DH_SetWaterMark(10); 												// watermark for irq generation from fifo 

//Direct IRQ from watemark and overrun to 1st pin. 
LIS3DH_SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_DISABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE | 
									LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE	| 
									LIS3DH_I1_DRDY2_ON_INT1_ENABLE | LIS3DH_WTM_ON_INT1_DISABLE | LIS3DH_INT1_OVERRUN_DISABLE);

	LIS3DH_SetODR(LIS3DH_ODR_10Hz);
  LIS3DH_SetMode(LIS3DH_NORMAL);
  LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
  LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE );

while (1) {
	LIS3DH_GetFifoSourceFSS(&resp);
	printf("%i recs in FIFO\r\n", resp);
	
	if (resp > 25) {
		IND2_ON;
		rep:
			LIS3DH_GetAccAxesRaw(&data);		
			LIS3DH_GetFifoSourceFSS(&resp);
			printf("> %i recs in FIFO (while)\r\n", resp);
			HAL_Delay(5);
		if (resp != 0) goto rep;
		IND2_OFF;
	}
	/*	if(LIS3DH_GetAccAxesRaw(&data)==1){
			printf("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z); 
		} else {
			printf("ER\r\n");
		}*/
		
	HAL_Delay(5);
}
//ENABLE ALL IRQs
//LIS3DH_SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_DISABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE | LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE | LIS3DH_I1_DRDY2_ON_INT1_ENABLE | LIS3DH_WTM_ON_INT1_ENABLE | LIS3DH_INT1_OVERRUN_ENABLE);
//LIS3DH_SetInt2Pin(LIS3DH_CLICK_ON_PIN_INT2_DISABLE | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE | LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE | LIS3DH_INT_ACTIVE_HIGH);
//	HAL_Delay(2000);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if(LIS3DH_GetAccAxesRaw(&data)==1){
			printf("X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z); 
		} else {
			printf("ER\r\n");
		}
		HAL_Delay(50);

		//	lowest_power();		//go to stop. Wakeup on RTC wakeup or 1st or 2d PIN wakeup
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

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

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 0x12;
  sDate.Year = 0x16;

  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BCD);

    /**Enable the WakeUp 
    */
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

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
  hsd.Init.ClockDiv = 20;

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
  hspi1.Init.NSS = SPI_NSS_SOFT;
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
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /*Configure GPIO pin : ACC_INT2_Pin */
  GPIO_InitStruct.Pin = ACC_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nDISCHARGE_Pin */
  GPIO_InitStruct.Pin = nDISCHARGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(nDISCHARGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_TO_2_8V_Pin ENABLE_2_5V_Pin INDICATOR2_Pin WIFI_PWR_Pin */
  GPIO_InitStruct.Pin = PWR_TO_2_8V_Pin|ENABLE_2_5V_Pin|INDICATOR2_Pin|WIFI_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ACC_INT1_Pin */
  GPIO_InitStruct.Pin = ACC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ACC_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_Pin ACC_CS_Pin HALL_CLAMP_PWR_Pin INDICATOR3_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin|ACC_CS_Pin|HALL_CLAMP_PWR_Pin|INDICATOR3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SENS_CLAMP_A_Pin SENS_TAKEOFF_Pin */
  GPIO_InitStruct.Pin = SENS_CLAMP_A_Pin|SENS_TAKEOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SHUNT_0_06_DISABLE_Pin RF_CE_Pin */
  GPIO_InitStruct.Pin = SHUNT_0_06_DISABLE_Pin|RF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_SENS_PWR_Pin */
  GPIO_InitStruct.Pin = HALL_SENS_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(HALL_SENS_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RF_PWR_Pin SD_PWR_Pin */
  GPIO_InitStruct.Pin = RF_PWR_Pin|SD_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_TO2_8AND2_9V_Pin INDICATOR1_Pin */
  GPIO_InitStruct.Pin = PWR_TO2_8AND2_9V_Pin|INDICATOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_VERY_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SENS_CLAMP_B_Pin RF_IRQ_Pin SENS_OPEN_Pin */
  GPIO_InitStruct.Pin = SENS_CLAMP_B_Pin|RF_IRQ_Pin|SENS_OPEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nDISCHARGE_Pin|WIFI_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR_TO_2_8V_Pin|ENABLE_2_5V_Pin|INDICATOR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, STATUS_Pin|ACC_CS_Pin|HALL_CLAMP_PWR_Pin|INDICATOR3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SHUNT_0_06_DISABLE_Pin|HALL_SENS_PWR_Pin|RF_CE_Pin|PWR_TO2_8AND2_9V_Pin 
                          |INDICATOR1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RF_PWR_Pin|SD_PWR_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
static void EXTI0_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIOC clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure PA.0 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the lowest priority */
  HAL_NVIC_SetPriority(EXTI0_IRQn , 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn );
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_0)
  {
    /* Toggle LED2 */
		printf("HELLO FROM CLBK\r\n");
  }
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
