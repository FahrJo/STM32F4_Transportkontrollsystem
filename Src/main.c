
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "card_operations.h"
#include "acceleration_Sensor.h"
#include "gps_modul.h"
#include "ini.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c3;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t GET_DATA;														 		/* Initialisierungswert fuer das Sammeln eines kompletten Datensatzes */

configuration config;

static int Preferences_Handler(void* user, const char* section, const char* name, const char* value);		/* Handler für Config-Parser */

const uint16_t 	datasetCount = 100;							/* Anzahl der Datensäetze, die gesammelt auf die Karte geschrieben werden */
uint16_t				actualSet = 0;									/* Momentan aktiver Datensatz */
ADC_AnalogWDGConfTypeDef AnalogWDGConf;

struct tm				clock_time;

FATFS 					myFatFS;
FIL 						logFile;												/* Dateihandle auf SD-Karte */
FIL 						configFile;											/* Dateihandle auf SD-Karte */
UINT 						cursor;													/* Letztes Zeichen in CSV-Datei */
UINT 						configCursor;										/* Letztes Zeichen in CSV-Datei */
char 						logFileName[] = "Log3.csv";
char 						configFileName[] = "config.ini";
char * 					configBuffer;
char 						header[] = "Tracking-Log vom 17.01.2019;;;;;;\n Date/Time;Location;Acceleration X; Acceleration Y; Acceleration Z;Temp;Open;Note\n";
uint32_t 				Temp_Raw;												/* Temperatur in 12 Bit aus ADC */
uint16_t 				Temp;														/* Temperatur in �C */
dataset 				sensor_set[datasetCount];				/* Datensatz, der im RAM gepuffert wird */
workmode_type 	operation_mode = workmode_log;	/* Betriebsmodus (Energiespar-Funktion) */
event_type 			event;													/* Event f�r die Detektierung einer Grenzwert�berschreitung */

s_accelerometerValues 			acceleration_actual_global;
s_accelerometerValuesFloat 	acceleration_actual_float;

uint8_t					g_newGPSData = 0;								/* Flag wenn neue Daten im Buffer anstehen wird von DMA IR-Handler gesetzt */
s_gpsSetOfData  gpsActualDataset;								/* Set mit aktuellsten GPS Daten */

/* Trigger-Flags f�r das Einlesen von Daten */
uint8_t 				getDataset;
uint8_t 				getTemp;
uint8_t					getLight;
uint8_t					getAcceleration;
uint8_t					getPosition;
uint8_t					writeDataset;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Timer4_Init(void);
void AnalogWDG_Init(void);
void error_blink(uint16_t GPIO_Pin, uint16_t n, uint16_t dt);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	/* Standartwerte */
	config.MAX_TEMP 						= 333;
	config.MIN_TEMP 						= 263;
	config.MAX_ACC 							= 4;
	config.ACCELERATION_ENABLE 	= 1;
	config.GNSS_ENABLE					= 1; 
	config.LIGHT_ENABLE					= 1;
	config.TEMP_ENABLE					= 1;
	config.ACC_MAX_ANZAHL_WERTE	= 20;
	
	char gpsRxRingBuffer[GPS_RINGBUFFER_SIZE] = {0};
	
	s_accelerometerValues acceleration_actual;
	//s_accelerometerValues acceleration_ringbuffer[ACC_MAX_ANZAHL_WERTE];
	HAL_StatusTypeDef i2c_status = HAL_BUSY;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_FATFS_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/* Vorbereiten der SD-Karte ------------------------------------------------*/
	if(f_mount(&myFatFS, SDPath, 1) == FR_OK){
		if(f_open(&configFile, configFileName, FA_READ | FA_OPEN_EXISTING) == FR_OK){
			configBuffer = calloc((f_size(&configFile) + 2), sizeof(char));		/* Reserviert Speicherbereich für den String der ganzen Konfigurationsdatei */ 
			f_read(&configFile, configBuffer, sizeof(configBuffer), &configCursor);		/* liest Konfigurationsdatei in den String für die folgende Auswertung */
			if (ini_parse_string(configBuffer, Preferences_Handler, &config) < 0) {		/* wertet den String aus und setzt die entsprechenden Parameter */
        error_blink(LED5_Pin, 5, 100);					/* wenn Lesefehler dann blinkt LED5 3 mal schnell */
			}
			free(configBuffer);												/* Gibt dynamisch reservierten Speicherbereich wieder frei */
		}
		write_string_to_file(&logFile, logFileName, header,	sizeof(header), &cursor);
	}
	else{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);		/* Orangene LED leuchtet dauerhaft, wenn SD-Karte  nicht initialisiert werden konnte */
	}

	Timer4_Init();
	
		
	/* Acc-Sensor ueber I2C deaktivieren und dann wieder aktivieren, damit dieser sicher aktiviert wird */
	if(config.ACCELERATION_ENABLE){
		HAL_Delay(500);
		ACC_deactivate(&hi2c3);
		HAL_Delay(500);
		ACC_activate(&hi2c3);
		HAL_Delay(500);
	}
	
	if(config.GNSS_ENABLE) {
		config.GNSS_ENABLE = 0; 										/* erst mal deaktiveren, wenn GPS aktivierung erfolgreich wird es wieder ENABLED */
		HAL_GPIO_WritePin(GPS_Reset_out_GPIO_Port, GPS_Reset_out_Pin, GPIO_PIN_SET);
		for(int i = 0; i < 4; i++){									/* wenn nach 4 Versuchen, das Modul nicht aktivert werden kann, bleibt GNSS_ENABLE=0 und das Programm lauft "normal" weiter */
				if(-1 != GPS_activateReceiver()){ 			/* GPS erfolgreich aktiviert, for-Schleife verlassen */
				config.GNSS_ENABLE=1;
				i = 5;

				/* Starte GPS UART DMA */
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)&gpsRxRingBuffer, GPS_RINGBUFFER_SIZE);
				__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC );
				__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE );
			}
			HAL_Delay(100);
		}
	}

	/* Starte ADC --------------------------------------------------------------*/
	if(config.TEMP_ENABLE){
		AnalogWDG_Init();														/* Analog Watchdog konfigurieren */
		HAL_ADC_Start_DMA(&hadc1, &Temp_Raw, 1);
		HAL_ADC_Start_IT(&hadc1);
	}
	
	GET_DATA = config.ACCELERATION_ENABLE + config.GNSS_ENABLE + config.LIGHT_ENABLE + config.TEMP_ENABLE;		/* initialisiere Counter zum sammeln eines Datensatzes */
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		if(getLight | getTemp | getAcceleration | getPosition | getDataset){
		
			/* Einlesen der Fotodiode und Ablage in den Datensatz ------------------*/
			if(config.LIGHT_ENABLE){
				if(getLight | getDataset){
					getLight = 0;
					getDataset--;
					
					if(HAL_GPIO_ReadPin(INT_Photodiode_GPIO_Port, INT_Photodiode_Pin) == 1){
						sensor_set[actualSet].open = 1;
					}
					else{
						sensor_set[actualSet].open = 0;
					}
				}
			}
			
			/* Einlesen der Temperatur und Ablage in den Datensatz -----------------*/
			if(config.TEMP_ENABLE){
				HAL_ADC_Start_IT(&hadc1);
				
				if(getTemp | getDataset){
					getTemp = 0;
					getDataset--;
					
					sensor_set[actualSet].temperature = 300 * 2 * Temp_Raw / 4096 - 273;    /* Umrechnung der 12-Bit Rohdaten in Grad Celsius */
					Temp = sensor_set[actualSet].temperature;
				}
			}
			
			/* Einlesen der Beschleunigungsdaten und Ablage in den Datensatz -------*/
			if(config.ACCELERATION_ENABLE){
				
				if(getAcceleration | getDataset){
					getAcceleration = 0;
					getDataset--;
					
					i2c_status = ACC_getAllValues(&hi2c3, &acceleration_actual);
					if(i2c_status == HAL_OK){
//						acceleration_actual_float.x_Value = ACC_convertAccelToFloat(acceleration_actual.x_Value, 12, 2);
//						acceleration_actual_float.y_Value = ACC_convertAccelToFloat(acceleration_actual.y_Value, 12, 2);
//						acceleration_actual_float.z_Value = ACC_convertAccelToFloat(acceleration_actual.z_Value, 12, 2);
						sensor_set[actualSet].acceleration = acceleration_actual;
					}
					else{																	
						error_blink(LED3_Pin, 3, 200);			/* wenn Lesefehler dann blinkt LED3 3 mal */
					}
				}
			}
			
			/* Einlesen der Positionsdaten und Ablage in den Datensatz -------------*/
			if(config.GNSS_ENABLE){
				if(getPosition | getDataset){
					getPosition = 0;
					getDataset--;
					if(g_newGPSData){											/* check for new UART GPS Data, and read in before using GPS dataset */
						GPS_sortInNewData(&gpsActualDataset, gpsRxRingBuffer);		/* read in new data */
						g_newGPSData = 0; 									/* reset flag	*/
					}
					/* aus aktuellem GPS Set */
				//	strcpy(sensor_set[actualSet].NMEA_GPGGA, gpsActualDataset.NMEA_GPGGA);
			  //	strcpy(sensor_set[actualSet].NMEA_GPRMC, gpsActualDataset.NMEA_GPRMC); // so wenn sensorSet zweimal char[80] enthält
					sensor_set[actualSet].NMEA_GPRMC = gpsActualDataset.NMEA_GPRMC;
					sensor_set[actualSet].NMEA_GPGGA = gpsActualDataset.NMEA_GPGGA;
					sensor_set[actualSet].timestamp = clock_time;		/* von Timer incrementiert, kann aber auch noch von GPS ab und so korrigiert werden */
				}
			}
			
			actualSet++;
		}
		
		
		/* Fuellen des Datensatzes und Abspeichern auf die SD-Karte ---------------*/
		if((actualSet == datasetCount) | writeDataset | (event == eject_card)) {
			HAL_NVIC_DisableIRQ(TIM4_IRQn);						/* Deaktiviere zyklischen Timer Interrupt um SD-Zugriff nicht zu unterbrechen */
			write_dataset_to_file(&logFile, logFileName, sensor_set, actualSet, &cursor);		/* Schreibe Datensatz auf die Speicherkarte */
			writeDataset = 0;
			actualSet = 0;
			if(event == eject_card){									/* SD-Karte auswerfen */
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);		/* Grüne LED leuchtet auf, sobald der SD-Karte entnommen werden kann */
				event = deactivate_gnss;               	/* Deaktiviere GNSS-Modul vor dem Abschalten */
			}
			else{
				HAL_NVIC_EnableIRQ(TIM4_IRQn);					/* Reaktiviere zyklischen Timer Interrupt */
			}
		}
		
		/* Deaktivieren des GNSS-Moduls -------------------------------------------*/
		if(event == deactivate_gnss) {
			if(GPS_deactivateReceiver() != -1) {
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);			/* Blaue LED leuchtet auf, sobald der GPS-Sensor sicher abgeschaltet ist */
				event = no_event;
			}
		}
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 3;

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PB10   ------> I2S2_CK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PB6   ------> I2C1_SCL
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPS_ONOFF_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPS_Reset_out_GPIO_Port, GPS_Reset_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED4_Pin|LED3_Pin|LED5_Pin|LED6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin GPS_ONOFF_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPS_ONOFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : User_Button_Pin INT_Photodiode_Pin INT_Acceleration_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin|INT_Photodiode_Pin|INT_Acceleration_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INT_Acceleration2_Pin */
  GPIO_InitStruct.Pin = INT_Acceleration2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_Acceleration2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_Reset_out_Pin */
  GPIO_InitStruct.Pin = GPS_Reset_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPS_Reset_out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin SD_Detect_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|SD_Detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : GPS_WakeUp_Pin */
  GPIO_InitStruct.Pin = GPS_WakeUp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPS_WakeUp_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin LED5_Pin LED6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin|LED5_Pin|LED6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Audio_SCL_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(Audio_SCL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void Timer4_Init(void){
	RCC->APB1ENR |= (1<<2); 					// APB: Advanced Perifery Bridge
	TIM4->PSC = 4200-1;								// Prescaler f�r Timer 4
	TIM4->ARR = 20000-1;							// Autoreload Limit
	
	TIM4->DIER |= 0x1;								// Enable Timer 4 for set an Interrupt
	NVIC_EnableIRQ(TIM4_IRQn);				// Enable Interrupt f�r Timer 4
	TIM4->CR1 |= 0x1;									// Konfig-Register f�r Timer 4 / Channel 1
}

void AnalogWDG_Init(void){
	AnalogWDGConf.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
	AnalogWDGConf.Channel = ADC_ALL_CHANNELS;
	AnalogWDGConf.HighThreshold = config.MAX_TEMP_RAW;
	AnalogWDGConf.LowThreshold = config.MIN_TEMP_RAW;
	AnalogWDGConf.ITMode = ENABLE;
	
	HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConf);
}

void error_blink(uint16_t GPIO_Pin, uint16_t n, uint16_t dt) {
	for(int i = 0; i < n; i++) {
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
		HAL_Delay(dt);
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
		HAL_Delay(dt);
	}
}


/* Interrupt Handler (ISR) ---------------------------------------------------*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	operation_mode = workmode_log;
	if(GPIO_Pin == User_Button_Pin){
		event = eject_card;
	}
	else if(GPIO_Pin == INT_Photodiode_Pin){
		event = open_event;
	}
	else if(GPIO_Pin == INT_Acceleration_Pin){
		event = vibration_event;
	}
}

void TIM4_IRQHandler() {												
	TIM4->SR &=~ (0x1);
	if(operation_mode == workmode_log) {
		clock_time.tm_sec++;
		getDataset = GET_DATA;
	}
}

void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {		
	operation_mode = workmode_log;
	event = temp_event;
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
}

/* INIH Handler --------------------------------------------------------------*/

static int Preferences_Handler(void* user, const char* section, const char* name, const char* value) {
    configuration* pconfig = (configuration*)user;

    #define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
    if (MATCH("functions", "ACCELERATION_ENABLE")) {
        pconfig->ACCELERATION_ENABLE = atoi(value);
    } else if (MATCH("functions", "GNSS_ENABLE")) {
        pconfig->GNSS_ENABLE = atoi(value);
		}	else if (MATCH("functions", "LIGHT_ENABLE")) {
        pconfig->LIGHT_ENABLE = atoi(value);
		} else if (MATCH("functions", "TEMP_ENABLE")) {
        pconfig->TEMP_ENABLE = atoi(value);
		} else if (MATCH("values", "ACC_MAX_ANZAHL_WERTE")) {
        pconfig->ACC_MAX_ANZAHL_WERTE = atoi(value);
    } else if (MATCH("values", "MIN_TEMP")) {
        pconfig->MIN_TEMP = atoi(value);
    } else if (MATCH("values", "MAX_TEMP")) {
        pconfig->MAX_TEMP = atoi(value);
    } else if (MATCH("values", "MAX_ACC")) {
        pconfig->MAX_ACC = atoi(value);
    } else {
        return 0;  																						/* unknown section/name, error */
    }
    		
		pconfig->MAX_TEMP_RAW = pconfig->MAX_TEMP * 4096 / 600;		/* Umrechnen der Werte von K in AD-Werte */
	  pconfig->MIN_TEMP_RAW = pconfig->MIN_TEMP * 4096 / 600;		/* Umrechnen der Werte von K in AD-Werte */
		
		return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
