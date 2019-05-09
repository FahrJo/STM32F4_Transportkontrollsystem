/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "card_operations.h"
#include "acceleration_Sensor.h"
#include "gps_modul.h"
#include "ini.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

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
GPIO_InitTypeDef GPIO_InitStruct = {0};

static int Preferences_Handler(void* user, const char* section, const char* name, const char* value);		/* Handler für Config-Parser */

const uint16_t 	datasetCount = 100;							/* Anzahl der Datensäetze, die gesammelt auf die Karte geschrieben werden */
uint16_t				actualSet = 0;									/* Momentan aktiver Datensatz */
ADC_AnalogWDGConfTypeDef AnalogWDGConf;

struct tm				clock_time;

FATFS 					myFatFS;
FIL 						logFile;												/* Dateihandle auf SD-Karte (Log-Datei) */
FIL 						configFile;											/* Dateihandle auf SD-Karte (Initialisierungsdatei)*/
UINT 						cursor;													/* Letztes Zeichen in CSV-Datei (Log-Datei) */
UINT 						configCursor;										/* Letztes Zeichen in CSV-Datei Initialisierungsdatei) */
char 						logFileName[] = "0:/LogFile.csv";
char 						configFileName[] = "0:/config.ini";
char * 					configBuffer;
char 						header[200];
uint32_t 				Temp_Raw;												/* Temperatur in 12 Bit aus ADC */
uint16_t 				Temp;														/* Temperatur in Grad Celsius */
dataset 				sensor_set[datasetCount];				/* Datensatz, der im RAM gepuffert wird */

/* Flags und Wartezeiten für den Sleep Mode */
uint16_t				sleepTimer;
uint16_t				activeTime = 300;								/* Zeit in Sekunden nach einem Ereigniss für den Sleep Mode (max 65.536 s) */
workmode_type 	operation_mode = workmode_log;	/* Betriebsmodus (Energiespar-Funktion) */
event_type 			event;													/* Event fuer die Detektierung einer Grenzwertueberschreitung */

s_accelerometerValues 			acceleration_actual_global;
s_accelerometerValuesFloat 	acceleration_actual_float;
float 											max_acceleration = 2.0;					/* Messbereich (+/-) in g */

uint8_t					g_newGPSData = 0;								/* Flag wenn neue Daten im Buffer anstehen wird von DMA IR-Handler gesetzt */
s_gpsSetOfData  gpsActualDataset;								/* Set mit aktuellsten GPS Daten */

/* Trigger-Flags fuer das Einlesen von Daten */
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
void writeToFile(void);
int ini_parse_fatfs(FIL* file, const char* filename, ini_handler handler, void* user);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	/* Standartwerte */
	config.MAX_TEMP 						= 333;
	config.MIN_TEMP 						= 263;
	config.MAX_ACC 							= 2;
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
  

  /* MCU Configuration--------------------------------------------------------*/

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
	
	/* I2C-Bus auf 3 Volt ziehen (Bug-Workaround) ------------------------------*/
	HAL_GPIO_WritePin(I2C_INIT_SCL_GPIO_Port, I2C_INIT_SCL_Pin|I2C_INIT_SDA_Pin, GPIO_PIN_SET);		/* Pull der Busleitungen auf VDD */
	HAL_Delay(200);
	GPIO_InitStruct.Pin = I2C_INIT_SCL_Pin|I2C_INIT_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;						/* HiZ */
  HAL_GPIO_Init(I2C_INIT_SCL_GPIO_Port, &GPIO_InitStruct);		/* Umkonfiguration der Ausgäge zu Aingängen */
	
	/* Vorbereiten der SD-Karte ------------------------------------------------*/
	if(f_mount(&myFatFS, SDPath, 1) == FR_OK){
		if(ini_parse_fatfs(&configFile, configFileName, Preferences_Handler, &config) != FR_OK) {		/* Auslesen der Initialisierungsdatei */
			error_blink(LED3_Pin, 5, 500);					/* wenn Lesefehler an config.ini dann blinkt orangene LED3 5 mal lang */
		}
		sprintf(header, "Tracking-Log;t_min:;%i;t_max:;%i;Acc_max:;%1.3f;\n Date/Time;Location (GPRMC);Location (GPGGA);Acceleration X; Acceleration Y; Acceleration Z;Temp;Open;Note\n", config.MIN_TEMP, config.MAX_TEMP, config.MAX_ACC);
		write_string_to_file(&logFile, logFileName, header,	sizeof(header), &cursor);
	}
	else{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);		/* Orangene LED3 leuchtet dauerhaft, wenn SD-Karte  nicht initialisiert werden konnte */
	}
	
	Timer4_Init();
	
		
	/* Acc-Sensor ueber I2C deaktivieren und dann wieder aktivieren, damit dieser sicher aktiviert wird */
	if(config.ACCELERATION_ENABLE){
		HAL_Delay(500);
		ACC_deactivate(&hi2c3);
		HAL_Delay(500);
		ACC_activate(&hi2c3);
	}
	
	/* GPS-Empfänger aktivieren ------------------------------------------------*/
	if(config.GNSS_ENABLE) {
		config.GNSS_ENABLE = 0; 										/* erst mal deaktiveren, wenn GPS aktivierung erfolgreich wird es wieder ENABLED */
		HAL_GPIO_WritePin(GPS_Reset_out_GPIO_Port, GPS_Reset_out_Pin, GPIO_PIN_SET);
		for(int i = 0; i < 4; i++){									/* wenn nach 4 Versuchen, das Modul nicht aktivert werden kann, bleibt GNSS_ENABLE=0 und das Programm lauft "normal" weiter */
			if(-1 != GPS_activateReceiver()){ 				/* GPS erfolgreich aktiviert, for-Schleife verlassen */
				config.GNSS_ENABLE=1;
				i = 5;

				/* Starte GPS UART DMA */
				HAL_UART_Receive_DMA(&huart3, (uint8_t*)&gpsRxRingBuffer, GPS_RINGBUFFER_SIZE);
				__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC );
				__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE );
			}
			else if(i == 4){
				error_blink(LED6_Pin, 5, 500);					/* wenn GPS nicht aktiviert werden kann blinkt LED6 (Blau) 5 mal lang */
			}
			HAL_Delay(100);
		}
	}

	/* Starte ADC --------------------------------------------------------------*/
	if(config.TEMP_ENABLE){
		AnalogWDG_Init();														/* Analog Watchdog konfigurieren */
		HAL_ADC_Start_DMA(&hadc1, &Temp_Raw, 1);		/* ADC->Mem DMA starten */
		HAL_ADC_Start_IT(&hadc1);										/* ADC-WDG Interrupt aktivieren */
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
						acceleration_actual_float.x_Value = ACC_convertAccelToFloat(acceleration_actual.x_Value, 12, max_acceleration);		/* Umrechnung in Float über Registerbreite und Messbereich in g */
						acceleration_actual_float.y_Value = ACC_convertAccelToFloat(acceleration_actual.y_Value, 12, max_acceleration);		/* Umrechnung in Float über Registerbreite und Messbereich in g */
						acceleration_actual_float.z_Value = ACC_convertAccelToFloat(acceleration_actual.z_Value, 12, max_acceleration);		/* Umrechnung in Float über Registerbreite und Messbereich in g */
						sensor_set[actualSet].acceleration = acceleration_actual_float;
					}
					else{																	
						error_blink(LED5_Pin, 3, 100);			/* wenn Lesefehler dann blinkt LED5 3 mal kurz */
					}
				}
			}
			
			/* Einlesen der Positionsdaten und Ablage in den Datensatz -------------*/
			if(config.GNSS_ENABLE){
				if(getPosition | getDataset){
					getPosition = 0;
					getDataset--;
					g_newGPSData = 1; // always read in. TODO: this is a workaround as long the DMA-IRQ does not do what it should
					if(g_newGPSData){					/* check for new UART GPS Data, and read in before using GPS dataset */
						HAL_UART_DMAPause(&huart3); //pause DMA um Datenkonstistenz zu gewährleisten
						int returnValue = GPS_sortInNewData(&gpsActualDataset, gpsRxRingBuffer);		/* read in new data */
						HAL_UART_DMAResume(&huart3);
						// read in new time from GPS only when GPS Timestamp is newer than old timestamp from GPS. Else we jump back in time (bc. Time get incremented intern) 
						// TODO: add gps parse time in this struct -> then incomment the next line						
						//if(returnValue==0) clock_time = gpsActualDataset.gps_timestamp; // check if this is ok, or only copy a pointer
						g_newGPSData = 0; 									/* reset flag	*/
					}
					/* aus aktuellem GPS Set Position immer kopieren, weil da ist es besser die letzte zu haben als keine*/
					strcpy(sensor_set[actualSet].NMEA_GPGGA, gpsActualDataset.NMEA_GPGGA);
			  	strcpy(sensor_set[actualSet].NMEA_GPRMC, gpsActualDataset.NMEA_GPRMC); 					
					sensor_set[actualSet].timestamp = clock_time;		/* von Timer incrementiert, wird aber auch noch von GPS ab und so korrigiert werden */
				}
			}
			
			actualSet++;
		}		
		
		/* Fuellen des Datensatzes und Abspeichern auf die SD-Karte --------------*/
		if((actualSet == datasetCount) | writeDataset) {
			writeToFile();
		}
		
		/* SD-Karte auswerfen ----------------------------------------------------*/
		if(event == eject_card){										
			writeToFile();														/* letzten Datensatz auf Karte speichern */
			HAL_NVIC_DisableIRQ(TIM4_IRQn);						/* Deaktiviere zyklischen Timer Interrupt um Logging zu unterbrechen */
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);		/* Grüne LED leuchtet auf, sobald der SD-Karte entnommen werden kann */
			event = deactivate_gnss;               		/* Deaktiviere GNSS-Modul vor dem Abschalten */
		}
		
		/* Deaktivieren des GNSS-Moduls -------------------------------------------*/
		if(event == deactivate_gnss) {
			if(GPS_deactivateReceiver() != -1) {
				HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);			/* Blaue LED leuchtet auf, sobald der GPS-Sensor sicher abgeschaltet ist */
				event = no_event;
			}
		}
		
		/* Sleep Mode nach x Minuten ohne Ereigniss/Interrupt --------------------*/
		if(operation_mode == workmode_sleep) {
			GPS_deactivateReceiver();
			writeToFile();
			HAL_NVIC_DisableIRQ(TIM4_IRQn);
			HAL_SuspendTick();
			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
			HAL_ResumeTick();
			HAL_NVIC_EnableIRQ(TIM4_IRQn);
			GPS_activateReceiver();
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
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
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 5;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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
  HAL_GPIO_WritePin(GPIOC, OTG_FS_PowerSwitchOn_Pin|I2C_INIT_SCL_Pin|I2C_INIT_SDA_Pin, GPIO_PIN_SET);

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

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin I2C_INIT_SCL_Pin I2C_INIT_SDA_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|I2C_INIT_SCL_Pin|I2C_INIT_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pins : BOOT1_Pin SD_detect_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|SD_detect_Pin;
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
	RCC->APB1ENR |= (1<<2); 											/* APB: Advanced Perifery Bridge */
	TIM4->PSC = 4200-1;														/* Prescaler fuer Timer 4 */
	TIM4->ARR = 20000-1;													/* Autoreload Limit */
	
	TIM4->DIER |= 0x1;														/* Enable Timer 4 for set an Interrupt */
	NVIC_EnableIRQ(TIM4_IRQn);										/* Enable Interrupt fuer Timer 4 */
	TIM4->CR1 |= 0x1;															/* Konfig-Register fuer Timer 4 / Channel 1 */
}

void AnalogWDG_Init(void){
	config.MAX_TEMP_RAW = config.MAX_TEMP * 4096 / 600;		/* Umrechnen der Werte von K in AD-Werte */
	config.MIN_TEMP_RAW = config.MIN_TEMP * 4096 / 600;		/* Umrechnen der Werte von K in AD-Werte */
	
	AnalogWDGConf.WatchdogMode = ADC_ANALOGWATCHDOG_ALL_REG;
	AnalogWDGConf.Channel = ADC_ALL_CHANNELS;
	AnalogWDGConf.HighThreshold = config.MAX_TEMP_RAW;
	AnalogWDGConf.LowThreshold = config.MIN_TEMP_RAW;
	AnalogWDGConf.ITMode = ENABLE;
	
	HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConf);
}

void error_blink(uint16_t GPIO_Pin, uint16_t n, uint16_t dt) {		/* GPIO_Pin: Pin, n: Anzahl der Impulse, dt: Dauer der Impulse */
	for(int i = 0; i < n; i++) {
		HAL_GPIO_WritePin(LED3_GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(dt);
		HAL_GPIO_WritePin(LED3_GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(dt);
	}
}

void writeToFile(void) {
	HAL_NVIC_DisableIRQ(TIM4_IRQn);								/* Deaktiviere zyklischen Timer Interrupt um SD-Zugriff nicht zu unterbrechen */
	write_dataset_to_file(&logFile, logFileName, sensor_set, actualSet, &cursor);		/* Schreibe Datensatz auf die Speicherkarte */
	writeDataset = 0;
	actualSet = 0;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);								/* Reaktiviere zyklischen Timer Interrupt */
}


/* Interrupt Handler (ISR) ---------------------------------------------------*/

/* GPIO Interrupt Handler */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	operation_mode = workmode_log;								/* Ändern des Betriebsmodus in Log-Modus */
	sleepTimer = 0;																/* Zurücksetzen des Sleep-Timers bei neuem Interrupt */
	if(GPIO_Pin == User_Button_Pin){
		event = eject_card;
	}	else if(GPIO_Pin == INT_Photodiode_Pin){
		event = open_event;
	}	else if(GPIO_Pin == INT_Acceleration_Pin){
		event = moving_event;
	}	else if(GPIO_Pin == INT_Acceleration2_Pin){
		event = hit_event;
	}
}

/* Analog Watchdog Interrupt Handler */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc) {		
	operation_mode = workmode_log;								/* Ändern des Betriebsmodus in Log-Modus */
	sleepTimer = 0;																/* Zurücksetzen des Sleep-Timers bei neuem Interrupt */
	event = temp_event;
	HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);		/* Aktivieren der roten LED als Indikator */
}

/* Zyklischer Timer interrupt im Sekundentakt */
void TIM4_IRQHandler() {												
	TIM4->SR &=~ (0x1);
	if(++sleepTimer == activeTime) {							/* Ändern des Betriebsmodus nach voreingestelter Zeit ohne Interrupt */
		operation_mode = workmode_sleep;						/* Ändern des Betriebsmodus in Ruhemodus */
		sleepTimer = 0;
	}
	if(operation_mode == workmode_log) {					/* Routine zum hochzählen der Zeit im Sekundentakt. Diese muss jedoch zu Beginn bzw. nach jedem Ruhezustand einmalig vom GPS-Signal aktualisiert werden. TODO! */
		getDataset = GET_DATA;
		if(++clock_time.tm_sec == 60) {
			clock_time.tm_sec = 0;
			if(++clock_time.tm_min == 60) {
				clock_time.tm_min = 0;
				if(++clock_time.tm_hour == 24) {
					clock_time.tm_hour = 0;
					if(++clock_time.tm_yday == 365) {
						clock_time.tm_yday = 0;
						++clock_time.tm_year;
					}
				}
			}
		}
	}
}

/* INIH Handler --------------------------------------------------------------*/
int ini_parse_fatfs(FIL* file, const char* filename, ini_handler handler, void* user)
{
    FRESULT result;
    int error;

    result = f_open(file, filename, FA_OPEN_EXISTING | FA_READ);
    if (result != FR_OK)
        return -1;
    error = ini_parse_stream((ini_reader)f_gets, file, handler, user);
    f_close(file);
    return error;
}

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
        pconfig->MAX_ACC = atof(value);
    } else {
        return 0;  															/* unknown section/name, error */
    }
		
		return 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
