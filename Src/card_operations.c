//#include "card_operations.h"

//FRESULT write_dataset_to_file(FIL* logFile_p, 

//	if(f_open(logFile_p, logFileName_p, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK){
//		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
//		
//		char header[] = "Tracking-Log vom 17.01.2019;;;\n Date/Time;Location;Temp;Note\n";
//		if(f_write(&logFile, header, sizeof(header), &cursor) == FR_OK){
//			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
//		}
//		f_close(&logFile);
//	}
//	else{
//		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
//	}
