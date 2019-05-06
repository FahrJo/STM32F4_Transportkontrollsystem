/* Includes ------------------------------------------------------------------*/
#include "card_operations.h"


/* Private variables ---------------------------------------------------------*/
FRESULT return_value;
char dataset_string[512];
char buffer_string[64];
BYTE mode0 = FA_WRITE | FA_CREATE_ALWAYS;
BYTE mode = FA_WRITE | FA_OPEN_APPEND;

/* Write a string into a file on the SD-Card ---------------------------------*/
FRESULT write_string_to_file(FIL* logFile_p, const TCHAR* logFileName_p, const void* string_p,	UINT size_of_string, UINT* cursor){
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	
	/* �ffnen und beschreiben der Datei */
	return_value = f_open(logFile_p, logFileName_p, mode0);
	if(return_value == FR_OK){
		return_value = f_write(logFile_p, string_p, size_of_string, cursor);
		f_close(logFile_p);
	}
	
	/* Fehler beim schreiben oder �ffnen der Datei */
	if(return_value != FR_OK){
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	return return_value;
}


/* Write a full dataset into the logfile on the SD-Card (CSV format) ---------*/
FRESULT write_dataset_to_file(FIL* logFile_p, const TCHAR* logFileName_p, dataset* dataset_p, UINT size_of_dataset, UINT* cursor){
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	
	
	
	
	/* �ffnen und beschreiben der Datei */
	return_value = f_open(logFile_p, logFileName_p, mode);
	if(return_value == FR_OK){
		for(int count = 0; count < size_of_dataset; count++){									// TODO: geht das so???
			convert_dataset_to_string(dataset_p + count, dataset_string);												// Konvertieren des Datensatzes in einen String
			for(int block = 0; block < 4; block++){
				return_value = f_write(logFile_p, dataset_string + (block * 32), 32, cursor);
			}
			//f_puts(logFile_p, dataset_string);
		}
		f_close(logFile_p);
	}
	
	/* Fehler beim schreiben oder �ffnen der Datei */
	if(return_value != FR_OK){
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	return return_value;
}


/* Convert the data in the struct to a string --------------------------------*/
void convert_dataset_to_string(dataset* dataset_p, char* dataset_string){
	for(int i = 0; i < 128; i++){
		dataset_string[i] = 0;
	}
	sprintf(dataset_string, "%04i-%02i-%02i-%02i:%02i:%02i;", dataset_p->timestamp.tm_year, dataset_p->timestamp.tm_mon, dataset_p->timestamp.tm_mday, dataset_p->timestamp.tm_hour, dataset_p->timestamp.tm_min, dataset_p->timestamp.tm_sec);
	strcat(dataset_string, dataset_p->NMEA_GPGGA);
	strcat(dataset_string, ";");
	strcat(dataset_string, dataset_p->NMEA_GPRMC);
	sprintf(buffer_string, ";%f;%f;%f;", dataset_p->acceleration.x_Value, dataset_p->acceleration.y_Value, dataset_p->acceleration.z_Value);
	strcat(dataset_string, buffer_string);
	sprintf(buffer_string, "%i;", dataset_p->temperature);
	strcat(dataset_string, buffer_string);
	sprintf(buffer_string, "%i;", dataset_p->open);
	strcat(dataset_string, buffer_string);
	strcat(dataset_string, dataset_p->notes);
	strcat(dataset_string, "\n");
}
