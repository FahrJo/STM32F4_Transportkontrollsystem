/* Includes ------------------------------------------------------------------*/
#include "card_operations.h"


/* Private variables ---------------------------------------------------------*/
FRESULT return_value;
char dataset_string[128];
char buffer_string[32];
BYTE mode = FA_WRITE | FA_CREATE_ALWAYS;

/* Write a string into a file on the SD-Card ---------------------------------*/
FRESULT write_string_to_file(FIL* logFile_p, const TCHAR* logFileName_p, const void* string_p,	UINT size_of_string, UINT* cursor){
	return_value = f_open(logFile_p, logFileName_p, mode);
	
	if(return_value == FR_OK){
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		
		return_value = f_write(logFile_p, string_p, size_of_string, cursor);
		if(return_value == FR_OK){
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		}
		f_close(logFile_p);
	}
	else{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	return return_value;
}


/* Write a full dataset into the logfile on the SD-Card (CSV format) ---------*/
FRESULT write_dataset_to_file(FIL* logFile_p, const TCHAR* logFileName_p, dataset* dataset_p, UINT* cursor){
	convert_dataset_to_string(dataset_p, dataset_string);
	
	return_value = f_open(logFile_p, logFileName_p, mode);
	
	if(return_value == FR_OK){
		HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		
		return_value = f_write(logFile_p, dataset_string, sizeof(dataset_string), cursor);
		if(return_value == FR_OK){
			HAL_GPIO_WritePin(LED6_GPIO_Port, LED6_Pin, GPIO_PIN_SET);
		}
		f_close(logFile_p);
	}
	else{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
	}
	return return_value;
}


/* Convert the data in the struct to a string --------------------------------*/
void convert_dataset_to_string(dataset* dataset_p, char* dataset_string){
	sprintf(dataset_string, "%i-%i-%i-%i:%i:%i;", dataset_p->timestamp.tm_year, dataset_p->timestamp.tm_mon, dataset_p->timestamp.tm_mday, dataset_p->timestamp.tm_hour, dataset_p->timestamp.tm_min, dataset_p->timestamp.tm_sec);
	strcat(dataset_string, dataset_p->position);
	sprintf(buffer_string, "%i;%i;%i;", dataset_p->acceleration.x_Value, dataset_p->acceleration.y_Value, dataset_p->acceleration.z_Value);
	strcat(dataset_string, buffer_string);
	sprintf(buffer_string, "%i;", dataset_p->temperature);
	strcat(dataset_string, buffer_string);
	strcat(dataset_string, dataset_p->open != 0 ? "YES;" : "NO;");
	strcat(dataset_string, dataset_p->notes);
}
