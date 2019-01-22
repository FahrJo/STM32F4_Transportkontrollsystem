/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "main.h"
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
typedef struct {
	struct tm timestamp;									/* Timestamp of the dataset (from GNSS) */
	char position[25];										/* actual position of the system (from GNSS) */
	s_accelerometerValues acceleration;		/* actual acceleration of the system (from MEMS) */
	int8_t temperature;										/* actual temperature (from LM335) */
	char open;														/* is package open (from photo diode) */
	char notes[25];												/* additional notes (e.g. exceed a limit) */ 
}dataset;

/* Private function prototypes -----------------------------------------------*/

/* Write a string into a file on the SD-Card */
FRESULT write_string_to_file(FIL* logFile_p, const TCHAR* logFileName_p, const void* string_p,	UINT size_of_string, UINT* cursor);

/* Write a full dataset into the logfile on the SD-Card (CSV format) */
FRESULT write_dataset_to_file(FIL* logFile_p, const TCHAR* logFileName_p, dataset* dataset_p, UINT* cursor);
	
/* Convert the data in the struct to a string */
void convert_dataset_to_string(dataset* dataset_p, char* dataset_string);
