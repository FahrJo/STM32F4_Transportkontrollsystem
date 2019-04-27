#include "acceleration_sensor.h"
#include "hilfsfunktionen.h"

//uint8_t ACCELEROMETER_I2C_ADRESS = 0x0;
uint8_t	 	adress[] = {0x1C, 0x1D, 0x38, 0x3A};
uint8_t		adressCounter;

/********************************************************************************
****************** I2C i2c_ *****************************************************
********************************************************************************/

// HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT -> HAL_StatusTypeDef;
// Schreibt Daten an ein Register eines I2C Gerätes
HAL_StatusTypeDef i2c_write_register(I2C_HandleTypeDef *i2cHandler, uint8_t device_slave_adress, uint8_t register_pointer 
																		,uint16_t register_data_to_write, uint16_t number_bytes_to_write)
{
	HAL_StatusTypeDef status = HAL_OK;
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

	status = HAL_I2C_Mem_Write(i2cHandler, device_slave_adress, (uint16_t)register_pointer, I2C_MEMADD_SIZE_8BIT 
														,(uint8_t*)(&register_data_to_write), number_bytes_to_write, 100); 

	/* Check the communication status */
	if(status != HAL_OK)
	{
			// Error handling, for example re-initialization of the I2C peripheral
	}
	else{
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	}
	return status;
}

// beliebiges Register lesen. 
HAL_StatusTypeDef i2c_read_register(I2C_HandleTypeDef *i2cHandler,uint8_t device_slave_adress, uint8_t register_pointer
																		,uint8_t *register_data_read_buffer, uint8_t number_bytes_to_read)
{
	HAL_StatusTypeDef status = HAL_OK;
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
	
	status = HAL_I2C_Mem_Read(i2cHandler, device_slave_adress, (uint16_t)register_pointer, 
														I2C_MEMADD_SIZE_8BIT, (uint8_t*) register_data_read_buffer, 
														number_bytes_to_read, 100);
	/* Check the communication status */
	if(status != HAL_OK)
	{
			// Error handling, for example re-initialization of the I2C peripheral
	}
	else{
		HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	}
	return status;
}

/********************************************************************************
****************** GPS/GNSS Modul GPS_ ******************************************
********************************************************************************/
// ueber WakeupPin Status abfragen, Sensor de-/aktiveren und Ergebnis ueberpruefen. 
// zurueckgeben ob erfolgreich, entsprechend reagieren.
// returns:  -1 -> Fehler.. nicht erfolgreich
int GPS_activateReceiver(void)
{
	//status ueberpruefen, wenn WAKEUP==1, ist es entweder schon An oder es gibt im moment einen 200ms Puls aus. darum 210ms spaeter nochmal schauen
	if(HAL_GPIO_ReadPin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin)){
		HAL_Delay(210);
		if(HAL_GPIO_ReadPin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin)){
			return 0; //ist bereits an
		}	// hat puls ausgegeben -> ist noch aus
	} // dauerhaft 0 -> ist noch aus
	// anschalten durch 200ms High-Puls auf ON_OFF Pin
	HAL_GPIO_WritePin(GPS_ONOFF_GPIO_Port, GPS_ONOFF_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPS_ONOFF_GPIO_Port, GPS_ONOFF_Pin, GPIO_PIN_RESET);
	HAL_Delay(150); // warten damit Modul auch sicher hochgefahren ist und nicht fehlerhaft ein fehlschlag detectiert wird
	if(HAL_GPIO_ReadPin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin)) return 1; // erfolgreich aktiviert
	else return -1; // aktivieren fehlgeschlagen
}

// optimierungspotential: nebenlaufig umsetzen, bzw die 1s wartezeit rausnehmen und nebenlaufig oder ueber abfrage ob schon schneller aus, ersetzen
// aktuelle Umsetzung ist aber auf jeden Fall die sicherere Option
int GPS_deactivateReceiver(void)
{
// nach Shutdown initialisierung dauert es maximal 1s bis Spannung weggenommen werden kann.
	//status ueberpruefen, wenn WAKEUP==0, ist es schon Aus
	if(0==HAL_GPIO_ReadPin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin)){
		return 0; //ist bereits aus
	}	 
	// ausschalten durch 200ms High-Puls auf ON_OFF Pin
	HAL_GPIO_WritePin(GPS_ONOFF_GPIO_Port, GPS_ONOFF_Pin, GPIO_PIN_SET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPS_ONOFF_GPIO_Port, GPS_ONOFF_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000); // warten damit Modul auch sicher runtergefahren ist und nicht fehlerhaft ein fehlschlag detektiert wird
	if(0==HAL_GPIO_ReadPin(GPS_WakeUp_GPIO_Port, GPS_WakeUp_Pin)) return 1; // erfolgreich deaktiviert 
	else return -1; // deaktivieren fehlgeschlagen. TODO: kann bei eigenständigem ONpuls fehlerhaft sein
}



// zwei Schleifen schachteln. äußere sucht nach "§" und gibt dann jeweilige Stringteil, also eine NMEA Nachricht weiter.
// innere sucht entsprechend der MessageID nach einem bestimmten Paramter -> Komma zählen
// oder hier identifiziert Message und speichert sie entsprechend weg
// bricht Buffersuche ab, nachdem beide gesuchten Stings gefunen wurden
/* GPS Stuct Entwurf siehe oneNote */
int GPS_sortInNewData(s_gpsSetOfData* gpsActualDataset, char* pNewNmeaString)
{
	char gpsTypFound = 0; // GPRMC (0000 0001 -> 0x1) , GPGGA (0000 0010 -> 0x2) , ((other (other)
	int16_t cursor = 0; 
	char copyindex=0;
	// fehler abfangen falls pNewNmeaString kurzer ist und kein \0 enthält
	// nach gefunden testen, ob zwischen '$' und '*' <=80 Zeichen 
	while((GPS_RINGBUFFER_SIZE-2) > cursor && pNewNmeaString[cursor] != '\0' ){
		if(pNewNmeaString[cursor] == '$'){
			// NMEA-Sentence Anfang gefunden an stelle cursor
			if(pNewNmeaString[cursor+1]=='G' && pNewNmeaString[cursor+2]=='P' 
				&& pNewNmeaString[cursor+3]=='R' && pNewNmeaString[cursor+4]=='M' 
				&& pNewNmeaString[cursor+5]=='C' && pNewNmeaString[cursor+6]==','){
					// GPRMC Sentence entdeckt
					// Datensatz von Null ab füllen, aus Eingangs-String mit offset, wo das $ beginnt ($ mitkopieren)
					while(80 > copyindex && pNewNmeaString[cursor+copyindex] != '\0' && pNewNmeaString[cursor+copyindex] != '*')	{
						/* vom $ an werden bis * oder Stringende maximal 80 Zeichen kopiert. 
						 kommt cursor+copyindex ans Ende des Buffers, zB. Size=500, ans 500 Zeichen, also Index 499 so wird das kopiert
						danach beim index 500 greift Modulo und man kopiert vom 1. Zeichen mit index 0 (->ringbuffer)*/
						gpsActualDataset->NMEA_GPRMC[copyindex] = pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE];
						copyindex++;
					}
					gpsActualDataset->NMEA_GPRMC[copyindex] = '\0'; // stringende ans Ende
					gpsTypFound |= 0x1; //erfolgreich GPRMC eingelesen
				}
			else if(pNewNmeaString[cursor+1]=='G' && pNewNmeaString[cursor+2]=='P' 
				&& pNewNmeaString[cursor+3]=='G' && pNewNmeaString[cursor+4]=='G' 
				&& pNewNmeaString[cursor+5]=='A' && pNewNmeaString[cursor+6]==','){
					// GPGGA Sentence entdeckt
					// Datensatz von Null ab füllen, aus Eingangs-String mit offset, wo das $ beginnt ($ mitkopieren)
					while(80 > copyindex && pNewNmeaString[cursor+copyindex] != '\0' && pNewNmeaString[cursor+copyindex] != '*')	{
						gpsActualDataset->NMEA_GPGGA[copyindex] = pNewNmeaString[cursor+copyindex];
						copyindex++;
					}
					gpsActualDataset->NMEA_GPGGA[copyindex] = '\0'; // stringende ans Ende
					gpsTypFound |= 0x2; //erfolgreich GPGGA eingelesen
				}
				else {} // nop; einen anderen Sentences gefunden, buffer enthält aber mehrere also weitersuchen
		} // ende if der '$' suche
		if(gpsTypFound==0x3) return 0; // abbruch wenn beide NMEA mindestens einmal gefunden wurden
	} // ende while der '$' suche
	return -1; // string enthielt kein $

	
// so kann man schauen, ob in dem Datensatz schon was steht oder noch leer
//gpsActualDataset->NMEA_GPGGA[0] != '$'..... strcmp(
}

int GPS_getVelocity(s_gpsSetOfData* gpsActualDataset) 
{
// als INT in ZentiMeter pro Sekunde ( GG.G -> int
	char cursor=0, i=0;
	char commaCounter=0;
	char velocityString[7]; // xxx.x\0 =6
	uint8_t anzahlStellenGenutzt;
	while(80 > cursor && gpsActualDataset->NMEA_GPRMC[cursor] != '\0' && gpsActualDataset->NMEA_GPRMC[cursor] != '*'){
		if(gpsActualDataset->NMEA_GPRMC[cursor] == ','){
			commaCounter++;
		}
		// nach dem 7. Komma steht die Geschwindigkeit
		if(commaCounter==7){
			i=0;
			while(gpsActualDataset->NMEA_GPRMC[cursor+1 + i] != ','){
				velocityString[i] = gpsActualDataset->NMEA_GPRMC[cursor+1 + i];
				i++;
				if(i>6)return -1; // sollte nicht so weit gehen->Fehler
			}
			velocityString[i]='\0';
			
			return (int)10*myParseFloatNumber(velocityString, &anzahlStellenGenutzt);
			// hier jetzt aus eingebundener Lib zu fload oder Int parsen
			// und funktion return richtige Value
		}
		cursor++;
	}
	return -1; //error
}
	
int GPS_getMovedDistance(s_gpsSetOfData* gpsActualDataset); // in Meter

/********************************************************************************
****************** ACCELEROMETER ACC_ *******************************************
********************************************************************************/
/*
Info wie ACCELEROMETER bestimmte ControlRegister geändert werden müsssen:
	- in Standby wechseln CTRL_REG1 &= ~(ACTIVE_BIT) 
	- Ändern
	- wieder in Active Mode CTRL_REG1 |= ACTIVE_BIT
*/

HAL_StatusTypeDef ACC_activate(I2C_HandleTypeDef *i2cHandler)
{
	// better version would be first read actual Value - modify - write
	//Schreibe SleepRate50Hz(00), DataRate50Hz(100), NO_lowNoise(0), NO_fastread(0), Activ(1) = 0010 0001 = 0x21
	return i2c_write_register(i2cHandler, ACCELEROMETER_I2C_ADRESS, ACC_CTRL_REG1, 0x21, 1);
}

HAL_StatusTypeDef ACC_deactivate(I2C_HandleTypeDef *i2cHandler)
{
	// better version would be first read actual Value - modify - write
	//Schreibe SleepRate50Hz(00), DataRate50Hz(100), NO_lowNoise(0), NO_fastread(0), NOT_Activ(0) = 0010 0000 = 0x20
	return i2c_write_register(i2cHandler, ACCELEROMETER_I2C_ADRESS, ACC_CTRL_REG1, 0x20, 1);
}

//Accelerometer alle Werte auslesen. Returns HAL_OK when successfull
HAL_StatusTypeDef ACC_getAllValues(I2C_HandleTypeDef *i2cHandler, s_accelerometerValues *accValues)
{
	HAL_StatusTypeDef status = HAL_OK;
	int8_t receiveBuffer[6];
	
	status = i2c_read_register(i2cHandler, ACCELEROMETER_I2C_ADRESS, ACC_OUT_X_MSB, (uint8_t*)&receiveBuffer, 6);
	
	if (status!=HAL_OK)return status; // bei Fehler Funktion hier mit status Rueckgabe verlassen
	//X_Wert_14_0 = (X_MSB<<5) || (X_LSB>>2)
	//bei Fehlerhaften Werten, evtl vor "<<" typecast auf int16 
	accValues->x_Value = 0;
	accValues->y_Value = 0;
	accValues->z_Value = 0;
	
	int8_t zwischenbuffer_byte[6];
	int16_t* zwischenbuffer = (int16_t*)zwischenbuffer_byte;
	
	zwischenbuffer_byte[0] = receiveBuffer[1];
	zwischenbuffer_byte[1] = receiveBuffer[0];
	zwischenbuffer_byte[2] = receiveBuffer[3];
	zwischenbuffer_byte[3] = receiveBuffer[2];
	zwischenbuffer_byte[4] = receiveBuffer[5];
	zwischenbuffer_byte[5] = receiveBuffer[4];
	
	accValues->x_Value = zwischenbuffer[0] / 16;
	accValues->y_Value = zwischenbuffer[1] / 16;
	accValues->z_Value = zwischenbuffer[2] / 16;
	//accValues->x_Value = (receiveBuffer[0]<<8 | receiveBuffer[1]) / 16;
	//accValues->y_Value = (receiveBuffer[2]<<8 | receiveBuffer[3]) / 16;
	//accValues->z_Value = (receiveBuffer[4]<<8 | receiveBuffer[5]) / 16;

	return	HAL_OK;
}

// Convert Received Data in Acceleration float
// Pram: breiteInBit ist entweder 8 oder 12bit / messbereich +-2, 4, 8 g
// für das speichern der Werte wäre uint16_t sinnvoller als float
float ACC_convertAccelToFloat(int16_t rohDaten, uint8_t breiteInBit, uint8_t messbereich)
{
	// 0111 1111 1111 = +1.999 / 3.998 / 7.996
	// 0000 0000 0001 = 0.001 / 0.002 / 0.004
	// 1111 1111 1111 = -0.001 / -0.002 / -0.004
	// 1000 0000 0000 = -2 / -4 / -8
	float accelerationFloat;
	
	accelerationFloat = (rohDaten * messbereich) / (float)(1<<(breiteInBit-1)); //TODO?
	
	return accelerationFloat;
}

/*void ACC_incrementAdress(void){
	ACCELEROMETER_I2C_ADRESS++;
}

void ACC_setAdress(uint8_t adress){
	ACCELEROMETER_I2C_ADRESS = adress;
}*/

/*uint8_t ACC_Search(I2C_HandleTypeDef *i2cHandler){
	while(ACC_activate(&i2cHandler) != HAL_OK){
		adressCounter = (adressCounter < 4) ? adressCounter + 1 : 0;
		ACC_setAdress(adress[adressCounter]);
		i2c_status = ACC_activate(&i2cHandler);
		HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		HAL_Delay(10);
	}
	return adress[adressCounter];
}*/
