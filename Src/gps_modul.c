
#include "gps_modul.h"

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

/*
weitere Funktionen, die Implementiert werden können: 
- Pausieren, um Strom zu sparen: DMA pausieren, ModulDeactivate, prüfen ob erfolgreich
- Resume: ModulActivate, wenn erfolgreich -> DMAResume

Dazu hilfreich: HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
								HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);

*/

// zwei Schleifen schachteln. äußere sucht nach "§" und gibt dann jeweilige Stringteil, also eine NMEA Nachricht weiter.
// innere sucht entsprechend der MessageID nach einem bestimmten Paramter -> Komma zählen
// oder hier identifiziert Message und speichert sie entsprechend weg
// bricht Buffersuche ab, nachdem beide gesuchten Stings gefunen wurden
/* GPS Stuct Entwurf siehe oneNote */
int GPS_sortInNewData(s_gpsSetOfData* gpsActualDataset, char* pNewNmeaString)
{
	char gpsTypFound = 0; // GPRMC (0000 0001 -> 0x1) , GPGGA (0000 0010 -> 0x2) , ((other (other)
	int16_t cursor = 0; 
	char copyindex = 0;
	char kommaZaehlerValidierung = 0;
	// fehler abfangen falls pNewNmeaString kurzer ist und kein \0 enthält
	// nach gefunden testen, ob zwischen '$' und '*' <=80 Zeichen 
	// wenn erwartete Anzahl an Komma enthalten, ist Satz richtig und wird in diesem Buffer einleseVorgang nicht mehr eingelesen
	// ist es nicht, ist er moeglicherweise korruptiert und wird beim naechsten mal eingelesen
	while((GPS_RINGBUFFER_SIZE-2) > cursor){
		if(pNewNmeaString[cursor] == '$'){
			// NMEA-Sentence Anfang gefunden an stelle cursor // wenn neuer satz durch $ gefunden. diese Var zuruecksetzen
			copyindex = 0;
			kommaZaehlerValidierung = 0;
		
			// wenn dieser Typ noch nicht gefunden wurde, gefundene NMEA-Sentence weiter danach untersuchen
			if(((gpsTypFound & 0x1) != 0)
				&& pNewNmeaString[cursor+1]=='G' && pNewNmeaString[cursor+2]=='P' 
				&& pNewNmeaString[cursor+3]=='R' && pNewNmeaString[cursor+4]=='M' 
				&& pNewNmeaString[cursor+5]=='C' && pNewNmeaString[cursor+6]==','){
					// GPRMC Sentence entdeckt
					// Datensatz von Null ab füllen, aus Eingangs-String mit offset, wo das $ beginnt ($ mitkopieren)
					while(80 > copyindex && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\0' && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '*'
							&& pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\r' && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\n')	{
						/* vom $ an werden bis * oder Stringende maximal 80 Zeichen kopiert. 
						 kommt cursor+copyindex ans Ende des Buffers, zB. Size=500, ans 500 Zeichen, also Index 499 so wird das kopiert
						danach beim index 500 greift Modulo und man kopiert vom 1. Zeichen mit index 0 (->ringbuffer)*/
						gpsActualDataset->NMEA_GPRMC[copyindex] = pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE];
						if(gpsActualDataset->NMEA_GPRMC[copyindex] == ',')kommaZaehlerValidierung++;
						copyindex++;
					}
					gpsActualDataset->NMEA_GPRMC[copyindex] = '\0'; // stringende ans Ende.  Eigentlich könnte zwischen Text und \0 ein <CR><LF> bzw \r\n wird aber formatierung in CSV
					if(kommaZaehlerValidierung == GPRMC_ANZAHL_KOMMA){
						pNewNmeaString[cursor]='X'; // overwrite $ of this sentence, so this string got not read in multiple times
						gpsTypFound |= 0x1; //erfolgreich GPRMC eingelesen
					}
				}
			// wenn dieser Typ noch nicht gefunden wurde, gefundene NMEA-Sentence weiter danach untersuchen
			else if(((gpsTypFound & 0x2) != 0)
				&& pNewNmeaString[(cursor+1)%GPS_RINGBUFFER_SIZE]=='G' && pNewNmeaString[(cursor+2)%GPS_RINGBUFFER_SIZE]=='P' 
				&& pNewNmeaString[(cursor+3)%GPS_RINGBUFFER_SIZE]=='G' && pNewNmeaString[(cursor+4)%GPS_RINGBUFFER_SIZE]=='G' 
				&& pNewNmeaString[(cursor+5)%GPS_RINGBUFFER_SIZE]=='A' && pNewNmeaString[(cursor+6)%GPS_RINGBUFFER_SIZE]==','){
					// GPGGA Sentence entdeckt
					// Datensatz von Null ab füllen, aus Eingangs-String mit offset, wo das $ beginnt ($ mitkopieren)
					while(80 > copyindex && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\0' && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '*'
								&& pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\r' && pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE] != '\n')	{
						gpsActualDataset->NMEA_GPGGA[copyindex] = pNewNmeaString[(cursor+copyindex)%GPS_RINGBUFFER_SIZE];
						if(gpsActualDataset->NMEA_GPGGA[copyindex] == ',')kommaZaehlerValidierung++;
						copyindex++;
					}
					gpsActualDataset->NMEA_GPGGA[copyindex] = '\0'; // stringende ans Ende
					if(kommaZaehlerValidierung == GPGGA_ANZAHL_KOMMA){
						pNewNmeaString[cursor]='X'; // overwrite $, so this string got not read in multiple times
						gpsTypFound |= 0x2; //erfolgreich GPGGA eingelesen
					}
				}
				else {} // nop; einen anderen Sentences gefunden, buffer enthält aber mehrere also weitersuchen
		} // ende if der '$' suche
		// NMEA Senstence Suche abgeschlossen
		// TODO: read in new timestamp (if newer) 
		//if(gpsTypFound & 0x01){ gpsActualDataset;	} //new RMC found read in new Time in first datafield after first comma (hhmmss)
			// and also date 9th datefield after 9th comma (ddmmyy)
		if(gpsTypFound==0x3) return 0; // abbruch Erfolgreich wenn beide NMEA mindestens einmal gefunden wurden
		cursor++;
	} // ende while der '$' suche
	
	// string enthielt nicht beide Sätze vollständig und Buffer ist komplett durchsucht worden
	// TODO: this is for debugging, delete this later to preserve last NMEA String and dont overwrite it with this ****
	/*if(gpsTypFound==0x01) strcpy(gpsActualDataset->NMEA_GPGGA, "kein GPGGA empfangen * ");
	else if(gpsTypFound==0x02) strcpy(gpsActualDataset->NMEA_GPRMC, "kein GPRMC empfangen * ");
	*/
	return gpsTypFound-1; // so its always >0 only if no used NMEA found then its -1 (error"")
	
	
// so kann man schauen, ob in dem Datensatz schon was steht oder noch leer
//gpsActualDataset->NMEA_GPGGA[0] != '$'..... strcmp(
}

int GPS_getVelocity(s_gpsSetOfData* gpsActualDataset) 
{
// als INT in ZentiMeter pro Sekunde ( GGG.G -> int
	const float knotsToCentimeterPerSecond = 100*0.514;
	char cursor=0, i=0;
	char commaCounter=0;
	char velocityString[6]; // xxx.x\0 =6
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
				if(i>=6)return -1; // sollte nicht so weit gehen->Fehler
			}
			velocityString[i]='\0'; // an 6. Stelle, i=5
			
			return (int)(knotsToCentimeterPerSecond * myParseFloatNumber(velocityString, &anzahlStellenGenutzt));
		}
		cursor++;
	}
	return -1; //error
}
	
int GPS_getMovedDistance(s_gpsSetOfData* gpsActualDataset); // in Meter
