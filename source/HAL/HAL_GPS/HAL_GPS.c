/*
 * HAL_GPS.c
 *
 *  Created on: 25 FEB 2023
 *      Author: morgan.venandy
 */

/*--------------------------------------------------------------------------*/
/*! ... INCLUDES ...                                                        */
/*--------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "HAL_GPS.h"
#include "HAL_UART.h"

/*--------------------------------------------------------------------------*/
/* ... DATATYPES ...                                                        */
/*--------------------------------------------------------------------------*/
Coordinates _tLatitude;
Coordinates _tLongitude;

uint8_t _uMinutesGpsAcquisition;
uint8_t _uHoursGpsAcquisition;
uint8_t _uMonthsGpsAcquisition;
uint8_t _uDaysGpsAcquisition;
/*--------------------------------------------------------------------------*/
/*! ... LOCAL FUNCTIONS DECLARATIONS ...                                    */
/*--------------------------------------------------------------------------*/
void HAL_GPS_initBufferNmea(BufferNmea*);
void HAL_GPS_initDataRmc(DataNmea_RMC*);
uint8_t HAL_GPS_decodeNmeaBuffer(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_decodeNmeaRmc(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_getNmeaUart(BufferNmea*, DataNmea_RMC*);
uint8_t HAL_GPS_getNmeaChecksum(char*);
uint8_t HAL_GPS_getNmeaBuffer(BufferNmea*, char);
void HAL_GPS_decodeNmeaForMaster(DataNmea_RMC*);
void HAL_GPS_rmcUtcTime(DataNmea_RMC*);
void HAL_GPS_rmcDate(DataNmea_RMC*);
void HAL_GPS_rmcLatLong(DataNmea_RMC*);
/*--------------------------------------------------------------------------*/
/*! ... FUNCTIONS DEFINITIONS    ...                                        */
/*--------------------------------------------------------------------------*/
void HAL_GPS_Init() 
{
    _uMinutesGpsAcquisition = 0;
	_uHoursGpsAcquisition = 0;
	_uMonthsGpsAcquisition = 0;
	_uDaysGpsAcquisition = 0;

    _tLatitude.degrees = 0;
    _tLatitude.minutes = 0;
    _tLatitude.decimalMSB = 0;
    _tLatitude.decimalB = 0;
    _tLatitude.decimalLSB = 0;

    _tLongitude.degrees = 0;
    _tLongitude.minutes = 0;
    _tLongitude.decimalMSB = 0;
    _tLongitude.decimalB = 0;
    _tLongitude.decimalLSB = 0;
}
void HAL_GPS_startGpsAcquisition() {
	uint8_t _bDecodeNmea = 0;

    BufferNmea _pBuffer;
    DataNmea_RMC _pNmeaRmc;
    
    HAL_GPS_initBufferNmea(&_pBuffer);
	HAL_GPS_initDataRmc(&_pNmeaRmc);
	
	_bDecodeNmea = HAL_GPS_getNmeaUart(&_pBuffer, &_pNmeaRmc);
	
	if(_bDecodeNmea) {
		HAL_GPS_decodeNmeaForMaster(&_pNmeaRmc);
	}
}

void HAL_GPS_initBufferNmea(BufferNmea *pBuffer) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        pBuffer->data[i] = 0;
    }
    pBuffer->indice = 0;
    pBuffer->nmea = 0;
}

void HAL_GPS_initDataRmc(DataNmea_RMC *pNmeaRmc) {
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->utcTime); i++) {
        pNmeaRmc->utcTime[i] = 0;
        pNmeaRmc->utcDate[i] = 0;
    }
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->latitude); i++) {
        pNmeaRmc->latitude[i] = 0;
        pNmeaRmc->longitude[i] = 0;
    }
    for (unsigned int i = 0; i<sizeof(pNmeaRmc->speed); i++) {
        pNmeaRmc->speed[i] = 0;
        pNmeaRmc->cap[i] = 0;
        pNmeaRmc->declMagn[i] = 0;
    }
    pNmeaRmc->latitudeDir = 0;
    pNmeaRmc->longitudeDir = 0;
    pNmeaRmc->declMagnDir = 0;
}

uint8_t HAL_GPS_getNmeaUart(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bTrameNmeaUart = 0;
	uint8_t _bTrameNmeaBuffer = 0;
	uint8_t _bDecodeNmeaBuffer = 0;
	uint8_t _cUartRxCounter = 0;
    uint8_t u8_returnValueUART = 0;
	char _tUartRxBuffer[BUFFER_SIZE] = { 0 };
    char _tBuffer[BUFFER_SIZE] = { 0 };
	
    u8_returnValueUART = HAL_UART_ReceptionGPS(_tUartRxBuffer, BUFFER_SIZE);

    if (u8_returnValueUART)
    {
        _tBuffer[0] = '$';

        for(_cUartRxCounter = 0; _cUartRxCounter < (BUFFER_SIZE - 1); _cUartRxCounter++) 
        {
            _tBuffer[_cUartRxCounter + 1] = _tUartRxBuffer[_cUartRxCounter];

		    if(_tBuffer[_cUartRxCounter] == '\n') 
            {
			    _bTrameNmeaUart = 1;
			    _cUartRxCounter = 0;
			    break;
		    }
	    }
    }
	
	if(_bTrameNmeaUart) 
    {
		while(_tBuffer[_cUartRxCounter] != '\n') 
        {
			_bTrameNmeaBuffer = HAL_GPS_getNmeaBuffer(pBuffer, _tBuffer[_cUartRxCounter]);
			_cUartRxCounter++;
		}
        if(_bTrameNmeaBuffer) 
        {
            _bTrameNmeaBuffer = HAL_GPS_getNmeaBuffer(pBuffer, _tBuffer[_cUartRxCounter]);
            if(_bTrameNmeaBuffer) {
                _bDecodeNmeaBuffer = HAL_GPS_decodeNmeaBuffer(pBuffer, pNmeaRmc);
            }
        }
	}

	return _bDecodeNmeaBuffer;
}

uint8_t HAL_GPS_decodeNmeaBuffer(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bDecodeRmc = 0;
	char *ptr = &pBuffer->data[3];
	
	if(!(strncmp(ptr, "RMC", 3))) {
		_bDecodeRmc = HAL_GPS_decodeNmeaRmc(pBuffer, pNmeaRmc);
	}
	
	return _bDecodeRmc;
}

uint8_t HAL_GPS_decodeNmeaRmc(BufferNmea *pBuffer, DataNmea_RMC *pNmeaRmc) {
	uint8_t _bDecodeRmc = 0;
	uint8_t _cDataBufferCounter = 0;
	uint8_t _cDataFieldCounter = 0;
	
	pBuffer->nmea = RMC_MESSAGE;
	
	while((pBuffer->data[_cDataBufferCounter] != '*') || (_cDataBufferCounter >= BUFFER_SIZE)) {
		switch(pBuffer->nmea) {
			case RMC_MESSAGE:
				if(pBuffer->data[_cDataBufferCounter] == ',') {
					pBuffer->nmea = RMC_UTC_TIME;
				}
				break;
				
			case RMC_UTC_TIME:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->utcTime[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_STATUS;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_STATUS:
				if(pBuffer->data[_cDataBufferCounter] == ',') {
					pBuffer->nmea = RMC_LAT;
				}
				break;
					
			case RMC_LAT:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->latitude[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_LAT_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_LAT_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->latitudeDir = pBuffer->data[_cDataBufferCounter];
				}
				else {
					pBuffer->nmea = RMC_LONG;
				}
				break;
				
			case RMC_LONG:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->longitude[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_LONG_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_LONG_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->longitudeDir = pBuffer->data[_cDataBufferCounter];
				}
				else {
					pBuffer->nmea = RMC_SPEED;
				}
				break;
				
			case RMC_SPEED:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->speed[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_CAP;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_CAP:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->cap[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_UTC_DATE;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_UTC_DATE:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->utcDate[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_DECL_MAGN;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_DECL_MAGN:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->declMagn[_cDataFieldCounter] = pBuffer->data[_cDataBufferCounter];
					_cDataFieldCounter++;
				}
				else {
					pBuffer->nmea = RMC_DECL_MAGN_DIR;
					_cDataFieldCounter = 0;
				}
				break;
				
			case RMC_DECL_MAGN_DIR:
				if(pBuffer->data[_cDataBufferCounter] != ',') {
					pNmeaRmc->declMagnDir = pBuffer->data[_cDataBufferCounter];
					pBuffer->nmea = RMC_MODE;
					_bDecodeRmc = 1;
				}
				break;
				
			case RMC_MODE:
				break;
				
			default:
				pBuffer->nmea = RMC_MESSAGE;
				_cDataBufferCounter = 0;
				_cDataFieldCounter = 0;
		}
		_cDataBufferCounter++;
	}
	
	return _bDecodeRmc;
}

uint8_t HAL_GPS_getNmeaChecksum(char *dataChecksum) {
    char checksum = 0;
    char values[3];
    uint8_t flagStartData = 0;
    uint8_t indiceStartChecksum = 0;
    
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if ((dataChecksum[i] == '$' || flagStartData) && dataChecksum[i] != '*') {
            if (dataChecksum[i] == '$') {
                i++;
            }
            flagStartData = 1;
            checksum ^= dataChecksum[i];
        }
        else if (dataChecksum[i] == '*') {
            flagStartData = 0;
            indiceStartChecksum = i;
            i = BUFFER_SIZE;
        }
    }
    
    sprintf(values, "%X", checksum);
    if (values[0] == dataChecksum[indiceStartChecksum+1] && values[1] == dataChecksum[indiceStartChecksum+2]) {
        return 1;
    }
    else
        return 0;
}

uint8_t HAL_GPS_getNmeaBuffer(BufferNmea *pBuffer, char byte) {
    uint8_t packetNmea = 0;
    
    if (byte == '$') {
        pBuffer->indice = 0;
        pBuffer->nmea = NMEA_START;
    }
    
    switch (pBuffer->nmea) {
        case NMEA_START:
            if (byte == '$') {
                pBuffer->data[0] = '$';
                pBuffer->nmea = NMEA_G;
            }
            break;
            
        case NMEA_G:
            if (byte == 'G') {
                pBuffer->data[1] = 'G';
                pBuffer->nmea = NMEA_P_A_L_N;
            }
            else {
                pBuffer->nmea = NMEA_START;
            }
            break;
        
        case NMEA_P_A_L_N:
            if (byte == 'P' || byte == 'A' || byte == 'L' || byte == 'N') {
                pBuffer->data[2] = byte;
                pBuffer->indice = 3;
                pBuffer->nmea = NMEA_DATA;
            }
            else {
                pBuffer->nmea = NMEA_START;
            }
            break;
        
        case NMEA_DATA:
            if (byte == '\r') {
                pBuffer->nmea = NMEA_END;
                packetNmea = 1;
            }
            else {
                pBuffer->data[pBuffer->indice++] = byte;
            }
            break;
            
        case NMEA_END:
            if (byte == '\n') {
                pBuffer->data[pBuffer->indice] = 0;
                packetNmea = HAL_GPS_getNmeaChecksum(pBuffer->data);
            }
            break;
            
        default:
            pBuffer->indice = 0;
            pBuffer->nmea = NMEA_START;
            break;
    }
    
    if (pBuffer->indice >= BUFFER_SIZE) {
        pBuffer->indice = 0;
        pBuffer->nmea = NMEA_START;
    }
    
    return packetNmea;
}

void HAL_GPS_decodeNmeaForMaster(DataNmea_RMC *pNmeaRmc) {
	
	HAL_GPS_rmcUtcTime(pNmeaRmc);
	HAL_GPS_rmcDate(pNmeaRmc);
	HAL_GPS_rmcLatLong(pNmeaRmc);
	
}

void HAL_GPS_rmcUtcTime(DataNmea_RMC *pNmeaRmc){
    char tabTemp[3] = {0,0,0};
    
    tabTemp[0] = pNmeaRmc->utcTime[0];
    tabTemp[1] = pNmeaRmc->utcTime[1];
    _uHoursGpsAcquisition = (atoi(tabTemp));
    
    tabTemp[0] = pNmeaRmc->utcTime[2];
    tabTemp[1] = pNmeaRmc->utcTime[3];
    _uMinutesGpsAcquisition = (atoi(tabTemp));
}

void HAL_GPS_rmcDate(DataNmea_RMC *pNmeaRmc){
    char tabTemp[3] = {0,0,0};

    tabTemp[0] = pNmeaRmc->utcDate[0];
    tabTemp[1] = pNmeaRmc->utcDate[1];
    _uDaysGpsAcquisition = (atoi(tabTemp));

    tabTemp[0] = pNmeaRmc->utcDate[2];
    tabTemp[1] = pNmeaRmc->utcDate[3];
    _uMonthsGpsAcquisition = (atoi(tabTemp));
}

void HAL_GPS_rmcLatLong(DataNmea_RMC *pNmeaRmc) {
    /*
     * Format lat ddmm.mmmmm -> dd uint8_t ; mm uint8_t ; mmmmm uint32_t  (uint32)MSB << 16 | (uint32)MiSB << 8 | (uint32)LSB
     * Format long dddmm.mmmmm -> ddd uint8_t ; mm uint8_t ; mmmmm uint32_t (uint32)MSB << 16 | (uint32)MiSB << 8 | (uint32)LSB
     */
	char latitudeDegrees[3] = { 0 };
    char latitudeMinutes[3] = { 0 };
    char latitudeDecimal[6] = { 0 };
    char longitudeDegrees[4] = { 0 };
    char longitudeMinutes[3] = { 0 };
    char longitudeDecimal[6] = { 0 };
    uint32_t decimalTemp;
	
    latitudeDegrees[0] = pNmeaRmc->latitude[0];
    latitudeDegrees[1] = pNmeaRmc->latitude[1];
    _tLatitude.degrees = (uint8_t)(atoi(latitudeDegrees));
    
    latitudeMinutes[0] = pNmeaRmc->latitude[2];
    latitudeMinutes[1] = pNmeaRmc->latitude[3];
    _tLatitude.minutes = (uint8_t)(atoi(latitudeMinutes));
    
    latitudeDecimal[0] = pNmeaRmc->latitude[5];
    latitudeDecimal[1] = pNmeaRmc->latitude[6];
    latitudeDecimal[2] = pNmeaRmc->latitude[7];
    latitudeDecimal[3] = pNmeaRmc->latitude[8];
    latitudeDecimal[4] = pNmeaRmc->latitude[9];
    decimalTemp = (uint32_t)(atoi(latitudeDecimal));
    _tLatitude.decimalMSB = (uint8_t)(decimalTemp >> 16);
    _tLatitude.decimalB = (uint8_t)(decimalTemp >> 8);
    _tLatitude.decimalLSB = (uint8_t)(decimalTemp);
    
    longitudeDegrees[0] = pNmeaRmc->longitude[0];
    longitudeDegrees[1] = pNmeaRmc->longitude[1];
    longitudeDegrees[2] = pNmeaRmc->longitude[2];
    _tLongitude.degrees = (uint8_t)(atoi(longitudeDegrees));
    
    longitudeMinutes[0] = pNmeaRmc->longitude[3];
    longitudeMinutes[1] = pNmeaRmc->longitude[4];
    _tLongitude.minutes = (uint8_t)(atoi(longitudeMinutes));
    
    longitudeDecimal[0] = pNmeaRmc->longitude[6];
    longitudeDecimal[1] = pNmeaRmc->longitude[7];
    longitudeDecimal[2] = pNmeaRmc->longitude[8];
    longitudeDecimal[3] = pNmeaRmc->longitude[9];
    longitudeDecimal[4] = pNmeaRmc->longitude[10];
    decimalTemp = (uint32_t)(atoi(longitudeDecimal));
    _tLongitude.decimalMSB = (uint8_t)(decimalTemp >> 16);
    _tLongitude.decimalB = (uint8_t)(decimalTemp >> 8);
    _tLongitude.decimalLSB = (uint8_t)(decimalTemp);
}
