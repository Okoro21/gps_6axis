/*
 * gps.h
 *
 *  Created on: Jun 25, 2023
 *      Author: chris
 */

/*
 * I am referencing ADAfruits ultimate GPS github repo
 * Check the repo out here -> https://github.com/adafruit/Adafruit_GPS/
 */

#ifndef INC_GPS_H_
#define INC_GPS_H_

#include "stm32f7xx_hal.h"

uint8_t const MIL_HZ_100[] = "$PMTK220,10000*2F\r\n";
uint8_t const PMTK_Q_RELEASE[] = "$PMTK605*31\r\n";
uint8_t const PMTK_SET_NMEA_OUTPUT_RMCONLY[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";



#endif /* INC_GPS_H_ */
