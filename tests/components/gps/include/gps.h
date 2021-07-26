/**
*******************************************************************************
* @file    gps.h
* @author  gsaurabhr
* @version V1.0.0
* @date    July 2021
* @brief   GPS device agnostic file for interacting with the GPS module
*
*******************************************************************************
*/
#ifndef GPS_H
#define GPS_H

#include "GPSProvider.h"
#include "GPSProviderImplBase.h"

GPSProviderImplBase * createGPSProviderInstance(void);

#endif // GPS_H