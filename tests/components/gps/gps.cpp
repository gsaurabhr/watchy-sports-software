// This file glues together the device-independent GPSProvider implementation,
// platform-independent but device-specific driver implementation and the ESP32 platform
#include <stdio.h>
#include "gps.h"
#include "esp_err.h"

#include "GPSProviderImplBase.h"
#include "GPSProvider.h"
#include "GPSDeviceBase.h"

#ifdef CONFIG_GPS_TESEO
#include "Teseo.h"
#endif

GPSProviderImplBase *
createGPSProviderInstance(void)
{
#ifdef  CONFIG_GPS_TESEO

    static Teseo gnss();

    //TODO register all the functions

    return &gnss;

#endif
// other GPS device implementations
}