/* mbed Microcontroller Library
 * Copyright (c) 2006-2014 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __GPS_PROVIDER_INSTANCE_BASE_H__
#define __GPS_PROVIDER_INSTANCE_BASE_H__

#include <stdint.h>

#include "GPSProviderCommon.h"
#include "GPSProvider.h"
#include "GPSGeofence.h"
#include "GPSDatalog.h"

class GPSProviderImplBase {
public:
    virtual bool setPowerMode(GPSProvider::PowerMode_t power) = 0;
    virtual void reset(void)   = 0;
    virtual void start(void)   = 0;
    virtual void stop(void)    = 0;
    virtual void process(void) = 0;
    virtual void lpmGetImmediateLocation(void) = 0;
    virtual uint32_t ioctl(uint32_t command, void *arg) = 0;

    /** [ST-GNSS ] - Enable verbose NMEA stream */
    virtual void setVerboseMode(int level);

    /** [ST-GNSS] - Geofencing API */
    virtual gps_provider_error_t enableGeofence(void) = 0;
    virtual gps_provider_error_t configGeofences(GPSGeofence *geofences[], unsigned GeofenceCount) = 0;
    virtual gps_provider_error_t geofenceReq(void) = 0;

    /** [ST-GNSS] - Datalogging API */
    virtual gps_provider_error_t enableDatalog(void) = 0;
    virtual gps_provider_error_t configDatalog(GPSDatalog *datalog) = 0;
    virtual gps_provider_error_t startDatalog(void) = 0;
    virtual gps_provider_error_t stopDatalog(void) = 0;
    virtual gps_provider_error_t eraseDatalog(void) = 0;
    virtual gps_provider_error_t logReqStatus(void) = 0;
    virtual gps_provider_error_t logReqQuery(GPSProvider::LogQueryParams_t &logReqQuery) = 0;

    /**  [ST-GNSS] - Odometer API*/
    virtual gps_provider_error_t enableOdo(void) = 0;
    virtual gps_provider_error_t startOdo(unsigned alarmDistance) = 0;
    virtual gps_provider_error_t stopOdo(void) = 0;
    virtual gps_provider_error_t resetOdo(void) = 0;

    virtual bool haveDeviceInfo(void) const {
        return (deviceInfo != NULL);
    }
    virtual const char *getDeviceInfo(void) const {
        return deviceInfo;
    }
    virtual bool locationAvailable(void) const {
        return lastLocation.valid;
    }
    virtual const GPSProvider::LocationUpdateParams_t *getLastLocation(void) const {
        return (lastLocation.valid) ? &lastLocation : NULL;
    }
    virtual void onLocationUpdate(GPSProvider::LocationUpdateCallback_t callback) {
        locationCallback = callback;
    }
    /** [ST-GNSS] - Geofencing API */
    virtual bool isGeofencingSupported(void) { 
        return false; /* Requesting action from porters: override this API if this capability is supported. */ 
    }
    void onGeofenceCfgMessage(GPSProvider::GeofenceCfgMessageCallback_t callback) {
        geofenceCfgMessageCallback = callback;
    }
    void onGeofenceStatusMessage(GPSProvider::GeofenceStatusMessageCallback_t callback) {
        geofenceStatusMessageCallback = callback;
    }
    /** [ST-GNSS] - Datalogging API */
    virtual bool isDataloggingSupported(void) { 
        return false; /* Requesting action from porters: override this API if this capability is supported. */ 
    }
    virtual void onLogStatus(GPSProvider::LogStatusCallback_t callback) {
        logStatusCallback = callback;
    }
    virtual void onLogQuery(GPSProvider::LogQueryCallback_t callback) {
        logQueryCallback = callback;
    }
    /** [ST-GNSS] - Odometer API */
    virtual bool isOdometerSupported(void) { 
        return false; /* Requesting action from porters: override this API if this capability is supported. */ 
    }
    virtual void onOdo(GPSProvider::OdoCallback_t callback) {
        odoCallback = callback;
    }
    
protected:
    GPSProvider::LocationUpdateParams_t          lastLocation;
    const char                                   *deviceInfo;
    GPSProvider::LocationUpdateCallback_t        locationCallback;

    /** [ST-GNSS] - Geofencing API */
    GPSProvider::GeofenceStatusParams_t          geofenceStatus;
    GPSProvider::GeofenceCfgMessageCallback_t    geofenceCfgMessageCallback;
    GPSProvider::GeofenceStatusMessageCallback_t geofenceStatusMessageCallback;

    /** [ST-GNSS] - Datalogging API */
    GPSProvider::LogStatusCallback_t             logStatusCallback;
    GPSProvider::LogQueryCallback_t              logQueryCallback;

    /** [ST-GNSS] - Ododmeter API */
    GPSProvider::OdoCallback_t                   odoCallback; 
};

#endif /* __GPS_PROVIDER_INSTANCE_BASE_H__ */
