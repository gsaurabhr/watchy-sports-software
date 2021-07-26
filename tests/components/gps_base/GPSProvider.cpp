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

#include <stdint.h>
#include <stdlib.h>

#include "GPSProviderImplBase.h"
#include "GPSProvider.h"

bool GPSProvider::setPowerMode(PowerMode_t power)
{
    return impl->setPowerMode(power);
}

void GPSProvider::reset(void)
{
    impl->reset();
}

void
GPSProvider::start(void)
{
    impl->start();
}

void
GPSProvider::stop(void)
{
    impl->stop();
}

void
GPSProvider::process(void)
{
    impl->process();
}

bool
GPSProvider::haveDeviceInfo(void) const
{
    return impl->haveDeviceInfo();
}

const char *
GPSProvider::getDeviceInfo(void) const
{
    return impl->getDeviceInfo();
}

uint32_t
GPSProvider::ioctl(uint32_t command, void *arg)
{
    return impl->ioctl(command, arg);
}

bool
GPSProvider::locationAvailable(void) const
{
    return impl->locationAvailable();
}

const GPSProvider::LocationUpdateParams_t *
GPSProvider::getLastLocation(void) const
{
    return impl->getLastLocation();
}

void
GPSProvider::onLocationUpdate(LocationUpdateCallback_t callback)
{
    impl->onLocationUpdate(callback);
}

void
GPSProvider::lpmGetImmediateLocation(void)
{
    impl->lpmGetImmediateLocation();
}

/** [ST-GNSS ] - Enable verbose NMEA stream */
void
GPSProvider::setVerboseMode(int level)
{
  impl->setVerboseMode(level);
}

/** [ST-GNSS] - Geofencing API */
bool
GPSProvider::isGeofencingSupported(void)
{
    return impl->isGeofencingSupported();
}

/** [ST-GNSS] - Geofencing API */
gps_provider_error_t
GPSProvider::enableGeofence(void)
{
  return impl->enableGeofence();
}

/** [ST-GNSS] - Geofencing API */
gps_provider_error_t
GPSProvider::configGeofences(GPSGeofence *geofences[], unsigned geofenceCount)
{  
  return impl->configGeofences(geofences, geofenceCount);
}

/** [ST-GNSS] - Geofencing API */
gps_provider_error_t
GPSProvider::geofenceReq(void)
{  
  return impl->geofenceReq();
}

/** [ST-GNSS] - Geofencing API */
void
GPSProvider::onGeofenceCfgMessage(GeofenceCfgMessageCallback_t callback)
{
  impl->onGeofenceCfgMessage(callback);
}

/** [ST-GNSS] - Geofencing API */
void
GPSProvider::onGeofenceStatusMessage(GeofenceStatusMessageCallback_t callback)
{
  impl->onGeofenceStatusMessage(callback);
}

/** [ST-GNSS] - Datalogging API */
bool
GPSProvider::isDataloggingSupported(void)
{
    return impl->isDataloggingSupported();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::enableDatalog(void)
{
  return impl->enableDatalog();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::configDatalog(GPSDatalog *datalog)
{
  return impl->configDatalog(datalog);
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::startDatalog(void)
{
  return impl->startDatalog();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::stopDatalog(void)
{
  return impl->stopDatalog();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::eraseDatalog(void)
{
  return impl->eraseDatalog();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::logReqStatus(void)
{
  return impl->logReqStatus();
}

/** [ST-GNSS] - Datalogging API */
gps_provider_error_t
GPSProvider::logReqQuery(LogQueryParams_t &logReqQuery)
{
  return impl->logReqQuery(logReqQuery);
}

/** [ST-GNSS] - Datalogging API */
void
GPSProvider::onLogStatus(LogStatusCallback_t callback)
{
  impl->onLogStatus(callback);
}

/** [ST-GNSS] - Datalogging API */
void
GPSProvider::onLogQuery(LogQueryCallback_t callback)
{
  impl->onLogQuery(callback);
}

/** [ST-GNSS] - Odometer API */
bool
GPSProvider::isOdometerSupported(void)
{
  return impl->isOdometerSupported();
}

/** [ST-GNSS] - Odometer API */
gps_provider_error_t
GPSProvider::enableOdo(void)
{
  return impl->enableOdo();
}

/**  [ST-GNSS] - Odometer API */
gps_provider_error_t
GPSProvider::startOdo(unsigned alarmDistance)
{
  return impl->startOdo(alarmDistance);
}

/** [ST-GNSS] - Odometer API */
gps_provider_error_t
GPSProvider::stopOdo(void)
{
  return impl->stopOdo();
}

/** [ST-GNSS] - Odometer API */
gps_provider_error_t
GPSProvider::resetOdo(void)
{
  return impl->resetOdo();
}

/** [ST-GNSS] - Odometer API */
void
GPSProvider::onOdo(OdoCallback_t callback)
{
  impl->onOdo(callback);
}