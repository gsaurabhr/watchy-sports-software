/**
 ******************************************************************************
 * @file    GPSGeofence.h
 * @author  AST/CL
 * @version V1.1.0
 * @date    Jun, 2017
 * @brief   .
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#ifndef __GPS_GEOFENCE_H__
#define __GPS_GEOFENCE_H__

class GPSGeofence {
  public:
      
      static const int NEVER_EXPIRE = -1;

      struct GeofenceCircle_t {
        int id;
        bool enabled;
        int tolerance;
        GPSProvider::LocationType_t lat;
        GPSProvider::LocationType_t lon;
        GPSProvider::LocationType_t radius;
        int status;
      };

      /** 
       * Construct a GPSGeofence instance. 
       */ 
      GPSGeofence() :
          _expirationDuration(-1),
          _notificationResponsiveness(0),
          _transitionTypes(0) {
      }
      
      /** 
       * Construct a GPSGeofence instance. 
       */ 
      GPSGeofence(const GeofenceCircle_t &geofenceCircle) :
          _geofenceCircle(geofenceCircle),
          _expirationDuration(-1),
          _notificationResponsiveness(0),
          _transitionTypes(0) {
      }
      
      void setGeofenceCircle(const GeofenceCircle_t &geofenceCircle) {
          _geofenceCircle.id = geofenceCircle.id;
          _geofenceCircle.enabled = geofenceCircle.enabled;
          _geofenceCircle.tolerance = geofenceCircle.tolerance;
          _geofenceCircle.lat = geofenceCircle.lat;
          _geofenceCircle.lon = geofenceCircle.lon;
          _geofenceCircle.radius = geofenceCircle.radius;
          _geofenceCircle.status = 0;
      }
      
      void updateGeofenceCircleStatus(int status) {
        _geofenceCircle.status = status;
      }
      
      const GeofenceCircle_t & getGeofenceCircle(void) {
        return _geofenceCircle;
      }
      
      virtual void setExpirationDuration (long durationMillis) {
        _expirationDuration = durationMillis;
      }
      
      virtual void setNotificationResponsiveness (int notificationResponsivenessMs) {
          _notificationResponsiveness = notificationResponsivenessMs;
      }
      
      virtual void setTransitionTypes(int transitionTypes) {
          _transitionTypes = transitionTypes;
      }

  protected:
      GeofenceCircle_t            _geofenceCircle;
      long                        _expirationDuration;
      int                         _notificationResponsiveness;
      int                         _transitionTypes;

  private:
      /* disallow copy constructor and assignment operators */
      GPSGeofence(const GPSGeofence&);
      GPSGeofence & operator= (const GPSGeofence&);
};

#endif /* __GPS_GEOFENCE_H__ */
