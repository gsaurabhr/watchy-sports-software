/**
 ******************************************************************************
 * @file    GPSProviderCommon.h
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

#ifndef __GPS_PROVIDER_COMMON_H__
#define __GPS_PROVIDER_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

   
enum {
  GEOFENCE_STATUS_UNKNOWN               = 0,
  GEOFENCE_STATUS_OUTSIDE_CIRCLE        = 1,
  GEOFENCE_STATUS_BOUNDARY_CIRCLE       = 2,
  GEOFENCE_STATUS_INSIDE_CIRCLE         = 3
};

/*! @brief Error codes for the GNSS API. */
enum gps_provider_error_t {
    GPS_ERROR_NONE                      = 0, /**< No error. */
    GPS_ERROR_FEATURE_ENABLING          = 1, /**< Feature enabling failed. */
    GPS_ERROR_GEOFENCE_MAX_EXCEEDED     = 2, /**< Max num of Geofence regions exceeded error. */
    GPS_ERROR_GEOFENCE_CFG              = 3, /**< Geofence regions config failed. */
    GPS_ERROR_GEOFENCE_STATUS           = 4, /**< Geofence regions status failed. */
    GPS_ERROR_GEOFENCE_NOT_IMPLEMENTED  = 5, /**< Geofence feature is not supported. */
    GPS_ERROR_DATALOG_CFG               = 6, /**< Datalog config error. */
    GPS_ERROR_DATALOG_START             = 7, /**< Datalog start error. */
    GPS_ERROR_DATALOG_STOP              = 8, /**< Datalog stop error. */
    GPS_ERROR_DATALOG_ERASE             = 9, /**< Datalog erase error. */
    GPS_ERROR_LOG_REQ_STATUS            = 10, /**< Datalog req status error. */
    GPS_ERROR_LOG_REQ_QUERY             = 11, /**< Datalog req query error. */
    GPS_ERROR_LOG_NOT_IMPLEMENTED       = 12, /**< Datalog feature is not supported. */
    GPS_ERROR_ODO_START                 = 13, /**< Odometer start error. */
    GPS_ERROR_ODO_STOP                  = 14, /**< Odometer start error. */
    GPS_ERROR_ODO_RESET                 = 15, /**< Odometer reset error. */
    GPS_ERROR_ODO_NOT_IMPLEMENTED       = 16, /**< Odometer feature is not supported. */
    GPS_ERROR_MSGLIST_CFG               = 17, /**< Msg List config error. */
    GPS_ERROR_SAVEPAR                   = 18, /**< Save parmas error. */
    GPS_ERROR_NO_MEM                    = 19
};
  
#ifdef __cplusplus
}
#endif


#endif /* __GPS_PROVIDER_COMMON_H__ */
