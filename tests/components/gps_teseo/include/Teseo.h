/**
*******************************************************************************
* @file    Teseo.h
* @author  AST / Central Lab
* @version V1.0.0
* @date    May-2017
* @brief   Teseo Location Class
*
*******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
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
********************************************************************************
*/

#ifndef __TESEO_H__
#define __TESEO_H__

#include "GPSDeviceBase.h"
#include "GPSProviderImplBase.h"

#include "NMEAParser.h"

/** Indicates the outputted location information */
#define LOC_OUTPUT_LOCATION             (1)
#define LOC_OUTPUT_NMEA                 (2)
#define LOC_OUTPUT_PSTM                 (3)

/**
 * @brief Constant that indicates the maximum number of nmea messages to be processed.
 */
#define NMEA_MSGS_NUM 6 //Note: update this constant coherently to eMsg enum type

/**
 * @brief Constant that indicates the maximum number of proprietary nmea messages to be processed.
 */
#define PSTM_NMEA_MSGS_NUM 5 //Note: update this constant coherently to ePSTMsg enum type

/**
 * @brief Constant that indicates the maximum number of positions that can be stored.
 */
#define MAX_STOR_POS 64
   
/**
 * @brief Constant that indicates the lenght of the buffer that stores the GPS data read by the GPS expansion.
 */

#define TESEO_RXBUF_LEN		256//90
#define TESEO_RXQUEUE_LEN	8

/** Application register this out callback function and Teseo class will pass outputted information to application */
typedef void (*teseo_app_output_callback)(uint32_t msgId, uint32_t msgType, tDeviceData *pData);
/** Application register this event callback function and Teseo class will pass internal processing event to application */
typedef void (*teseo_app_event_callback)(eDeviceLocEventType event, uint32_t data);
/** Function to set state of wakeup pin high or low */
typedef void (*teseo_setstate_wakeup_fn)(bool high);
/** Function to get state of wakeup pin high or low */
typedef void (*teseo_getstate_wakeup_fn)();
/** Function to set state of reset pin high or low */
typedef void (*teseo_setstate_reset_fn)(bool high);
/** write command handler */
typedef void (*teseo_write_cmd)(char *cmd, int len);
/** read command handler */
typedef void (*teseo_read_msg)(char *msg, int len);
/** delay function */
typedef void (*teseo_delay_ms)(int delay);

class Teseo : public GPSProviderImplBase {
  public:

    typedef enum {
      TEST,
      GETSWVER,
      SYSTEMRESET,
      FORCESTANDBY,
      RFTESTON,
      RFTESTOFF,
      LOWPOWER
    } eCmd;

    /** NMEA messages types */
    typedef enum {
      GPGGA,
      GNS,
      GPGST,
      GPRMC,
      GSA,
      GSV,
    } eMsg;
    
    /** NMEA proprietary messages types */
    typedef enum {
      PSTMGEOFENCE,
      PSTMODO,
      PSTMDATALOG,
      PSTMSGL,
      PSTMSAVEPAR
    } ePSTMsg;

  private:

    eDeviceLocState _locState;
          
    tDeviceData pData;
    GPGGA_Infos stored_positions[MAX_STOR_POS];

    int FwWaitAck();

    /**
     * Command string
     */
    char _teseoCmd[TESEO_RXBUF_LEN];

    /**
     * Message struct
     */
    struct _teseoMsg {
      uint8_t len;
      uint8_t buf[TESEO_RXBUF_LEN];
    };
  
  public:
    
    /** Constructor: Teseo
     * Create the Teseo, accept specified configuration
     */
    Teseo();
    
    /** Register output callback and event callback functions
     * @param app_output_cb Teseo class output the location and satellite information to application
     * @param app_event_cb Teseo class output the start and stop result to application
     */
    void TeseoLocRegOutput(teseo_app_output_callback app_output_cb, teseo_app_event_callback app_event_cb);
    void TeseoSetRWfunctions(teseo_write_cmd teseo_writer, teseo_read_msg teseo_reader);
    void TeseoSetPinFunctions(teseo_setstate_wakeup_fn wakeup_set, teseo_getstate_wakeup_fn wakeup_get, teseo_setstate_reset_fn reset_set);
    void TeseoSetDelayFunction(teseo_delay_ms teseo_delay);
    
    void SendCommand(Teseo::eCmd c);
    void SendCommand(char *cmd);
    
    int EnableLowPower();
      
    eStatus WakeStatus(void){
      return DEVICE_STATUS_SUCCESS;
      // return wakeup_getter() ? DEVICE_STATUS_SUCCESS : DEVICE_STATUS_FAILURE;
    }

  private:
    
    virtual bool setPowerMode(GPSProvider::PowerMode_t pwrMode);
    virtual void start(void);
    virtual void stop(void);
    virtual void process(void);
    virtual uint32_t ioctl(uint32_t command, void *arg);
    virtual void lpmGetImmediateLocation(void);
    virtual void reset(void);
    virtual const GPSProvider::LocationUpdateParams_t *getLastLocation(void) const;

    gps_provider_error_t cfgMessageList(int level);
    gps_provider_error_t saveConfigParams(void);
    
    /** Set NMEA stream verbosity */
    virtual void setVerboseMode(int level);

    /** Geofencing */
    virtual bool isGeofencingSupported(void);
    virtual gps_provider_error_t enableGeofence(void);
    virtual gps_provider_error_t configGeofences(GPSGeofence *geofences[], unsigned geofenceCount);
    virtual gps_provider_error_t geofenceReq(void);
    gps_provider_error_t cfgGeofenceCircle(void);

    /** Datalogging */
    virtual bool isDataloggingSupported(void);
    virtual gps_provider_error_t enableDatalog(void);
    virtual gps_provider_error_t configDatalog(GPSDatalog *datalog);
    virtual gps_provider_error_t startDatalog(void);
    virtual gps_provider_error_t stopDatalog(void);
    virtual gps_provider_error_t eraseDatalog(void);
    virtual gps_provider_error_t logReqStatus(void);
    virtual gps_provider_error_t logReqQuery(GPSProvider::LogQueryParams_t &logReqQuery);
    
    /* Odometer */
    virtual bool isOdometerSupported(void);
    virtual gps_provider_error_t enableOdo(void);
    virtual gps_provider_error_t startOdo(unsigned alarmDistance);
    virtual gps_provider_error_t stopOdo(void);
    virtual gps_provider_error_t resetOdo(void);

    void _ResetFast();
    void _Reset();
    void _SendString(char *buf, int len);
    int _WakeUp();
    int16_t _CRC(char *buf, int size);
    /**
     * @brief  This function gets a chunck of NMEA messages
     * @param  msg NMEA message to search for
     * @retval eStatus DEVICE_STATUS_SUCCESS if the parse process goes ok, DEVICE_FAILURE if it doesn't
     */
    eStatus _GetMsg(Teseo::eMsg msg, uint8_t *buffer);
    /**
     * @brief  This function gets a chunck of PSTM NMEA messages
     * @param  msg PSTM NMEA message to search for
     * @retval eStatus DEVICE_STATUS_SUCCESS if the parse process goes ok, DEVICE_FAILURE if it doesn't
     */
    void _GetPSTMsg(Teseo::ePSTMsg msg, uint8_t *buffer);
    void _GetLocationMsg(Teseo::eMsg msg, uint8_t *buffer);
    
    // void _LocLed2Set(void){
    //   _loc_led2.write(1);
    // }
    // void _LocLed2Reset(void){
    //   _loc_led2.write(0);
    // }
    
    void outputHandler(uint32_t msgId, uint32_t msgType, tDeviceData *pData);
    void eventHandler(eDeviceLocEventType event, uint32_t data);
    
    teseo_app_output_callback appOutCb;
    teseo_app_event_callback appEventCb;
    teseo_setstate_wakeup_fn wakeup_setter;
    teseo_getstate_wakeup_fn wakeup_getter;
    teseo_setstate_reset_fn reset_setter;
    teseo_write_cmd teseo_writer;
    teseo_read_msg teseo_reader;
    teseo_delay_ms teseo_delay;
    
    // MemoryPool<struct _teseoMsg, TESEO_RXQUEUE_LEN> mpool;
    // Queue<struct _teseoMsg, TESEO_RXQUEUE_LEN> queue;
};

#endif /*__TESEO_H__*/
