#ifndef __GPS_DEVICE_H__
#define __GPS_DEVICE_H__

/**
 * @brief Enumeration structure that containes the two success states of a process
 */
typedef enum {
  DEVICE_STATUS_SUCCESS = 0, /**< Success status */
  DEVICE_STATUS_FAILURE = 1  /**< Failure status */
} eStatus;

/** Location event definitions */
typedef enum {
    /** Start result event */
    DEVICE_LOC_EVENT_START_RESULT,
    /** Stop result event */
    DEVICE_LOC_EVENT_STOP_RESULT,
} eDeviceLocEventType;

/** Device Location state */
typedef enum {
    DEVICE_LOC_STATE_IDLE,
    DEVICE_LOC_STATE_RUN,
    DEVICE_LOC_STATE_FEATURE,
    DEVICE_LOC_STATE_DEBUG
} eDeviceLocState;

/**
 * @brief Enumeration structure that containes the two states of a debug process
 */
typedef enum {
  DEBUG_OFF = 0, /**< In this case, nothing will be printed on the console (nmea strings, positions and so on) */
  DEBUG_ON = 1   /**< In this case, nmea strings and just acquired positions will be printed on the console */
} eDebugState;

#endif//__GPS_DEVICE_H__