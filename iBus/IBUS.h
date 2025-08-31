/* BEGIN Header */
/**
 ******************************************************************************
 * \file            iBus.h
 * \author          Andrea Vivani
 * \brief           FlySky iBUS (AFHDS 2A) – protocol decoder
 ******************************************************************************
 * \copyright
 *
 * Copyright 2025 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */
/* END Header */

#ifndef __IBUS_H__
#define __IBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>

/* Configuration Options -----------------------------------------------------*/

#ifndef IBUS_ENABLE_TELEMETRY
#define IBUS_ENABLE_TELEMETRY 0
#endif

#ifndef IBUS_ENABLE_STATS
#define IBUS_ENABLE_STATS 1
#endif

#ifndef IBUS_ENABLE_FRESHNESS_CHECK
#define IBUS_ENABLE_FRESHNESS_CHECK 1
#endif

#ifndef IBUS_TEL_MAX_SENSORS
#define IBUS_TEL_MAX_SENSORS 16U
#endif

#ifndef IBUS_MAX_CHANNELS
#define IBUS_MAX_CHANNELS 14U
#endif

/* iBus Protocol Constants ---------------------------------------------------*/

#define IBUS_SERVO_FRAME_LEN     32U   /** 0x20 bytes total. */
#define IBUS_SERVO_HDR0          0x20U /** Length field. */
#define IBUS_SERVO_HDR1          0x40U /** Command code. */

// Telemetry commands
#define IBUS_TEL_CMD_DISCOVER    0x80U /** Discovery request/ack. */
#define IBUS_TEL_CMD_TYPE        0x90U /** Query type/size. */
#define IBUS_TEL_CMD_MEAS        0xA0U /** Read value. */
#define IBUS_TEL_CMD_MASK        0xF0U /** High nibble is command. */
#define IBUS_TEL_ADDR_MASK       0x0FU /** Low nibble is slot (1..15). */

// Freshness tracking indices
#define IBUS_TRACKED_FRAME_TYPES 2
#define IBUS_FRAME_TYPE_RC       0
#define IBUS_FRAME_TYPE_TEL      1

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return values
 */
typedef enum {
    IBUS_OK = 0,                 /** Operation completed successfully */
    IBUS_ERROR_NULL_POINTER,     /** Null pointer provided */
    IBUS_ERROR_INVALID_FRAME,    /** Invalid packet length or format */
    IBUS_ERROR_CHECKSUM_FAIL,    /** Packet checksum verification failed */
    IBUS_ERROR_BUFFER_OVERFLOW,  /** Buffer overflow detected */
    IBUS_ERROR_SENSOR_EXISTS,    /** Requested sensor already registered */
    IBUS_ERROR_SENSOR_NOT_FOUND, /** Requested sensor not found */
    IBUS_TEL_NO_REPLY = 100,     /** No telemetry reply needed */
    IBUS_TEL_REPLY_READY         /** Telemetry reply is ready */
} iBus_Status_t;

/**
 * Sensor type struct
 */
#if IBUS_ENABLE_TELEMETRY
typedef enum {
    IBUS_MEAS_TYPE_INTERNAL_VOLTAGE = 0x00, /** decivolts (0.1V units) */
    IBUS_MEAS_TYPE_TEMPERATURE = 0x01,      /** degrees Celsius */
    IBUS_MEAS_TYPE_RPM = 0x02,              /** revolutions per minute */
    IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE = 0x03, /** decivolts (0.1V units) */
    IBUS_MEAS_TYPE_HEADING = 0x04,          /** degrees (0-359) */
    IBUS_MEAS_TYPE_CURRENT = 0x05,          /** 0.01A units */
    IBUS_MEAS_TYPE_CLIMB = 0x06,            /** m/s (vertical speed) */
    IBUS_MEAS_TYPE_ACC_Z = 0x07,            /** m/s^2 */
    IBUS_MEAS_TYPE_ACC_Y = 0x08,            /** m/s^2 */
    IBUS_MEAS_TYPE_ACC_X = 0x09,            /** m/s^2 */
    IBUS_MEAS_TYPE_VSPEED = 0x0A,           /** cm/s (some firmwares) */
    IBUS_MEAS_TYPE_SPEED = 0x0B,            /** km/h */
    IBUS_MEAS_TYPE_DIST = 0x0C,             /** meters */
    IBUS_MEAS_TYPE_ARMED = 0x0D,            /** 0 = disarmed, 1 = armed */
    IBUS_MEAS_TYPE_MODE = 0x0E,             /** flight mode ID */
    IBUS_MEAS_TYPE_PRES = 0x41,             /** hPa (pressure) */
    IBUS_MEAS_TYPE_COG = 0x80,              /** course over ground, degrees */
    IBUS_MEAS_TYPE_GPS_STATUS = 0x81,       /** bitfield: sats, fix type */
    IBUS_MEAS_TYPE_GPS_LON = 0x82,          /** 1E-7 deg (signed int32) */
    IBUS_MEAS_TYPE_GPS_LAT = 0x83,          /** 1E-7 deg (signed int32) */
    IBUS_MEAS_TYPE_ALT = 0x84,              /** meters */
    IBUS_MEAS_TYPE_HOME_DIST = 0x85,        /** meters */
    IBUS_MEAS_TYPE_GROUND_SPEED = 0x86,     /** km/h */
    IBUS_MEAS_TYPE_GPS_HEADING = 0x87,      /** degrees */
    IBUS_MEAS_TYPE_HOME_DIR = 0x88,         /** degrees */
    IBUS_MEAS_TYPE_RSSI = 0x89,             /** dBm or percent (radio dependent) */
    IBUS_MEAS_TYPE_FLIGHT_MODE = 0x8A,      /** flight mode ID string index */
    IBUS_MEAS_TYPE_GALT = 0xF9,             /** meters (GPS altitude legacy) */
    IBUS_MEAS_TYPE_GPS = 0xFD               /** compound GPS frame, multiple fields */
} iBus_SensorType_t;

#pragma pack(push, 1)

/**
 * Sensor data struct
 */
typedef struct {
    iBus_SensorType_t type; /** iBus_SensorType_t. */
    uint8_t size;           /** 2 or 4; if 0, auto-deduced by library. */
    volatile int32_t value; /** Latest value in raw iBUS units (signed). */
#if IBUS_ENABLE_FRESHNESS_CHECK
    uint32_t updated_ms; /** Timestamp of last write to value. */
#endif
} iBus_Sensor_t;

#pragma pack(pop)
#endif /* IBUS_ENABLE_TELEMETRY */

/**
 * Statistics struct
 */
#if IBUS_ENABLE_STATS
typedef struct {
    uint32_t frames_total;       /** Total frames parsed (any kind). */
    uint32_t frames_bad_crc;     /** Checksum errors. */
    uint32_t servo_frames_ok;    /** Valid servo frames parsed. */
    uint32_t polls_rx;           /** Telemetry polls received (any kind). */
    uint32_t polls_unknown_addr; /** Polls for non-registered sensors. */
} iBus_Stats_t;
#endif /* IBUS_ENABLE_STATS */

/**
 * iBus struct
 */
typedef struct {

    uint16_t channels[IBUS_MAX_CHANNELS]; /** Raw iBUS channel values, typically ~1000–2000 range (µs equivalent). */

#if IBUS_ENABLE_TELEMETRY
    iBus_Sensor_t sensors[IBUS_TEL_MAX_SENSORS];
    uint8_t sensorCount;
#endif

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
    uint32_t (*getTimestamp_ms)(void);
    uint32_t _packet_times[IBUS_TRACKED_FRAME_TYPES];
#endif
} iBus_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Initialize iBus decoder
 * 
 * \param[in]       ibus: iBus decoder
 */
void iBus_init(iBus_t* iBus);

#if IBUS_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Set timestamp callback for packet timestamping
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       getTimestamp_ms: Function pointer to get current timestamp in milliseconds
 */
void iBus_setTimestampCallback(iBus_t* iBus, uint32_t (*getTimestamp_ms)(void));
#endif

#if IBUS_ENABLE_TELEMETRY
/**
 * \brief           Register a new sensor for telemetry measurements
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       sensor: Sensor type to be registered
 * 
 * \return          IBUS_OK if sensor registered successfully
 *                  IBUS_ERROR_NULL_POINTER if ibus is NULL, 
 *                  IBUS_ERROR_BUFFER_OVERFLOW if max sensors reached,
 *                  IBUS_ERROR_SENSOR_EXISTS if sensor already registered
 */
iBus_Status_t iBus_registerSensor(iBus_t* iBus, const iBus_SensorType_t sensor);

/**
 * \brief           Update sensor value and timestamp
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       type: Sensor type to be updated
 * \param[in]       value: New sensor value in raw iBus units
 * 
 * \return          IBUS_OK if value updated successfully,
 *                  IBUS_ERROR_NULL_POINTER if ibus is NULL,
 *                  IBUS_ERROR_SENSOR_NOT_FOUND if sensor address not found
 */
static inline iBus_Status_t iBus_writeSensor(iBus_t* iBus, iBus_SensorType_t type, int32_t value) {
    if (!iBus) {
        return IBUS_ERROR_NULL_POINTER;
    }

    for (uint8_t i = 0; i < iBus->sensorCount; i++) {
        if (iBus->sensors[i].type == type) {
            iBus->sensors[i].value = value;
#if IBUS_ENABLE_FRESHNESS_CHECK
            if (iBus->getTimestamp_ms) {
                iBus->sensors[i].updated_ms = iBus->getTimestamp_ms();
            }
#endif
            return IBUS_OK;
        }
    }

    return IBUS_ERROR_SENSOR_NOT_FOUND;
}
#endif /* IBUS_ENABLE_TELEMETRY */

/**
 * \brief           Process received frame
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       frame: RF frame data
 *
 * \return          IBUS_OK if frame processed successfully,
 *                  IBUS_ERROR_NULL_POINTER if pointers are NULL,
 *                  IBUS_ERROR_INVALID_FRAME if packet format is invalid,
 *                  IBUS_ERROR_CHECKSUM_FAIL if checksum verification fails
 */
iBus_Status_t iBus_processFrame(iBus_t* iBus, const uint8_t* frame);

#if IBUS_ENABLE_TELEMETRY
/**
 * \brief           Handle telemetry packets at ISR level. Prepares reply if rx is a DISC/TYPE/MEAS poll
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       rx: Received bytes (full packet, including length and CRC)
 * \param[out]      tx: Output buffer for reply bytes
 * \param[out]      tx_len: Length to be written
 * 
 * \return          IBUS_TEL_REPLY_READY if reply is ready in tx,
 *                  IBUS_TEL_NO_REPLY if no reply needed/unknown,
 *                  IBUS_ERROR_NULL_POINTER if pointers are NULL,
 *                  IBUS_ERROR_SENSOR_NOT_FOUND if requested sensor address is not registered,
 *                  IBUS_ERROR_CHECKSUM_FAIL if checksum verification fails
 */
iBus_Status_t iBus_handleTelemetryFromISR(iBus_t* iBus, const uint8_t* rx, uint8_t* tx, uint8_t* tx_len);
#endif

#if IBUS_ENABLE_STATS
/**
 * \brief           Get statistics about the decoder
 * 
 * \param[in]       ibus: iBus decoder
 * \param[out]      stats: Output statistics structure
 */
void iBus_getStats(const iBus_t* iBus, iBus_Stats_t* stats);

/**
 * \brief           Reset statistics counters
 * 
 * \param[out]      ibus: iBus decoder
 */
void iBus_resetStats(iBus_t* iBus);
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Check if a frame type was received recently
 * 
 * \param[in]       ibus: iBus decoder
 * \param[in]       frame_type: Frame type to check (IBUS_FRAME_TYPE_RC, IBUS_FRAME_TYPE_TEL)
 * \param[in]       max_age_ms: Maximum age in milliseconds
 * 
 * \return          1 if frame is fresh, 0 if stale or never received
 */
uint8_t iBus_isFrameFresh(const iBus_t* iBus, uint8_t frame_type, uint32_t max_age_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __IBUS_H__ */
