/* BEGIN Header */
/**
 ******************************************************************************
 * \file            CRSF.h
 * \author          Andrea Vivani
 * \brief           CRSF protocol decoder/encoder
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

#ifndef __CRSF_H__
#define __CRSF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "CRSF_types.h"

/* Configuration Options -----------------------------------------------------*/

// Configuration validation - ensure at least one mode is selected
#if !defined(CRSF_CONFIG_RX) && !defined(CRSF_CONFIG_TX)
#define CRSF_CONFIG_RX
#endif

#ifdef CRSF_CONFIG_TX
#undef CRSF_ENABLE_FRESHNESS_CHECK
#define CRSF_ENABLE_FRESHNESS_CHECK 0
#endif

#ifndef CRSF_ENABLE_STATS
#define CRSF_ENABLE_STATS 0
#endif

#ifndef CRSF_ENABLE_FRESHNESS_CHECK
#define CRSF_ENABLE_FRESHNESS_CHECK 1
#endif

#ifndef CRSF_ENABLE_ADDRESS_VALIDATION
#define CRSF_ENABLE_ADDRESS_VALIDATION 1
#endif

#ifndef CRSF_USE_CRC_CALCULATION
#define CRSF_USE_CRC_CALCULATION 0 /* Use CRC calculation instead of lookup tables (saves flash). */
#endif

#ifndef CRSF_USE_RC_DIRECT_CONVERSION
#define CRSF_USE_RC_DIRECT_CONVERSION 0
#endif

#ifndef CRSF_TEL_ENABLE_GPS
#define CRSF_TEL_ENABLE_GPS 1
#endif
#ifndef CRSF_TEL_ENABLE_GPS_TIME
#define CRSF_TEL_ENABLE_GPS_TIME 1
#endif
#ifndef CRSF_TEL_ENABLE_GPS_EXTENDED
#define CRSF_TEL_ENABLE_GPS_EXTENDED 1
#endif
#ifndef CRSF_TEL_ENABLE_VARIO
#define CRSF_TEL_ENABLE_VARIO 1
#endif
#ifndef CRSF_TEL_ENABLE_BATTERY_SENSOR
#define CRSF_TEL_ENABLE_BATTERY_SENSOR 1
#endif
#ifndef CRSF_TEL_ENABLE_BAROALT_VSPEED
#define CRSF_TEL_ENABLE_BAROALT_VSPEED 1
#endif
#ifndef CRSF_TEL_ENABLE_AIRSPEED
#define CRSF_TEL_ENABLE_AIRSPEED 1
#endif
#ifndef CRSF_TEL_ENABLE_HEARTBEAT
#define CRSF_TEL_ENABLE_HEARTBEAT 1
#endif
#ifndef CRSF_TEL_ENABLE_RPM
#define CRSF_TEL_ENABLE_RPM 1
#endif
#ifndef CRSF_TEL_ENABLE_TEMPERATURE
#define CRSF_TEL_ENABLE_TEMPERATURE 1
#endif
#ifndef CRSF_TEL_ENABLE_VOLTAGES
#define CRSF_TEL_ENABLE_VOLTAGES 1
#endif
#ifndef CRSF_TEL_ENABLE_VTX
#define CRSF_TEL_ENABLE_VTX 1
#endif
#ifndef CRSF_TEL_ENABLE_LINK_STATISTICS
#define CRSF_TEL_ENABLE_LINK_STATISTICS 1
#endif
#ifndef CRSF_ENABLE_RC_CHANNELS
#define CRSF_ENABLE_RC_CHANNELS 1
#endif
#ifndef CRSF_TEL_ENABLE_LINK_STATISTICS_RX
#define CRSF_TEL_ENABLE_LINK_STATISTICS_RX 1
#endif
#ifndef CRSF_TEL_ENABLE_LINK_STATISTICS_TX
#define CRSF_TEL_ENABLE_LINK_STATISTICS_TX 1
#endif
#ifndef CRSF_TEL_ENABLE_ATTITUDE
#define CRSF_TEL_ENABLE_ATTITUDE 1
#endif
#ifndef CRSF_TEL_ENABLE_MAVLINK_FC
#define CRSF_TEL_ENABLE_MAVLINK_FC 1
#endif
#ifndef CRSF_TEL_ENABLE_FLIGHT_MODE
#define CRSF_TEL_ENABLE_FLIGHT_MODE 1
#endif
#ifndef CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
#define CRSF_TEL_ENABLE_ESP_NOW_MESSAGES 1
#endif
#ifndef CRSF_TEL_ENABLE_PARAMETER_GROUP
#define CRSF_TEL_ENABLE_PARAMETER_GROUP 1
#endif
#ifndef CRSF_ENABLE_COMMAND
#define CRSF_ENABLE_COMMAND 1
#endif
#ifndef CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
#define CRSF_TEL_ENABLE_MAVLINK_ENVELOPE 1
#endif
#ifndef CRSF_TEL_ENABLE_MAVLINK_STATUS
#define CRSF_TEL_ENABLE_MAVLINK_STATUS 1
#endif

/* Status codes ---------------------------------------------------------------*/
typedef enum {
    CRSF_OK = 0,                  /** Operation completed successfully */
    CRSF_ERROR_NULL_POINTER,      /** Null pointer provided */
    CRSF_ERROR_INVALID_FRAME,     /** Invalid frame type */
    CRSF_ERROR_CHECKSUM_FAIL,     /** Frame checksum verification failed */
    CRSF_ERROR_CMD_CHECKSUM_FAIL, /** Command checksum verification failed */
    CRSF_ERROR_LENGTH,            /** Received frame length error */
    CRSF_ERROR_TYPE_LENGTH,       /** Frame length invalid for type */
    CRSF_ERROR_ADDR,              /** Invalid address */
} CRSF_Status_t;

/* Tracked packet classes -----------------------------------------------------*/
#define CRSF_TRACKED_FRAME_TYPES                    16

#define CRSF_TRK_FRAMETYPE_GPS                      0
#define CRSF_TRK_FRAMETYPE_GPS_TIME                 1
#define CRSF_TRK_FRAMETYPE_GPS_EXTENDED             2
#define CRSF_TRK_FRAMETYPE_VARIO                    3
#define CRSF_TRK_FRAMETYPE_BATTERY_SENSOR           4
#define CRSF_TRK_FRAMETYPE_BAROALT_VSPEED           5
#define CRSF_TRK_FRAMETYPE_AIRSPEED                 6
#define CRSF_TRK_FRAMETYPE_HEARTBEAT                0xFF
#define CRSF_TRK_FRAMETYPE_RPM                      7
#define CRSF_TRK_FRAMETYPE_TEMPERATURE              8
#define CRSF_TRK_FRAMETYPE_VOLTAGES                 9
#define CRSF_TRK_FRAMETYPE_VTX                      10
#define CRSF_TRK_FRAMETYPE_LINK_STATISTICS          11
#define CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED       12
#define CRSF_TRK_FRAMETYPE_LINK_STATISTICS_RX       13
#define CRSF_TRK_FRAMETYPE_LINK_STATISTICS_TX       14
#define CRSF_TRK_FRAMETYPE_ATTITUDE                 15
#define CRSF_TRK_FRAMETYPE_MAVLINK_FC               0xFF
#define CRSF_TRK_FRAMETYPE_FLIGHT_MODE              0xFF
#define CRSF_TRK_FRAMETYPE_ESP_NOW_MESSAGES         0xFF
#define CRSF_TRK_FRAMETYPE_DEVICE_PING              0xFF
#define CRSF_TRK_FRAMETYPE_DEVICE_INFO              0xFF
#define CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY 0xFF
#define CRSF_TRK_FRAMETYPE_PARAMETER_READ           0xFF
#define CRSF_TRK_FRAMETYPE_PARAMETER_WRITE          0xFF
#define CRSF_TRK_FRAMETYPE_COMMAND                  0xFF
#define CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE         0xFF
#define CRSF_TRK_FRAMETYPE_MAVLINK_STATUS           0xFF

/* Statistics -----------------------------------------------------------------*/
typedef struct {
    uint32_t frames_total;       /**< Total frames parsed. */
    uint32_t frames_bad_crc;     /**< CRC errors. */
    uint32_t frames_bad_addr;    /**< Address validation errors. */
    uint32_t frames_bad_len;     /**< Length errors. */
    uint32_t frames_unsupported; /**< Unsupported types. */
    uint32_t commands_rx;        /**< Commands received. */
} CRSF_Stats_t;

/* Core instance --------------------------------------------------------------*/
typedef struct {
#if CRSF_TEL_ENABLE_GPS
    CRSF_GPS_t GPS;
#endif
#if CRSF_TEL_ENABLE_GPS_TIME
    CRSF_GPS_Time_t GPS_Time;
#endif
#if CRSF_TEL_ENABLE_GPS_EXTENDED
    CRSF_GPS_Ext_t GPS_Ext;
#endif
#if CRSF_TEL_ENABLE_VARIO
    CRSF_Vario_t Vario;
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR
    CRSF_Battery_t Battery;
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED
    CRSF_BaroAlt_VS_t BaroAlt_VS;
#endif
#if CRSF_TEL_ENABLE_AIRSPEED
    CRSF_Airspeed_t Airspeed;
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
    CRSF_Heartbeat_t Heartbeat;
#endif
#if CRSF_TEL_ENABLE_RPM
    CRSF_RPM_t RPM;
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE
    CRSF_Temperature_t Temperature;
#endif
#if CRSF_TEL_ENABLE_VOLTAGES
    CRSF_Voltages_t Voltages;
#endif
#if CRSF_TEL_ENABLE_VTX
    CRSF_VTX_t VTX;
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS
    CRSF_LinkStatistics_t LinkStatistics;
#endif
#if CRSF_ENABLE_RC_CHANNELS
    CRSF_RC_Channels_t RC;
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
    CRSF_LinkStatisticsRX_t LinkStatisticsRX;
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
    CRSF_LinkStatisticsTX_t LinkStatisticsTX;
#endif
#if CRSF_TEL_ENABLE_ATTITUDE
    CRSF_Attitude_t Attitude;
#endif
#if CRSF_TEL_ENABLE_MAVLINK_FC
    CRSF_MAVLinkFC_t MAVLinkFC;
#endif
#if CRSF_TEL_ENABLE_FLIGHT_MODE
    CRSF_FlightMode_t FlightMode;
#endif
#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
    CRSF_ESPNowMessages_t ESPNowMessages;
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
    CRSF_Ping_t Ping;
    CRSF_DeviceInfo_t DeviceInfo;
    CRSF_ParamSettingsEntry_t ParamSettingsEntry;
    CRSF_ParamRead_t ParamRead;
    CRSF_ParamWrite_t ParamWrite;
#endif
#if CRSF_ENABLE_COMMAND
    CRSF_Command_t Command;
#endif
#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
    CRSF_MAVLinkEnv_t MAVLinkEnv;
#endif
#if CRSF_TEL_ENABLE_MAVLINK_STATUS
    CRSF_MAVLinkStat_t MAVLinkStat;
#endif

#if CRSF_ENABLE_STATS
    CRSF_Stats_t Stats; /**< Statistics. */
#endif
#if CRSF_ENABLE_FRESHNESS_CHECK
    uint32_t (*getTimestamp_ms)(void);                /**< Timestamp callback [ms]. */
    uint32_t _packet_times[CRSF_TRACKED_FRAME_TYPES]; /**< Touch stamps. */
#endif
} CRSF_t;

/* Public API -----------------------------------------------------------------*/

/**
 * \brief           Initialize CRSF decoder
 * 
 * \param[in]       crsf: CRSF decoder
 */
void CRSF_init(CRSF_t* crsf);

#if CRSF_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Set timestamp callback for packet timestamping
 * 
 * \param[in]       crsf: CRSF decoder
 * \param[in]       getTimestamp_ms: Function pointer to get current timestamp in milliseconds
 */
void CRSF_setTimestampCallback(CRSF_t* crsf, uint32_t (*get_ms)(void));
#endif

/**
 * @brief           Build a frame to be sent, using values stored in CRSF struct
 *
 * @param[in]       crsf: CRSF decoder
 * @param[in]       bus_addr: first header byte (bus address)
 * @param[in]       type: frame type to be encoded
 * @param[in]       values: optional number of values (for variable length payloads)
 * @param[out]      frame: output buffer with full frame
 * @param[out]      frameLength: written frame length
 * 
 * @return          CRSF_OK on success, error otherwise.
 */
CRSF_Status_t CRSF_buildFrame(CRSF_t* crsf, uint8_t bus_addr, CRSF_FrameType_t type, uint8_t values, uint8_t* frame, uint8_t* frameLength);

/**
 * \brief           Process received frame
 * 
 * \param[in]       crsf: CRSF decoder
 * \param[in]       frame: RF frame data
 * \param[out]      recType: type of frame received
 *
 * \return          CRSF_OK on success, error otherwise.
 */
CRSF_Status_t CRSF_processFrame(CRSF_t* crsf, const uint8_t* frame, CRSF_FrameType_t* recType);

#if CRSF_ENABLE_STATS
/**
 * \brief           Get statistics about the decoder
 * 
 * \param[in]       crsf: CRSF decoder
 * \param[out]      stats: Output statistics structure
 */
void CRSF_getStats(const CRSF_t* crsf, CRSF_Stats_t* stats);

/**
 * \brief           Reset statistics counters
 * 
 * \param[out]      crsf: CRSF decoder
 */
void CRSF_resetStats(CRSF_t* crsf);
#endif

#if CRSF_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Check if a frame type was received recently
 * 
 * \param[in]       crsf: CRSF decoder
 * \param[in]       frame_type: Frame type to check (CRSF_FRAME_TYPE_RC, CRSF_FRAME_TYPE_TEL)
 * \param[in]       max_age_ms: Maximum age in milliseconds
 * 
 * \return          1 if frame is fresh, 0 if stale or never received
 */
uint8_t CRSF_isFrameFresh(const CRSF_t* CRSF, uint8_t frame_type, uint32_t max_age_ms);
#endif

#if CRSF_ENABLE_COMMAND
/**
 * @brief   Encode a Direct Command payload (0x32) including inner CRC (0xBA).
 *
 * This helper produces the **payload** for a Direct Command frame:
 *   [CMD_ID][CMD_PAYLOAD...][INNER_CRC]
 *
 * @param[in]  dest_addr   Destination address (extended header).
 * @param[in]  origin_addr Origin address (extended header).
 * @param[in]  cmd_id      Command ID.
 * @param[in]  cmd_payload Pointer to command payload bytes (may be NULL if none).
 * @param[in]  cmd_len     Command payload length in bytes.
 * @param[out] out_payload Output buffer for command payload (+1 for inner CRC).
 * @param[out] out_len     Written payload length (cmd_len + 2 incl. cmd_id + inner CRC).
 * @return     CRSF_OK on success, error otherwise.
 */
// CRSF_Status_t CRSF_encodeDirectCommand(uint8_t dest_addr, uint8_t origin_addr, uint8_t cmd_id, const uint8_t* cmd_payload, uint8_t cmd_len,
//                                        uint8_t* out_payload, uint8_t* out_len);

/**
 * @brief   Build a complete Direct Command frame (0x32) into @p out.
 * @param[in]  bus_addr    First header byte (bus address).
 * @param[in]  dest_addr   Destination address (extended header).
 * @param[in]  origin_addr Origin address (extended header).
 * @param[in]  cmd_id      Command ID.
 * @param[in]  cmd_payload Pointer to command payload (may be NULL).
 * @param[in]  cmd_len     Command payload length in bytes.
 * @param[out] out         Output buffer for full CRSF frame (addr..crc).
 * @param[out] out_len     Written frame length.
 * @return     CRSF_OK on success, error otherwise.
 */
// CRSF_Status_t CRSF_buildDirectCommandFrame(uint8_t bus_addr, uint8_t dest_addr, uint8_t origin_addr, uint8_t cmd_id, const uint8_t* cmd_payload,
//                                            uint8_t cmd_len, uint8_t* out, uint8_t* out_len);
#endif /* CRSF_ENABLE_COMMAND */

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_H__ */
