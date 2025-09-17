/* BEGIN Header */
/**
 ******************************************************************************
 * \file            CRSF.c
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

#include "CRSF.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "CRSF_CRC.h"
#include "CRSF_types.h"

/* Macros --------------------------------------------------------------------*/

// Helper macros
#define CRSF_RC_TICKS_TO_US(x) (((x) - 992) * 5 / 8 + 1500)
#define CRSF_RC_US_TO_TICKS(x) (((x) - 1500) * 8 / 5 + 992)

#define ABS(value)             (((value) >= 0) ? (value) : (-value))
#define SIGN(value)            (((value) > 0) - ((value) < 0))

#define BUILD_FRAME(TYPE, VAR)                                                                                                                                 \
    case CRSF_FRAMETYPE_##TYPE: memcpy(payload, &crsf->VAR, sizeof(CRSF_##VAR##_t))
#define UPDATE_LENGTH(VAR)                                                                                                                                     \
    *frameLength += sizeof(CRSF_##VAR##_t);                                                                                                                    \
    break

#define PROCESS_FRAME(TYPE, VAR)                                                                                                                               \
    case CRSF_FRAMETYPE_##TYPE: memcpy(&crsf->VAR, payload, sizeof(CRSF_##VAR##_t))
#define UPDATE_FRESHNESS(TYPE)                                                                                                                                 \
    if (CRSF_ENABLE_FRESHNESS_CHECK && (CRSF_TRK_FRAMETYPE_##TYPE < 0xFF)) {                                                                                   \
        CRSF_updateTimestamp(crsf, CRSF_TRK_FRAMETYPE_##TYPE);                                                                                                 \
    }                                                                                                                                                          \
    break

#define CHECK_LENGTH(LEN, TYPE, VAR) ((LEN) >= sizeof(CRSF_##VAR##_t))
/* Private Function Prototypes -----------------------------------------------*/

static uint8_t CRSF_validateFrameLength(CRSF_FrameType_t type, uint8_t payloadLength);

#if CRSF_ENABLE_ADDRESS_VALIDATION
CRSF_Status_t CRSF_isValidAddress(CRSF_Address_t addr);
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
static void CRSF_packBaroAltVSpeed(uint8_t* payload, const CRSF_BaroAlt_VS_t* baroAltVS);
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
static void CRSF_unpackBaroAltVSpeed(const uint8_t* payload, CRSF_BaroAlt_VS_t* baroAltVS);
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
static void CRSF_packRC(uint8_t* payload, const uint16_t* channels);
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
static void CRSF_unpackRC(const uint8_t* payload, uint16_t* channels);
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static CRSF_Status_t CRSF_encodeCommandPayload(CRSF_CommandID_t commandID, const CRSF_CommandPayload_t* in, uint8_t* payload, uint8_t* length);
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static CRSF_Status_t CRSF_decodeCommandPayload(CRSF_CommandID_t commandID, CRSF_CommandPayload_t* out, const uint8_t* payload, uint8_t length);
#endif

static inline void CRSF_updateTimestamp(CRSF_t* crsf, uint8_t frame_type);

// Helpers
static inline void CRSF_packBE16(uint8_t* dest, const uint16_t value);
static inline void CRSF_packBE32(uint8_t* dest, const uint32_t value);
static inline void CRSF_packHSV(uint8_t* dest, uint16_t H, uint8_t S, uint8_t V);
static inline uint16_t CRSF_unpackBE16(const uint8_t* src);
static inline uint32_t CRSF_unpackBE32(const uint8_t* src);
static inline void CRSF_unpackHSV(const uint8_t* src, uint16_t* H, uint8_t* S, uint8_t* V);

/* Functions -----------------------------------------------------------------*/

void CRSF_init(CRSF_t* crsf) {
    if (!crsf) {
        return;
    }
    memset(crsf, 0x00, sizeof(*crsf));
}

#if CRSF_ENABLE_FRESHNESS_CHECK
void CRSF_setTimestampCallback(CRSF_t* crsf, uint32_t (*get_ms)(void)) {
    if (!crsf) {
        return;
    }
    crsf->getTimestamp_ms = get_ms;
}
#endif

CRSF_Status_t CRSF_buildFrame(CRSF_t* crsf, uint8_t bus_addr, CRSF_FrameType_t type, uint8_t values, uint8_t* frame, uint8_t* frameLength) {
    if (!crsf || !frame || !frameLength) {
        return CRSF_ERROR_NULL_POINTER;
    }

#if CRSF_ENABLE_ADDRESS_VALIDATION
    if (CRSF_isValidAddress((CRSF_Address_t)bus_addr) == CRSF_ERROR_ADDR) {
        return CRSF_ERROR_ADDR;
    }
#endif
    memset(frame, 0x00, CRSF_MAX_FRAME_LEN + 2U);

    // Header
    frame[0] = bus_addr;
    frame[2] = type;
    *frameLength = 1;

    uint8_t* payload = frame + CRSF_STD_HDR_SIZE;
    uint8_t off = 0;
    (void)off;

    switch (type) {
#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_GPS:
            CRSF_packBE32(payload, crsf->GPS.latitude);
            off += sizeof(uint32_t);
            CRSF_packBE32(payload + off, crsf->GPS.longitude);
            off += sizeof(uint32_t);
            CRSF_packBE16(payload + off, crsf->GPS.groundspeed);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS.heading);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS.altitude);
            off += sizeof(uint16_t);
            payload[off] = crsf->GPS.satellites;
            UPDATE_LENGTH(GPS);
#endif

#if CRSF_TEL_ENABLE_GPS_TIME && defined(CRSF_CONFIG_RX)
            BUILD_FRAME(GPS_TIME, GPS_Time);
            CRSF_packBE16(payload, crsf->GPS_Time.year);
            off += sizeof(uint16_t) + 5U * sizeof(uint8_t);
            CRSF_packBE16(payload + off, crsf->GPS_Time.millisecond);
            UPDATE_LENGTH(GPS_Time);
#endif

#if CRSF_TEL_ENABLE_GPS_EXTENDED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_GPS_EXTENDED:
            payload[off++] = crsf->GPS_Ext.fix_type;
            CRSF_packBE16(payload + off, crsf->GPS_Ext.n_speed);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.e_speed);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.v_speed);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.h_speed_acc);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.track_acc);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.alt_ellipsoid);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.h_acc);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->GPS_Ext.v_acc);
            off += sizeof(uint16_t);
            payload[off++] = crsf->GPS_Ext.reserved;
            payload[off++] = crsf->GPS_Ext.hDOP;
            payload[off] = crsf->GPS_Ext.vDOP;
            UPDATE_LENGTH(GPS_Ext);

#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_VARIO: CRSF_packBE16(payload, crsf->Vario.v_speed); UPDATE_LENGTH(Vario);
#endif

#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            CRSF_packBE16(payload, crsf->Battery.voltage);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->Battery.current);
            off += sizeof(uint16_t);
            payload[off++] = (uint8_t)(crsf->Battery.capacity_used >> 16);
            payload[off++] = (uint8_t)((crsf->Battery.capacity_used >> 8) & 0xFFU);
            payload[off++] = (uint8_t)((crsf->Battery.capacity_used) & 0xFFU);
            payload[off++] = crsf->Battery.remaining;
            *frameLength += off;
            break;
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_BAROALT_VSPEED:
            CRSF_packBaroAltVSpeed(payload, &crsf->BaroAlt_VS);
            *frameLength += sizeof(uint16_t) + sizeof(uint8_t);
            break;
#endif

#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_AIRSPEED: CRSF_packBE16(payload, crsf->Airspeed.speed); UPDATE_LENGTH(Airspeed);
#endif

#if CRSF_TEL_ENABLE_HEARTBEAT
        case CRSF_FRAMETYPE_HEARTBEAT: CRSF_packBE16(payload, crsf->Heartbeat.origin_address); UPDATE_LENGTH(Heartbeat);
#endif

#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_RPM: {
            payload[off++] = crsf->RPM.rpm_source_id;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_RPM_VALUES; ii++) {
                uint32_t val = (uint32_t)(crsf->RPM.rpm_value[ii] & 0xFFFFFFU);
                payload[off++] = (uint8_t)((val >> 16) & 0xFFU);
                payload[off++] = (uint8_t)((val >> 8) & 0xFFU);
                payload[off++] = (uint8_t)(val & 0xFFU);
            }

            if (off < 4U) {
                payload[off++] = 0;
                payload[off++] = 0;
                payload[off++] = 0;
            }

            *frameLength += off;
            break;
        }
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_TEMPERATURE:
            payload[off++] = crsf->Temperature.temp_source_id;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_TEMPERATURE_VALUES; ii++) {
                CRSF_packBE16(payload + off, crsf->Temperature.temperature[ii]);
                off += sizeof(uint16_t);
            }

            if (off < 3U) {
                payload[off++] = 0;
                payload[off++] = 0;
            }

            *frameLength += off;
            break;
#endif

#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_VOLTAGES:
            payload[off++] = crsf->Voltages.Voltage_source_id;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_VOLTAGE_VALUES; ii++) {
                CRSF_packBE16(payload + off, crsf->Voltages.Voltage_values[ii]);
                off += sizeof(uint16_t);
            }

            if (off < 3U) {
                payload[off++] = 0;
                payload[off++] = 0;
            }

            *frameLength += off;
            break;
#endif

#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_RX)
            BUILD_FRAME(VTX, VTX);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->VTX.frequency_MHz);
            UPDATE_LENGTH(VTX);
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS
            BUILD_FRAME(LINK_STATISTICS, LinkStatistics);
            UPDATE_LENGTH(LinkStatistics);
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            CRSF_packRC(payload, crsf->RC.channels);
            *frameLength += 22;
            break;
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
            BUILD_FRAME(LINK_STATISTICS_RX, LinkStatisticsRX);
            UPDATE_LENGTH(LinkStatisticsRX);
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
            BUILD_FRAME(LINK_STATISTICS_TX, LinkStatisticsTX);
            UPDATE_LENGTH(LinkStatisticsTX);

#endif

#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_ATTITUDE:
            CRSF_packBE16(payload, crsf->Attitude.pitch);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->Attitude.roll);
            off += sizeof(uint16_t);
            CRSF_packBE16(payload + off, crsf->Attitude.yaw);
            UPDATE_LENGTH(Attitude);
#endif

#if CRSF_TEL_ENABLE_MAVLINK_FC
            BUILD_FRAME(MAVLINK_FC, MAVLinkFC);
            CRSF_packBE16(payload, crsf->MAVLinkFC.airspeed);
            off += sizeof(uint16_t) + sizeof(uint8_t);
            CRSF_packBE32(payload + off, crsf->MAVLinkFC.custom_mode);
            UPDATE_LENGTH(MAVLinkFC);
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            strncpy((char*)payload, crsf->FlightMode.flight_mode, CRSF_MAX_FLIGHT_MODE_NAME_LEN - 1U);
            payload[CRSF_MAX_FLIGHT_MODE_NAME_LEN - 1U] = '\0'; // Ensure null termination
            *frameLength += strlen((char*)payload) + 1U;        // Adding also null termination
            break;
#endif

#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
            BUILD_FRAME(ESP_NOW_MESSAGES, ESPNowMessages);
            UPDATE_LENGTH(ESPNowMessages);
#endif

#if CRSF_TEL_ENABLE_PARAMETER_GROUP
            BUILD_FRAME(DEVICE_PING, Ping);
            UPDATE_LENGTH(Ping);

        case CRSF_FRAMETYPE_DEVICE_INFO: {
            uint16_t nameLen = strlen(crsf->DeviceInfo.Device_name);
            payload[off++] = crsf->DeviceInfo.dest_address;
            payload[off++] = crsf->DeviceInfo.origin_address;
            strncpy((char*)(payload + off), crsf->DeviceInfo.Device_name, nameLen);
            off += nameLen;
            payload[off++] = '\0'; //Ensure null termination
            CRSF_packBE32(payload + off, crsf->DeviceInfo.Serial_number);
            off += sizeof(uint32_t);
            CRSF_packBE32(payload + off, crsf->DeviceInfo.Hardware_ID);
            off += sizeof(uint32_t);
            CRSF_packBE32(payload + off, crsf->DeviceInfo.Firmware_ID);
            off += sizeof(uint32_t);
            payload[off++] = crsf->DeviceInfo.Parameters_total;
            payload[off++] = crsf->DeviceInfo.Parameter_version_number;
            *frameLength += off;
            break;
        }

        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: {
            uint8_t paramLen = (values > CRSF_MAX_PARAM_SETTINGS_PAYLOAD ? CRSF_MAX_PARAM_SETTINGS_PAYLOAD : values) + 4U;
            memcpy(payload, &crsf->ParamSettingsEntry, paramLen);
            *frameLength += paramLen;
            if (paramLen == 4U) {
                payload[4] = 0;
                *frameLength += sizeof(uint8_t);
            }
            break;
        }

            BUILD_FRAME(PARAMETER_READ, ParamRead);
            UPDATE_LENGTH(ParamRead);

        case CRSF_FRAMETYPE_PARAMETER_WRITE: {
            uint8_t paramLen = (values > CRSF_MAX_PARAM_DATA_LEN ? CRSF_MAX_PARAM_DATA_LEN : values) + 3U;
            memcpy(payload, &crsf->ParamWrite, paramLen);
            *frameLength += paramLen;
            if (paramLen == 3U) {
                payload[3] = 0;
                *frameLength += sizeof(uint8_t);
            }
            break;
        }
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_COMMAND: {
            payload[off++] = crsf->Command.dest_address;
            payload[off++] = crsf->Command.origin_address;
            payload[off++] = (uint8_t)crsf->Command.Command_ID;
            CRSF_Status_t retStatus = CRSF_encodeCommandPayload(crsf->Command.Command_ID, &crsf->Command.payload, payload + off, &off);
            if (retStatus != CRSF_OK) {
                return retStatus;
            }
            payload[off] = CRSF_calcChecksumCMD(frame + 2U, off + sizeof(uint8_t));
            *frameLength += off + sizeof(uint8_t); //including also inner CRC
            break;
        }
#endif

#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
        case CRSF_FRAMETYPE_MAVLINK_ENVELOPE:
            crsf->MAVLinkEnv.data_size = crsf->MAVLinkEnv.data_size > CRSF_MAX_MAVLINK_PAYLOAD ? CRSF_MAX_MAVLINK_PAYLOAD : crsf->MAVLinkEnv.data_size;
            memcpy(payload, &crsf->MAVLinkEnv, crsf->MAVLinkEnv.data_size + 2U);
            *frameLength += crsf->MAVLinkEnv.data_size + 2U;
            break;
#endif

#if CRSF_TEL_ENABLE_MAVLINK_STATUS
        case CRSF_FRAMETYPE_MAVLINK_STATUS:
            CRSF_packBE32(payload, crsf->MAVLinkStat.sensor_present);
            off += sizeof(uint32_t);
            CRSF_packBE32(payload + off, crsf->MAVLinkStat.sensor_enabled);
            off += sizeof(uint32_t);
            CRSF_packBE32(payload + off, crsf->MAVLinkStat.sensor_health);
            UPDATE_LENGTH(MAVLinkStat);
#endif
        default: (void)payload; return CRSF_ERROR_INVALID_FRAME;
    }

    *frameLength += CRSF_CRC_SIZE;
    frame[1] = *frameLength;

    frame[*frameLength + 1U] = CRSF_calcChecksum(frame + 2U, *frameLength - CRSF_CRC_SIZE);
    *frameLength += 2U; //Adding also address and length
    return CRSF_OK;
}

CRSF_Status_t CRSF_processFrame(CRSF_t* crsf, const uint8_t* frame, CRSF_FrameType_t* recType) {
    if (!crsf || !frame || !recType) {
        return CRSF_ERROR_NULL_POINTER;
    }

    const uint8_t payloadLength = frame[1] - 1U - CRSF_CRC_SIZE; // Removing the type and CRC bytes
    const CRSF_FrameType_t type = (CRSF_FrameType_t)frame[2];
    *recType = type;

#if CRSF_ENABLE_STATS
    crsf->Stats.frames_total++;
#endif

#if CRSF_ENABLE_ADDRESS_VALIDATION
    if (CRSF_isValidAddress((CRSF_Address_t)frame[0]) == CRSF_ERROR_ADDR) {
#if CRSF_ENABLE_STATS
        crsf->Stats.frames_bad_addr++;
#endif
        return CRSF_ERROR_ADDR;
    }
#endif

    if (frame[1] < CRSF_MIN_FRAME_LEN || frame[1] > CRSF_MAX_FRAME_LEN) {
#if CRSF_ENABLE_STATS
        crsf->Stats.frames_bad_len++;
#endif
        return CRSF_ERROR_LENGTH;
    }

    /* Frame length validation */
    if (!CRSF_validateFrameLength(type, payloadLength)) {
#if CRSF_ENABLE_STATS
        crsf->Stats.frames_bad_len++;
#endif
        return CRSF_ERROR_TYPE_LENGTH;
    }

    /* Payload CRC check */
    if (CRSF_calcChecksum(frame + 2U, payloadLength + 1U) != frame[payloadLength + CRSF_STD_HDR_SIZE]) {
#if CRSF_ENABLE_STATS
        crsf->Stats.frames_bad_crc++;
#endif
        return CRSF_ERROR_CHECKSUM_FAIL;
    }

    //Process
    const uint8_t* payload = frame + CRSF_STD_HDR_SIZE;
    uint8_t off = 0;
    (void)off;

    switch (type) {

#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS:
            crsf->GPS.latitude = CRSF_unpackBE32(payload);
            off += sizeof(uint32_t);
            crsf->GPS.longitude = CRSF_unpackBE32(payload + off);
            off += sizeof(uint32_t);
            crsf->GPS.groundspeed = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS.heading = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS.altitude = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS.satellites = payload[off];
            UPDATE_FRESHNESS(GPS);
#endif

#if CRSF_TEL_ENABLE_GPS_TIME && defined(CRSF_CONFIG_TX)
            PROCESS_FRAME(GPS_TIME, GPS_Time);
            crsf->GPS_Time.year = CRSF_unpackBE16(payload);
            off += sizeof(uint16_t) + 5U * sizeof(uint8_t);
            crsf->GPS_Time.millisecond = CRSF_unpackBE16(payload + 7U);
            UPDATE_FRESHNESS(GPS_TIME);
#endif

#if CRSF_TEL_ENABLE_GPS_EXTENDED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS_EXTENDED:
            crsf->GPS_Ext.fix_type = payload[off++];
            crsf->GPS_Ext.n_speed = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.e_speed = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.v_speed = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.h_speed_acc = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.track_acc = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.alt_ellipsoid = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.h_acc = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.v_acc = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->GPS_Ext.reserved = payload[off++];
            crsf->GPS_Ext.hDOP = payload[off++];
            crsf->GPS_Ext.vDOP = payload[off];
            UPDATE_FRESHNESS(GPS_EXTENDED);
#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VARIO: crsf->Vario.v_speed = CRSF_unpackBE16(payload); UPDATE_FRESHNESS(VARIO);
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            crsf->Battery.voltage = CRSF_unpackBE16(payload);
            off += sizeof(uint16_t);
            crsf->Battery.current = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->Battery.capacity_used = ((uint32_t)payload[off] << 16) | ((uint32_t)payload[off + 1U] << 8) | ((uint32_t)payload[off + 2U]);
            off += 3U; //sizeof uint24_t
            crsf->Battery.remaining = payload[off];
            UPDATE_FRESHNESS(BATTERY_SENSOR);
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_BAROALT_VSPEED: CRSF_unpackBaroAltVSpeed(payload, &crsf->BaroAlt_VS); UPDATE_FRESHNESS(BAROALT_VSPEED);
#endif

#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_AIRSPEED: crsf->Airspeed.speed = CRSF_unpackBE16(payload); UPDATE_FRESHNESS(AIRSPEED);
#endif

#if CRSF_TEL_ENABLE_HEARTBEAT
        case CRSF_FRAMETYPE_HEARTBEAT: crsf->Heartbeat.origin_address = CRSF_unpackBE16(payload); UPDATE_FRESHNESS(HEARTBEAT);
#endif

#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_RPM: {
            uint8_t rpmCount = (payloadLength - sizeof(crsf->RPM.rpm_source_id)) / 3U;
            crsf->RPM.rpm_source_id = payload[off++];
            for (uint8_t ii = 0; ii < rpmCount && ii < CRSF_MAX_RPM_VALUES; ii++) {
                uint32_t val = ((uint32_t)payload[off] << 16) | ((uint32_t)payload[off + 1U] << 8) | ((uint32_t)payload[off + 2U]);
                crsf->RPM.rpm_value[ii] = val & 0x800000 ? (int32_t)(val | 0xFF000000) : (int32_t)val; // Sign extend if negative
                off += 3U;
            }
            UPDATE_FRESHNESS(RPM);
        }
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_TEMPERATURE: {
            uint8_t tempCount = (payloadLength - sizeof(crsf->Temperature.temp_source_id)) / sizeof(uint16_t);
            crsf->Temperature.temp_source_id = payload[off++];
            for (uint8_t ii = 0; ii < tempCount && ii < CRSF_MAX_TEMPERATURE_VALUES; ii++) {
                crsf->Temperature.temperature[ii] = CRSF_unpackBE16(payload + off);
                off += sizeof(uint16_t);
            }
            UPDATE_FRESHNESS(TEMPERATURE);
        }
#endif

#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VOLTAGES: {
            uint8_t voltCount = (payloadLength - sizeof(crsf->Voltages.Voltage_source_id)) / 2;
            crsf->Voltages.Voltage_source_id = payload[off++];
            for (uint8_t ii = 0; ii < voltCount && ii < CRSF_MAX_VOLTAGE_VALUES; ii++) {
                crsf->Voltages.Voltage_values[ii] = CRSF_unpackBE16(payload + off);
                off += sizeof(uint16_t);
            }
            UPDATE_FRESHNESS(VOLTAGES);
        }
#endif

#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_TX)
            PROCESS_FRAME(VTX, VTX);
            off += sizeof(uint16_t);
            crsf->VTX.frequency_MHz = CRSF_unpackBE16(payload + off);
            UPDATE_FRESHNESS(VTX);
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS
            PROCESS_FRAME(LINK_STATISTICS, LinkStatistics);
            UPDATE_FRESHNESS(LINK_STATISTICS);
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: CRSF_unpackRC(payload, crsf->RC.channels); UPDATE_FRESHNESS(RC_CHANNELS_PACKED);
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
            PROCESS_FRAME(LINK_STATISTICS_RX, LinkStatisticsRX);
            UPDATE_FRESHNESS(LINK_STATISTICS_RX);
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
            PROCESS_FRAME(LINK_STATISTICS_TX, LinkStatisticsTX);
            UPDATE_FRESHNESS(LINK_STATISTICS_TX);
#endif

#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_ATTITUDE:
            crsf->Attitude.pitch = CRSF_unpackBE16(payload);
            off += sizeof(uint16_t);
            crsf->Attitude.roll = CRSF_unpackBE16(payload + off);
            off += sizeof(uint16_t);
            crsf->Attitude.yaw = CRSF_unpackBE16(payload + off);
            UPDATE_FRESHNESS(ATTITUDE);
#endif

#if CRSF_TEL_ENABLE_MAVLINK_FC
            PROCESS_FRAME(MAVLINK_FC, MAVLinkFC);
            crsf->MAVLinkFC.airspeed = CRSF_unpackBE16(payload);
            off += sizeof(uint16_t) + sizeof(uint8_t);
            crsf->MAVLinkFC.custom_mode = CRSF_unpackBE32(payload + off);
            UPDATE_FRESHNESS(MAVLINK_FC);
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
        case CRSF_FRAMETYPE_FLIGHT_MODE: {
            strncpy(crsf->FlightMode.flight_mode, (char*)payload, CRSF_MAX_FLIGHT_MODE_NAME_LEN - 1U); // Last charachter is always \0
            crsf->FlightMode.flight_mode[CRSF_MAX_FLIGHT_MODE_NAME_LEN - 1U] = '\0';
            UPDATE_FRESHNESS(FLIGHT_MODE);
        }
#endif

#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
            PROCESS_FRAME(ESP_NOW_MESSAGES, ESPNowMessages);
            UPDATE_FRESHNESS(ESP_NOW_MESSAGES);
#endif

#if CRSF_TEL_ENABLE_PARAMETER_GROUP
            PROCESS_FRAME(DEVICE_PING, Ping);
            UPDATE_FRESHNESS(DEVICE_PING);

        case CRSF_FRAMETYPE_DEVICE_INFO: {
            crsf->DeviceInfo.dest_address = payload[off++];
            crsf->DeviceInfo.origin_address = payload[off++];
            uint16_t nameLen = strlen((char*)(payload + off)) + 1U;                                       // Including null termination
            strncpy(crsf->DeviceInfo.Device_name, (char*)(payload + off), CRSF_MAX_DEVICE_NAME_LEN - 1U); // Last charachter is always \0
            crsf->DeviceInfo.Device_name[CRSF_MAX_DEVICE_NAME_LEN - 1U] = '\0';
            off += nameLen;
            crsf->DeviceInfo.Serial_number = CRSF_unpackBE32(payload + off);
            off += sizeof(uint32_t);
            crsf->DeviceInfo.Hardware_ID = CRSF_unpackBE32(payload + off);
            off += sizeof(uint32_t);
            crsf->DeviceInfo.Firmware_ID = CRSF_unpackBE32(payload + off);
            off += sizeof(uint32_t);
            crsf->DeviceInfo.Parameters_total = payload[off++];
            crsf->DeviceInfo.Parameter_version_number = payload[off];
            UPDATE_FRESHNESS(DEVICE_INFO);
        }
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            memcpy(&crsf->ParamSettingsEntry, payload,
                   payloadLength < (CRSF_MAX_PARAM_SETTINGS_PAYLOAD + 4U) ? payloadLength : (CRSF_MAX_PARAM_SETTINGS_PAYLOAD + 4U));
            UPDATE_FRESHNESS(PARAMETER_SETTINGS_ENTRY);

            PROCESS_FRAME(PARAMETER_READ, ParamRead);
            UPDATE_FRESHNESS(PARAMETER_READ);

        case CRSF_FRAMETYPE_PARAMETER_WRITE:
            memcpy(&crsf->ParamWrite, payload, payloadLength < (CRSF_MAX_PARAM_DATA_LEN + 3U) ? payloadLength : (CRSF_MAX_PARAM_DATA_LEN + 3U));
            UPDATE_FRESHNESS(PARAMETER_WRITE);
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_COMMAND:
            if (payload[payloadLength - 1U] != CRSF_calcChecksumCMD(payload - 1U, payloadLength)) {
                return CRSF_ERROR_CMD_CHECKSUM_FAIL;
            }
            crsf->Command.dest_address = payload[off++];
            crsf->Command.origin_address = payload[off++];
            crsf->Command.Command_ID = (CRSF_CommandID_t)payload[off++];
            if (CRSF_decodeCommandPayload(crsf->Command.Command_ID, &crsf->Command.payload, payload + off, payloadLength - 4U) != CRSF_OK) {
                return CRSF_ERROR_TYPE_LENGTH;
            };

#if CRSF_ENABLE_STATS
            crsf->Stats.commands_rx++;
#endif
            UPDATE_FRESHNESS(COMMAND);
            break;

#endif

#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
        case CRSF_FRAMETYPE_MAVLINK_ENVELOPE: memcpy(&crsf->MAVLinkEnv, payload, (payload[1] > 58 ? 58 : payload[1]) + 2U); UPDATE_FRESHNESS(MAVLINK_ENVELOPE);
#endif

#if CRSF_TEL_ENABLE_MAVLINK_STATUS
        case CRSF_FRAMETYPE_MAVLINK_STATUS:
            crsf->MAVLinkStat.sensor_present = CRSF_unpackBE32(payload);
            off += sizeof(uint32_t);
            crsf->MAVLinkStat.sensor_enabled = CRSF_unpackBE32(payload + off);
            off += sizeof(uint32_t);
            crsf->MAVLinkStat.sensor_health = CRSF_unpackBE32(payload + off);
            UPDATE_FRESHNESS(MAVLINK_STATUS);
#endif
        default: (void)payload;
#if CRSF_ENABLE_STATS
            crsf->Stats.frames_unsupported++;
#endif
            return CRSF_ERROR_INVALID_FRAME;
    }
    return CRSF_OK;
}

#if CRSF_ENABLE_STATS
void CRSF_getStats(const CRSF_t* crsf, CRSF_Stats_t* stats) {
    if (!crsf || !stats) {
        return;
    }
    *stats = crsf->Stats;
}

void CRSF_resetStats(CRSF_t* crsf) {
    if (!crsf) {
        return;
    }
    memset(&crsf->Stats, 0x00, sizeof(crsf->Stats));
}
#endif

#if CRSF_ENABLE_FRESHNESS_CHECK
uint8_t CRSF_isFrameFresh(const CRSF_t* crsf, uint8_t frame_type, uint32_t max_age_ms) {
    if (!crsf || !crsf->getTimestamp_ms) {
        return 0;
    }
    if (frame_type >= CRSF_TRACKED_FRAME_TYPES) {
        return 0;
    }

    return (crsf->getTimestamp_ms() - crsf->_packet_times[frame_type]) <= max_age_ms;
}
#endif

/* Private Functions ---------------------------------------------------------*/

static uint8_t CRSF_validateFrameLength(CRSF_FrameType_t type, uint8_t payloadLength) {
    switch (type) {
#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS: return CHECK_LENGTH(payloadLength, type, GPS);
#endif
#if CRSF_TEL_ENABLE_GPS_TIME && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS_TIME: return CHECK_LENGTH(payloadLength, type, GPS_Time);
#endif
#if CRSF_TEL_ENABLE_GPS_EXTENDED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS_EXTENDED: return CHECK_LENGTH(payloadLength, type, GPS_Ext);
#endif
#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VARIO: return CHECK_LENGTH(payloadLength, type, Vario);
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_BATTERY_SENSOR: return ((payloadLength) >= sizeof(CRSF_Battery_t) - 1U);
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_BAROALT_VSPEED: return ((payloadLength) >= 3);
#endif
#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_AIRSPEED: return CHECK_LENGTH(payloadLength, type, Airspeed);
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
        case CRSF_FRAMETYPE_HEARTBEAT: return CHECK_LENGTH(payloadLength, type, Heartbeat);
#endif
#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_RPM: return (payloadLength >= 4U);
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_TEMPERATURE: return (payloadLength >= 3U);
#endif
#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VOLTAGES: return (payloadLength >= 3U);
#endif
#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VTX: return CHECK_LENGTH(payloadLength, type, VTX);
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_LINK_STATISTICS: return CHECK_LENGTH(payloadLength, type, LinkStatistics);
#endif
#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: return (payloadLength >= 22U);
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
        case CRSF_FRAMETYPE_LINK_STATISTICS_RX: return CHECK_LENGTH(payloadLength, type, LinkStatisticsRX);
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
        case CRSF_FRAMETYPE_LINK_STATISTICS_TX: return CHECK_LENGTH(payloadLength, type, LinkStatisticsTX);
#endif
#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_ATTITUDE: return CHECK_LENGTH(payloadLength, type, Attitude);
#endif
#if CRSF_TEL_ENABLE_MAVLINK_FC
        case CRSF_FRAMETYPE_MAVLINK_FC: return CHECK_LENGTH(payloadLength, type, MAVLinkFC);
#endif
#if CRSF_TEL_ENABLE_FLIGHT_MODE
        case CRSF_FRAMETYPE_FLIGHT_MODE: return (payloadLength >= 1U && payloadLength <= CRSF_MAX_FLIGHT_MODE_NAME_LEN);
#endif
#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
        case CRSF_FRAMETYPE_ESP_NOW_MESSAGES: return CHECK_LENGTH(payloadLength, type, ESPNowMessages);
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
        case CRSF_FRAMETYPE_DEVICE_PING: return CHECK_LENGTH(payloadLength, type, Ping);
        case CRSF_FRAMETYPE_DEVICE_INFO: return (payloadLength >= 15U);
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: return (payloadLength >= 5U);
        case CRSF_FRAMETYPE_PARAMETER_READ: return CHECK_LENGTH(payloadLength, type, ParamRead);
        case CRSF_FRAMETYPE_PARAMETER_WRITE: return (payloadLength >= CRSF_MIN_FRAME_LEN);
#endif
#if CRSF_ENABLE_COMMAND
        case CRSF_FRAMETYPE_COMMAND: return (payloadLength >= 3U);
#endif
#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
        case CRSF_FRAMETYPE_MAVLINK_ENVELOPE: return (payloadLength >= CRSF_MIN_FRAME_LEN && payloadLength <= (CRSF_MAX_FRAME_LEN - 2U));
#endif
#if CRSF_TEL_ENABLE_MAVLINK_STATUS
        case CRSF_FRAMETYPE_MAVLINK_STATUS: return CHECK_LENGTH(payloadLength, type, MAVLinkStat);
#endif
        default: break;
    }
    return (payloadLength <= (CRSF_MAX_FRAME_LEN - 2U));
}

#if CRSF_ENABLE_ADDRESS_VALIDATION
CRSF_Status_t CRSF_isValidAddress(CRSF_Address_t addr) {
    // Check specific known addresses
    switch (addr) {
        case CRSF_ADDRESS_BROADCAST:
        case CRSF_ADDRESS_USB:
        case CRSF_ADDRESS_TBS_CORE_PNP_PRO:
        case CRSF_ADDRESS_CURRENT_SENSOR:
        case CRSF_ADDRESS_GPS:
        case CRSF_ADDRESS_TBS_BLACKBOX:
        case CRSF_ADDRESS_FLIGHT_CONTROLLER:
        case CRSF_ADDRESS_RACE_TAG:
        case CRSF_ADDRESS_RADIO_TRANSMITTER:
        case CRSF_ADDRESS_CRSF_RECEIVER:
        case CRSF_ADDRESS_CRSF_TRANSMITTER: return CRSF_OK;
        default: return CRSF_ERROR_ADDR;
    }
}
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
static void CRSF_packBaroAltVSpeed(uint8_t* payload, const CRSF_BaroAlt_VS_t* baroAltVS) {

    const int Kl = 100;    // linearity constant;
    const float Kr = .026; // range constant;

    // Pack altitude
    uint16_t altPacked;
    if (baroAltVS->altitude < -10000) {       //less than minimum altitude
        altPacked = 0;                        //minimum
    } else if (baroAltVS->altitude < 22768) { //dm-resolution range
        altPacked = baroAltVS->altitude + 10000;
    } else if (baroAltVS->altitude > 327655) { //more than maximum
        altPacked = 0xFFFE;                    //maximum
    } else {
        altPacked = ((baroAltVS->altitude + 5) / 10) | 0x8000; //meter-resolution range
    }

    // Pack vertical speed
    int8_t vsPacked;
    if (baroAltVS->vertical_speed == 0) {
        vsPacked = 0;
    } else {
        vsPacked = (int8_t)roundf(logf((float)ABS(baroAltVS->vertical_speed) / Kl + 1) / Kr) * SIGN(baroAltVS->vertical_speed);
    }

    payload[0] = (uint8_t)((altPacked >> 8U) & 0xFFU);
    payload[1] = (uint8_t)(altPacked & 0xFFU);
    payload[2] = (uint8_t)(vsPacked);
}
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
static void CRSF_unpackBaroAltVSpeed(const uint8_t* payload, CRSF_BaroAlt_VS_t* baroAltVS) {

    const int Kl = 100;    // linearity constant;
    const float Kr = .026; // range constant;

    uint16_t altPacked = (uint16_t)(payload[1] | (payload[0] << 8U));
    int8_t vsPacked = (int8_t)payload[2];
    baroAltVS->altitude = (altPacked & 0x8000) ? (altPacked & 0x7FFF) * 10 : (altPacked - 10000);
    baroAltVS->vertical_speed = (expf((float)ABS(vsPacked) * Kr) - 1.0f) * Kl * SIGN(vsPacked);
}

#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
static void CRSF_packRC(uint8_t* payload, const uint16_t* channels) {
    uint32_t bitbuf = 0;
    uint8_t bitcnt = 0;
    uint8_t* p = payload;

    for (uint8_t ii = 0; ii < CRSF_RC_CHANNELS; ii++) {
        // Map 1000..2000us <-> 172..1811 (11-bit)
        uint16_t val = CRSF_RC_US_TO_TICKS(channels[ii]);
        bitbuf |= ((uint32_t)val & 0x7FFU) << bitcnt;
        bitcnt += 11;

        while (bitcnt >= 8) {
            *p++ = (uint8_t)bitbuf;
            bitbuf >>= 8;
            bitcnt -= 8;
        }
    }
}
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
static void CRSF_unpackRC(const uint8_t* payload, uint16_t* channels) {
    uint32_t bitbuf = 0;
    uint8_t bitcnt = 0;
    const uint8_t* p = payload;

    for (uint8_t ii = 0; ii < CRSF_RC_CHANNELS; ii++) {
        while (bitcnt < 11U) {
            bitbuf |= ((uint32_t)(*p++)) << bitcnt;
            bitcnt += 8U;
        }
        // Map 1000..2000us <-> 172..1811 (11-bit)
        channels[ii] = CRSF_RC_TICKS_TO_US((uint16_t)(bitbuf & 0x7FFU));
        bitbuf >>= 11U;
        bitcnt -= 11U;
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static CRSF_Status_t CRSF_encodeCommandPayload(CRSF_CommandID_t commandID, const CRSF_CommandPayload_t* in, uint8_t* payload, uint8_t* length) {
    uint8_t off = 0;

    switch (commandID) {
        case CRSF_CMDID_COMMAND_ACK:
            payload[off++] = in->ACK.Command_ID;
            payload[off++] = in->ACK.SubCommand_ID;
            payload[off++] = in->ACK.Action;

            // Add information string if present and space available
            if (in->ACK.Information[0] != '\0' && (CRSF_MAX_COMMAND_PAYLOAD - off) > 0) {
                uint8_t strLen = strlen(in->ACK.Information);
                strncpy((char*)(payload + off), in->ACK.Information, strLen);
                off += strLen;
                payload[off++] = '\0'; // Add null terminator
            }
            break;

        case CRSF_CMDID_FC: payload[off++] = (uint8_t)in->FC.subCommand; break;

        case CRSF_CMDID_BLUETOOTH:
            payload[off++] = (uint8_t)in->Bluetooth.subCommand;
            if (in->Bluetooth.subCommand == CRSF_CMD_BT_ENABLE) {
                payload[off++] = in->Bluetooth.Enable;
            }
            break;

        case CRSF_CMDID_OSD:
            payload[off++] = (uint8_t)in->OSD.subCommand;
            if (in->OSD.subCommand == CRSF_CMD_OSD_SEND_BUTTONS) {
                payload[off++] = in->OSD.buttons;
            }
            break;

        case CRSF_CMDID_VTX:
            payload[off++] = (uint8_t)in->VTX.subCommand;
            switch (in->VTX.subCommand) {
                case CRSF_CMD_VTX_SET_FREQUENCY:
                    CRSF_packBE16(payload + off, in->VTX.FrequencyMHz);
                    off += 2;
                    break;
                case CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP: payload[off++] = in->VTX.pitModeCfg; break;
                case CRSF_CMD_VTX_SET_DYNAMIC_POWER:
                case CRSF_CMD_VTX_SET_POWER: payload[off++] = in->VTX.Power_dBm; break;
                default: break;
            }
            break;

        case CRSF_CMDID_LED:
            payload[off++] = (uint8_t)in->LED.subCommand;
            switch (in->LED.subCommand) {
                case CRSF_CMD_LED_OVERRIDE_COLOR:
                    CRSF_packHSV(payload + off, in->LED.overrideColor.H, in->LED.overrideColor.S, in->LED.overrideColor.V);
                    off += 3;
                    break;
                case CRSF_CMD_LED_OVERRIDE_PULSE:
                    CRSF_packBE16(payload + off, in->LED.overridePulse.duration_ms);
                    off += 2;
                    CRSF_packHSV(payload + off, in->LED.overridePulse.H_start, in->LED.overridePulse.S_start, in->LED.overridePulse.V_start);
                    off += 3;
                    CRSF_packHSV(payload + off, in->LED.overridePulse.H_stop, in->LED.overridePulse.S_stop, in->LED.overridePulse.V_stop);
                    off += 3;
                    break;
                case CRSF_CMD_LED_OVERRIDE_BLINK:
                    CRSF_packBE16(payload + off, in->LED.overrideBlink.interval_ms);
                    off += 2;
                    CRSF_packHSV(payload + off, in->LED.overrideBlink.H_start, in->LED.overrideBlink.S_start, in->LED.overrideBlink.V_start);
                    off += 3;
                    CRSF_packHSV(payload + off, in->LED.overrideBlink.H_stop, in->LED.overrideBlink.S_stop, in->LED.overrideBlink.V_stop);
                    off += 3;
                    break;
                case CRSF_CMD_LED_OVERRIDE_SHIFT:
                    CRSF_packBE16(payload + off, in->LED.overrideShift.interval_ms);
                    off += 2;
                    CRSF_packHSV(payload + off, in->LED.overrideShift.H, in->LED.overrideShift.S, in->LED.overrideShift.V);
                    off += 3;
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_GENERAL:
            payload[off++] = (uint8_t)in->general.subCommand;
            switch (in->general.subCommand) {
                case CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL:
                    payload[off++] = in->general.protocolSpeedProposal.port_id;
                    CRSF_packBE32(payload + off, in->general.protocolSpeedProposal.proposed_baudrate);
                    off += 4;
                    break;
                case CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE:
                    payload[off++] = in->general.protocolSpeedResponse.port_id;
                    payload[off++] = in->general.protocolSpeedResponse.response ? 1 : 0;
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_CROSSFIRE:
            payload[off++] = (uint8_t)in->crossfire.subCommand;
            switch (in->crossfire.subCommand) {
                case CRSF_CMD_CF_SET_BIND_ID:
                    memcpy(payload + off, in->crossfire.setBindId.bytes, in->crossfire.setBindId.len);
                    off += in->crossfire.setBindId.len;
                    break;
                case CRSF_CMD_CF_MODEL_SELECTION:
                case CRSF_CMD_CF_CURRENT_MODEL_REPLY: payload[off++] = in->crossfire.Model_Number; break;
                default: break;
            }
            break;

        case CRSF_CMDID_FLOW_CTRL:
            payload[off++] = (uint8_t)in->flow.subCommand;
            switch (in->flow.subCommand) {
                case CRSF_CMD_FLOW_SUBSCRIBE:
                    payload[off++] = in->flow.Frame_type;
                    CRSF_packBE16(payload + off, in->flow.Max_interval_time_ms);
                    off += 2;
                    break;
                case CRSF_CMD_FLOW_UNSUBSCRIBE: payload[off++] = in->flow.Frame_type; break;
                default: break;
            }
            break;

        case CRSF_CMDID_SCREEN:
            payload[off++] = (uint8_t)in->screen.subCommand;
            switch (in->screen.subCommand) {
                case CRSF_CMD_SCREEN_POPUP_MESSAGE_START: {
                    // Pack header string
                    uint8_t strLen = strnlen(in->screen.popupMessageStart.Header, CRSF_MAX_COMMAND_PAYLOAD_STRINGS);
                    // No check on the first item, as it is by design smaller than CRSF_MAX_COMMAND_PAYLOAD
                    memcpy(payload + off, in->screen.popupMessageStart.Header, strLen);
                    off += strLen;
                    payload[off++] = '\0';

                    // Pack info message string
                    strLen = strnlen(in->screen.popupMessageStart.Info_message, CRSF_MAX_COMMAND_PAYLOAD_STRINGS);
                    if ((off + strLen + 1U + 2U) >= CRSF_MAX_COMMAND_PAYLOAD) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    memcpy(payload + off, in->screen.popupMessageStart.Info_message, strLen);
                    off += strLen;
                    payload[off++] = '\0';

                    // Pack timeout and close button option
                    payload[off++] = in->screen.popupMessageStart.Max_timeout_interval;
                    payload[off++] = in->screen.popupMessageStart.Close_button_option ? 1U : 0U;

                    // Pack additional data if present
                    if (in->screen.popupMessageStart.add_data.present) {
                        strLen = strnlen(in->screen.popupMessageStart.add_data.selectionText, CRSF_MAX_COMMAND_PAYLOAD_STRINGS);
                        if ((off + strLen + 1 + 4 + strlen(in->screen.popupMessageStart.add_data.unit) + 1) >= CRSF_MAX_COMMAND_PAYLOAD) {
                            return CRSF_ERROR_TYPE_LENGTH;
                        }
                        memcpy(payload + off, in->screen.popupMessageStart.add_data.selectionText, strLen);
                        off += strLen;
                        payload[off++] = '\0';

                        payload[off++] = in->screen.popupMessageStart.add_data.value;
                        payload[off++] = in->screen.popupMessageStart.add_data.minValue;
                        payload[off++] = in->screen.popupMessageStart.add_data.maxValue;
                        payload[off++] = in->screen.popupMessageStart.add_data.defaultValue;

                        strLen = strnlen(in->screen.popupMessageStart.add_data.unit, 5U);
                        memcpy(payload + off, in->screen.popupMessageStart.add_data.unit, strLen);
                        off += strLen;
                        payload[off++] = '\0';
                    }

                    // Pack possible values if present
                    if (in->screen.popupMessageStart.has_possible_values) {
                        strLen = strnlen(in->screen.popupMessageStart.possible_values, CRSF_MAX_COMMAND_PAYLOAD_STRINGS);
                        if ((off + strLen + 1) >= CRSF_MAX_COMMAND_PAYLOAD) {
                            return CRSF_ERROR_TYPE_LENGTH;
                        }
                        memcpy(payload + off, in->screen.popupMessageStart.possible_values, strLen);
                        off += strLen;
                        payload[off++] = '\0';
                    }
                    break;
                }
                case CRSF_CMD_SCREEN_SELECTION_RETURN:
                    payload[off++] = in->screen.selectionReturn.value;
                    payload[off++] = in->screen.selectionReturn.response ? 1U : 0U;
                    break;
                default: break;
            }
            break;

        default: return CRSF_ERROR_INVALID_FRAME;
    }
    *length += off;
    return CRSF_OK;
}
#endif /* CRSF_ENABLE_COMMAND && && defined(CRSF_CONFIG_TX) */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static CRSF_Status_t CRSF_decodeCommandPayload(CRSF_CommandID_t commandID, CRSF_CommandPayload_t* out, const uint8_t* payload, uint8_t length) {
    uint8_t off = 0;

    switch (commandID) {
        case CRSF_CMDID_COMMAND_ACK:
            if (length < 3U) {
                return CRSF_ERROR_TYPE_LENGTH; // Command_ID, SubCommand_ID, Action
            }
            out->ACK.Command_ID = payload[off++];
            out->ACK.SubCommand_ID = payload[off++];
            out->ACK.Action = payload[off++];
            if (length > off + 1) {
                strncpy(out->ACK.Information, (char*)(payload + off), CRSF_MAX_COMMAND_PAYLOAD - 4U);
                out->ACK.Information[CRSF_MAX_COMMAND_PAYLOAD - 4U] = '\0';
            } else {
                out->ACK.Information[0] = '\0';
            }
            break;

        case CRSF_CMDID_FC:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->FC.subCommand = (CRSF_CommandFC_subCMD_t)payload[off];
            break;

        case CRSF_CMDID_BLUETOOTH:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->Bluetooth.subCommand = (CRSF_CommandBT_subCMD_t)payload[off++];
            if (out->Bluetooth.subCommand == CRSF_CMD_BT_ENABLE) {
                out->Bluetooth.Enable = payload[off];
            }
            break;

        case CRSF_CMDID_OSD:
            if (length < 2U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->OSD.subCommand = (CRSF_CommandOSD_subCMD_t)payload[off++];
            if (out->OSD.subCommand == CRSF_CMD_OSD_SEND_BUTTONS) {
                out->OSD.buttons = payload[off];
            }
            break;

        case CRSF_CMDID_VTX:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->VTX.subCommand = (CRSF_CommandVTX_subCMD_t)payload[off++];
            switch (out->VTX.subCommand) {
                case CRSF_CMD_VTX_SET_FREQUENCY:
                    if (length < off + 2U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->VTX.FrequencyMHz = CRSF_unpackBE16(payload + off);
                    break;
                case CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP:
                    if (length < off + 1U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->VTX.pitModeCfg = payload[off];
                    break;
                case CRSF_CMD_VTX_SET_DYNAMIC_POWER:
                case CRSF_CMD_VTX_SET_POWER:
                    if (length < off + 1U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->VTX.Power_dBm = payload[off];
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_LED:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->LED.subCommand = (CRSF_CommandLED_subCMD_t)payload[off++];
            switch (out->LED.subCommand) {
                case CRSF_CMD_LED_OVERRIDE_COLOR:
                    if (length < off + 3U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    CRSF_unpackHSV(payload + off, &out->LED.overrideColor.H, &out->LED.overrideColor.S, &out->LED.overrideColor.V);
                    break;
                case CRSF_CMD_LED_OVERRIDE_PULSE:
                    if (length < off + 8U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->LED.overridePulse.duration_ms = CRSF_unpackBE16(payload + off);
                    off += sizeof(uint16_t);
                    CRSF_unpackHSV(payload + off, &out->LED.overridePulse.H_start, &out->LED.overridePulse.S_start, &out->LED.overridePulse.V_start);
                    off += 3U;
                    CRSF_unpackHSV(payload + off, &out->LED.overridePulse.H_stop, &out->LED.overridePulse.S_stop, &out->LED.overridePulse.V_stop);
                    break;
                case CRSF_CMD_LED_OVERRIDE_BLINK:
                    if (length < off + 8U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->LED.overrideBlink.interval_ms = CRSF_unpackBE16(payload + off);
                    off += sizeof(uint16_t);
                    CRSF_unpackHSV(payload + off, &out->LED.overrideBlink.H_start, &out->LED.overrideBlink.S_start, &out->LED.overrideBlink.V_start);
                    off += 3U;
                    CRSF_unpackHSV(payload + off, &out->LED.overrideBlink.H_stop, &out->LED.overrideBlink.S_stop, &out->LED.overrideBlink.V_stop);
                    break;
                case CRSF_CMD_LED_OVERRIDE_SHIFT:
                    if (length < off + 5U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->LED.overrideShift.interval_ms = CRSF_unpackBE16(payload + off);
                    off += sizeof(uint16_t);
                    CRSF_unpackHSV(payload + off, &out->LED.overrideShift.H, &out->LED.overrideShift.S, &out->LED.overrideShift.V);
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_GENERAL:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->general.subCommand = (CRSF_CommandGen_subCMD_t)payload[off++];
            switch (out->general.subCommand) {
                case CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL:
                    if (length < off + 5U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->general.protocolSpeedProposal.port_id = payload[off++];
                    out->general.protocolSpeedProposal.proposed_baudrate = CRSF_unpackBE32(payload + off);
                    break;
                case CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE:
                    if (length < off + 2U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->general.protocolSpeedResponse.port_id = payload[off++];
                    out->general.protocolSpeedResponse.response = (payload[off] != 0);
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_CROSSFIRE:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->crossfire.subCommand = (CRSF_CommandCF_subCMD_t)payload[off++];
            switch (out->crossfire.subCommand) {
                case CRSF_CMD_CF_SET_BIND_ID:
                    memcpy(out->crossfire.setBindId.bytes, payload + off, length - off);
                    out->crossfire.setBindId.len = length - off;
                    break;
                case CRSF_CMD_CF_MODEL_SELECTION:
                case CRSF_CMD_CF_CURRENT_MODEL_REPLY:
                    if (length < off + 1U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->crossfire.Model_Number = payload[off];
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_FLOW_CTRL:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->flow.subCommand = (CRSF_CommandFlow_subCMD_t)payload[off++];
            switch (out->flow.subCommand) {
                case CRSF_CMD_FLOW_SUBSCRIBE:
                    if (length < off + 3U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->flow.Frame_type = payload[off++];
                    out->flow.Max_interval_time_ms = CRSF_unpackBE16(payload + off);
                    break;
                case CRSF_CMD_FLOW_UNSUBSCRIBE:
                    if (length < off + 1U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->flow.Frame_type = payload[off++];
                    break;
                default: break;
            }
            break;

        case CRSF_CMDID_SCREEN:
            if (length < 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->screen.subCommand = (CRSF_CommandScreen_subCMD_t)payload[off++];
            switch (out->screen.subCommand) {
                case CRSF_CMD_SCREEN_POPUP_MESSAGE_START: {
                    uint8_t strLen = 0;
                    strLen = strlen((char*)(payload + off)) + 1U;
                    strncpy(out->screen.popupMessageStart.Header, (char*)(payload + off),
                            ((strLen > CRSF_MAX_COMMAND_PAYLOAD_STRINGS) ? CRSF_MAX_COMMAND_PAYLOAD_STRINGS : strLen) - 1U);
                    out->screen.popupMessageStart.Header[CRSF_MAX_COMMAND_PAYLOAD_STRINGS - 1U] = '\0';
                    off += strLen;
                    strLen = strlen((char*)(payload + off)) + 1U;
                    strncpy(out->screen.popupMessageStart.Info_message, (char*)(payload + off),
                            ((strLen > CRSF_MAX_COMMAND_PAYLOAD_STRINGS) ? CRSF_MAX_COMMAND_PAYLOAD_STRINGS : strLen) - 1U);
                    out->screen.popupMessageStart.Info_message[CRSF_MAX_COMMAND_PAYLOAD_STRINGS - 1U] = '\0';
                    off += strLen;
                    out->screen.popupMessageStart.Max_timeout_interval = payload[off++];
                    out->screen.popupMessageStart.Close_button_option = (payload[off++] != 0);

                    // Additional data
                    strLen = strnlen((char*)(payload + off), length - off - 1U) + 1U;
                    out->screen.popupMessageStart.add_data.present = 0;
                    if ((uint8_t)(length - strLen - off) >= 4U) { // If additional data is present, it must contain value, minValue,maxValue and defaultValue
                        out->screen.popupMessageStart.add_data.present = 1U;
                        strncpy(out->screen.popupMessageStart.add_data.selectionText, (char*)(payload + off),
                                ((strLen > CRSF_MAX_COMMAND_PAYLOAD_STRINGS) ? CRSF_MAX_COMMAND_PAYLOAD_STRINGS : strLen) - 1U);
                        out->screen.popupMessageStart.add_data.selectionText[CRSF_MAX_COMMAND_PAYLOAD_STRINGS - 1U] = '\0';
                        off += strLen;
                        out->screen.popupMessageStart.add_data.value = payload[off++];
                        out->screen.popupMessageStart.add_data.minValue = payload[off++];
                        out->screen.popupMessageStart.add_data.maxValue = payload[off++];
                        out->screen.popupMessageStart.add_data.defaultValue = payload[off++];
                        strLen = strlen((char*)(payload + off)) + 1U;
                        strncpy(out->screen.popupMessageStart.add_data.unit, (char*)(payload + off), ((strLen > 5U) ? 5U : strLen) - 1U);
                        out->screen.popupMessageStart.add_data.unit[4] = '\0';
                        off += strLen;
                    }

                    // Possible values
                    out->screen.popupMessageStart.has_possible_values = 0;
                    if (length > off + 0U) {
                        out->screen.popupMessageStart.has_possible_values = 1;
                        strLen = strlen((char*)(payload + off)) + 1U;
                        strncpy(out->screen.popupMessageStart.possible_values, (char*)(payload + off),
                                ((strLen > CRSF_MAX_COMMAND_PAYLOAD_STRINGS) ? CRSF_MAX_COMMAND_PAYLOAD_STRINGS : strLen) - 1U);
                        out->screen.popupMessageStart.possible_values[CRSF_MAX_COMMAND_PAYLOAD_STRINGS - 1U] = '\0';
                    }

                    break;
                }
                case CRSF_CMD_SCREEN_SELECTION_RETURN:
                    if (length < off + 2U) {
                        return CRSF_ERROR_TYPE_LENGTH;
                    }
                    out->screen.selectionReturn.value = payload[off++];
                    out->screen.selectionReturn.response = (payload[off] != 0);
                    break;
                default: break;
            }
            break;

        default: break;
    }
    return CRSF_OK;
}
#endif /* CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX) */

static inline void CRSF_updateTimestamp(CRSF_t* crsf, uint8_t frame_type) {
#if CRSF_ENABLE_FRESHNESS_CHECK
    if (crsf && crsf->getTimestamp_ms && frame_type < CRSF_TRACKED_FRAME_TYPES) {
        crsf->_packet_times[frame_type] = crsf->getTimestamp_ms();
    }
#else
    (void)crsf;
    (void)frame_type;
#endif /* CRSF_ENABLE_FRESHNESS_CHECK */
}

/* Helpers -------------------------------------------------------------------*/

static inline void CRSF_packBE16(uint8_t* dest, const uint16_t value) {
    uint16_t be_value = HTOBE16(value);
    memcpy(dest, &be_value, sizeof(uint16_t));
}

static inline void CRSF_packBE32(uint8_t* dest, const uint32_t value) {
    uint32_t be_value = HTOBE32(value);
    memcpy(dest, &be_value, sizeof(uint32_t));
}

static inline void CRSF_packHSV(uint8_t* dest, uint16_t H, uint8_t S, uint8_t V) {
    uint32_t v = ((uint32_t)(H & 0x1FF) << 15) | ((uint32_t)(S & 0x7F) << 8) | (uint32_t)V;
    *dest++ = (uint8_t)(v >> 16);
    *dest++ = (uint8_t)(v >> 8);
    *dest++ = (uint8_t)v;
}

static inline uint16_t CRSF_unpackBE16(const uint8_t* src) {
    uint16_t value;
    memcpy(&value, src, sizeof(uint16_t));
    return BE16TOH(value);
}

static inline uint32_t CRSF_unpackBE32(const uint8_t* src) {
    uint32_t value;
    memcpy(&value, src, sizeof(uint32_t));
    return BE32TOH(value);
}

static inline void CRSF_unpackHSV(const uint8_t* src, uint16_t* H, uint8_t* S, uint8_t* V) {
    uint32_t v = ((uint32_t)src[0] << 16) | ((uint32_t)src[1] << 8) | src[2];
    *H = (uint16_t)((v >> 15) & 0x1FF);
    *S = (uint8_t)((v >> 8) & 0x7F);
    *V = (uint8_t)(v & 0xFF);
}
