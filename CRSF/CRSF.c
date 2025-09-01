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
#include <string.h>
#include "CRSF_CRC.h"
#include "CRSF_types.h"

/* Macros --------------------------------------------------------------------*/

#define PC16(X)                *(uint16_t*)&X
#define PC32(X)                *(uint32_t*)&X

// Helper macros
#define CRSF_RC_TICKS_TO_US(x) (((x) - 992) * 5 / 8 + 1500)
#define CRSF_RC_US_TO_TICKS(x) (((x) - 1500) * 8 / 5 + 992)

#define SIGN(x)                (((x) > 0) - ((x) < 0))

#define BUILD_FRAME(TYPE, VAR)                                                                                                                                                                         \
    case CRSF_FRAMETYPE_##TYPE: memcpy(payload, &crsf->VAR, sizeof(CRSF_##VAR##_t))
#define UPDATE_LENGTH(VAR)                                                                                                                                                                             \
    *frameLength += sizeof(CRSF_##VAR##_t);                                                                                                                                                            \
    break

#define PROCESS_FRAME(TYPE, VAR)                                                                                                                                                                       \
    case CRSF_FRAMETYPE_##TYPE: memcpy(&crsf->VAR, payload, sizeof(CRSF_##VAR##_t))
#define UPDATE_FRESHNESS(TYPE)                                                                                                                                                                         \
    if (CRSF_ENABLE_FRESHNESS_CHECK && (CRSF_TRK_FRAMETYPE_##TYPE < 0xFF)) {                                                                                                                           \
        CRSF_updateTimestamp(crsf, CRSF_TRK_FRAMETYPE_##TYPE);                                                                                                                                         \
    }                                                                                                                                                                                                  \
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

static inline void CRSF_updateTimestamp(CRSF_t* crsf, uint8_t frame_type);

// Helpers
static inline void CRSF_packBE16(uint8_t* dest, const uint16_t value);
static inline void CRSF_packBE32(uint8_t* dest, const uint32_t value);
static inline uint16_t CRSF_unpackBE16(const uint8_t* src);
static inline uint32_t CRSF_unpackBE32(const uint8_t* src);

/* Functions -----------------------------------------------------------------*/

void CRSF_init(CRSF_t* crsf) {
    if (!crsf) {
        return;
    }
    memset(crsf, 0, sizeof(*crsf));
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

    switch (type) {
#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_GPS:
            CRSF_packBE32(payload, crsf->GPS.latitude);
            CRSF_packBE32(payload + 4U, crsf->GPS.longitude);
            CRSF_packBE16(payload + 8U, crsf->GPS.groundspeed);
            CRSF_packBE16(payload + 10U, crsf->GPS.heading);
            CRSF_packBE16(payload + 12U, crsf->GPS.altitude);
            payload[14] = crsf->GPS.satellites;
            UPDATE_LENGTH(GPS);
#endif

#if CRSF_TEL_ENABLE_GPS_TIME && defined(CRSF_CONFIG_RX)
            BUILD_FRAME(GPS_TIME, GPS_Time);
            CRSF_packBE16(payload, crsf->GPS_Time.year);
            CRSF_packBE16(payload + 7U, crsf->GPS_Time.millisecond);
            UPDATE_LENGTH(GPS_Time);
#endif

#if CRSF_TEL_ENABLE_GPS_EXTENDED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_GPS_EXTENDED:
            payload[0] = crsf->GPS_Ext.fix_type;
            CRSF_packBE16(payload + 1U, crsf->GPS_Ext.n_speed);
            CRSF_packBE16(payload + 3U, crsf->GPS_Ext.e_speed);
            CRSF_packBE16(payload + 5U, crsf->GPS_Ext.v_speed);
            CRSF_packBE16(payload + 7U, crsf->GPS_Ext.h_speed_acc);
            CRSF_packBE16(payload + 9U, crsf->GPS_Ext.track_acc);
            CRSF_packBE16(payload + 11U, crsf->GPS_Ext.alt_ellipsoid);
            CRSF_packBE16(payload + 13U, crsf->GPS_Ext.h_acc);
            CRSF_packBE16(payload + 15U, crsf->GPS_Ext.v_acc);
            payload[17] = crsf->GPS_Ext.reserved;
            payload[18] = crsf->GPS_Ext.hDOP;
            payload[19] = crsf->GPS_Ext.vDOP;
            UPDATE_LENGTH(GPS_Ext);

#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_VARIO: CRSF_packBE16(payload, crsf->Vario.v_speed); UPDATE_LENGTH(Vario);
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            CRSF_packBE16(payload, crsf->Battery.voltage);
            CRSF_packBE16(payload + 2U, crsf->Battery.current);
            payload[4] = (uint8_t)(crsf->Battery.capacity_used >> 16);
            payload[5] = (uint8_t)((crsf->Battery.capacity_used >> 8) & 0xFFU);
            payload[6] = (uint8_t)((crsf->Battery.capacity_used) & 0xFFU);
            payload[7] = crsf->Battery.remaining;
            *frameLength += sizeof(CRSF_Battery_t) - 1;
            break;
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_BAROALT_VSPEED: CRSF_packBaroAltVSpeed(payload, &crsf->BaroAlt_VS); UPDATE_LENGTH(BaroAlt_VS);
#endif

#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_AIRSPEED: CRSF_packBE16(payload, crsf->Airspeed.speed); UPDATE_LENGTH(Airspeed);
#endif

#if CRSF_TEL_ENABLE_HEARTBEAT
        case CRSF_FRAMETYPE_HEARTBEAT: CRSF_packBE16(payload, crsf->Heartbeat.origin_address); UPDATE_LENGTH(Heartbeat);
#endif

#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_RPM: {
            payload[0] = crsf->RPM.rpm_source_id;
            *frameLength += 1U;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_RPM_VALUES; ii++) {
                uint32_t val = (uint32_t)(crsf->RPM.rpm_value[ii] & 0xFFFFFFU);
                payload[ii * 3 + 1] = (uint8_t)((val >> 16) & 0xFFU);
                payload[ii * 3 + 2] = (uint8_t)((val >> 8) & 0xFFU);
                payload[ii * 3 + 3] = (uint8_t)(val & 0xFFU);
                *frameLength += 3U; // int24_t size;
            }

            if (*frameLength < 4U) {
                payload[1] = 0;
                payload[2] = 0;
                payload[3] = 0;
                *frameLength += 3U;
            }
            break;
        }
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_TEMPERATURE:
            payload[0] = crsf->Temperature.temp_source_id;
            *frameLength += 1U;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_TEMPERATURE_VALUES; ii++) {
                CRSF_packBE16(payload + ii * 2 + 1U, crsf->Temperature.temperature[ii]);
                *frameLength += 2U;
            }

            if (*frameLength < 3U) {
                payload[1] = 0;
                payload[2] = 0;
                *frameLength += 2U;
            }
            break;
#endif

#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_VOLTAGES:
            payload[0] = crsf->Voltages.Voltage_source_id;
            *frameLength += 1U;

            for (uint8_t ii = 0; ii < values && ii < CRSF_MAX_VOLTAGE_VALUES; ii++) {
                CRSF_packBE16(payload + ii * 2 + 1U, crsf->Voltages.Voltage_values[ii]);
                *frameLength += 2U;
            }

            if (*frameLength < 3U) {
                payload[1] = 0;
                payload[2] = 0;
                *frameLength += 2U;
            }
            break;
#endif

#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_RX)
            BUILD_FRAME(VTX, VTX);
            CRSF_packBE16(payload + 2U, crsf->VTX.frequency_MHz);
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
            CRSF_packBE16(payload + 2U, crsf->Attitude.roll);
            CRSF_packBE16(payload + 4U, crsf->Attitude.yaw);
            UPDATE_LENGTH(Attitude);
#endif

#if CRSF_TEL_ENABLE_MAVLINK_FC
            BUILD_FRAME(MAVLINK_FC, MAVLinkFC);
            CRSF_packBE16(payload, crsf->MAVLinkFC.airspeed);
            CRSF_packBE32(payload + 3U, crsf->MAVLinkFC.custom_mode);
            UPDATE_LENGTH(MAVLinkFC);
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
        case CRSF_FRAMETYPE_FLIGHT_MODE:
            strcpy((char*)payload, crsf->FlightMode.flight_mode);
            *frameLength += strlen(crsf->FlightMode.flight_mode) + 1U; // Adding also null termination
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
            uint16_t nameLen = strlen(crsf->DeviceInfo.Device_name) + 1U; //Adding also null termination
            payload[0] = crsf->DeviceInfo.dest_address;
            payload[1] = crsf->DeviceInfo.origin_address;
            strcpy((char*)(payload + 2U), crsf->DeviceInfo.Device_name);
            CRSF_packBE32(payload + nameLen + 2U, crsf->DeviceInfo.Serial_number);
            CRSF_packBE32(payload + nameLen + 6U, crsf->DeviceInfo.Hardware_ID);
            CRSF_packBE32(payload + nameLen + 10U, crsf->DeviceInfo.Firmware_ID);
            payload[nameLen + 14U] = crsf->DeviceInfo.Parameters_total;
            payload[nameLen + 15U] = crsf->DeviceInfo.Parameter_version_number;
            *frameLength += nameLen + 16U;
            break;
        }

        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: {
            uint8_t paramLen = (values > CRSF_MAX_PARAM_SETTINGS_PAYLOAD ? CRSF_MAX_PARAM_SETTINGS_PAYLOAD : values) + 4U;
            memcpy(payload, &crsf->ParamSettingsEntry, paramLen);
            *frameLength += paramLen;
            if (paramLen == 4U) {
                payload[4] = 0;
                *frameLength += 1U;
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
                *frameLength += 1U;
            }
            break;
        }
#endif

#if CRSF_ENABLE_COMMAND
        case CRSF_FRAMETYPE_COMMAND: {
            uint8_t cmdLen = (values > CRSF_MAX_COMMAND_PAYLOAD ? CRSF_MAX_COMMAND_PAYLOAD : values) + 3U;
            memcpy(payload, &crsf->Command, cmdLen);
            payload[cmdLen] = CRSF_calcChecksumCMD(frame + 2U, cmdLen + 1U);
            *frameLength += cmdLen + 1U; //including also inner CRC
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
            CRSF_packBE32(payload + 4U, crsf->MAVLinkStat.sensor_enabled);
            CRSF_packBE32(payload + 8U, crsf->MAVLinkStat.sensor_health);
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
    switch (type) {

#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS:
            crsf->GPS.latitude = CRSF_unpackBE32(payload);
            crsf->GPS.longitude = CRSF_unpackBE32(payload + 4U);
            crsf->GPS.groundspeed = CRSF_unpackBE16(payload + 8U);
            crsf->GPS.heading = CRSF_unpackBE16(payload + 10U);
            crsf->GPS.altitude = CRSF_unpackBE16(payload + 12U);
            crsf->GPS.satellites = payload[14];
            UPDATE_FRESHNESS(GPS);
#endif

#if CRSF_TEL_ENABLE_GPS_TIME && defined(CRSF_CONFIG_TX)
            PROCESS_FRAME(GPS_TIME, GPS_Time);
            crsf->GPS_Time.year = CRSF_unpackBE16(payload);
            crsf->GPS_Time.millisecond = CRSF_unpackBE16(payload + 7U);
            UPDATE_FRESHNESS(GPS_TIME);
#endif

#if CRSF_TEL_ENABLE_GPS_EXTENDED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_GPS_EXTENDED:
            crsf->GPS_Ext.fix_type = payload[0];
            crsf->GPS_Ext.n_speed = CRSF_unpackBE16(payload + 1U);
            crsf->GPS_Ext.e_speed = CRSF_unpackBE16(payload + 3U);
            crsf->GPS_Ext.v_speed = CRSF_unpackBE16(payload + 5U);
            crsf->GPS_Ext.h_speed_acc = CRSF_unpackBE16(payload + 7U);
            crsf->GPS_Ext.track_acc = CRSF_unpackBE16(payload + 9U);
            crsf->GPS_Ext.alt_ellipsoid = CRSF_unpackBE16(payload + 11U);
            crsf->GPS_Ext.h_acc = CRSF_unpackBE16(payload + 13U);
            crsf->GPS_Ext.v_acc = CRSF_unpackBE16(payload + 15U);
            crsf->GPS_Ext.reserved = payload[17];
            crsf->GPS_Ext.hDOP = payload[18];
            crsf->GPS_Ext.vDOP = payload[19];
            UPDATE_FRESHNESS(GPS_EXTENDED);
#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VARIO: crsf->Vario.v_speed = CRSF_unpackBE16(payload); UPDATE_FRESHNESS(VARIO);
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            crsf->Battery.voltage = CRSF_unpackBE16(payload);
            crsf->Battery.current = CRSF_unpackBE16(payload + 2U);
            crsf->Battery.capacity_used = ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 8) | ((uint32_t)payload[6]);
            crsf->Battery.remaining = payload[7];
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
            uint8_t rpmCount = (payloadLength - sizeof(crsf->RPM.rpm_source_id)) / 3;
            crsf->RPM.rpm_source_id = payload[0];
            for (uint8_t ii = 0; ii < rpmCount && ii < CRSF_MAX_RPM_VALUES; ii++) {
                uint32_t val = ((uint32_t)payload[ii * 3 + 1] << 16) | ((uint32_t)payload[ii * 3 + 2] << 8) | ((uint32_t)payload[ii * 3 + 3]);
                crsf->RPM.rpm_value[ii] = val & 0x800000 ? (int32_t)(val | 0xFF000000) : (int32_t)val; // Sign extend if negative
            }
            UPDATE_FRESHNESS(RPM);
        }
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_TEMPERATURE: {
            uint8_t tempCount = (payloadLength - sizeof(crsf->Temperature.temp_source_id)) / 2;
            crsf->Temperature.temp_source_id = payload[0];
            for (uint8_t ii = 0; ii < tempCount && ii < CRSF_MAX_TEMPERATURE_VALUES; ii++) {
                crsf->Temperature.temperature[ii] = CRSF_unpackBE16(payload + ii * 2U + 1U);
            }
            UPDATE_FRESHNESS(TEMPERATURE);
        }
#endif

#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VOLTAGES: {
            uint8_t voltCount = (payloadLength - sizeof(crsf->Voltages.Voltage_source_id)) / 2;
            crsf->Voltages.Voltage_source_id = payload[0];
            for (uint8_t ii = 0; ii < voltCount && ii < CRSF_MAX_VOLTAGE_VALUES; ii++) {
                crsf->Voltages.Voltage_values[ii] = CRSF_unpackBE16(payload + ii * 2U + 1U);
            }
            UPDATE_FRESHNESS(VOLTAGES);
        }
#endif

#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_TX)
            PROCESS_FRAME(VTX, VTX);
            crsf->VTX.frequency_MHz = CRSF_unpackBE16(payload + 2U);
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
            crsf->Attitude.roll = CRSF_unpackBE16(payload + 2U);
            crsf->Attitude.yaw = CRSF_unpackBE16(payload + 4U);
            UPDATE_FRESHNESS(ATTITUDE);
#endif

#if CRSF_TEL_ENABLE_MAVLINK_FC
            PROCESS_FRAME(MAVLINK_FC, MAVLinkFC);
            crsf->MAVLinkFC.airspeed = CRSF_unpackBE16(payload);
            crsf->MAVLinkFC.custom_mode = CRSF_unpackBE32(payload + 3U);
            UPDATE_FRESHNESS(MAVLINK_FC);
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
        case CRSF_FRAMETYPE_FLIGHT_MODE: {
            strncpy(crsf->FlightMode.flight_mode, (char*)payload, CRSF_MAX_FLIGHT_MODE_NAME_LEN - 1); // Last charachter is always \0
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
            uint16_t nameLen = strlen((char*)(payload + 2U)) + 1U; // Including null termination
            crsf->DeviceInfo.dest_address = payload[0];
            crsf->DeviceInfo.origin_address = payload[1];
            strncpy(crsf->DeviceInfo.Device_name, (char*)(payload + 2U), CRSF_MAX_DEVICE_NAME_LEN - 1U); // Last charachter is always \0
            crsf->DeviceInfo.Serial_number = CRSF_unpackBE32(payload + nameLen + 2U);
            crsf->DeviceInfo.Hardware_ID = CRSF_unpackBE32(payload + nameLen + 6U);
            crsf->DeviceInfo.Firmware_ID = CRSF_unpackBE32(payload + nameLen + 10U);
            crsf->DeviceInfo.Parameters_total = payload[nameLen + 14U];
            crsf->DeviceInfo.Parameter_version_number = payload[nameLen + 15U];
            UPDATE_FRESHNESS(DEVICE_INFO);
        }
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            memcpy(&crsf->ParamSettingsEntry, payload, payloadLength < (CRSF_MAX_PARAM_SETTINGS_PAYLOAD + 4U) ? payloadLength : (CRSF_MAX_PARAM_SETTINGS_PAYLOAD + 4U));
            UPDATE_FRESHNESS(PARAMETER_SETTINGS_ENTRY);

            PROCESS_FRAME(PARAMETER_READ, ParamRead);
            UPDATE_FRESHNESS(PARAMETER_READ);

        case CRSF_FRAMETYPE_PARAMETER_WRITE:
            memcpy(&crsf->ParamWrite, payload, payloadLength < (CRSF_MAX_PARAM_DATA_LEN + 3U) ? payloadLength : (CRSF_MAX_PARAM_DATA_LEN + 3U));
            UPDATE_FRESHNESS(PARAMETER_WRITE);
#endif

#if CRSF_ENABLE_COMMAND
        case CRSF_FRAMETYPE_COMMAND:
            if (payload[payloadLength - 1U] != CRSF_calcChecksumCMD(payload - 1U, payloadLength)) {
                return CRSF_ERROR_CMD_CHECKSUM_FAIL;
            }
            memcpy(&crsf->Command, payload, (payloadLength - 1U) < (CRSF_MAX_COMMAND_PAYLOAD + 3U) ? (payloadLength - 1U) : (CRSF_MAX_COMMAND_PAYLOAD + 3U));
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
            crsf->MAVLinkStat.sensor_enabled = CRSF_unpackBE32(payload + 4U);
            crsf->MAVLinkStat.sensor_health = CRSF_unpackBE32(payload + 8U);
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
    memset(&crsf->Stats, 0, sizeof(crsf->Stats));
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
        case CRSF_FRAMETYPE_BAROALT_VSPEED: return CHECK_LENGTH(payloadLength, type, BaroAlt_VS);
#endif
#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_AIRSPEED: return CHECK_LENGTH(payloadLength, type, Airspeed);
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
        case CRSF_FRAMETYPE_HEARTBEAT: return CHECK_LENGTH(payloadLength, type, Heartbeat);
#endif
#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_RPM: return (payloadLength >= 4);
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_TEMPERATURE: return (payloadLength >= 3);
#endif
#if CRSF_TEL_ENABLE_VOLTAGES && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VOLTAGES: return (payloadLength >= 3);
#endif
#if CRSF_TEL_ENABLE_VTX && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_VTX: return CHECK_LENGTH(payloadLength, type, VTX);
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS && defined(CRSF_CONFIG_TX)
        case CRSF_FRAMETYPE_LINK_STATISTICS: return CHECK_LENGTH(payloadLength, type, LinkStatistics);
#endif
#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED: return (payloadLength >= 22);
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
        case CRSF_FRAMETYPE_FLIGHT_MODE: return (payloadLength >= 1 && payloadLength <= 16);
#endif
#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
        case CRSF_FRAMETYPE_ESP_NOW_MESSAGES: return CHECK_LENGTH(payloadLength, type, ESPNowMessages);
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
        case CRSF_FRAMETYPE_DEVICE_PING: return CHECK_LENGTH(payloadLength, type, Ping);
        case CRSF_FRAMETYPE_DEVICE_INFO: return (payloadLength >= 15);
        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY: return (payloadLength >= 5);
        case CRSF_FRAMETYPE_PARAMETER_READ: return CHECK_LENGTH(payloadLength, type, ParamRead);
        case CRSF_FRAMETYPE_PARAMETER_WRITE: return (payloadLength >= 2);
#endif
#if CRSF_ENABLE_COMMAND
        case CRSF_FRAMETYPE_COMMAND: return (payloadLength >= 3);
#endif
#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
        case CRSF_FRAMETYPE_MAVLINK_ENVELOPE: return (payloadLength >= 2 && payloadLength <= 60);
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
        vsPacked = (int8_t)roundf(logf(fabsf(baroAltVS->vertical_speed) / Kl + 1) / Kr) * SIGN(baroAltVS->vertical_speed);
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
    baroAltVS->vertical_speed = (expf(fabsf((float)vsPacked) * Kr) - 1.0f) * Kl * SIGN(vsPacked);
}

#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
static void CRSF_packRC(uint8_t* payload, const uint16_t* channels) {

#if CRSF_USE_RC_DIRECT_CONVERSION
    CRSF_RC_Channels_Packed_t* pack = (CRSF_RC_Channels_Packed_t*)payload;
    pack->channel_01 = CRSF_RC_US_TO_TICKS(channels[0]) & 0x7FFU;
    pack->channel_02 = CRSF_RC_US_TO_TICKS(channels[1]) & 0x7FFU;
    pack->channel_03 = CRSF_RC_US_TO_TICKS(channels[2]) & 0x7FFU;
    pack->channel_04 = CRSF_RC_US_TO_TICKS(channels[3]) & 0x7FFU;
    pack->channel_05 = CRSF_RC_US_TO_TICKS(channels[4]) & 0x7FFU;
    pack->channel_06 = CRSF_RC_US_TO_TICKS(channels[5]) & 0x7FFU;
    pack->channel_07 = CRSF_RC_US_TO_TICKS(channels[6]) & 0x7FFU;
    pack->channel_08 = CRSF_RC_US_TO_TICKS(channels[7]) & 0x7FFU;
    pack->channel_09 = CRSF_RC_US_TO_TICKS(channels[8]) & 0x7FFU;
    pack->channel_10 = CRSF_RC_US_TO_TICKS(channels[9]) & 0x7FFU;
    pack->channel_11 = CRSF_RC_US_TO_TICKS(channels[10]) & 0x7FFU;
    pack->channel_12 = CRSF_RC_US_TO_TICKS(channels[11]) & 0x7FFU;
    pack->channel_13 = CRSF_RC_US_TO_TICKS(channels[12]) & 0x7FFU;
    pack->channel_14 = CRSF_RC_US_TO_TICKS(channels[13]) & 0x7FFU;
    pack->channel_15 = CRSF_RC_US_TO_TICKS(channels[14]) & 0x7FFU;
    pack->channel_16 = CRSF_RC_US_TO_TICKS(channels[15]) & 0x7FFU;
#else
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

    if (bitcnt > 0) {
        *p = (uint8_t)bitbuf;
    }
#endif
}
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
static void CRSF_unpackRC(const uint8_t* payload, uint16_t* channels) {

#if CRSF_USE_RC_DIRECT_CONVERSION
    CRSF_RC_Channels_Packed_t* pack = (CRSF_RC_Channels_Packed_t*)payload;
    channels[0] = CRSF_RC_TICKS_TO_US(pack->channel_01);
    channels[1] = CRSF_RC_TICKS_TO_US(pack->channel_02);
    channels[2] = CRSF_RC_TICKS_TO_US(pack->channel_03);
    channels[3] = CRSF_RC_TICKS_TO_US(pack->channel_04);
    channels[4] = CRSF_RC_TICKS_TO_US(pack->channel_05);
    channels[5] = CRSF_RC_TICKS_TO_US(pack->channel_06);
    channels[6] = CRSF_RC_TICKS_TO_US(pack->channel_07);
    channels[7] = CRSF_RC_TICKS_TO_US(pack->channel_08);
    channels[8] = CRSF_RC_TICKS_TO_US(pack->channel_09);
    channels[9] = CRSF_RC_TICKS_TO_US(pack->channel_10);
    channels[10] = CRSF_RC_TICKS_TO_US(pack->channel_11);
    channels[11] = CRSF_RC_TICKS_TO_US(pack->channel_12);
    channels[12] = CRSF_RC_TICKS_TO_US(pack->channel_13);
    channels[13] = CRSF_RC_TICKS_TO_US(pack->channel_14);
    channels[14] = CRSF_RC_TICKS_TO_US(pack->channel_15);
    channels[15] = CRSF_RC_TICKS_TO_US(pack->channel_16);
#else
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
#endif
}
#endif

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
