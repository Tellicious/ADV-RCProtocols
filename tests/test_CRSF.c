/* BEGIN Header */
/**
 ******************************************************************************
 * \file            test_CRSF.c
 * \author          Andrea Vivani
 * \brief           CRSF protocol decoder test suite
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

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmocka.h>

#include "CRSF.h"

/* ============================================================================
 * CRSF test packets
 * ============================================================================ */

/* 0x02 GPS
 * lat = 45.463681°  ->  454636810 (int32, deg * 1e7, BE) → 0x1B19350A
 * lon = 9.188171°   ->   91881710 (int32, deg * 1e7, BE) → 0x057A00EE
 * gs  = 55.1 km/h   ->        551 (uint16, km/h * 10, BE) → 0x0227
 * hdg = 123.45°     ->      12345 (uint16, deg * 100, BE) → 0x3039
 * alt = 250 m       ->       1250 (uint16, meters + 1000, BE) → 0x04E2
 * sats = 9          ->       0x09
 */
const uint8_t test_gps_packet[] = {
    0xC8, 0x11, 0x02, 0x1B, 0x19, 0x35, 0x0A, // lat
    0x05, 0x7A, 0x00, 0xEE,                   // lon
    0x02, 0x27,                               // ground speed
    0x30, 0x39,                               // heading
    0x04, 0xE2,                               // altitude
    0x09,                                     // satellites
    0xD9                                      // CRC
};

/* 0x07 Variometer
 * v_speed = +20.12 m/s -> 2012 (int16, cm/s, BE) → 0x07DC
 */
const uint8_t test_vario_packet[] = {0xC8, 0x04, 0x07, 0x07, 0xDC, // v_speed
                                     0xE5};

/* 0x08 Battery Sensor  (per spec)
 * voltage  = 16.8 V ->  168 (uint16, dV, BE)      → 0x00A8
 * current  = 12.3 A ->  123 (uint16, dA, BE)      → 0x007B
 * used     = 1500 mAh (uint24, BE)                → 0x0005DC
 * remaining= 78 %  (uint8)
 */
const uint8_t test_battery_packet[] = {0xC8, 0x0A, 0x08, 0x00, 0xA8, // voltage = 168 *0.1V = 16.8V
                                       0x00, 0x7B,                   // current = 123 *0.1A = 12.3A
                                       0x00, 0x05, 0xDC,             // used = 1500 mAh
                                       0x4E,                         // remaining = 78%
                                       0x02};

/* 0x09 Barometric Altitude (& VSpeed)
 * altitude = 1234 dm -> 11234 (int16, decimeters + 10000, BE) → 0x2BE2
 * v_speed  = -150 cm/s -> -150 (int8, packed, BE)            → 0xDD
 */
const uint8_t test_baro_packet[] = {0xC8, 0x05, 0x09, 0x2B, 0xE2, // altitude
                                    0xDD,                         // v_speed
                                    0x03};

/* 0x0A Airspeed
 * airspeed = 36.0 km/h -> 360 (uint16, km/h * 10, BE) → 0x0168
 */
const uint8_t test_airspeed_packet[] = {0xC8, 0x04, 0x0A, 0x01, 0x68, 0x2E};

/* 0x0B Heartbeat (no payload) */
const uint8_t test_heartbeat_packet[] = {0xC8, 0x04, 0x0B, 0x00, 0xEA, 0x36};

/* 0x0C RPM
 * source_id = 1
 * rpm 0 = 15000 (int24, BE) → 0x003A98
 * rpm 1 = -122000 (int24, BE) → 0xFE2370
 */
const uint8_t test_rpm_packet[] = {0xC8, 0x09, 0x0C, 0x01, 0x00, 0x3A, 0x98, 0xFE, 0x23, 0x70, 0x58};

/* 0x0D Temperature
 * source_id = 11
 * temp 0 = 25.3 °C -> 253 (int16, 0.1 °C, BE) → 0x00FD
 * temp 1 = -12.1 °C -> -121 (int16, 0.1 °C, BE) → 0xFF87
 */
const uint8_t test_temp_packet[] = {0xC8, 0x07, 0x0D, 0x0B, 0x00, 0xFD, 0xFF, 0x87, 0x7F};

/* 0x14 Link Statistics
 * upRSSI1=0x41 (−65 dBm), upRSSI2=0x42 (−66 dBm), upLQ=98% (0x62), upSNR=−7 dB (0xF9)
 * ant=1, rfMode=3, upTxPwr=5
 * dnRSSI=0x46 (−70 dBm), dnLQ=99% (0x63), dnSNR=−9 dB (0xF7)
 */
const uint8_t test_linkstats_packet[] = {0xC8, 0x0C, 0x14, 0x41, 0x42, 0x62, 0xF9, 0x01, 0x03, 0x05, 0x46, 0x63, 0xF7, 0xEB};

/* 0x16 RC Channels Packed (16ch @ 1500)
 * ch0..15 = 1500 (11-bit, LSB-first across 22 bytes)
 */
const uint8_t test_rc_channels_packet[] = {0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F,
                                           0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xAD};

/* 0x1C Link Statistics RX
 * rssi = 86
 * rssi_percent = 92
 * link_quality = 80
 * snr = -10
 * rf_power = 14
 */
const uint8_t test_link_rx_id_packet[] = {0xC8, 0x07, 0x1C, 0x56, 0x5C, 0x50, 0xF6, 0x0E, 0x87};

/* 0x1D Link Statistics TX
 * rssi = 40
 * rssi_percent = 22
 * link_quality = 50
 * snr = -7
 * rf_power = 12
 * fps = 30
 */
const uint8_t test_link_tx_id_packet[] = {0xC8, 0x08, 0x1D, 0x28, 0x16, 0x32, 0xF9, 0x0C, 0x1E, 0xA3};

/* 0x1E Attitude
 * pitch = −0.78 rad -> −7800 (int16, rad * 10000, BE) → 0xE188
 * roll  = +0.12 rad ->  1200 (int16, rad * 10000, BE) → 0x04B0
 * yaw   = +1.57 rad -> 15700 (int16, rad * 10000, BE) → 0x3D54
 */
const uint8_t test_attitude_packet[] = {0xC8, 0x08, 0x1E, 0xE1, 0x88, 0x04, 0xB0, 0x3D, 0x54, 0x81};

/* 0x21 Flight Mode = "ANGLE" + NUL */
const uint8_t test_flightmode_packet[] = {0xC8, 0x08, 0x21, 'A', 'N', 'G', 'L', 'E', 0x00, 0x87};

/* ===== Extended frames (0x28+): payload begins [DEST, ORIGIN] ===== */

/* 0x28 Device Ping (DEST=BROADCAST(0x00), ORIGIN=RADIO_TX(0xEA)) */
const uint8_t test_device_ping_packet[] = {0xC8, 0x04, 0x28, 0x00, 0xEA, 0x54};

/* 0x29 Device Info (DEST=RADIO_TX, ORIGIN=FLIGHT_CONTROLLER)
 * serial=0x12345678, hw=0x00010002, sw=0x00030004, fieldCnt=5, paramVer=2, name="CRSF-DEV"
 */
const uint8_t test_device_info_packet[] = {0xC8, 0x1B, 0x29, 0xEA, 0xC8,                      // DEST, ORIGIN
                                           'C',  'R',  'S',  'F',  '-',  'D', 'E', 'V', 0x00, // name + NUL
                                           0x12, 0x34, 0x56, 0x78,                            // serial
                                           0x00, 0x01, 0x00, 0x02,                            // hw
                                           0x00, 0x03, 0x00, 0x04,                            // sw
                                           0x05, 0x02,                                        // fieldCnt, paramVer
                                           0xC6};

/* 0x2C Parameter Read request (DEST=FC, ORIGIN=RADIO_TX) number = 0x01, chunk_number=0x02 */
const uint8_t test_param_read_packet[] = {0xC8, 0x06, 0x2C, 0xC8, 0xEA, 0x01, 0x02, 0x3B};

/* 0x2D Parameter Write request (DEST=FC, ORIGIN=RADIO_TX) field id 0x0001, value=0x2A */
const uint8_t test_param_write_packet[] = {0xC8, 0x06, 0x2D, 0xC8, 0xEA, 0x01, 0x2A, 0x00};

/* 0x32 Command (DEST=CRSF_RX, ORIGIN=RADIO_TX) command_ID=Crossfire(0x10), Payload=BIND(0x01) */
const uint8_t test_command_packet[] = {0xC8, 0x07, 0x32, 0xEC, 0xEA, 0x10, 0x01, 0x34, 0xF6};

/* Test Constants and Helpers */

#if CRSF_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 12345;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

uint8_t test_calc_checksum(const uint8_t* data, uint8_t length, uint8_t poly) {
    uint8_t crc = 0x00; // Initialize CRC to 0
    for (size_t ii = 0; ii < length; ii++) {
        crc ^= data[ii];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void test_build_dummy_RC_frame(uint8_t* frame, uint8_t* frameLength) {
    memset(frame, 0x00, CRSF_MAX_FRAME_LEN + 2U);

    *frameLength = 22U + 3U + 1U;
    frame[0] = CRSF_ADDRESS_BROADCAST;
    frame[1] = 22U + 2U;
    frame[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    for (uint8_t ii = 0; ii < 22; ii++) {
        frame[3 + ii] = ii + 80;
    }
    frame[25] = test_calc_checksum(frame + 2U, 23U, 0xD5U);
}

/* ============================================================================
 * INITIALIZATION AND CONFIGURATION TESTS
 * ============================================================================ */

static void test_init_null(void** state) {
    (void)state;
    CRSF_init(NULL); // Should not crash
}

static void test_init_defaults_and_clear(void** state) {
    (void)state;
    CRSF_t crsf;
    memset(&crsf, 0xFF, sizeof(crsf));
    CRSF_init(&crsf);

    /* Spot-check a few fields are 0 */
    for (uint8_t ii = 0; ii < CRSF_RC_CHANNELS; ii++) {
        assert_int_equal(crsf.RC.channels[ii], 0);
    }

#if CRSF_ENABLE_STATS
    CRSF_Stats_t stats;
    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.commands_rx, 0);
    assert_int_equal(stats.frames_bad_addr, 0);
    assert_int_equal(stats.frames_bad_crc, 0);
    assert_int_equal(stats.frames_bad_len, 0);
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.frames_unsupported, 0);
#endif

#if CRSF_ENABLE_FRESHNESS_CHECK
    assert_true(crsf.getTimestamp_ms == NULL);
    for (int ii = 0; ii < CRSF_TRACKED_FRAME_TYPES; ii++) {
        assert_int_equal(crsf._packet_times[ii], 0);
    }
#endif
}

#if CRSF_ENABLE_FRESHNESS_CHECK
static void test_timestamp_callback_setting(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);

    // Test setting callback
    CRSF_setTimestampCallback(&crsf, test_getTimestamp_ms);
    assert_false(crsf.getTimestamp_ms == NULL);

    // Test with NULL state
    CRSF_setTimestampCallback(NULL, test_getTimestamp_ms); // Should not crash
}
#endif

/* ============================================================================
 * BASIC TESTS
 * ============================================================================ */

static void test_build_null_ptrs(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    assert_int_equal(CRSF_buildFrame(NULL, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, &frameLength), CRSF_ERROR_NULL_POINTER);
    assert_int_equal(CRSF_buildFrame(&crsf, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, NULL, &frameLength), CRSF_ERROR_NULL_POINTER);
    assert_int_equal(CRSF_buildFrame(&crsf, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, NULL), CRSF_ERROR_NULL_POINTER);
}

static void test_process_null_ptrs(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U] = {0};
    CRSF_FrameType_t frameType = 0;
    assert_int_equal(CRSF_processFrame(NULL, frame, &frameType), CRSF_ERROR_NULL_POINTER);
    assert_int_equal(CRSF_processFrame(&crsf, NULL, &frameType), CRSF_ERROR_NULL_POINTER);
    assert_int_equal(CRSF_processFrame(&crsf, frame, NULL), CRSF_ERROR_NULL_POINTER);
}

#if CRSF_ENABLE_ADDRESS_VALIDATION
static void test_all_valid_addresses(void** state) {
    (void)state;
    CRSF_t s;
    CRSF_init(&s);
    uint8_t buf[128];
    uint8_t frameLength = 0;

    /* Test all valid addresses from the enum */
    CRSF_Address_t valid_addresses[] = {
        CRSF_ADDRESS_BROADCAST,
        CRSF_ADDRESS_USB,
        CRSF_ADDRESS_TBS_CORE_PNP_PRO,
        CRSF_ADDRESS_CURRENT_SENSOR,
        CRSF_ADDRESS_GPS,
        CRSF_ADDRESS_TBS_BLACKBOX,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_ADDRESS_RACE_TAG,
        CRSF_ADDRESS_RADIO_TRANSMITTER,
        CRSF_ADDRESS_CRSF_RECEIVER,
        CRSF_ADDRESS_CRSF_TRANSMITTER,
    };

    for (size_t ii = 0; ii < sizeof(valid_addresses) / sizeof(valid_addresses[0]); ii++) {
        frameLength = 0;
        assert_int_not_equal(CRSF_buildFrame(&s, valid_addresses[ii], CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, buf, &frameLength), CRSF_ERROR_ADDR);
    }

    /* Test some invalid addresses */
    uint8_t invalid_addresses[] = {0x01, 0x02, 0x50, 0xAA, 0xFF};
    for (size_t ii = 0; ii < sizeof(invalid_addresses) / sizeof(invalid_addresses[0]); ii++) {
        frameLength = 0;
        assert_int_equal(CRSF_buildFrame(&s, invalid_addresses[ii], CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, buf, &frameLength), CRSF_ERROR_ADDR);
    }
}

static void test_build_invalid_address(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    assert_int_equal(CRSF_buildFrame(&crsf, 0xFF, CRSF_FRAMETYPE_HEARTBEAT, 0, frame, &frameLength), CRSF_ERROR_ADDR);
}

static void test_process_invalid_address(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    CRSF_FrameType_t frameType = 0;
    frame[0] = 0xFF;
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_ADDR);
}
#endif

static void test_build_invalid_frame(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    assert_int_equal(CRSF_buildFrame(&crsf, CRSF_ADDRESS_BROADCAST, (CRSF_FrameType_t)0xFF, 0, frame, &frameLength), CRSF_ERROR_INVALID_FRAME);
}

static void test_process_invalid_frame(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    CRSF_FrameType_t frameType = 0;
    uint8_t frameLength = 0;
    test_build_dummy_RC_frame(frame, &frameLength);
    frame[2] = 0xFF;
    frame[frameLength - 1U] = test_calc_checksum(frame + 2U, frameLength - 3U, 0xD5U); // re-calculate checksum
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_INVALID_FRAME);
}

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_valid_lengths(void** state) {
    (void)state;
    CRSF_t s;
    CRSF_init(&s);
    CRSF_FrameType_t frameType = 0;
    uint8_t frame[128];

    /* Test each frame type's length validation by constructing invalid frames */
    struct {
        CRSF_FrameType_t type;
        uint8_t min_payload_len;
        uint8_t valid_payload_len;
    } frame_tests[] = {{CRSF_FRAMETYPE_GPS, sizeof(CRSF_GPS_t), sizeof(CRSF_GPS_t)},
                       {CRSF_FRAMETYPE_GPS_TIME, sizeof(CRSF_GPS_Time_t), sizeof(CRSF_GPS_Time_t)},
                       {CRSF_FRAMETYPE_GPS_EXTENDED, sizeof(CRSF_GPS_Ext_t), sizeof(CRSF_GPS_Ext_t)},
                       {CRSF_FRAMETYPE_VARIO, sizeof(CRSF_Vario_t), sizeof(CRSF_Vario_t)},
                       {CRSF_FRAMETYPE_BATTERY_SENSOR, sizeof(CRSF_Battery_t) - 1U, sizeof(CRSF_Battery_t) - 1U},
                       {CRSF_FRAMETYPE_BAROALT_VSPEED, 3, 3},
                       {CRSF_FRAMETYPE_AIRSPEED, sizeof(CRSF_Airspeed_t), sizeof(CRSF_Airspeed_t)},
                       {CRSF_FRAMETYPE_HEARTBEAT, sizeof(CRSF_Heartbeat_t), sizeof(CRSF_Heartbeat_t)},
                       {CRSF_FRAMETYPE_RPM, 4, 7},         /* minimum: id + 1 rpm value */
                       {CRSF_FRAMETYPE_TEMPERATURE, 3, 5}, /* minimum: id + 1 temp value */
                       {CRSF_FRAMETYPE_VOLTAGES, 3, 5},    /* minimum: id + 1 voltage value */
                       {CRSF_FRAMETYPE_VTX, sizeof(CRSF_VTX_t), sizeof(CRSF_VTX_t)},
                       {CRSF_FRAMETYPE_LINK_STATISTICS, sizeof(CRSF_LinkStatistics_t), sizeof(CRSF_LinkStatistics_t)},
                       {CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 22, 22},
                       {CRSF_FRAMETYPE_LINK_STATISTICS_RX, sizeof(CRSF_LinkStatisticsRX_t), sizeof(CRSF_LinkStatisticsRX_t)},
                       {CRSF_FRAMETYPE_LINK_STATISTICS_TX, sizeof(CRSF_LinkStatisticsTX_t), sizeof(CRSF_LinkStatisticsTX_t)},
                       {CRSF_FRAMETYPE_ATTITUDE, sizeof(CRSF_Attitude_t), sizeof(CRSF_Attitude_t)},
                       {CRSF_FRAMETYPE_MAVLINK_FC, sizeof(CRSF_MAVLinkFC_t), sizeof(CRSF_MAVLinkFC_t)},
                       {CRSF_FRAMETYPE_FLIGHT_MODE, 1, 16},
                       {CRSF_FRAMETYPE_ESP_NOW_MESSAGES, sizeof(CRSF_ESPNowMessages_t), sizeof(CRSF_ESPNowMessages_t)},
                       {CRSF_FRAMETYPE_DEVICE_PING, sizeof(CRSF_Ping_t), sizeof(CRSF_Ping_t)},
                       {CRSF_FRAMETYPE_DEVICE_INFO, 15, 20},
                       {CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 5, 10},
                       {CRSF_FRAMETYPE_PARAMETER_READ, sizeof(CRSF_ParamRead_t), sizeof(CRSF_ParamRead_t)},
                       {CRSF_FRAMETYPE_PARAMETER_WRITE, 2, 5},
                       {CRSF_FRAMETYPE_COMMAND, 3, 6},
                       {CRSF_FRAMETYPE_MAVLINK_ENVELOPE, 2, 10},
                       {CRSF_FRAMETYPE_MAVLINK_STATUS, sizeof(CRSF_MAVLinkStat_t), sizeof(CRSF_MAVLinkStat_t)}};

    for (uint8_t ii = 0; ii < sizeof(frame_tests) / sizeof(frame_tests[0]); ii++) {
        /* Test frame too short */
        frame[0] = CRSF_ADDRESS_BROADCAST;
        frame[1] = frame_tests[ii].min_payload_len + 2U - 1U; // +1 for type +1 for CRC, -1 to make it invalid
        frame[2] = frame_tests[ii].type;
        memset(&frame[3], 0x00, frame[1] - 2U); /* fill payload */
        frame[frame[1] + 1U] = test_calc_checksum(frame + 2U, frame[1] - 1U, 0xD5U);

        assert_int_equal(CRSF_processFrame(&s, frame, &frameType), CRSF_ERROR_TYPE_LENGTH);

        /* Test valid frame length */
        frame[1] = frame_tests[ii].valid_payload_len + 2U; /* +1 for type +1 for CRC */
        memset(&frame[3], 0x00, frame[1] - 2U);            /* fill payload */
        frame[frame[1] + 1U] = test_calc_checksum(&frame[2], frame[1] - 1, 0xD5U);

        /* This might fail due to content but should pass length validation */
        assert_true(CRSF_processFrame(&s, frame, &frameType) != CRSF_ERROR_TYPE_LENGTH);
    }
}
#endif

static void test_process_wrong_length(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    CRSF_FrameType_t frameType = 0;
    uint8_t frameLength = 0;
    test_build_dummy_RC_frame(frame, &frameLength);

    frame[1] = 0xFF; // Bigger than maximum
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_LENGTH);
    frame[1] = 1; // Smaller than minimum
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_LENGTH);
    frame[1] = 4; // Wrong for RC frame
#ifdef CRSF_CONFIG_RX
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_TYPE_LENGTH);
#endif
}

static void test_process_wrong_CRC(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    CRSF_FrameType_t frameType = 0;
    uint8_t frameLength = 0;
    test_build_dummy_RC_frame(frame, &frameLength);

    frame[frameLength - 3U] += 1;
    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_CHECKSUM_FAIL);
}

/* ============================================================================
 * STATISTICS TESTS
 * ============================================================================ */

#if CRSF_ENABLE_STATS
static void test_stats_getter_and_reset(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);

    CRSF_Stats_t stats;
    CRSF_getStats(&crsf, &stats);

    // Initial state
    assert_int_equal(stats.commands_rx, 0);
    assert_int_equal(stats.frames_bad_addr, 0);
    assert_int_equal(stats.frames_bad_crc, 0);
    assert_int_equal(stats.frames_bad_len, 0);
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.frames_unsupported, 0);

    // Process some data to change stats
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength;
    CRSF_FrameType_t frameType;
    test_build_dummy_RC_frame(frame, &frameLength);
    CRSF_processFrame(&crsf, frame, &frameType);

    // Create bad frame
    frame[1] = 1; // Bad length
    CRSF_processFrame(&crsf, frame, &frameType);

    CRSF_getStats(&crsf, &stats);

    assert_int_equal(stats.frames_total, 2);
    assert_int_equal(stats.frames_bad_len, 1);

    // Test reset
    CRSF_resetStats(&crsf);
    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.commands_rx, 0);
    assert_int_equal(stats.frames_bad_addr, 0);
    assert_int_equal(stats.frames_bad_crc, 0);
    assert_int_equal(stats.frames_bad_len, 0);
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.frames_unsupported, 0);
}

static void test_stats_null_arguments(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);

    CRSF_Stats_t stats;

    // Test NULL arguments - should not crash
    CRSF_getStats(NULL, &stats);
    CRSF_getStats(&crsf, NULL);
    CRSF_resetStats(NULL);
}

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_stats_comprehensive_counting(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);

    CRSF_Stats_t stats;

    // Process some data to change stats
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength;
    CRSF_FrameType_t frameType;
    uint8_t tmp = 0;
    test_build_dummy_RC_frame(frame, &frameLength);

    // Good servo frames
    for (int i = 0; i < 3; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }

    // Create bad frames
    tmp = frame[1];

    frame[1] = 1; // Bad length
    for (int i = 0; i < 2; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }

    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.frames_total, 5);
    assert_int_equal(stats.frames_bad_len, 2);

    frame[1] = 21; // Bad type length

    for (int i = 0; i < 2; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }

    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.frames_total, 7);
    assert_int_equal(stats.frames_bad_len, 4);
    frame[1] = tmp;

    tmp = frame[frameLength - 1U];
    frame[frameLength - 1U] += 1; // Bad CRC
    for (int i = 0; i < 3; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }

    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.frames_total, 10);
    assert_int_equal(stats.frames_bad_len, 4);
    assert_int_equal(stats.frames_bad_crc, 3);
    frame[frameLength - 1U] = tmp;

    tmp = frame[2];
    frame[2] = 0xFF; // Bad type
    frame[frameLength - 1] = test_calc_checksum(frame + 2U, frameLength - 3U, 0xD5U);
    for (int i = 0; i < 3; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }
    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.frames_total, 13);
    assert_int_equal(stats.frames_bad_len, 4);
    assert_int_equal(stats.frames_bad_crc, 3);
    assert_int_equal(stats.frames_unsupported, 3);
    frame[2] = tmp;
    frame[frameLength - 1] = test_calc_checksum(frame + 2U, frameLength - 3U, 0xD5U);

#if CRSF_ENABLE_ADDRESS_VALIDATION
    tmp = frame[0];
    frame[0] = 0xFF; // Bad Address
    for (int i = 0; i < 3; i++) {
        CRSF_processFrame(&crsf, frame, &frameType);
    }
    CRSF_getStats(&crsf, &stats);
    assert_int_equal(stats.frames_total, 16);
    assert_int_equal(stats.frames_bad_len, 4);
    assert_int_equal(stats.frames_bad_crc, 3);
    assert_int_equal(stats.frames_unsupported, 3);
    assert_int_equal(stats.frames_bad_addr, 3);
    frame[0] = tmp;
#endif
}
#endif
#endif

/* ============================================================================
 * FRESHNESS CHECKING TESTS
 * ============================================================================ */

#if CRSF_ENABLE_FRESHNESS_CHECK
static void test_freshness_basic_functionality(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    CRSF_setTimestampCallback(&crsf, test_getTimestamp_ms);

    // Initially no frames should be fresh
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 100));
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_GPS, 100));

    // Process some data
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength;
    CRSF_FrameType_t frameType;
    test_build_dummy_RC_frame(frame, &frameLength);
    CRSF_processFrame(&crsf, frame, &frameType);

    // Should be fresh immediately
    assert_true(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 0));
    assert_true(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 100));
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_GPS, 100)); // Different type

    // Advance time
    mock_timestamp += 50;
    assert_true(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 100)); // Still fresh
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 30)); // Too old

    // Advance time further
    mock_timestamp += 100;
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 100)); // Now stale
}

static void test_freshness_invalid_arguments(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    CRSF_setTimestampCallback(&crsf, test_getTimestamp_ms);

    // Process some data
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength;
    CRSF_FrameType_t frameType;
    test_build_dummy_RC_frame(frame, &frameLength);
    CRSF_processFrame(&crsf, frame, &frameType);

    // Test invalid frame type
    assert_false(CRSF_isFrameFresh(&crsf, CRSF_TRACKED_FRAME_TYPES, 10));
    assert_false(CRSF_isFrameFresh(&crsf, 255, 10));

    // Test NULL state
    assert_false(CRSF_isFrameFresh(NULL, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 10));

    // Test no callback set
    CRSF_t no_callback;
    CRSF_init(&no_callback);
    assert_false(CRSF_isFrameFresh(&no_callback, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 10));
}

static void test_freshness_edge_cases(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    CRSF_setTimestampCallback(&crsf, test_getTimestamp_ms);

    // Process some data
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength;
    CRSF_FrameType_t frameType;
    test_build_dummy_RC_frame(frame, &frameLength);
    CRSF_processFrame(&crsf, frame, &frameType);

    assert_true(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 0));

    // Test timestamp wrap-around scenario
    mock_timestamp = UINT32_MAX - 10;
    CRSF_processFrame(&crsf, frame, &frameType);

    mock_timestamp = 10; // Wrapped around
    assert_true(CRSF_isFrameFresh(&crsf, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 30));
}
#endif

/* ============================================================================
 * BUILD TESTS
 * ============================================================================ */

#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_RX)
static void test_build_gps(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.GPS.latitude = 454636810; // 45.463681° * 1e7
    crsf.GPS.longitude = 91881710; //  9.188171° * 1e7
    crsf.GPS.groundspeed = 551;    // 55.1 km/h * 10
    crsf.GPS.heading = 12345;      // 123.45° * 100
    crsf.GPS.altitude = 1250;      // meters + 1000
    crsf.GPS.satellites = 9;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_GPS, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_gps_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_RX)
static void test_build_vario(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Vario.v_speed = (int16_t)(20.12f * 100); // cm/s
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_VARIO, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_vario_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_RX)
static void test_build_battery(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Battery.voltage = 168;        // 16.8V
    crsf.Battery.current = 123;        // 12.3A
    crsf.Battery.capacity_used = 1500; // 1500 mAh
    crsf.Battery.remaining = 78;       // 78%
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BATTERY_SENSOR, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_battery_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
static void test_build_baro(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.BaroAlt_VS.altitude = 1234; // 1234 dm
    crsf.BaroAlt_VS.vertical_speed = -150;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BAROALT_VSPEED, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_baro_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_RX)
static void test_build_airspeed(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Airspeed.speed = 360; // 36.0 km/h
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_AIRSPEED, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_airspeed_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_HEARTBEAT
static void test_build_heartbeat(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Heartbeat.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_HEARTBEAT, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_heartbeat_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_RX)
static void test_build_rpm(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.RPM.rpm_source_id = 1;
    crsf.RPM.rpm_value[0] = 15000;
    crsf.RPM.rpm_value[1] = -122000;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RPM, 2, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_rpm_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_RX)
static void test_build_temp(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Temperature.temp_source_id = 11;
    crsf.Temperature.temperature[0] = 25.3 * 10;
    crsf.Temperature.temperature[1] = -12.1 * 10;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_TEMPERATURE, 2, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_temp_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS
static void test_build_linkstats(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.LinkStatistics.up_rssi_ant1 = 0x41;
    crsf.LinkStatistics.up_rssi_ant2 = 0x42;
    crsf.LinkStatistics.up_link_quality = 0x62;
    crsf.LinkStatistics.up_snr = -7;
    crsf.LinkStatistics.active_antenna = 1;
    crsf.LinkStatistics.rf_profile = 3;
    crsf.LinkStatistics.up_rf_power = 5;
    crsf.LinkStatistics.down_rssi = 0x46;
    crsf.LinkStatistics.down_link_quality = 0x63;
    crsf.LinkStatistics.down_snr = -9;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_linkstats_packet[ii]);
    }
}
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
static void test_build_rc_channels(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    for (int ch = 0; ch < 16; ch++) {
        crsf.RC.channels[ch] = 1500;
    }
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_rc_channels_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
static void test_build_link_rx_id(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.LinkStatisticsRX.rssi_db = 86;
    crsf.LinkStatisticsRX.rssi_percent = 92;
    crsf.LinkStatisticsRX.link_quality = 80;
    crsf.LinkStatisticsRX.snr = -10;
    crsf.LinkStatisticsRX.rf_power_db = 14;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS_RX, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_link_rx_id_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
static void test_build_link_tx_id(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.LinkStatisticsTX.rssi_db = 40;
    crsf.LinkStatisticsTX.rssi_percent = 22;
    crsf.LinkStatisticsTX.link_quality = 50;
    crsf.LinkStatisticsTX.snr = -7;
    crsf.LinkStatisticsTX.rf_power_db = 12;
    crsf.LinkStatisticsTX.fps = 30;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_LINK_STATISTICS_TX, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_link_tx_id_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_RX)
static void test_build_attitude(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Attitude.roll = 1200; // rad * 10000
    crsf.Attitude.pitch = -7800;
    crsf.Attitude.yaw = 15700;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_ATTITUDE, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_attitude_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
static void test_build_flightmode(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    strcpy((char*)crsf.FlightMode.flight_mode, "ANGLE");
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_FLIGHT_MODE, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_flightmode_packet[ii]);
    }
}
#endif

#if CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_build_device_ping(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Ping.dest_address = CRSF_ADDRESS_BROADCAST;
    crsf.Ping.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_DEVICE_PING, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_device_ping_packet[ii]);
    }
}

static void test_build_device_info(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.DeviceInfo.dest_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    crsf.DeviceInfo.origin_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsf.DeviceInfo.Serial_number = 0x12345678;
    crsf.DeviceInfo.Hardware_ID = 0x00010002;
    crsf.DeviceInfo.Firmware_ID = 0x00030004;
    crsf.DeviceInfo.Parameters_total = 5;
    crsf.DeviceInfo.Parameter_version_number = 2;
    strcpy((char*)crsf.DeviceInfo.Device_name, "CRSF-DEV");
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_DEVICE_INFO, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_device_info_packet[ii]);
    }
}

static void test_build_param_read(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.ParamRead.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsf.ParamRead.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    crsf.ParamRead.Parameter_number = 1;
    crsf.ParamRead.Parameter_chunk_number = 2;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_PARAMETER_READ, 0, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_param_read_packet[ii]);
    }
}

static void test_build_param_write(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.ParamWrite.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    crsf.ParamWrite.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    crsf.ParamWrite.Parameter_number = 1;
    crsf.ParamWrite.Data[0] = 0x2A;
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_PARAMETER_WRITE, 1, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_param_write_packet[ii]);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_command(void** state) {
    (void)state;
    CRSF_t crsf;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_init(&crsf);
    crsf.Command.dest_address = CRSF_ADDRESS_CRSF_RECEIVER;
    crsf.Command.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    crsf.Command.Command_ID = CRSF_CMDID_CROSSFIRE;                           // FC
    crsf.Command.payload.crossfire.subCommand = CRSF_CMD_CF_SET_RX_BIND_MODE; // Force Disarm
    assert_true(CRSF_buildFrame(&crsf, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 1, frame, &frameLength) == CRSF_OK);
    for (uint8_t ii = 0; ii < frameLength; ii++) {
        assert_int_equal(frame[ii], test_command_packet[ii]);
    }
}
#endif

/* ============================================================================
 * PROCESS TESTS
 * ============================================================================ */

#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_TX)
static void test_process_gps(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_gps_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_GPS);
    assert_int_equal(crsf.GPS.latitude, 454636810);
    assert_int_equal(crsf.GPS.longitude, 91881710);
    assert_int_equal(crsf.GPS.groundspeed, 551);
    assert_int_equal(crsf.GPS.heading, 12345);
    assert_int_equal(crsf.GPS.altitude, 1250);
    assert_int_equal(crsf.GPS.satellites, 9);
}
#endif

#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_TX)
static void test_process_vario(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_vario_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_VARIO);
    assert_int_equal(crsf.Vario.v_speed, (int16_t)(20.12f * 100));
}
#endif

#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_TX)
static void test_process_battery(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_battery_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_BATTERY_SENSOR);
    assert_int_equal(crsf.Battery.voltage, 168);
    assert_int_equal(crsf.Battery.current, 123);
    assert_int_equal(crsf.Battery.capacity_used, 1500);
    assert_int_equal(crsf.Battery.remaining, 78);
}
#endif

#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
static void test_process_baro(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_baro_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_BAROALT_VSPEED);
    assert_int_equal(crsf.BaroAlt_VS.altitude, 1234);
    assert_int_in_range(crsf.BaroAlt_VS.vertical_speed, -155, -145);
}
#endif

#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_TX)
static void test_process_airspeed(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_airspeed_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_AIRSPEED);
    assert_int_equal(crsf.Airspeed.speed, 360);
}
#endif

#if CRSF_TEL_ENABLE_HEARTBEAT
static void test_process_heartbeat(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_heartbeat_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_HEARTBEAT);
}
#endif

#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_TX)
static void test_process_rpm(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_rpm_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_RPM);
    assert_int_equal(crsf.RPM.rpm_source_id, 1);
    assert_int_equal(crsf.RPM.rpm_value[0], 15000);
    assert_int_equal(crsf.RPM.rpm_value[1], -122000);
}
#endif

#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
static void test_process_temp(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_temp_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_TEMPERATURE);
    assert_int_equal(crsf.Temperature.temp_source_id, 11);
    assert_int_equal(crsf.Temperature.temperature[0], 25.3 * 10);
    assert_int_equal(crsf.Temperature.temperature[1], -12.1 * 10);
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS
static void test_process_linkstats(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_linkstats_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_LINK_STATISTICS);
    assert_int_equal(crsf.LinkStatistics.up_rssi_ant1, 0x41);
    assert_int_equal(crsf.LinkStatistics.up_rssi_ant2, 0x42);
    assert_int_equal(crsf.LinkStatistics.up_link_quality, 0x62);
    assert_int_equal(crsf.LinkStatistics.up_snr, (int8_t)0xF9);
    assert_int_equal(crsf.LinkStatistics.active_antenna, 1);
    assert_int_equal(crsf.LinkStatistics.rf_profile, 3);
    assert_int_equal(crsf.LinkStatistics.up_rf_power, 5);
    assert_int_equal(crsf.LinkStatistics.down_rssi, 0x46);
    assert_int_equal(crsf.LinkStatistics.down_link_quality, 0x63);
    assert_int_equal(crsf.LinkStatistics.down_snr, (int8_t)0xF7);
}
#endif

#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
static void test_process_rc_channels(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_rc_channels_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_RC_CHANNELS_PACKED);
    for (uint8_t ch = 0; ch < CRSF_RC_CHANNELS; ch++) {
        assert_int_equal(crsf.RC.channels[ch], 1500);
    }
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
static void test_process_link_rx_id(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_link_rx_id_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_LINK_STATISTICS_RX);
    assert_int_equal(crsf.LinkStatisticsRX.rssi_db, 86);
    assert_int_equal(crsf.LinkStatisticsRX.rssi_percent, 92);
    assert_int_equal(crsf.LinkStatisticsRX.link_quality, 80);
    assert_int_equal(crsf.LinkStatisticsRX.snr, -10);
    assert_int_equal(crsf.LinkStatisticsRX.rf_power_db, 14);
}
#endif

#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
static void test_process_link_tx_id(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_link_tx_id_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_LINK_STATISTICS_TX);
    assert_int_equal(crsf.LinkStatisticsTX.rssi_db, 40);
    assert_int_equal(crsf.LinkStatisticsTX.rssi_percent, 22);
    assert_int_equal(crsf.LinkStatisticsTX.link_quality, 50);
    assert_int_equal(crsf.LinkStatisticsTX.snr, -7);
    assert_int_equal(crsf.LinkStatisticsTX.rf_power_db, 12);
    assert_int_equal(crsf.LinkStatisticsTX.fps, 30);
}
#endif

#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_TX)
static void test_process_attitude(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_attitude_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_ATTITUDE);
    assert_int_equal(crsf.Attitude.roll, 1200);
    assert_int_equal(crsf.Attitude.pitch, -7800);
    assert_int_equal(crsf.Attitude.yaw, 15700);
}
#endif

#if CRSF_TEL_ENABLE_FLIGHT_MODE
static void test_process_flightmode(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_flightmode_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_FLIGHT_MODE);
    assert_string_equal(crsf.FlightMode.flight_mode, "ANGLE");
}
#endif

#if CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_process_device_ping(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_device_ping_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_DEVICE_PING);
    assert_int_equal(crsf.Ping.dest_address, CRSF_ADDRESS_BROADCAST);
    assert_int_equal(crsf.Ping.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
}

static void test_process_device_info(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_device_info_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_DEVICE_INFO);
    assert_int_equal(crsf.DeviceInfo.dest_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(crsf.DeviceInfo.origin_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(crsf.DeviceInfo.Serial_number, 0x12345678);
    assert_int_equal(crsf.DeviceInfo.Hardware_ID, 0x00010002);
    assert_int_equal(crsf.DeviceInfo.Firmware_ID, 0x00030004);
    assert_int_equal(crsf.DeviceInfo.Parameters_total, 5);
    assert_int_equal(crsf.DeviceInfo.Parameter_version_number, 2);
    assert_string_equal((char*)crsf.DeviceInfo.Device_name, "CRSF-DEV");
}

static void test_process_param_read(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_param_read_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_PARAMETER_READ);
    assert_int_equal(crsf.ParamRead.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(crsf.ParamRead.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(crsf.ParamRead.Parameter_number, 1);
    assert_int_equal(crsf.ParamRead.Parameter_chunk_number, 2);
}

static void test_process_param_write(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_param_write_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_PARAMETER_WRITE);
    assert_int_equal(crsf.ParamWrite.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(crsf.ParamWrite.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(crsf.ParamWrite.Parameter_number, 1);
    assert_int_equal(crsf.ParamWrite.Data[0], 0x2A);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_process_command(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_FrameType_t frameType;
    CRSF_init(&crsf);
    assert_true(CRSF_processFrame(&crsf, test_command_packet, &frameType) == CRSF_OK);
    assert_int_equal(frameType, CRSF_FRAMETYPE_COMMAND);
    assert_int_equal(crsf.Command.dest_address, CRSF_ADDRESS_CRSF_RECEIVER);
    assert_int_equal(crsf.Command.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(crsf.Command.Command_ID, CRSF_CMDID_CROSSFIRE);
    assert_int_equal(crsf.Command.payload.crossfire.subCommand, CRSF_CMD_CF_SET_RX_BIND_MODE);
}
#endif

/* ============================================================================
 * ROUNDTRIP TESTS
 * ============================================================================ */

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_GPS
static void test_roundtrip_gps(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case GPS packet */
    tx.GPS.altitude = 0xABCD;
    tx.GPS.groundspeed = 40;
    tx.GPS.heading = 3;
    tx.GPS.latitude = (int32_t)0xABCDEF12;
    tx.GPS.longitude = (int32_t)0x8BCDEF12;
    tx.GPS.satellites = 2;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_GPS, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_GPS_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_GPS);

    /* Check received frame */
    assert_int_equal(rx.GPS.altitude, 0xABCD);
    assert_int_equal(rx.GPS.groundspeed, 40);
    assert_int_equal(rx.GPS.heading, 3);
    assert_int_equal(rx.GPS.latitude, (int32_t)0xABCDEF12);
    assert_int_equal(rx.GPS.longitude, (int32_t)0x8BCDEF12);
    assert_int_equal(rx.GPS.satellites, 2);

#if CRSF_ENABLE_STATS
    assert_int_equal(rx.Stats.frames_total, 1);
#endif
#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_GPS < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_GPS_TIME
static void test_roundtrip_gps_time(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case GPS time packet */
    tx.GPS_Time.year = 2025;
    tx.GPS_Time.month = 12;
    tx.GPS_Time.day = 31;
    tx.GPS_Time.hour = 23;
    tx.GPS_Time.minute = 59;
    tx.GPS_Time.second = 59;
    tx.GPS_Time.millisecond = 999;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_GPS, CRSF_FRAMETYPE_GPS_TIME, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_GPS_Time_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_GPS_TIME);

    /* Check received frame */
    assert_int_equal(rx.GPS_Time.year, 2025);
    assert_int_equal(rx.GPS_Time.month, 12);
    assert_int_equal(rx.GPS_Time.day, 31);
    assert_int_equal(rx.GPS_Time.hour, 23);
    assert_int_equal(rx.GPS_Time.minute, 59);
    assert_int_equal(rx.GPS_Time.second, 59);
    assert_int_equal(rx.GPS_Time.millisecond, 999);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_GPS_TIME < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS_TIME, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS_TIME, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_GPS_EXTENDED
static void test_roundtrip_gps_extended(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case GPS extended packet */
    tx.GPS_Ext.fix_type = 3;
    tx.GPS_Ext.n_speed = 0x7BCD;
    tx.GPS_Ext.e_speed = (int16_t)0xABCD;
    tx.GPS_Ext.v_speed = -500;
    tx.GPS_Ext.h_speed_acc = 100;
    tx.GPS_Ext.track_acc = 50;
    tx.GPS_Ext.alt_ellipsoid = 1500;
    tx.GPS_Ext.h_acc = 300;
    tx.GPS_Ext.v_acc = 200;
    tx.GPS_Ext.reserved = 0xAA;
    tx.GPS_Ext.hDOP = 15;
    tx.GPS_Ext.vDOP = 25;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_GPS, CRSF_FRAMETYPE_GPS_EXTENDED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_GPS_Ext_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_GPS_EXTENDED);

    /* Check received frame */
    assert_int_equal(rx.GPS_Ext.fix_type, 3);
    assert_int_equal(rx.GPS_Ext.n_speed, 0x7BCD);
    assert_int_equal(rx.GPS_Ext.e_speed, (int16_t)0xABCD);
    assert_int_equal(rx.GPS_Ext.v_speed, -500);
    assert_int_equal(rx.GPS_Ext.h_speed_acc, 100);
    assert_int_equal(rx.GPS_Ext.track_acc, 50);
    assert_int_equal(rx.GPS_Ext.alt_ellipsoid, 1500);
    assert_int_equal(rx.GPS_Ext.h_acc, 300);
    assert_int_equal(rx.GPS_Ext.v_acc, 200);
    assert_int_equal(rx.GPS_Ext.reserved, 0xAA);
    assert_int_equal(rx.GPS_Ext.hDOP, 15);
    assert_int_equal(rx.GPS_Ext.vDOP, 25);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_GPS_EXTENDED < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS_EXTENDED, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_GPS_EXTENDED, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_VARIO
static void test_roundtrip_vario(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case vario packet */
    tx.Vario.v_speed = (int16_t)0xABCD; // Strong downward speed

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_VARIO, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Vario_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_VARIO);

    /* Check received frame */
    assert_int_equal(rx.Vario.v_speed, (int16_t)0xABCD);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_VARIO < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VARIO, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VARIO, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_BATTERY_SENSOR
static void test_roundtrip_battery_sensor(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case battery packet */
    tx.Battery.voltage = 0x7BCD;          // Maximum voltage
    tx.Battery.current = (int16_t)0xABCD; // Negative current (regenerative braking)
    tx.Battery.capacity_used = 0xFFFFFF;  // Maximum capacity used (24-bit)
    tx.Battery.remaining = 0;             // Empty battery

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CURRENT_SENSOR, CRSF_FRAMETYPE_BATTERY_SENSOR, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Battery_t) + 3U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_BATTERY_SENSOR);

    /* Check received frame */
    assert_int_equal(rx.Battery.voltage, 0x7BCD);
    assert_int_equal(rx.Battery.current, (int16_t)0xABCD);
    assert_int_equal(rx.Battery.capacity_used, 0xFFFFFF);
    assert_int_equal(rx.Battery.remaining, 0);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_BATTERY_SENSOR < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_BATTERY_SENSOR, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_BATTERY_SENSOR, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_BAROALT_VSPEED
static void test_roundtrip_baroalt_vspeed(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* First test */
    tx.BaroAlt_VS.altitude = -15000;
    tx.BaroAlt_VS.vertical_speed = 1500;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BAROALT_VSPEED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 3U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_BAROALT_VSPEED);

    /* Check received frame (note: packed format may have precision loss) */
    assert_int_equal(rx.BaroAlt_VS.altitude, -10000);
    assert_true(abs(rx.BaroAlt_VS.vertical_speed - tx.BaroAlt_VS.vertical_speed) < abs(rx.BaroAlt_VS.vertical_speed / 100) + 1);

    /* Second test */
    tx.BaroAlt_VS.altitude = 10001;
    tx.BaroAlt_VS.vertical_speed = -1500;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BAROALT_VSPEED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 3U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_BAROALT_VSPEED);

    /* Check received frame (note: packed format may have precision loss) */
    assert_int_equal(rx.BaroAlt_VS.altitude, tx.BaroAlt_VS.altitude);
    assert_true(abs(rx.BaroAlt_VS.vertical_speed - tx.BaroAlt_VS.vertical_speed) < abs(rx.BaroAlt_VS.vertical_speed / 100) + 1);

    /* Third test */
    tx.BaroAlt_VS.altitude = 35003;
    tx.BaroAlt_VS.vertical_speed = 0;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BAROALT_VSPEED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 3U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_BAROALT_VSPEED);

    /* Check received frame (note: packed format may have precision loss) */
    assert_true(abs(rx.BaroAlt_VS.altitude - tx.BaroAlt_VS.altitude) < 10);
    assert_int_equal(rx.BaroAlt_VS.vertical_speed, tx.BaroAlt_VS.vertical_speed);

    /* Fourth test */
    tx.BaroAlt_VS.altitude = 500000;
    tx.BaroAlt_VS.vertical_speed = 0;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_BAROALT_VSPEED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 3U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_BAROALT_VSPEED);

    /* Check received frame (note: packed format may have precision loss) */
    assert_int_equal(rx.BaroAlt_VS.altitude, 327660);
    assert_int_equal(rx.BaroAlt_VS.vertical_speed, tx.BaroAlt_VS.vertical_speed);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_BAROALT_VSPEED < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_BAROALT_VSPEED, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_BAROALT_VSPEED, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_AIRSPEED
static void test_roundtrip_airspeed(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case airspeed packet */
    tx.Airspeed.speed = (uint16_t)0xABCD; // Maximum airspeed

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_AIRSPEED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Airspeed_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_AIRSPEED);

    /* Check received frame */
    assert_int_equal(rx.Airspeed.speed, (uint16_t)0xABCD);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_AIRSPEED < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_AIRSPEED, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_AIRSPEED, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_HEARTBEAT
static void test_roundtrip_heartbeat(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Heartbeat packet */
    tx.Heartbeat.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_HEARTBEAT, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Heartbeat_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_HEARTBEAT);

    /* Check received frame */
    assert_int_equal(rx.Heartbeat.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_HEARTBEAT < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_HEARTBEAT, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_HEARTBEAT, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_RPM
static void test_roundtrip_rpm(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case RPM packet with multiple motors */
    tx.RPM.rpm_source_id = 5;
    tx.RPM.rpm_value[0] = 12021;   // Positive RPM
    tx.RPM.rpm_value[1] = -8122;   // Negative RPM (reverse)
    tx.RPM.rpm_value[2] = 8367053; // Maximum positive RPM
    tx.RPM.rpm_value[3] = -344866; // Maximum negative RPM

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RPM, 4, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 4 * 3 + 3U + 1U); // source_id + 4*int24 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_RPM);

    /* Check received frame */
    assert_int_equal(rx.RPM.rpm_source_id, 5);
    assert_int_equal(rx.RPM.rpm_value[0], 12021);
    assert_int_equal(rx.RPM.rpm_value[1], -8122);
    assert_int_equal(rx.RPM.rpm_value[2], 8367053);
    assert_int_equal(rx.RPM.rpm_value[3], -344866);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_RPM < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RPM, 20));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_RPM
static void test_roundtrip_rpm_single_motor(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Minimum RPM packet - single motor */
    tx.RPM.rpm_source_id = 0;
    tx.RPM.rpm_value[0] = -1;

    /* Test Build with 1 value */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RPM, 1, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 1 * 3 + 3U + 1U); // source_id + 1*int24 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_RPM);

    /* Check received frame */
    assert_int_equal(rx.RPM.rpm_source_id, 0);
    assert_int_equal(rx.RPM.rpm_value[0], -1);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_RPM < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RPM, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RPM, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_RPM
static void test_roundtrip_rpm_zero_values(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* RPM packet with 0 values - should create minimum frame */
    tx.RPM.rpm_source_id = 3;

    /* Test Build with 0 values */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_RPM, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 3 + 3U + 1U); // source_id + min 3 bytes + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_RPM);

    /* Check received frame */
    assert_int_equal(rx.RPM.rpm_source_id, 3);
    assert_int_equal(rx.RPM.rpm_value[0], 0); // Should be 0 from minimum frame

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_RPM < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RPM, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RPM, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_TEMPERATURE
static void test_roundtrip_temperature(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case temperature packet with multiple sensors */
    tx.Temperature.temp_source_id = 1;
    tx.Temperature.temperature[0] = 850;             // 85.0°C (hot ESC)
    tx.Temperature.temperature[1] = -50;             // -5.0°C (cold ambient)
    tx.Temperature.temperature[2] = 250;             // 25.0°C (normal)
    tx.Temperature.temperature[3] = 0x7BCD;          // Maximum temperature
    tx.Temperature.temperature[4] = (int16_t)0xABCD; // Minimum temperature

    /* Test Build with 5 values */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_TEMPERATURE, 5, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 5 * 2 + 3U + 1U); // source_id + 5*int16 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_TEMPERATURE);

    /* Check received frame */
    assert_int_equal(rx.Temperature.temp_source_id, 1);
    assert_int_equal(rx.Temperature.temperature[0], 850);
    assert_int_equal(rx.Temperature.temperature[1], -50);
    assert_int_equal(rx.Temperature.temperature[2], 250);
    assert_int_equal(rx.Temperature.temperature[3], 0x7BCD);
    assert_int_equal(rx.Temperature.temperature[4], (int16_t)0xABCD);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_TEMPERATURE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_TEMPERATURE
static void test_roundtrip_temperature_single_sensor(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Minimum temperature packet - single sensor */
    tx.Temperature.temp_source_id = 0;
    tx.Temperature.temperature[0] = 200; // 20.0°C

    /* Test Build with 1 value */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_TEMPERATURE, 1, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 1 * 2 + 3U + 1U); // source_id + 1*int16 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_TEMPERATURE);

    /* Check received frame */
    assert_int_equal(rx.Temperature.temp_source_id, 0);
    assert_int_equal(rx.Temperature.temperature[0], 200);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_TEMPERATURE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_TEMPERATURE
static void test_roundtrip_temperature_zero_values(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Temperature packet with 0 values - should create minimum frame */
    tx.Temperature.temp_source_id = 2;

    /* Test Build with 0 values */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_TEMPERATURE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 2 + 3U + 1U); // source_id + min 2 bytes + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_TEMPERATURE);

    /* Check received frame */
    assert_int_equal(rx.Temperature.temp_source_id, 2);
    assert_int_equal(rx.Temperature.temperature[0], 0); // Should be 0 from minimum frame

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_TEMPERATURE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_TEMPERATURE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_VOLTAGES
static void test_roundtrip_voltages(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case voltages packet with multiple cells */
    tx.Voltages.Voltage_source_id = 0;
    tx.Voltages.Voltage_values[0] = 4200;   // 4.2V (full LiPo cell)
    tx.Voltages.Voltage_values[1] = 3700;   // 3.7V (nominal)
    tx.Voltages.Voltage_values[2] = 3000;   // 3.0V (low)
    tx.Voltages.Voltage_values[3] = 0xABCD; // Maximum voltage
    tx.Voltages.Voltage_values[4] = 0;      // Minimum voltage

    /* Test Build with 5 values */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CURRENT_SENSOR, CRSF_FRAMETYPE_VOLTAGES, 5, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 5 * 2U + 3U + 1U); // source_id + 5*uint16 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_VOLTAGES);

    /* Check received frame */
    assert_int_equal(rx.Voltages.Voltage_source_id, 0);
    assert_int_equal(rx.Voltages.Voltage_values[0], 4200);
    assert_int_equal(rx.Voltages.Voltage_values[1], 3700);
    assert_int_equal(rx.Voltages.Voltage_values[2], 3000);
    assert_int_equal(rx.Voltages.Voltage_values[3], 0xABCD);
    assert_int_equal(rx.Voltages.Voltage_values[4], 0);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_VOLTAGES < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_VOLTAGES
static void test_roundtrip_voltages_single_cell(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Minimum voltages packet - single cell */
    tx.Voltages.Voltage_source_id = 1;
    tx.Voltages.Voltage_values[0] = 3850; // 3.85V

    /* Test Build with 1 value */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CURRENT_SENSOR, CRSF_FRAMETYPE_VOLTAGES, 1, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 1 * 2U + 3U + 1U); // source_id + 1*uint16 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_VOLTAGES);

    /* Check received frame */
    assert_int_equal(rx.Voltages.Voltage_source_id, 1);
    assert_int_equal(rx.Voltages.Voltage_values[0], 3850);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_VOLTAGES < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_VOLTAGES
static void test_roundtrip_voltages_zero_values(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Minimum voltages packet - single cell */
    tx.Voltages.Voltage_source_id = 1;

    /* Test Build with 1 value */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CURRENT_SENSOR, CRSF_FRAMETYPE_VOLTAGES, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 1 + 1 * 2U + 3U + 1U); // source_id + 1*uint16 + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_VOLTAGES);

    /* Check received frame */
    assert_int_equal(rx.Voltages.Voltage_source_id, 1);
    assert_int_equal(rx.Voltages.Voltage_values[0], 0);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_VOLTAGES < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VOLTAGES, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_VTX
static void test_roundtrip_vtx(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case VTX packet */
    tx.VTX.origin_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.VTX.power_dBm = 30;       // High power
    tx.VTX.frequency_MHz = 5800; // 5.8GHz band
    tx.VTX.pit_mode = 1;         // Pit mode on
    tx.VTX.pitmode_control = 2;  // Switch control
    tx.VTX.pitmode_switch = 8;   // Channel 9

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_VTX, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_VTX_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_VTX);

    /* Check received frame */
    assert_int_equal(rx.VTX.origin_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.VTX.power_dBm, 30);
    assert_int_equal(rx.VTX.frequency_MHz, 5800);
    assert_int_equal(rx.VTX.pit_mode, 1);
    assert_int_equal(rx.VTX.pitmode_control, 2);
    assert_int_equal(rx.VTX.pitmode_switch, 8);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_VTX < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VTX, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_VTX, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_LINK_STATISTICS
static void test_roundtrip_link_statistics(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case link statistics packet */
    tx.LinkStatistics.up_rssi_ant1 = 120;   // -120 dBm
    tx.LinkStatistics.up_rssi_ant2 = 110;   // -110 dBm
    tx.LinkStatistics.up_link_quality = 95; // 95%
    tx.LinkStatistics.up_snr = -10;         // -10 dB
    tx.LinkStatistics.active_antenna = 1;
    tx.LinkStatistics.rf_profile = 2;         // 150fps
    tx.LinkStatistics.up_rf_power = 8;        // 50mW
    tx.LinkStatistics.down_rssi = 100;        // -100 dBm
    tx.LinkStatistics.down_link_quality = 85; // 85%
    tx.LinkStatistics.down_snr = 5;           // 5 dB

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_LINK_STATISTICS, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_LinkStatistics_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_LINK_STATISTICS);

    /* Check received frame */
    assert_int_equal(rx.LinkStatistics.up_rssi_ant1, 120);
    assert_int_equal(rx.LinkStatistics.up_rssi_ant2, 110);
    assert_int_equal(rx.LinkStatistics.up_link_quality, 95);
    assert_int_equal(rx.LinkStatistics.up_snr, -10);
    assert_int_equal(rx.LinkStatistics.active_antenna, 1);
    assert_int_equal(rx.LinkStatistics.rf_profile, 2);
    assert_int_equal(rx.LinkStatistics.up_rf_power, 8);
    assert_int_equal(rx.LinkStatistics.down_rssi, 100);
    assert_int_equal(rx.LinkStatistics.down_link_quality, 85);
    assert_int_equal(rx.LinkStatistics.down_snr, 5);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_LINK_STATISTICS < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_ENABLE_RC_CHANNELS
static void test_roundtrip_rc_channels_packed(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case RC channels packet */
    tx.RC.channels[0] = 1000; // Minimum
    tx.RC.channels[1] = 1500; // Center
    tx.RC.channels[2] = 2000; // Maximum
    tx.RC.channels[3] = 1250; // Quarter
    tx.RC.channels[4] = 1750; // Three quarters
    for (uint8_t ii = 5; ii < CRSF_RC_CHANNELS; ii++) {
        tx.RC.channels[ii] = 1000 + (ii * 62); // Spread across range
    }

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 22 + 3U + 1U); // 22 bytes for packed channels + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED);

    /* Check received frame (allow small tolerance due to packing/unpacking) */
    for (uint8_t ii = 0; ii < CRSF_RC_CHANNELS; ii++) {
        assert_true(abs((int)rx.RC.channels[ii] - (int)tx.RC.channels[ii]) <= 2);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_RC_CHANNELS_PACKED, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_LINK_STATISTICS_RX
static void test_roundtrip_link_statistics_rx(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case link statistics RX packet */
    tx.LinkStatisticsRX.rssi_db = 80;      // -80 dBm
    tx.LinkStatisticsRX.rssi_percent = 75; // 75%
    tx.LinkStatisticsRX.link_quality = 90; // 90%
    tx.LinkStatisticsRX.snr = INT8_MIN;    // Minimum SNR
    tx.LinkStatisticsRX.rf_power_db = 20;  // 20 dBm

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CRSF_RECEIVER, CRSF_FRAMETYPE_LINK_STATISTICS_RX, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_LinkStatisticsRX_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_LINK_STATISTICS_RX);

    /* Check received frame */
    assert_int_equal(rx.LinkStatisticsRX.rssi_db, 80);
    assert_int_equal(rx.LinkStatisticsRX.rssi_percent, 75);
    assert_int_equal(rx.LinkStatisticsRX.link_quality, 90);
    assert_int_equal(rx.LinkStatisticsRX.snr, INT8_MIN);
    assert_int_equal(rx.LinkStatisticsRX.rf_power_db, 20);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_LINK_STATISTICS_RX < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS_RX, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS_RX, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_LINK_STATISTICS_TX
static void test_roundtrip_link_statistics_tx(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case link statistics TX packet */
    tx.LinkStatisticsTX.rssi_db = 60;       // -60 dBm
    tx.LinkStatisticsTX.rssi_percent = 100; // 100%
    tx.LinkStatisticsTX.link_quality = 0;   // 0% (worst case)
    tx.LinkStatisticsTX.snr = INT8_MAX;     // Maximum SNR
    tx.LinkStatisticsTX.rf_power_db = 30;   // 30 dBm
    tx.LinkStatisticsTX.fps = 150;          // 15.0 fps

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_CRSF_TRANSMITTER, CRSF_FRAMETYPE_LINK_STATISTICS_TX, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_LinkStatisticsTX_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_LINK_STATISTICS_TX);

    /* Check received frame */
    assert_int_equal(rx.LinkStatisticsTX.rssi_db, 60);
    assert_int_equal(rx.LinkStatisticsTX.rssi_percent, 100);
    assert_int_equal(rx.LinkStatisticsTX.link_quality, 0);
    assert_int_equal(rx.LinkStatisticsTX.snr, INT8_MAX);
    assert_int_equal(rx.LinkStatisticsTX.rf_power_db, 30);
    assert_int_equal(rx.LinkStatisticsTX.fps, 150);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_LINK_STATISTICS_TX < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS_TX, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_LINK_STATISTICS_TX, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_ATTITUDE
static void test_roundtrip_attitude(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case attitude packet */
    tx.Attitude.pitch = 0x7BCD;         // Maximum pitch
    tx.Attitude.roll = (int16_t)0xABCD; // Minimum roll
    tx.Attitude.yaw = 0;                // Zero yaw

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_ATTITUDE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Attitude_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_ATTITUDE);

    /* Check received frame */
    assert_int_equal(rx.Attitude.pitch, 0x7BCD);
    assert_int_equal(rx.Attitude.roll, (int16_t)0xABCD);
    assert_int_equal(rx.Attitude.yaw, 0);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_ATTITUDE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_ATTITUDE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_ATTITUDE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_MAVLINK_FC
static void test_roundtrip_mavlink_fc(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case MAVLink FC packet */
    tx.MAVLinkFC.airspeed = 0x7BCD;
    tx.MAVLinkFC.base_mode = 0xAB;
    tx.MAVLinkFC.custom_mode = 0xABCDEF12;
    tx.MAVLinkFC.autopilot_type = 3;
    tx.MAVLinkFC.firmware_type = 1;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_MAVLINK_FC, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_MAVLinkFC_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_MAVLINK_FC);

    /* Check received frame */
    assert_int_equal(rx.MAVLinkFC.airspeed, 0x7BCD);
    assert_int_equal(rx.MAVLinkFC.base_mode, 0xAB);
    assert_int_equal(rx.MAVLinkFC.custom_mode, 0xABCDEF12);
    assert_int_equal(rx.MAVLinkFC.autopilot_type, 3);
    assert_int_equal(rx.MAVLinkFC.firmware_type, 1);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_MAVLINK_FC < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_FC, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_FC, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_FLIGHT_MODE
static void test_roundtrip_flight_mode(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case flight mode packet - test maximum length */
    strcpy(tx.FlightMode.flight_mode, "ACRO_MODE_TEST_"); // 15 chars + null

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_FLIGHT_MODE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, strlen(tx.FlightMode.flight_mode) + 1 + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_FLIGHT_MODE);

    /* Check received frame */
    assert_string_equal(rx.FlightMode.flight_mode, "ACRO_MODE_TEST_");

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_FLIGHT_MODE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_FLIGHT_MODE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_FLIGHT_MODE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_FLIGHT_MODE
static void test_roundtrip_flight_mode_short(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Minimum flight mode packet - short name */
    strcpy(tx.FlightMode.flight_mode, "A"); // Single character + null

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_FLIGHT_MODE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 2 + 3U + 1U); // "A" + null + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_FLIGHT_MODE);

    /* Check received frame */
    assert_string_equal(rx.FlightMode.flight_mode, "A");

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_FLIGHT_MODE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_FLIGHT_MODE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_FLIGHT_MODE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
static void test_roundtrip_esp_now_messages(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case ESP-NOW messages packet */
    tx.ESPNowMessages.VAL1 = 0xAB; // Maximum seat position
    tx.ESPNowMessages.VAL2 = 0;    // First lap
    strcpy(tx.ESPNowMessages.VAL3, "12:34.567");
    strcpy(tx.ESPNowMessages.VAL4, "01:23.456");
    strcpy(tx.ESPNowMessages.FREE_TEXT, "RACE MODE ACTIVE!!!");

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RACE_TAG, CRSF_FRAMETYPE_ESP_NOW_MESSAGES, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_ESPNowMessages_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_ESP_NOW_MESSAGES);

    /* Check received frame */
    assert_int_equal(rx.ESPNowMessages.VAL1, 0xAB);
    assert_int_equal(rx.ESPNowMessages.VAL2, 0);
    assert_string_equal(rx.ESPNowMessages.VAL3, "12:34.567");
    assert_string_equal(rx.ESPNowMessages.VAL4, "01:23.456");
    assert_string_equal(rx.ESPNowMessages.FREE_TEXT, "RACE MODE ACTIVE!!!");

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_ESP_NOW_MESSAGES < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_ESP_NOW_MESSAGES, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_ESP_NOW_MESSAGES, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_device_ping(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Device ping packet */
    tx.Ping.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.Ping.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_DEVICE_PING, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_Ping_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_DEVICE_PING);

    /* Check received frame */
    assert_int_equal(rx.Ping.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.Ping.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_DEVICE_PING < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_DEVICE_PING, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_DEVICE_PING, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_device_info(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case device info packet */
    tx.DeviceInfo.dest_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.DeviceInfo.origin_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    strcpy(tx.DeviceInfo.Device_name, "TestFC");
    tx.DeviceInfo.Serial_number = 0xABCDEF12;
    tx.DeviceInfo.Hardware_ID = 0x12345678;
    tx.DeviceInfo.Firmware_ID = 0x87654321;
    tx.DeviceInfo.Parameters_total = 0xAB;
    tx.DeviceInfo.Parameter_version_number = 42;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_DEVICE_INFO, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 23U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_DEVICE_INFO);

    /* Check received frame */
    assert_int_equal(rx.DeviceInfo.dest_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.DeviceInfo.origin_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_string_equal(rx.DeviceInfo.Device_name, "TestFC");
    assert_int_equal(rx.DeviceInfo.Serial_number, 0xABCDEF12);
    assert_int_equal(rx.DeviceInfo.Hardware_ID, 0x12345678);
    assert_int_equal(rx.DeviceInfo.Firmware_ID, 0x87654321);
    assert_int_equal(rx.DeviceInfo.Parameters_total, 0xAB);
    assert_int_equal(rx.DeviceInfo.Parameter_version_number, 42);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_DEVICE_INFO < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_DEVICE_INFO, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_DEVICE_INFO, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_settings_entry(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case parameter settings entry packet */
    tx.ParamSettingsEntry.dest_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamSettingsEntry.origin_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamSettingsEntry.Parameter_number = 0xAB;
    tx.ParamSettingsEntry.Parameter_chunks_remaining = 5;
    // Fill payload with test data
    for (uint8_t ii = 0; ii < 10; ii++) {
        tx.ParamSettingsEntry.Payload[ii] = ii + 0xA0;
    }

    /* Test Build with 10 payload bytes */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 10, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 14U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY);

    /* Check received frame */
    assert_int_equal(rx.ParamSettingsEntry.dest_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.ParamSettingsEntry.origin_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.ParamSettingsEntry.Parameter_number, 0xAB);
    assert_int_equal(rx.ParamSettingsEntry.Parameter_chunks_remaining, 5);
    for (uint8_t ii = 0; ii < 10; ii++) {
        assert_int_equal(rx.ParamSettingsEntry.Payload[ii], ii + 0xA0);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_settings_entry_oversized(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;
    uint8_t testPattern[CRSF_MAX_PARAM_SETTINGS_PAYLOAD + 10];

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Create test pattern
    for (uint8_t ii = 0; ii < sizeof(testPattern); ii++) {
        testPattern[ii] = ii;
    }

    // Setup oversized payload
    tx.ParamSettingsEntry.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamSettingsEntry.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamSettingsEntry.Parameter_number = 0x42;
    tx.ParamSettingsEntry.Parameter_chunks_remaining = 1;
    memcpy(tx.ParamSettingsEntry.Payload, testPattern, sizeof(testPattern));

    /* Test Build with oversized payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, sizeof(testPattern), frame, &frameLength)
                == CRSF_OK);
    assert_int_equal(frameLength, 60U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY);

    /* Verify payload was truncated to max size */
    for (uint8_t ii = 0; ii < CRSF_MAX_PARAM_SETTINGS_PAYLOAD; ii++) {
        assert_int_equal(rx.ParamSettingsEntry.Payload[ii], testPattern[ii]);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_settings_entry_min_payload(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Test minimal payload case (tests paramLen == 4U branch)
    tx.ParamSettingsEntry.dest_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamSettingsEntry.origin_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamSettingsEntry.Parameter_number = 0xAB;
    tx.ParamSettingsEntry.Parameter_chunks_remaining = 5;

    /* Test Build with minimal payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 5U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY);

    /* Check received frame */
    assert_int_equal(rx.ParamSettingsEntry.dest_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.ParamSettingsEntry.origin_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.ParamSettingsEntry.Parameter_number, 0xAB);
    assert_int_equal(rx.ParamSettingsEntry.Parameter_chunks_remaining, 5);
    assert_int_equal(rx.ParamSettingsEntry.Payload[0], 0); // Should be zero-filled

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_SETTINGS_ENTRY, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_read(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case parameter read packet */
    tx.ParamRead.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamRead.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamRead.Parameter_number = 0xAB;
    tx.ParamRead.Parameter_chunk_number = 0xCD;

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_READ, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_ParamRead_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_READ);

    /* Check received frame */
    assert_int_equal(rx.ParamRead.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.ParamRead.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.ParamRead.Parameter_number, 0xAB);
    assert_int_equal(rx.ParamRead.Parameter_chunk_number, 0xCD);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_READ < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_READ, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_READ, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_write(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case parameter write packet */
    tx.ParamWrite.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamWrite.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamWrite.Parameter_number = 100;
    // Fill data with test pattern
    for (uint8_t ii = 0; ii < 15; ii++) {
        tx.ParamWrite.Data[ii] = 0x55 + ii;
    }

    /* Test Build with 15 data bytes */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_WRITE, 15, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 18U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_WRITE);

    /* Check received frame */
    assert_int_equal(rx.ParamWrite.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.ParamWrite.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.ParamWrite.Parameter_number, 100);
    for (uint8_t ii = 0; ii < 15; ii++) {
        assert_int_equal(rx.ParamWrite.Data[ii], 0x55 + ii);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_WRITE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_write_oversized(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;
    uint8_t testPattern[CRSF_MAX_PARAM_DATA_LEN + 10];

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Create test pattern
    for (uint8_t ii = 0; ii < sizeof(testPattern); ii++) {
        testPattern[ii] = ii;
    }

    // Setup oversized payload
    tx.ParamWrite.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamWrite.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamWrite.Parameter_number = 0x42;
    memcpy(tx.ParamWrite.Data, testPattern, sizeof(testPattern));

    /* Test Build with oversized payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_WRITE, sizeof(testPattern), frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, CRSF_MAX_PARAM_DATA_LEN + 3U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_WRITE);

    /* Verify payload was truncated to max size */
    for (uint8_t ii = 0; ii < CRSF_MAX_PARAM_DATA_LEN; ii++) {
        assert_int_equal(rx.ParamWrite.Data[ii], testPattern[ii]);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_WRITE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_PARAMETER_GROUP
static void test_roundtrip_parameter_write_min_payload(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Test minimal payload case (tests paramLen == 3U branch)
    tx.ParamWrite.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.ParamWrite.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.ParamWrite.Parameter_number = 100;

    /* Test Build with minimal payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_PARAMETER_WRITE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 4U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_PARAMETER_WRITE);

    /* Check received frame */
    assert_int_equal(rx.ParamWrite.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.ParamWrite.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.ParamWrite.Parameter_number, 100);
    assert_int_equal(rx.ParamWrite.Data[0], 0); // Should be zero-filled

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_PARAMETER_WRITE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_PARAMETER_WRITE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_ENABLE_COMMAND
static void test_roundtrip_command(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case command packet */
    tx.Command.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.Command.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.Command.Command_ID = CRSF_CMDID_SCREEN;
    tx.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
    strcpy(tx.Command.payload.screen.popupMessageStart.Header, "TestHH");
    strcpy(tx.Command.payload.screen.popupMessageStart.Info_message, "What");
    tx.Command.payload.screen.popupMessageStart.Max_timeout_interval = 3;
    tx.Command.payload.screen.popupMessageStart.Close_button_option = 1;
    tx.Command.payload.screen.popupMessageStart.add_data.present = 1;
    strcpy(tx.Command.payload.screen.popupMessageStart.add_data.selectionText, "qq");
    tx.Command.payload.screen.popupMessageStart.add_data.value = 5;
    tx.Command.payload.screen.popupMessageStart.add_data.minValue = 2;
    tx.Command.payload.screen.popupMessageStart.add_data.maxValue = 7;
    tx.Command.payload.screen.popupMessageStart.add_data.defaultValue = 4;
    strcpy(tx.Command.payload.screen.popupMessageStart.add_data.unit, "mV");
    tx.Command.payload.screen.popupMessageStart.has_possible_values = 1;
    strcpy(tx.Command.payload.screen.popupMessageStart.possible_values, "2;3");

    /* Test Build with 32 payload bytes */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_COMMAND, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 32U + 1U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_COMMAND);

    /* Check received frame */
    assert_int_equal(rx.Command.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.Command.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.Command.Command_ID, CRSF_CMDID_SCREEN);
    assert_int_equal(rx.Command.payload.screen.subCommand, CRSF_CMD_SCREEN_POPUP_MESSAGE_START);
    assert_string_equal(rx.Command.payload.screen.popupMessageStart.Header, "TestHH");
    assert_string_equal(rx.Command.payload.screen.popupMessageStart.Info_message, "What");
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.Max_timeout_interval, 3);
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.Close_button_option, 1);
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.add_data.present, 1);
    assert_string_equal(rx.Command.payload.screen.popupMessageStart.add_data.selectionText, "qq");
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.add_data.value, 5);
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.add_data.minValue, 2);
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.add_data.maxValue, 7);
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.add_data.defaultValue, 4);
    assert_string_equal(rx.Command.payload.screen.popupMessageStart.add_data.unit, "mV");
    assert_int_equal(rx.Command.payload.screen.popupMessageStart.has_possible_values, 1);
    assert_string_equal(rx.Command.payload.screen.popupMessageStart.possible_values, "2;3");

#if CRSF_ENABLE_STATS
    assert_int_equal(rx.Stats.commands_rx, 1);
#endif
#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_COMMAND < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_COMMAND, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_COMMAND, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_ENABLE_COMMAND
static void test_roundtrip_command_oversized(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Setup oversized payload
    tx.Command.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.Command.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.Command.Command_ID = CRSF_CMDID_SCREEN;
    tx.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
    strcpy(tx.Command.payload.screen.popupMessageStart.Header, "TestHH");
    strcpy(tx.Command.payload.screen.popupMessageStart.Info_message, "What");
    tx.Command.payload.screen.popupMessageStart.Max_timeout_interval = 3;
    tx.Command.payload.screen.popupMessageStart.Close_button_option = 1;
    tx.Command.payload.screen.popupMessageStart.add_data.present = 1;
    strcpy(tx.Command.payload.screen.popupMessageStart.add_data.selectionText, "qq");
    tx.Command.payload.screen.popupMessageStart.add_data.value = 5;
    tx.Command.payload.screen.popupMessageStart.add_data.minValue = 2;
    tx.Command.payload.screen.popupMessageStart.add_data.maxValue = 7;
    tx.Command.payload.screen.popupMessageStart.add_data.defaultValue = 4;
    strcpy(tx.Command.payload.screen.popupMessageStart.add_data.unit, "mV");
    tx.Command.payload.screen.popupMessageStart.has_possible_values = 1;
    strcpy(tx.Command.payload.screen.popupMessageStart.possible_values, "2;3;4;5;6;7");

    /* Test Build with oversized payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_COMMAND, 0, frame, &frameLength) == CRSF_ERROR_TYPE_LENGTH);

#if CRSF_ENABLE_STATS
    assert_int_equal(rx.Stats.commands_rx, 0);
#endif
#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_COMMAND < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_COMMAND, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_COMMAND, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_ENABLE_COMMAND
static void test_direct_command_bad_inner_crc(void** state) {
    (void)state;
    CRSF_t crsf;

    CRSF_init(&crsf);

    // Process some data to change stats
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    CRSF_FrameType_t frameType;
    frame[0] = CRSF_ADDRESS_BROADCAST;
    frame[1] = 10;
    frame[2] = CRSF_FRAMETYPE_COMMAND;
    frame[3] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    frame[4] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    frame[5] = 1;
    frame[6] = 'B';
    frame[7] = 'A';
    frame[8] = 'D';
    frame[9] = '\0';
    frame[10] = test_calc_checksum(frame + 2U, 8, 0xBAU) + 1U;
    frame[11] = test_calc_checksum(frame + 2U, 9, 0xD5U);

    assert_int_equal(CRSF_processFrame(&crsf, frame, &frameType), CRSF_ERROR_CMD_CHECKSUM_FAIL);
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
static void test_roundtrip_mavlink_envelope(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case MAVLink envelope packet */
    tx.MAVLinkEnv.total_chunks = 10; // Maximum chunks
    tx.MAVLinkEnv.current_chunk = 7; // Middle chunk
    tx.MAVLinkEnv.data_size = 40;    // Test data size
    // Fill data with test pattern
    for (uint8_t ii = 0; ii < 40; ii++) {
        tx.MAVLinkEnv.data[ii] = ii ^ 0xAA;
    }

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_MAVLINK_ENVELOPE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 40U + 2U + 3U + 1U); // data_size + 2 header bytes + frame header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_MAVLINK_ENVELOPE);

    /* Check received frame */
    assert_int_equal(rx.MAVLinkEnv.total_chunks, 10);
    assert_int_equal(rx.MAVLinkEnv.current_chunk, 7);
    assert_int_equal(rx.MAVLinkEnv.data_size, 40);
    for (uint8_t ii = 0; ii < 40; ii++) {
        assert_int_equal(rx.MAVLinkEnv.data[ii], ii ^ 0xAA);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
static void test_roundtrip_mavlink_envelope_max_data(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Maximum MAVLink envelope packet */
    tx.MAVLinkEnv.total_chunks = 1;
    tx.MAVLinkEnv.current_chunk = 0;
    tx.MAVLinkEnv.data_size = CRSF_MAX_MAVLINK_PAYLOAD; // Maximum data size
    // Fill data with test pattern
    for (uint8_t ii = 0; ii < CRSF_MAX_MAVLINK_PAYLOAD; ii++) {
        tx.MAVLinkEnv.data[ii] = ii;
    }

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_MAVLINK_ENVELOPE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, CRSF_MAX_MAVLINK_PAYLOAD + 2 + 3U + 1U); // max data + 2 header bytes + frame header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_MAVLINK_ENVELOPE);

    /* Check received frame */
    assert_int_equal(rx.MAVLinkEnv.total_chunks, 1);
    assert_int_equal(rx.MAVLinkEnv.current_chunk, 0);
    assert_int_equal(rx.MAVLinkEnv.data_size, CRSF_MAX_MAVLINK_PAYLOAD);
    for (uint8_t ii = 0; ii < CRSF_MAX_MAVLINK_PAYLOAD; ii++) {
        assert_int_equal(rx.MAVLinkEnv.data[ii], ii);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
static void test_roundtrip_mavlink_envelope_limited_size(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Test oversized data case (tests data_size > CRSF_MAX_MAVLINK_PAYLOAD branch)
    tx.MAVLinkEnv.total_chunks = 1;
    tx.MAVLinkEnv.current_chunk = 0;
    tx.MAVLinkEnv.data_size = 100; // Larger than max allowed (58)

    // Fill data with test pattern
    for (uint8_t ii = 0; ii < CRSF_MAX_MAVLINK_PAYLOAD; ii++) {
        tx.MAVLinkEnv.data[ii] = ii;
    }

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_MAVLINK_ENVELOPE, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, CRSF_MAX_MAVLINK_PAYLOAD + 2 + 3U + 1U); // max data + 2 header bytes + frame header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_MAVLINK_ENVELOPE);

    /* Check received frame */
    assert_int_equal(rx.MAVLinkEnv.total_chunks, 1);
    assert_int_equal(rx.MAVLinkEnv.current_chunk, 0);
    assert_int_equal(rx.MAVLinkEnv.data_size, CRSF_MAX_MAVLINK_PAYLOAD); // Should be limited to max size

    // Verify data was truncated correctly
    for (uint8_t ii = 0; ii < CRSF_MAX_MAVLINK_PAYLOAD; ii++) {
        assert_int_equal(rx.MAVLinkEnv.data[ii], ii);
    }

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_ENVELOPE, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX) && CRSF_TEL_ENABLE_MAVLINK_STATUS
static void test_roundtrip_mavlink_status(void** state) {
    (void)state;

    CRSF_t tx, rx;
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t frameLength = 0;
    CRSF_FrameType_t frameType;

    CRSF_init(&tx);
    CRSF_init(&rx);
#if CRSF_ENABLE_FRESHNESS_CHECK
    CRSF_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    /* Edge case MAVLink status packet */
    tx.MAVLinkStat.sensor_present = 0xABCDEF12; // All sensors present
    tx.MAVLinkStat.sensor_enabled = 0x55555555; // Alternating pattern
    tx.MAVLinkStat.sensor_health = 0xAAAAAAAA;  // Alternating pattern

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_MAVLINK_STATUS, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, sizeof(CRSF_MAVLinkStat_t) + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_MAVLINK_STATUS);

    /* Check received frame */
    assert_int_equal(rx.MAVLinkStat.sensor_present, 0xABCDEF12);
    assert_int_equal(rx.MAVLinkStat.sensor_enabled, 0x55555555);
    assert_int_equal(rx.MAVLinkStat.sensor_health, 0xAAAAAAAA);

#if CRSF_ENABLE_FRESHNESS_CHECK
    if (CRSF_TRK_FRAMETYPE_MAVLINK_STATUS < 0xFF) {
        mock_timestamp += 10;
        assert_true(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_STATUS, 20));
        assert_false(CRSF_isFrameFresh(&rx, CRSF_TRK_FRAMETYPE_MAVLINK_STATUS, 5));
        mock_timestamp -= 10;
    }
#endif
}
#endif

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================ */

int main(void) {
    const struct CMUnitTest tests[] = {
        /* Initialization */
        cmocka_unit_test(test_init_null),
        cmocka_unit_test(test_init_defaults_and_clear),
#if CRSF_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_timestamp_callback_setting),
#endif

        /* Basic Tests */
        cmocka_unit_test(test_build_null_ptrs),
        cmocka_unit_test(test_process_null_ptrs),
#if CRSF_ENABLE_ADDRESS_VALIDATION
        cmocka_unit_test(test_all_valid_addresses),
        cmocka_unit_test(test_build_invalid_address),
        cmocka_unit_test(test_process_invalid_address),
#endif
        cmocka_unit_test(test_build_invalid_frame),
        cmocka_unit_test(test_process_invalid_frame),
#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_valid_lengths),
#endif
        cmocka_unit_test(test_process_wrong_length),
        cmocka_unit_test(test_process_wrong_CRC),

#if CRSF_ENABLE_STATS
        /* Statistics Tests */
        cmocka_unit_test(test_stats_getter_and_reset),
        cmocka_unit_test(test_stats_null_arguments),
#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_stats_comprehensive_counting),
#endif
#endif

#if CRSF_ENABLE_FRESHNESS_CHECK
        /* Freshness Tests */
        cmocka_unit_test(test_freshness_basic_functionality),
        cmocka_unit_test(test_freshness_invalid_arguments),
        cmocka_unit_test(test_freshness_edge_cases),
#endif

/* Build Tests */
#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_gps),
#endif
#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_vario),
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_battery),
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_baro),
#endif
#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_airspeed),
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
        cmocka_unit_test(test_build_heartbeat),
#endif
#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_rpm),
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_temp),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS
        cmocka_unit_test(test_build_linkstats),
#endif
#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_rc_channels),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
        cmocka_unit_test(test_build_link_rx_id),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
        cmocka_unit_test(test_build_link_tx_id),
#endif
#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_build_attitude),
#endif
#if CRSF_TEL_ENABLE_FLIGHT_MODE
        cmocka_unit_test(test_build_flightmode),
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
        cmocka_unit_test(test_build_device_ping),
        cmocka_unit_test(test_build_device_info),
        cmocka_unit_test(test_build_param_read),
        cmocka_unit_test(test_build_param_write),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_command),
#endif

/* Process Tests */
#if CRSF_TEL_ENABLE_GPS && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_gps),
#endif
#if CRSF_TEL_ENABLE_VARIO && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_vario),
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_battery),
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_baro),
#endif
#if CRSF_TEL_ENABLE_AIRSPEED && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_airspeed),
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
        cmocka_unit_test(test_process_heartbeat),
#endif
#if CRSF_TEL_ENABLE_RPM && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_rpm),
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_temp),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS
        cmocka_unit_test(test_process_linkstats),
#endif
#if CRSF_ENABLE_RC_CHANNELS && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_process_rc_channels),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
        cmocka_unit_test(test_process_link_rx_id),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
        cmocka_unit_test(test_process_link_tx_id),
#endif
#if CRSF_TEL_ENABLE_ATTITUDE && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_process_attitude),
#endif
#if CRSF_TEL_ENABLE_FLIGHT_MODE
        cmocka_unit_test(test_process_flightmode),
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
        cmocka_unit_test(test_process_device_ping),
        cmocka_unit_test(test_process_device_info),
        cmocka_unit_test(test_process_param_read),
        cmocka_unit_test(test_process_param_write),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_process_command),
#endif

/* Roundtrip Tests */
#if defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
#if CRSF_TEL_ENABLE_GPS
        cmocka_unit_test(test_roundtrip_gps),
#endif
#if CRSF_TEL_ENABLE_GPS_TIME
        cmocka_unit_test(test_roundtrip_gps_time),
#endif
#if CRSF_TEL_ENABLE_GPS_EXTENDED
        cmocka_unit_test(test_roundtrip_gps_extended),
#endif
#if CRSF_TEL_ENABLE_VARIO
        cmocka_unit_test(test_roundtrip_vario),
#endif
#if CRSF_TEL_ENABLE_BATTERY_SENSOR
        cmocka_unit_test(test_roundtrip_battery_sensor),
#endif
#if CRSF_TEL_ENABLE_BAROALT_VSPEED
        cmocka_unit_test(test_roundtrip_baroalt_vspeed),
#endif
#if CRSF_TEL_ENABLE_AIRSPEED
        cmocka_unit_test(test_roundtrip_airspeed),
#endif
#if CRSF_TEL_ENABLE_HEARTBEAT
        cmocka_unit_test(test_roundtrip_heartbeat),
#endif
#if CRSF_TEL_ENABLE_RPM
        cmocka_unit_test(test_roundtrip_rpm),
        cmocka_unit_test(test_roundtrip_rpm_single_motor),
        cmocka_unit_test(test_roundtrip_rpm_zero_values),
#endif
#if CRSF_TEL_ENABLE_TEMPERATURE
        cmocka_unit_test(test_roundtrip_temperature),
        cmocka_unit_test(test_roundtrip_temperature_single_sensor),
        cmocka_unit_test(test_roundtrip_temperature_zero_values),
#endif
#if CRSF_TEL_ENABLE_VOLTAGES
        cmocka_unit_test(test_roundtrip_voltages),
        cmocka_unit_test(test_roundtrip_voltages_single_cell),
        cmocka_unit_test(test_roundtrip_voltages_zero_values),
#endif
#if CRSF_TEL_ENABLE_VTX
        cmocka_unit_test(test_roundtrip_vtx),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS
        cmocka_unit_test(test_roundtrip_link_statistics),
#endif
#if CRSF_ENABLE_RC_CHANNELS
        cmocka_unit_test(test_roundtrip_rc_channels_packed),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_RX
        cmocka_unit_test(test_roundtrip_link_statistics_rx),
#endif
#if CRSF_TEL_ENABLE_LINK_STATISTICS_TX
        cmocka_unit_test(test_roundtrip_link_statistics_tx),
#endif
#if CRSF_TEL_ENABLE_ATTITUDE
        cmocka_unit_test(test_roundtrip_attitude),
#endif
#if CRSF_TEL_ENABLE_MAVLINK_FC
        cmocka_unit_test(test_roundtrip_mavlink_fc),
#endif
#if CRSF_TEL_ENABLE_FLIGHT_MODE
        cmocka_unit_test(test_roundtrip_flight_mode),
        cmocka_unit_test(test_roundtrip_flight_mode_short),
#endif
#if CRSF_TEL_ENABLE_ESP_NOW_MESSAGES
        cmocka_unit_test(test_roundtrip_esp_now_messages),
#endif
#if CRSF_TEL_ENABLE_PARAMETER_GROUP
        cmocka_unit_test(test_roundtrip_device_ping),
        cmocka_unit_test(test_roundtrip_device_info),
        cmocka_unit_test(test_roundtrip_parameter_settings_entry),
        cmocka_unit_test(test_roundtrip_parameter_settings_entry_oversized),
        cmocka_unit_test(test_roundtrip_parameter_settings_entry_min_payload),
        cmocka_unit_test(test_roundtrip_parameter_read),
        cmocka_unit_test(test_roundtrip_parameter_write),
        cmocka_unit_test(test_roundtrip_parameter_write_oversized),
        cmocka_unit_test(test_roundtrip_parameter_write_min_payload),
#endif
#if CRSF_ENABLE_COMMAND
        cmocka_unit_test(test_roundtrip_command),
        cmocka_unit_test(test_roundtrip_command_oversized),
        cmocka_unit_test(test_direct_command_bad_inner_crc),

#endif
#if CRSF_TEL_ENABLE_MAVLINK_ENVELOPE
        cmocka_unit_test(test_roundtrip_mavlink_envelope),
        cmocka_unit_test(test_roundtrip_mavlink_envelope_max_data),
        cmocka_unit_test(test_roundtrip_mavlink_envelope_limited_size),
#endif
#if CRSF_TEL_ENABLE_MAVLINK_STATUS
        cmocka_unit_test(test_roundtrip_mavlink_status),
#endif
#endif
    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}
