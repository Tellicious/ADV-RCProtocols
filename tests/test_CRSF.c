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

/* Test Constants and Helpers */

#if CRSF_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 12345;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

uint8_t test_calc_checksum(const uint8_t* data, uint8_t length, uint8_t poly) {
    uint8_t crc = 0x00; // Initialize CRC to 0
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
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
    for (int i = 0; i < CRSF_TRACKED_FRAME_TYPES; i++) {
        assert_int_equal(crsf._packet_times[i], 0);
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
    uint8_t len = 0;
    assert_int_equal(CRSF_buildFrame(NULL, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, &len), CRSF_ERROR_NULL_POINTER);
    assert_int_equal(CRSF_buildFrame(&crsf, CRSF_ADDRESS_BROADCAST, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, NULL, &len), CRSF_ERROR_NULL_POINTER);
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
    uint8_t len = 0;

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

    for (size_t i = 0; i < sizeof(valid_addresses) / sizeof(valid_addresses[0]); i++) {
        len = 0;
        assert_int_not_equal(CRSF_buildFrame(&s, valid_addresses[i], CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, buf, &len), CRSF_ERROR_ADDR);
    }

    /* Test some invalid addresses */
    uint8_t invalid_addresses[] = {0x01, 0x02, 0x50, 0xAA, 0xFF};
    for (size_t i = 0; i < sizeof(invalid_addresses) / sizeof(invalid_addresses[0]); i++) {
        len = 0;
        assert_int_equal(CRSF_buildFrame(&s, invalid_addresses[i], CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, buf, &len), CRSF_ERROR_ADDR);
    }
}

static void test_build_invalid_address(void** state) {
    (void)state;
    CRSF_t crsf;
    CRSF_init(&crsf);
    uint8_t frame[CRSF_MAX_FRAME_LEN + 2U];
    uint8_t len = 0;
    assert_int_equal(CRSF_buildFrame(&crsf, 0xFF, CRSF_FRAMETYPE_HEARTBEAT, 0, frame, &len), CRSF_ERROR_ADDR);
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
    uint8_t len = 0;
    assert_int_equal(CRSF_buildFrame(&crsf, CRSF_ADDRESS_BROADCAST, (CRSF_FrameType_t)0xFF, 0, frame, &len), CRSF_ERROR_INVALID_FRAME);
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
    CRSF_FrameType_t t = 0;
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
                       {CRSF_FRAMETYPE_BAROALT_VSPEED, sizeof(CRSF_BaroAlt_VS_t), sizeof(CRSF_BaroAlt_VS_t)},
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

        assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);

        /* Test valid frame length */
        frame[1] = frame_tests[ii].valid_payload_len + 2U; /* +1 for type +1 for CRC */
        memset(&frame[3], 0x00, frame[1] - 2U);            /* fill payload */
        frame[frame[1] + 1U] = test_calc_checksum(&frame[2], frame[1] - 1, 0xD5U);

        /* This might fail due to content but should pass length validation */
        assert_true(CRSF_processFrame(&s, frame, &t) != CRSF_ERROR_TYPE_LENGTH);
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
    assert_int_equal(frameLength, sizeof(CRSF_BaroAlt_VS_t) + 3U + 1U);

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
    assert_int_equal(frameLength, sizeof(CRSF_BaroAlt_VS_t) + 3U + 1U);

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
    assert_int_equal(frameLength, sizeof(CRSF_BaroAlt_VS_t) + 3U + 1U);

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
    assert_int_equal(frameLength, sizeof(CRSF_BaroAlt_VS_t) + 3U + 1U);

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
    for (uint8_t i = 5; i < CRSF_RC_CHANNELS; i++) {
        tx.RC.channels[i] = 1000 + (i * 62); // Spread across range
    }

    /* Test Build */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, 0, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 22 + 3U + 1U); // 22 bytes for packed channels + header + CRC

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED);

    /* Check received frame (allow small tolerance due to packing/unpacking) */
    for (uint8_t i = 0; i < CRSF_RC_CHANNELS; i++) {
        assert_true(abs((int)rx.RC.channels[i] - (int)tx.RC.channels[i]) <= 2);
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
    for (uint8_t i = 0; i < 10; i++) {
        tx.ParamSettingsEntry.Payload[i] = i + 0xA0;
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
    for (uint8_t i = 0; i < 10; i++) {
        assert_int_equal(rx.ParamSettingsEntry.Payload[i], i + 0xA0);
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
    for (uint8_t i = 0; i < sizeof(testPattern); i++) {
        testPattern[i] = i;
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
    for (uint8_t i = 0; i < CRSF_MAX_PARAM_SETTINGS_PAYLOAD; i++) {
        assert_int_equal(rx.ParamSettingsEntry.Payload[i], testPattern[i]);
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
    printf("%d\n", CRSF_processFrame(&rx, frame, &frameType));
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
    for (uint8_t i = 0; i < 15; i++) {
        tx.ParamWrite.Data[i] = 0x55 + i;
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
    for (uint8_t i = 0; i < 15; i++) {
        assert_int_equal(rx.ParamWrite.Data[i], 0x55 + i);
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
    for (uint8_t i = 0; i < sizeof(testPattern); i++) {
        testPattern[i] = i;
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
    for (uint8_t i = 0; i < CRSF_MAX_PARAM_DATA_LEN; i++) {
        assert_int_equal(rx.ParamWrite.Data[i], testPattern[i]);
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
    tx.Command.Command_ID = 0x42;
    // Fill payload with test data
    for (uint8_t i = 0; i < 20; i++) {
        tx.Command.Payload[i] = 0x10 + i;
    }

    /* Test Build with 20 payload bytes */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_COMMAND, 20, frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, 23U + 1U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_COMMAND);

    /* Check received frame */
    assert_int_equal(rx.Command.dest_address, CRSF_ADDRESS_FLIGHT_CONTROLLER);
    assert_int_equal(rx.Command.origin_address, CRSF_ADDRESS_RADIO_TRANSMITTER);
    assert_int_equal(rx.Command.Command_ID, 0x42);
    for (uint8_t i = 0; i < 20; i++) {
        assert_int_equal(rx.Command.Payload[i], 0x10 + i);
    }

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
    uint8_t testPattern[CRSF_MAX_COMMAND_PAYLOAD + 10];

    CRSF_init(&tx);
    CRSF_init(&rx);

    // Create test pattern
    for (uint8_t i = 0; i < sizeof(testPattern); i++) {
        testPattern[i] = i;
    }

    // Setup oversized payload
    tx.Command.dest_address = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    tx.Command.origin_address = CRSF_ADDRESS_RADIO_TRANSMITTER;
    tx.Command.Command_ID = 0x42;
    memcpy(tx.Command.Payload, testPattern, sizeof(testPattern));

    /* Test Build with oversized payload */
    assert_true(CRSF_buildFrame(&tx, CRSF_ADDRESS_RADIO_TRANSMITTER, CRSF_FRAMETYPE_COMMAND, sizeof(testPattern), frame, &frameLength) == CRSF_OK);
    assert_int_equal(frameLength, CRSF_MAX_COMMAND_PAYLOAD + 3U + 1U + 3U + 1U);

    /* Test Process */
    assert_true(CRSF_processFrame(&rx, frame, &frameType) == CRSF_OK);
    assert_true(frameType == CRSF_FRAMETYPE_COMMAND);

    /* Verify payload was truncated to max size */
    for (uint8_t i = 0; i < CRSF_MAX_COMMAND_PAYLOAD; i++) {
        assert_int_equal(rx.Command.Payload[i], testPattern[i]);
    }
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
    for (uint8_t i = 0; i < 40; i++) {
        tx.MAVLinkEnv.data[i] = i ^ 0xAA;
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
    for (uint8_t i = 0; i < 40; i++) {
        assert_int_equal(rx.MAVLinkEnv.data[i], i ^ 0xAA);
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
    for (uint8_t i = 0; i < CRSF_MAX_MAVLINK_PAYLOAD; i++) {
        tx.MAVLinkEnv.data[i] = i;
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
    for (uint8_t i = 0; i < CRSF_MAX_MAVLINK_PAYLOAD; i++) {
        assert_int_equal(rx.MAVLinkEnv.data[i], i);
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
    for (uint8_t i = 0; i < CRSF_MAX_MAVLINK_PAYLOAD; i++) {
        tx.MAVLinkEnv.data[i] = i;
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
    for (uint8_t i = 0; i < CRSF_MAX_MAVLINK_PAYLOAD; i++) {
        assert_int_equal(rx.MAVLinkEnv.data[i], i);
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

/* Integration Tests */
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
