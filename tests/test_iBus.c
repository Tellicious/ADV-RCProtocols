/* BEGIN Header */
/**
 ******************************************************************************
 * \file            test_iBus.c
 * \author          Andrea Vivani
 * \brief           FlySky iBUS (AFHDS 2A) protocol decoder test suite
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

#include "iBus.h"

/* Test Constants and Helpers */

#if IBUS_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 12345;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

/* Helper: calculate checksum matching IBUS implementation */
static uint16_t test_calc_checksum(const uint8_t* buf, uint8_t len) {
    uint16_t val = 0xFFFFU;
    for (uint8_t i = 0; i < len; ++i) {
        val -= buf[i];
    }
    return val;
}

/* Helper: create valid servo frame */
static void create_servo_frame(uint8_t* frame, const uint16_t* channels) {
    frame[0] = IBUS_SERVO_HDR0; // 0x20 (32 decimal - length)
    frame[1] = IBUS_SERVO_HDR1; // 0x40 (command)

    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        uint16_t val = (i < IBUS_MAX_CHANNELS && channels) ? channels[i] : 1500;
        frame[2 + i * 2] = (uint8_t)(val & 0xFF);
        frame[3 + i * 2] = (uint8_t)((val >> 8) & 0xFF);
    }

    uint16_t csum = test_calc_checksum(frame, IBUS_SERVO_FRAME_LEN - 2);
    frame[30] = (uint8_t)(csum & 0xFF);
    frame[31] = (uint8_t)(csum >> 8);
}

#if IBUS_ENABLE_TELEMETRY
/* Helper: create telemetry discover request */
static void create_discover_request(uint8_t* frame, uint8_t addr) {
    frame[0] = 0x04; // Length
    frame[1] = IBUS_TEL_CMD_DISCOVER | (addr & IBUS_TEL_ADDR_MASK);
    uint16_t csum = test_calc_checksum(frame, 2);
    frame[2] = (uint8_t)(csum & 0xFF);
    frame[3] = (uint8_t)(csum >> 8);
}

/* Helper: create telemetry type request */
static void create_type_request(uint8_t* frame, uint8_t addr) {
    frame[0] = 0x04; // Length
    frame[1] = IBUS_TEL_CMD_TYPE | (addr & IBUS_TEL_ADDR_MASK);
    uint16_t csum = test_calc_checksum(frame, 2);
    frame[2] = (uint8_t)(csum & 0xFF);
    frame[3] = (uint8_t)(csum >> 8);
}

/* Helper: create telemetry measurement request */
static void create_meas_request(uint8_t* frame, uint8_t addr) {
    frame[0] = 0x04; // Length
    frame[1] = IBUS_TEL_CMD_MEAS | (addr & IBUS_TEL_ADDR_MASK);
    uint16_t csum = test_calc_checksum(frame, 2);
    frame[2] = (uint8_t)(csum & 0xFF);
    frame[3] = (uint8_t)(csum >> 8);
}
#endif

/* ============================================================================
 * INITIALIZATION AND CONFIGURATION TESTS
 * ============================================================================ */

static void test_init_null_pointer(void** state) {
    (void)state;
    iBus_init(NULL); // Should not crash
}

static void test_init_defaults_and_clear(void** state) {
    (void)state;
    iBus_t ibus;

    // Initialize with some non-zero data to verify clearing
    memset(&ibus, 0xFF, sizeof(ibus));

    iBus_init(&ibus);

    // Verify RC channels are cleared
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        assert_int_equal(ibus.channels[i], 0);
    }

#if IBUS_ENABLE_TELEMETRY
    // Verify telemetry data contains only internal voltage sensor
    assert_int_equal(ibus.sensors[0].type, IBUS_MEAS_TYPE_INTERNAL_VOLTAGE);
    assert_int_equal(ibus.sensors[0].size, 2);
    assert_int_equal(ibus.sensors[0].value, 0);
    assert_int_equal(ibus.sensorCount, 1);
#endif

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.frames_bad_crc, 0);
    assert_int_equal(stats.servo_frames_ok, 0);
    assert_int_equal(stats.polls_rx, 0);
    assert_int_equal(stats.polls_unknown_addr, 0);
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
    assert_true(ibus.getTimestamp_ms == NULL);
    for (int i = 0; i < IBUS_TRACKED_FRAME_TYPES; i++) {
        assert_int_equal(ibus._packet_times[i], 0);
    }
#endif
}

#if IBUS_ENABLE_FRESHNESS_CHECK
static void test_timestamp_callback_setting(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Test setting callback
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);
    assert_false(ibus.getTimestamp_ms == NULL);

    // Test with NULL state
    iBus_setTimestampCallback(NULL, test_getTimestamp_ms); // Should not crash
}
#endif

/* ============================================================================
 * TELEMETRY SENSOR REGISTRATION TESTS
 * ============================================================================ */

#if IBUS_ENABLE_TELEMETRY
static void test_register_sensor_basic(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    iBus_Status_t result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(result, IBUS_OK); // Should return the specified address
    assert_int_equal(ibus.sensorCount, 2);
    assert_int_equal(ibus.sensors[0].type, IBUS_MEAS_TYPE_INTERNAL_VOLTAGE);
    assert_int_equal(ibus.sensors[0].size, 2);
    assert_int_equal(ibus.sensors[0].value, 0);
    assert_int_equal(ibus.sensors[1].type, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(ibus.sensors[1].size, 2);
    assert_int_equal(ibus.sensors[1].value, 0);
}

static void test_register_sensor_auto_size(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Test GPS coordinates (should auto-size to 4 bytes)
    iBus_Status_t result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_GPS_LAT);
    assert_int_equal(result, IBUS_OK);
    assert_int_equal(ibus.sensors[1].size, 4);

    result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_GPS_LON);
    assert_int_equal(result, IBUS_OK);
    assert_int_equal(ibus.sensors[2].size, 4);

    // Test temperature and RPM (should auto-size to 2 bytes)
    result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(result, IBUS_OK);
    assert_int_equal(ibus.sensors[3].size, 2);

    result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_RPM);
    assert_int_equal(result, IBUS_OK);
    assert_int_equal(ibus.sensors[4].size, 2);
}

static void test_register_sensor_type_collision(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register multiple sensors with auto-address to test collision avoidance
    iBus_Status_t result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_INTERNAL_VOLTAGE);
    assert_int_equal(result, IBUS_ERROR_SENSOR_EXISTS); // Should fail, already registered
    assert_int_equal(ibus.sensorCount, 1);              // Should still be 1 sensor

    result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(result, IBUS_OK);     // Should succeed, different type
    assert_int_equal(ibus.sensorCount, 2); // Now 2 sensors registered

    // Register another sensor of the same type
    result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(result, IBUS_ERROR_SENSOR_EXISTS); // Should fail, already registered
    assert_int_equal(ibus.sensorCount, 2);              // Should still be 2 sensors
}

static void test_register_sensor_max_limit(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    iBus_SensorType_t sensors[IBUS_TEL_MAX_SENSORS] = {IBUS_MEAS_TYPE_TEMPERATURE, IBUS_MEAS_TYPE_RPM,     IBUS_MEAS_TYPE_EXTERNAL_VOLTAGE,
                                                       IBUS_MEAS_TYPE_HEADING,     IBUS_MEAS_TYPE_CURRENT, IBUS_MEAS_TYPE_CLIMB,
                                                       IBUS_MEAS_TYPE_ACC_Z,       IBUS_MEAS_TYPE_ACC_Y,   IBUS_MEAS_TYPE_ACC_X,
                                                       IBUS_MEAS_TYPE_VSPEED,      IBUS_MEAS_TYPE_SPEED,   IBUS_MEAS_TYPE_DIST,
                                                       IBUS_MEAS_TYPE_ARMED,       IBUS_MEAS_TYPE_MODE,    IBUS_MEAS_TYPE_PRES,
                                                       IBUS_MEAS_TYPE_COG};

    // Fill up to maximum sensors
    for (uint8_t i = 0; i < IBUS_TEL_MAX_SENSORS - 1; i++) {
        iBus_Status_t result = iBus_registerSensor(&ibus, sensors[i]);
        assert_int_equal(result, IBUS_OK);
    }

    // Try to register one more (should fail)
    iBus_Status_t result = iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_GPS);
    assert_int_equal(result, IBUS_ERROR_BUFFER_OVERFLOW);     // Should fail, max reached
    assert_int_equal(ibus.sensorCount, IBUS_TEL_MAX_SENSORS); // Should still be max
}

static void test_register_sensor_null_pointer(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Test NULL ibus
    assert_int_equal(iBus_registerSensor(NULL, IBUS_MEAS_TYPE_TEMPERATURE), IBUS_ERROR_NULL_POINTER);
}

#if IBUS_ENABLE_FRESHNESS_CHECK
static void test_register_sensor_freshness_timestamp(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    assert_int_equal(ibus.sensors[1].updated_ms, mock_timestamp);
}
#endif

static void test_sensor_write_function(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

#if IBUS_ENABLE_FRESHNESS_CHECK
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);
#endif

    // Register a sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_RPM);

    // Test writing to the sensor
    iBus_Status_t result = iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_RPM, 5500);
    assert_int_equal(result, IBUS_OK); // Should succeed
    assert_int_equal(ibus.sensors[1].value, 5500);

#if IBUS_ENABLE_FRESHNESS_CHECK
    assert_int_equal(ibus.sensors[1].updated_ms, mock_timestamp);
#endif

    // Register another sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);

    // Test writing to the sensor
    result = iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE, 50);
    assert_int_equal(result, IBUS_OK); // Should succeed
    assert_int_equal(ibus.sensors[2].value, 50);

    // Test writing to non-existent sensor (should not crash)
    result = iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_CURRENT, 12345);
    assert_int_equal(result, IBUS_ERROR_SENSOR_NOT_FOUND); // Should return error

    // Test NULL ibus
    result = iBus_writeSensor(NULL, 3, 7777); // Should not crash
    assert_int_equal(result, IBUS_ERROR_NULL_POINTER);
}
#endif

/* ============================================================================
 * SERVO FRAME PROCESSING TESTS
 * ============================================================================ */

static void test_process_RC_frame_basic(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint16_t test_channels[IBUS_MAX_CHANNELS] = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 1050, 1150, 1250};

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, test_channels);

    iBus_Status_t result = iBus_processFrame(&ibus, frame);
    assert_int_equal(result, IBUS_OK);

    // Compare each channel
    for (uint8_t i = 0; i < IBUS_MAX_CHANNELS; i++) {
        assert_int_equal(ibus.channels[i], test_channels[i]);
    }

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
#if IBUS_ENABLE_TELEMETRY
    assert_int_equal(stats.frames_total, 0); // if telemetry is enabled, the increment happens inside the telemetry ISR
#else
    assert_int_equal(stats.frames_total, 1); // 1 good frame processed
#endif
    assert_int_equal(stats.servo_frames_ok, 1);
#endif
}

static void test_process_RC_frame_null_arguments(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, NULL);

    // Test NULL ibus
    assert_int_equal(iBus_processFrame(NULL, frame), IBUS_ERROR_NULL_POINTER);

    // Test NULL frame
    assert_int_equal(iBus_processFrame(&ibus, NULL), IBUS_ERROR_NULL_POINTER);
}

static void test_process_RC_frame_invalid_header(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, NULL);

    // Corrupt header
    frame[0] = 0x21; // Wrong length header
    int result = iBus_processFrame(&ibus, frame);
    assert_int_equal(result, IBUS_ERROR_INVALID_FRAME);

    // Reset and corrupt command header
    frame[0] = IBUS_SERVO_HDR0;
    frame[1] = 0x41; // Wrong command header
    result = iBus_processFrame(&ibus, frame);
    assert_int_equal(result, IBUS_ERROR_INVALID_FRAME);

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
#endif
}

static void test_process_RC_frame_invalid_checksum(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, NULL);

    // Corrupt checksum
    frame[30] = 0x00;
    frame[31] = 0x00;

    int result = iBus_processFrame(&ibus, frame);
    assert_int_equal(result, IBUS_ERROR_CHECKSUM_FAIL);

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.frames_bad_crc, 1);
    assert_int_equal(stats.servo_frames_ok, 0);
#endif
}

static void test_process_RC_frame_boundary_values(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Test with extreme channel values
    uint16_t boundary_channels[IBUS_MAX_CHANNELS] = {0, 65535, 1000, 2000, 500, 2500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, boundary_channels);

    int result = iBus_processFrame(&ibus, frame);
    assert_int_equal(result, IBUS_OK);

    // Verify extreme values were handled correctly
    assert_int_equal(ibus.channels[0], 0);
    assert_int_equal(ibus.channels[1], 65535);
    assert_int_equal(ibus.channels[2], 1000);
    assert_int_equal(ibus.channels[3], 2000);
}

#if IBUS_ENABLE_FRESHNESS_CHECK
static void test_process_RC_frame_freshness(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, NULL);

    iBus_processFrame(&ibus, frame);

    // Should update RC freshness
    assert_int_equal(ibus._packet_times[IBUS_FRAME_TYPE_RC], mock_timestamp);
}
#endif

/* ============================================================================
 * TELEMETRY FRAME HANDLING TESTS (FROM ISR)
 * ============================================================================ */

#if IBUS_ENABLE_TELEMETRY
static void test_telemetry_from_isr_basic_discover(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register a sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);

    uint8_t rx_frame[4];

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    // Create discover request for address 0
    create_discover_request(rx_frame, 0);
    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_REPLY_READY);
    assert_int_equal(tx_len, 4); // Discover reply is 4 bytes

    // Verify reply structure
    assert_int_equal(tx_frame[0], 0x04);                      // Length
    assert_int_equal(tx_frame[1], IBUS_TEL_CMD_DISCOVER | 0); // Command + address

    // Verify checksum
    uint16_t expected_csum = test_calc_checksum(tx_frame, 2);
    uint16_t reply_csum = (uint16_t)(tx_frame[2] | ((uint16_t)tx_frame[3] << 8));
    assert_int_equal(reply_csum, expected_csum);

    // Create another discover request for address 1
    create_discover_request(rx_frame, 1);
    result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_REPLY_READY);
    assert_int_equal(tx_len, 4); // Discover reply is 4 bytes

    // Verify reply structure
    assert_int_equal(tx_frame[0], 0x04);                      // Length
    assert_int_equal(tx_frame[1], IBUS_TEL_CMD_DISCOVER | 1); // Command + address

    // Verify checksum
    expected_csum = test_calc_checksum(tx_frame, 2);
    reply_csum = (uint16_t)(tx_frame[2] | ((uint16_t)tx_frame[3] << 8));
    assert_int_equal(reply_csum, expected_csum);

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.polls_rx, 2);
#endif
}

static void test_telemetry_from_isr_type_request(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register a 4-byte sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_GPS_LAT);

    uint8_t rx_frame[4];
    create_type_request(rx_frame, 1);

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_REPLY_READY);
    assert_int_equal(tx_len, 6); // Type reply is 6 bytes

    // Verify reply structure
    assert_int_equal(tx_frame[0], 0x06);                   // Length
    assert_int_equal(tx_frame[1], IBUS_TEL_CMD_TYPE | 1);  // Command + address
    assert_int_equal(tx_frame[2], IBUS_MEAS_TYPE_GPS_LAT); // Sensor type
    assert_int_equal(tx_frame[3], 4);                      // Size

    // Verify checksum
    uint16_t expected_csum = test_calc_checksum(tx_frame, 4);
    uint16_t reply_csum = (uint16_t)(tx_frame[4] | ((uint16_t)tx_frame[5] << 8));
    assert_int_equal(reply_csum, expected_csum);
}

static void test_telemetry_from_isr_measurement_request_2byte(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register a 2-byte sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE, 0x1234); // Set test value

    uint8_t rx_frame[4];
    create_meas_request(rx_frame, 1);

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_REPLY_READY); // Reply ready
    assert_int_equal(tx_len, 6);                    // Measurement reply for 2-byte sensor is 6 bytes

    // Verify reply structure
    assert_int_equal(tx_frame[0], 0x06);                  // Length
    assert_int_equal(tx_frame[1], IBUS_TEL_CMD_MEAS | 1); // Command + address
    assert_int_equal(tx_frame[2], 0x34);                  // Low byte of value
    assert_int_equal(tx_frame[3], 0x12);                  // High byte of value

    // Verify checksum
    uint16_t expected_csum = test_calc_checksum(tx_frame, 4);
    uint16_t reply_csum = (uint16_t)(tx_frame[4] | ((uint16_t)tx_frame[5] << 8));
    assert_int_equal(reply_csum, expected_csum);
}

static void test_telemetry_from_isr_measurement_request_4byte(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register a 4-byte sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_GPS_LAT);
    iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_GPS_LAT, 0x12345678);

    uint8_t rx_frame[4];
    create_meas_request(rx_frame, 1);

    uint8_t tx_frame[12];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_REPLY_READY); // Reply ready
    assert_int_equal(tx_len, 8);                    // Measurement reply for 4-byte sensor is 8 bytes

    // Verify reply structure
    assert_int_equal(tx_frame[0], 0x08);                  // Length
    assert_int_equal(tx_frame[1], IBUS_TEL_CMD_MEAS | 1); // Command + address
    assert_int_equal(tx_frame[2], 0x78);                  // Byte 0 of value
    assert_int_equal(tx_frame[3], 0x56);                  // Byte 1 of value
    assert_int_equal(tx_frame[4], 0x34);                  // Byte 2 of value
    assert_int_equal(tx_frame[5], 0x12);                  // Byte 3 of value

    // Verify checksum
    uint16_t expected_csum = test_calc_checksum(tx_frame, 6);
    uint16_t reply_csum = (uint16_t)(tx_frame[6] | ((uint16_t)tx_frame[7] << 8));
    assert_int_equal(reply_csum, expected_csum);
}

static void test_telemetry_from_isr_unknown_sensor(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register one sensor at address 1
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_RPM);

    uint8_t rx_frame[4];
    create_discover_request(rx_frame, 8); // Non-existent sensor

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_ERROR_SENSOR_NOT_FOUND);

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.polls_unknown_addr, 1);
#endif
}

static void test_telemetry_from_isr_invalid_checksum(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t rx_frame[4] = {0x04, IBUS_TEL_CMD_DISCOVER | 1, 0x00, 0x00}; // Bad checksum

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    int result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_ERROR_CHECKSUM_FAIL);

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.frames_bad_crc, 1);
    assert_int_equal(stats.frames_total, 1);
#endif
}

static void test_telemetry_from_isr_null_arguments(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t rx_frame[4];
    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    create_discover_request(rx_frame, 1);

    // Test NULL ibus
    assert_int_equal(iBus_handleTelemetryFromISR(NULL, rx_frame, tx_frame, &tx_len), IBUS_ERROR_NULL_POINTER);

    // Test NULL rx
    assert_int_equal(iBus_handleTelemetryFromISR(&ibus, NULL, tx_frame, &tx_len), IBUS_ERROR_NULL_POINTER);

    // Test NULL tx
    assert_int_equal(iBus_handleTelemetryFromISR(&ibus, rx_frame, NULL, &tx_len), IBUS_ERROR_NULL_POINTER);

    // Test NULL tx_len
    assert_int_equal(iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, NULL), IBUS_ERROR_NULL_POINTER);
}

static void test_telemetry_from_isr_invalid_length(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    uint8_t rx_frame[3] = {0x03, 0x81, 0x00}; // Too short
    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_NO_REPLY);
}

static void test_telemetry_from_isr_unknown_command(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    // Register a sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_RPM);

    // Create request with unknown command
    uint8_t rx_frame[4] = {0x04, 0xB1, 0x00, 0x00}; // 0xB0 is not a valid command
    uint16_t csum = test_calc_checksum(rx_frame, 2);
    rx_frame[2] = (uint8_t)(csum & 0xFF);
    rx_frame[3] = (uint8_t)(csum >> 8);

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_Status_t result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
    assert_int_equal(result, IBUS_TEL_NO_REPLY); // No reply for unknown command
}

#if IBUS_ENABLE_FRESHNESS_CHECK
static void test_telemetry_from_isr_freshness(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    // Register a sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);

    uint8_t rx_frame[4];
    create_discover_request(rx_frame, 1);

    uint8_t tx_frame[10];
    uint8_t tx_len = 0;

    iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);

    // Should update telemetry freshness
    assert_int_equal(ibus._packet_times[IBUS_FRAME_TYPE_TEL], mock_timestamp);
}
#endif
#endif

/* ============================================================================
 * STATISTICS TESTS
 * ============================================================================ */

#if IBUS_ENABLE_STATS
static void test_stats_getter_and_reset(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);

    // Initial state
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.frames_bad_crc, 0);
    assert_int_equal(stats.servo_frames_ok, 0);
    assert_int_equal(stats.polls_rx, 0);
    assert_int_equal(stats.polls_unknown_addr, 0);

    // Process some data to change stats
    uint8_t frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(frame, NULL);
    iBus_processFrame(&ibus, frame);

    // Create bad frame
    frame[0] = 0x21; // Bad length
    iBus_processFrame(&ibus, frame);

    iBus_getStats(&ibus, &stats);
#if IBUS_ENABLE_TELEMETRY
    assert_int_equal(stats.frames_total, 0); // If telemetry is enabled, the increment happens inside the telemetry ISR
#else
    assert_int_equal(stats.frames_total, 2); // 1 good + 1 bad
#endif
    assert_int_equal(stats.servo_frames_ok, 1);

    // Test reset
    iBus_resetStats(&ibus);
    iBus_getStats(&ibus, &stats);
    assert_int_equal(stats.frames_total, 0);
    assert_int_equal(stats.servo_frames_ok, 0);
}

static void test_stats_null_arguments(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

    iBus_Stats_t stats;

    // Test NULL arguments - should not crash
    iBus_getStats(NULL, &stats);
    iBus_getStats(&ibus, NULL);
    iBus_resetStats(NULL);
}

static void test_stats_comprehensive_counting(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

#if IBUS_ENABLE_TELEMETRY
    // Register some sensors
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);
    iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE, 400);
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_RPM);
    iBus_writeSensor(&ibus, IBUS_MEAS_TYPE_RPM, 5000);
#endif

    // Process various frames
    uint8_t servo_frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(servo_frame, NULL);

    // Good servo frames
    for (int i = 0; i < 3; i++) {
        iBus_processFrame(&ibus, servo_frame);
    }

    // Bad servo frames (wrong length)
    uint8_t bad_frame[20];
    for (int i = 0; i < 2; i++) {
        iBus_processFrame(&ibus, bad_frame);
    }

    // Bad servo frames (wrong checksum)
    servo_frame[30] = 0x00;
    servo_frame[31] = 0x00;
    iBus_processFrame(&ibus, servo_frame);

#if IBUS_ENABLE_TELEMETRY
    // Telemetry requests
    uint8_t tel_req[4];
    uint8_t tel_reply[10];
    uint8_t tel_len;

    // Valid telemetry requests
    create_discover_request(tel_req, 1);
    iBus_handleTelemetryFromISR(&ibus, tel_req, tel_reply, &tel_len);
    create_type_request(tel_req, 2);
    iBus_handleTelemetryFromISR(&ibus, tel_req, tel_reply, &tel_len);

    // Unknown sensor request
    create_discover_request(tel_req, 99);
    iBus_handleTelemetryFromISR(&ibus, tel_req, tel_reply, &tel_len);
#endif

    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);

    assert_int_equal(stats.servo_frames_ok, 3);
    assert_int_equal(stats.frames_bad_crc, 1);

#if IBUS_ENABLE_TELEMETRY
    assert_int_equal(stats.polls_rx, 2);
    assert_int_equal(stats.polls_unknown_addr, 1);
#endif
}
#endif

/* ============================================================================
 * FRESHNESS CHECKING TESTS
 * ============================================================================ */

#if IBUS_ENABLE_FRESHNESS_CHECK
static void test_freshness_basic_functionality(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    // Initially no frames should be fresh
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 100));
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 100));

    // Process servo frame
    uint8_t servo_frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(servo_frame, NULL);
    iBus_processFrame(&ibus, servo_frame);

    // Should be fresh immediately
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 0));
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 100));
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 100)); // Different type

    // Advance time
    mock_timestamp += 50;
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 100)); // Still fresh
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 30)); // Too old

    // Advance time further
    mock_timestamp += 100;
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 100)); // Now stale
}

#if IBUS_ENABLE_TELEMETRY
static void test_freshness_telemetry_tracking(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    // Register sensor
    iBus_registerSensor(&ibus, IBUS_MEAS_TYPE_TEMPERATURE);

    // Process telemetry request
    uint8_t tel_req[4];
    uint8_t tel_reply[10];
    uint8_t tel_len;

    create_discover_request(tel_req, 1);
    mock_timestamp = 100; // Set initial timestamp
    iBus_handleTelemetryFromISR(&ibus, tel_req, tel_reply, &tel_len);

    // Should be fresh
    mock_timestamp += 10;
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 20));
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 20)); // Different type

    // Test freshness aging
    mock_timestamp += 25;
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 50));
    assert_false(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 20));
}
#endif

static void test_freshness_invalid_arguments(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    // Test invalid frame type
    assert_false(iBus_isFrameFresh(&ibus, IBUS_TRACKED_FRAME_TYPES, 10));
    assert_false(iBus_isFrameFresh(&ibus, 255, 10));

    // Test NULL state
    assert_false(iBus_isFrameFresh(NULL, IBUS_FRAME_TYPE_RC, 10));

    // Test no callback set
    iBus_t no_callback;
    iBus_init(&no_callback);
    assert_false(iBus_isFrameFresh(&no_callback, IBUS_FRAME_TYPE_RC, 10));
}

static void test_freshness_edge_cases(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);

    // Test max_age_ms = 0 (should always be fresh if updated)
    uint8_t servo_frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(servo_frame, NULL);
    iBus_processFrame(&ibus, servo_frame);

    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 0));

    // Test timestamp wrap-around scenario
    mock_timestamp = UINT32_MAX - 10;
    iBus_processFrame(&ibus, servo_frame);

    mock_timestamp = 10; // Wrapped around
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 30));
}
#endif

/* ============================================================================
 * COMPREHENSIVE INTEGRATION TESTS
 * ============================================================================ */

static void test_complete_servo_telemetry_workflow(void** state) {
    (void)state;
    iBus_t ibus;
    iBus_init(&ibus);

#if IBUS_ENABLE_FRESHNESS_CHECK
    iBus_setTimestampCallback(&ibus, test_getTimestamp_ms);
#endif

#if IBUS_ENABLE_TELEMETRY
    // Register multiple sensors
    iBus_SensorType_t sensors[] = {IBUS_MEAS_TYPE_INTERNAL_VOLTAGE, IBUS_MEAS_TYPE_TEMPERATURE, IBUS_MEAS_TYPE_RPM, IBUS_MEAS_TYPE_GPS_LAT,
                                   IBUS_MEAS_TYPE_CURRENT};

    int32_t value[] = {4, 450, 5500, -123456789, -12345};

    for (int i = 1; i <= 4; i++) {
        iBus_registerSensor(&ibus, sensors[i]);
    }

    // Test sensor value updates, in reverse order
    for (int i = 4; i >= 0; i--) {
        iBus_writeSensor(&ibus, sensors[i], value[i]);
    }
#endif

    // Process servo frames with different channel values
    uint16_t test_channels[IBUS_MAX_CHANNELS] = {1000, 1200, 1400, 1600, 1800, 2000, 1100, 1300, 1500, 1700, 1900, 1050, 1150, 1250};

    uint8_t servo_frame[IBUS_SERVO_FRAME_LEN];
    create_servo_frame(servo_frame, test_channels);

    int result = iBus_processFrame(&ibus, servo_frame);
    assert_int_equal(result, IBUS_OK);

#if IBUS_ENABLE_TELEMETRY
    // Test telemetry workflow for each sensor
    uint8_t rx_frame[4];
    uint8_t tx_frame[10];
    uint8_t tx_len;

    // Test discovery
    for (uint8_t addr = 0; addr <= 4; addr++) {
        create_discover_request(rx_frame, addr);
        result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
        assert_int_equal(result, IBUS_TEL_REPLY_READY);
        assert_int_equal(tx_len, 4);
    }

    // Test type requests
    for (uint8_t addr = 0; addr <= 4; addr++) {
        create_type_request(rx_frame, addr);
        result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
        assert_int_equal(result, IBUS_TEL_REPLY_READY);
        assert_int_equal(tx_len, 6);
        assert_int_equal(tx_frame[2], sensors[addr]); // Check sensor type
    }

    // Test measurement requests
    for (uint8_t addr = 0; addr <= 4; addr++) {
        create_meas_request(rx_frame, addr);
        result = iBus_handleTelemetryFromISR(&ibus, rx_frame, tx_frame, &tx_len);
        assert_int_equal(result, IBUS_TEL_REPLY_READY);
        assert_int_equal(tx_len, sensors[addr] == IBUS_MEAS_TYPE_GPS_LAT ? 8 : 6); // GPS uses 4 bytes
        printf("Addr: %d, tx_len: %02x\n", addr, tx_frame[0]);
        assert_int_equal(tx_frame[2], value[addr] & 0xFF);        // Low byte
        assert_int_equal(tx_frame[3], (value[addr] >> 8) & 0xFF); // High byte
        if (tx_frame[0] == 0x08) {
            assert_int_equal(tx_frame[4], (value[addr] >> 16) & 0xFF); // Byte 2
            assert_int_equal(tx_frame[5], (value[addr] >> 24) & 0xFF); // Byte 3
        }
    }
#endif

#if IBUS_ENABLE_STATS
    iBus_Stats_t stats;
    iBus_getStats(&ibus, &stats);
#if IBUS_ENABLE_TELEMETRY
    assert_int_equal(stats.frames_total, 15); // 15 telemetry requests
#else
    assert_int_equal(stats.frames_total, 1); // 1 servo frame processed
#endif
    assert_int_equal(stats.servo_frames_ok, 1);
#if IBUS_ENABLE_TELEMETRY
    assert_int_equal(stats.polls_rx, 15); // 15 telemetry requests
#endif
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_RC, 100));
#if IBUS_ENABLE_TELEMETRY
    assert_true(iBus_isFrameFresh(&ibus, IBUS_FRAME_TYPE_TEL, 100));
#endif
#endif
}

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================ */

int main(void) {
    const struct CMUnitTest tests[] = {
        /* Initialization Tests */
        cmocka_unit_test(test_init_null_pointer),
        cmocka_unit_test(test_init_defaults_and_clear),
#if IBUS_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_timestamp_callback_setting),
#endif

    /* Telemetry Sensor Registration Tests */
#if IBUS_ENABLE_TELEMETRY
        cmocka_unit_test(test_register_sensor_basic),
        cmocka_unit_test(test_register_sensor_auto_size),
        cmocka_unit_test(test_register_sensor_type_collision),
        cmocka_unit_test(test_register_sensor_max_limit),
        cmocka_unit_test(test_register_sensor_null_pointer),
#if IBUS_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_register_sensor_freshness_timestamp),
#endif
        cmocka_unit_test(test_sensor_write_function),
#endif

        /* Servo Frame Tests */
        cmocka_unit_test(test_process_RC_frame_basic),
        cmocka_unit_test(test_process_RC_frame_null_arguments),
        cmocka_unit_test(test_process_RC_frame_invalid_header),
        cmocka_unit_test(test_process_RC_frame_invalid_checksum),
        cmocka_unit_test(test_process_RC_frame_boundary_values),
#if IBUS_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_process_RC_frame_freshness),
#endif

    /* Telemetry Frame Tests */
#if IBUS_ENABLE_TELEMETRY
        cmocka_unit_test(test_sensor_write_function),
        cmocka_unit_test(test_telemetry_from_isr_basic_discover),
        cmocka_unit_test(test_telemetry_from_isr_type_request),
        cmocka_unit_test(test_telemetry_from_isr_measurement_request_2byte),
        cmocka_unit_test(test_telemetry_from_isr_measurement_request_4byte),
        cmocka_unit_test(test_telemetry_from_isr_unknown_sensor),
        cmocka_unit_test(test_telemetry_from_isr_invalid_checksum),
        cmocka_unit_test(test_telemetry_from_isr_null_arguments),
        cmocka_unit_test(test_telemetry_from_isr_invalid_length),
        cmocka_unit_test(test_telemetry_from_isr_unknown_command),
#if IBUS_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_telemetry_from_isr_freshness),
#endif
#endif

#if IBUS_ENABLE_STATS
        /* Statistics Tests */
        cmocka_unit_test(test_stats_getter_and_reset),
        cmocka_unit_test(test_stats_null_arguments),
        cmocka_unit_test(test_stats_comprehensive_counting),
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
        /* Freshness Tests */
        cmocka_unit_test(test_freshness_basic_functionality),
#if IBUS_ENABLE_TELEMETRY
        cmocka_unit_test(test_freshness_telemetry_tracking),
#endif
        cmocka_unit_test(test_freshness_invalid_arguments),
        cmocka_unit_test(test_freshness_edge_cases),
#endif

        /* Integration Tests */
        cmocka_unit_test(test_complete_servo_telemetry_workflow),
    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}
