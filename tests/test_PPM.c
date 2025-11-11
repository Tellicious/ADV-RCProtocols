/* BEGIN Header */
/**
 ******************************************************************************
 * \file            test_PPM.c
 * \author          Andrea Vivani
 * \brief           PPM protocol decoder test suite
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

#include "PPM.h"

/* Test Constants and Helpers */
#define TEST_TIMER_FREQ_1MHZ   1000000U
#define TEST_TIMER_FREQ_2MHZ   2000000U
#define TEST_TIMER_RELOAD_MAX  65535U
#define TEST_TIMER_RELOAD_1000 1000U

#if PPM_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 0;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

/* ============================================================================
 * INITIALIZATION TESTS
 * ============================================================================ */

static void test_init_null_pointer(void** state) {
    (void)state;
    PPM_Status_t status = PPM_init(NULL, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX); // Should not crash
    assert_int_equal(status, PPM_ERROR_NULL_POINTER);
}

static void test_init_null_frequency(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_Status_t status = PPM_init(&ppm, 0, TEST_TIMER_RELOAD_MAX); // Should not crash
    assert_int_equal(status, PPM_ERROR);
}

static void test_init_defaults_and_clear(void** state) {
    (void)state;
    PPM_t ppm;

    // Initialize with some non-zero data to verify clearing
    memset(&ppm, 0xFF, sizeof(ppm));

    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Verify internal state is cleared
    assert_int_equal(ppm._timerAutoReload, TEST_TIMER_RELOAD_MAX);
    assert_int_equal(ppm._freqMultiplier, 1); // 1000000 / 1000000 = 1
    assert_int_equal(ppm._lastTimerCounter, 0);
    assert_int_equal(ppm._currentChannel, 0);

    // Verify channels are cleared
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        assert_int_equal(ppm.channels[ii], 0);
    }

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 0);
#endif
}

static void test_init_frequency_multiplier_calculation(void** state) {
    (void)state;
    PPM_t ppm;

    // Test 1MHz timer (most common)
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(ppm._freqMultiplier, 1, 1e-5);

    // Test 2MHz timer
    PPM_init(&ppm, TEST_TIMER_FREQ_2MHZ, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(ppm._freqMultiplier, 0.5, 1e-5);

    // Test 500kHz timer
    PPM_init(&ppm, 500000U, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(ppm._freqMultiplier, 2, 1e-5);

    // Test 100kHz timer
    PPM_init(&ppm, 100000U, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(ppm._freqMultiplier, 10, 1e-5);
}

static void test_init_packet_end_ticks_calculation(void** state) {
    (void)state;
    PPM_t ppm;

    // Test with 1MHz timer, multiplier = 1
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    // PPM_PACKET_END_US is typically 3000us, so 3000 / 1 = 3000
    assert_int_equal(ppm._packetEndTicks, PPM_PACKET_END_US / 1);

    // Test with 500kHz timer, multiplier = 2
    PPM_init(&ppm, 500000U, TEST_TIMER_RELOAD_MAX);
    assert_int_equal(ppm._packetEndTicks, PPM_PACKET_END_US / 2);
}

#if PPM_ENABLE_FRESHNESS_CHECK
static void test_timestamp_callback_setting(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test setting callback
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);

    // Test with NULL state
    PPM_setTimestampCallback(NULL, test_getTimestamp_ms); // Should not crash
}
#endif

/* ============================================================================
 * PACKET PROCESSING TESTS
 * ============================================================================ */

static void test_processPacket_null_pointer(void** state) {
    (void)state;
    PPM_Status_t status = PPM_processPacket(NULL, 100);
    assert_int_equal(status, PPM_ERROR_NULL_POINTER);
}

static void test_processPacket_normal_pulse(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Simulate receiving pulses for a channel (typical PPM pulse: 1000-2000us)
    uint16_t pulse1 = 1000; // 1ms pulse
    uint16_t pulse2 = 2500; // 1.5ms pulse (1500us from last)

    // First pulse
    PPM_Status_t status = PPM_processPacket(&ppm, pulse1);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, 1);
    assert_int_equal(ppm.channels[0], 1000); // 1000 ticks * 1 multiplier

    // Second pulse
    status = PPM_processPacket(&ppm, pulse2);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, 2);
    assert_int_equal(ppm.channels[1], 1500); // (2500 - 1000) * 1
}

static void test_processPacket_timer_overflow(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_1000);

    // Set last counter near max
    ppm._lastTimerCounter = 950;

    // Next counter wraps around
    uint16_t new_counter = 100; // Wrapped around

    PPM_Status_t status = PPM_processPacket(&ppm, new_counter);

    // Pulse length should be: (100 - 950) + (1000 + 1) = -850 + 1001 = 151
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm.channels[0], 151); // 151 ticks * 1 multiplier
}

static void test_processPacket_packet_end_detection(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    uint16_t counter = 0;

#if PPM_ENABLE_FRESHNESS_CHECK
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);
    mock_timestamp = 1000;
#endif

    // Fill all channels first
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS - 1; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }
    counter += 1500;
    PPM_Status_t status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_SUCCESS);

    // Now send a long pulse (packet end marker)
    counter += ppm._packetEndTicks + 100;

    status = PPM_processPacket(&ppm, counter);

    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, 0); // Reset to start

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);
#endif
}

static void test_processPacket_partial_packet_then_end(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    uint16_t counter = 0;

    // Fill only half the channels
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS / 2; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Send packet end marker before all channels filled
    counter += ppm._packetEndTicks + 100;
    PPM_Status_t status = PPM_processPacket(&ppm, counter);

    // Should detect end, reset and return OK
    assert_int_equal(status, PPM_SUCCESS);
    assert_int_equal(ppm._currentChannel, 0);

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);
#endif
}

static void test_processPacket_excess_channels(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    uint16_t counter = 0;

    // Fill all channels
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS - 1; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }
    counter += 1500;
    PPM_Status_t status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_SUCCESS);

    // Try to add more channels beyond the max
    counter += 1500;
    status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, PPM_MAX_CHANNELS + 1); // Continues incrementing

    // Continue adding excess channels
    counter += 1500;
    status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, PPM_MAX_CHANNELS + 2); // Continues incrementing

    // Send packet end marker before all channels filled
    counter += ppm._packetEndTicks + 100;

    status = PPM_processPacket(&ppm, counter);

    // Should detect end, reset and return OK
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, 0);

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);
#endif
}

static void test_processPacket_complete_frame(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

#if PPM_ENABLE_FRESHNESS_CHECK
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);
    mock_timestamp = 500;
#endif

    uint16_t counter = 0;
    uint16_t pulse_width = 1500;

    // Fill exactly PPM_MAX_CHANNELS
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS - 1; ii++) {
        counter += pulse_width;
        PPM_Status_t status = PPM_processPacket(&ppm, counter);
        assert_int_equal(status, PPM_WAIT);
    }

    // Last channel should complete the packet
    counter += pulse_width;
    PPM_Status_t status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_SUCCESS);
    assert_int_equal(ppm._currentChannel, PPM_MAX_CHANNELS);

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);
#endif

#if PPM_ENABLE_FRESHNESS_CHECK
    assert_int_equal(ppm._packet_time, 500);
#endif
}

static void test_processPacket_frequency_multiplier_effect(void** state) {
    (void)state;
    PPM_t ppm;

    // Test with 500kHz timer (multiplier = 2)
    PPM_init(&ppm, 500000U, TEST_TIMER_RELOAD_MAX);

    uint16_t counter = 1000;
    PPM_processPacket(&ppm, counter);

    // Channel value should be: 1000 ticks * 2 multiplier = 2000us
    assert_int_equal(ppm.channels[0], 2000);
}

static void test_processPacket_edge_cases_zero_pulse(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Send zero as first counter (edge case)
    PPM_Status_t status = PPM_processPacket(&ppm, 0);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm.channels[0], 0);
}

/* ============================================================================
 * STATISTICS TESTS
 * ============================================================================ */

#if PPM_ENABLE_STATS
static void test_stats_null_arguments(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    uint32_t packets;

    // Test NULL PPM
    PPM_getStats(NULL, &packets); // Should not crash

    // Test NULL packets pointer
    PPM_getStats(&ppm, NULL); // Should not crash
}

static void test_stats_packet_counting(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 0);

    // Complete one packet
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);

    counter += ppm._packetEndTicks + 10;
    PPM_processPacket(&ppm, counter);

    // Complete another packet
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 2);
}

static void test_stats_reset(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Complete some packets
    uint16_t counter = 0;
    for (int j = 0; j < 3; j++) {
        for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
            counter += 1500;
            PPM_processPacket(&ppm, counter);
        }
        // Send sync pulse (packet end marker)
        counter += ppm._packetEndTicks + 10;
        PPM_Status_t status = PPM_processPacket(&ppm, counter);
        assert_int_equal(status, PPM_WAIT);
        assert_int_equal(ppm._currentChannel, 0); // Reset for next frame
    }

    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 3);

    // Reset stats
    PPM_resetStats(&ppm);
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 0);

    // Test reset with NULL
    PPM_resetStats(NULL); // Should not crash
}

static void test_stats_packet_end_marker_counting(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Complete packet via packet end marker
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS / 2; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Send end marker
    counter += ppm._packetEndTicks + 100;
    PPM_processPacket(&ppm, counter);

    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 1);
}
#endif

/* ============================================================================
 * FRESHNESS CHECK TESTS
 * ============================================================================ */

#if PPM_ENABLE_FRESHNESS_CHECK
static void test_freshness_null_arguments(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test NULL PPM
    assert_false(PPM_isPacketFresh(NULL, 100));

    // Test NULL callback
    assert_false(PPM_isPacketFresh(&ppm, 100));
}

static void test_freshness_tracking(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete a packet
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Should be fresh immediately
    assert_true(PPM_isPacketFresh(&ppm, 0));
    assert_true(PPM_isPacketFresh(&ppm, 100));

    // Advance time
    mock_timestamp = 1050;
    assert_true(PPM_isPacketFresh(&ppm, 100)); // Still fresh (50ms old)
    assert_false(PPM_isPacketFresh(&ppm, 30)); // Too old

    // Advance time further
    mock_timestamp = 1200;
    assert_false(PPM_isPacketFresh(&ppm, 100)); // Now stale (200ms old)
}

static void test_freshness_multiple_packets(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete first packet
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Send sync pulse (packet end marker)
    counter += ppm._packetEndTicks + 10;
    PPM_Status_t status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_WAIT);
    assert_int_equal(ppm._currentChannel, 0); // Reset for next frame

    // Advance time
    mock_timestamp = 1100;

    // Complete second packet
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS - 1; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }
    // Send sync pulse (packet end marker)
    counter += ppm._packetEndTicks + 10;
    status = PPM_processPacket(&ppm, counter);
    assert_int_equal(status, PPM_SUCCESS);
    assert_int_equal(ppm._currentChannel, 0); // Reset for next frame

    mock_timestamp += 2;
    // Should use latest timestamp
    assert_true(PPM_isPacketFresh(&ppm, 10)); // Fresh (2ms old)
    assert_false(PPM_isPacketFresh(&ppm, 0)); // Not fresh at exactly 0ms tolerance
}

static void test_freshness_edge_case_max_age(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete packet
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Advance time to exactly max_age
    mock_timestamp = 1100;
    assert_true(PPM_isPacketFresh(&ppm, 100)); // Exactly at limit (100ms old)

    // Advance one more ms
    mock_timestamp = 1101;
    assert_false(PPM_isPacketFresh(&ppm, 100)); // Now too old (101ms old)
}

static void test_freshness_timestamp_wraparound(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);

    // Set timestamp near max
    mock_timestamp = UINT32_MAX - 50;

    // Complete packet
    uint16_t counter = 0;
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += 1500;
        PPM_processPacket(&ppm, counter);
    }

    // Wrap around
    mock_timestamp = 50; // Wrapped

    // Should handle wraparound correctly (100ms elapsed)
    assert_true(PPM_isPacketFresh(&ppm, 150));
    assert_false(PPM_isPacketFresh(&ppm, 50));
}
#endif

/* ============================================================================
 * INTEGRATION TESTS
 * ============================================================================ */

static void test_complete_ppm_frame_sequence(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

#if PPM_ENABLE_FRESHNESS_CHECK
    PPM_setTimestampCallback(&ppm, test_getTimestamp_ms);
    mock_timestamp = 0;
#endif

    // Simulate typical PPM frame: 8 channels, 1500us each, then sync pulse
    uint16_t counter = 0;
    uint16_t channel_values[PPM_MAX_CHANNELS] = {1000, 1200, 1500, 1800, 2000, 1500, 1300, 1100};

    for (int frame = 0; frame < 30; frame++) {
        // Send channel pulses
        for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
            counter += channel_values[ii];
            PPM_Status_t status = PPM_processPacket(&ppm, counter);

            if (ii < PPM_MAX_CHANNELS - 1) {
                assert_int_equal(status, PPM_WAIT);
            } else {
                assert_int_equal(status, PPM_SUCCESS);
            }
        }

        // Verify channel values
        for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
            assert_int_equal(ppm.channels[ii], channel_values[ii]);
        }

#if PPM_ENABLE_FRESHNESS_CHECK
        mock_timestamp += 20; // Advance 20ms per frame
        assert_true(PPM_isPacketFresh(&ppm, 50));
#endif

        // Send sync pulse (packet end marker)
        counter += ppm._packetEndTicks + 10;
        PPM_Status_t status = PPM_processPacket(&ppm, counter);
        assert_int_equal(status, PPM_WAIT);
        assert_int_equal(ppm._currentChannel, 0); // Reset for next frame
    }

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 30); // 30 frames
#endif
}

static void test_variable_pulse_widths(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test with varying pulse widths (typical RC range: 1000-2000us)
    uint16_t test_pulses[] = {1000, 1100, 1200, 1500, 1800, 1900, 2000, 1500};
    uint16_t counter = 0;

    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        counter += test_pulses[ii];
        PPM_processPacket(&ppm, counter);
        assert_int_equal(ppm.channels[ii], test_pulses[ii]);
    }

    // Verify channel values
    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        assert_int_equal(ppm.channels[ii], test_pulses[ii]);
    }
}

static void test_rapid_frame_processing(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Process 100 frames rapidly
    uint16_t counter = 0;
    for (int frame = 0; frame < 100; frame++) {
        for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
            counter += 1500;
            PPM_processPacket(&ppm, counter);
        }
        // Send sync pulse (packet end marker)
        counter += ppm._packetEndTicks + 10;
        PPM_Status_t status = PPM_processPacket(&ppm, counter);
        assert_int_equal(status, PPM_WAIT);
        assert_int_equal(ppm._currentChannel, 0); // Reset for next frame
    }

#if PPM_ENABLE_STATS
    uint32_t packets;
    PPM_getStats(&ppm, &packets);
    assert_int_equal(packets, 100);
#endif
}

static void test_timer_overflow_multiple_times(void** state) {
    (void)state;
    PPM_t ppm;
    PPM_init(&ppm, TEST_TIMER_FREQ_1MHZ, 1000);

    // Test multiple overflows in sequence
    uint16_t counters[] = {900, 100, 300, 700, 200, 500, 800, 50};

    for (uint8_t ii = 0; ii < PPM_MAX_CHANNELS; ii++) {
        PPM_processPacket(&ppm, counters[ii]);
        // Just verify it doesn't crash and completes
    }
}

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================ */

int main(void) {
    const struct CMUnitTest tests[] = {
        /* INITIALIZATION */
        cmocka_unit_test(test_init_null_pointer),
        cmocka_unit_test(test_init_null_frequency),
        cmocka_unit_test(test_init_defaults_and_clear),
        cmocka_unit_test(test_init_frequency_multiplier_calculation),
        cmocka_unit_test(test_init_packet_end_ticks_calculation),
#if PPM_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_timestamp_callback_setting),
#endif

        /* PACKET PROCESSING */
        cmocka_unit_test(test_processPacket_null_pointer),
        cmocka_unit_test(test_processPacket_normal_pulse),
        cmocka_unit_test(test_processPacket_timer_overflow),
        cmocka_unit_test(test_processPacket_packet_end_detection),
        cmocka_unit_test(test_processPacket_partial_packet_then_end),
        cmocka_unit_test(test_processPacket_excess_channels),
        cmocka_unit_test(test_processPacket_complete_frame),
        cmocka_unit_test(test_processPacket_frequency_multiplier_effect),
        cmocka_unit_test(test_processPacket_edge_cases_zero_pulse),

/* STATISTICS */
#if PPM_ENABLE_STATS
        cmocka_unit_test(test_stats_null_arguments),
        cmocka_unit_test(test_stats_packet_counting),
        cmocka_unit_test(test_stats_reset),
        cmocka_unit_test(test_stats_packet_end_marker_counting),
#endif

/* FRESHNESS CHECK */
#if PPM_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_freshness_null_arguments),
        cmocka_unit_test(test_freshness_tracking),
        cmocka_unit_test(test_freshness_multiple_packets),
        cmocka_unit_test(test_freshness_edge_case_max_age),
        cmocka_unit_test(test_freshness_timestamp_wraparound),
#endif

        /* INTEGRATION */
        cmocka_unit_test(test_complete_ppm_frame_sequence),
        cmocka_unit_test(test_variable_pulse_widths),
        cmocka_unit_test(test_rapid_frame_processing),
        cmocka_unit_test(test_timer_overflow_multiple_times),
    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}