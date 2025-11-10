/* BEGIN Header */
/**
 ******************************************************************************
 * \file            test_PWM.c
 * \author          Andrea Vivani
 * \brief           PWM protocol decoder test suite
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

#include "PWM.h"

/* Test Constants and Helpers */
#define TEST_TIMER_FREQ_1MHZ   1000000U
#define TEST_TIMER_FREQ_2MHZ   2000000U
#define TEST_TIMER_RELOAD_MAX  65535U
#define TEST_TIMER_RELOAD_1000 1000U

#if PWM_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 0;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

/* ============================================================================
 * INITIALIZATION TESTS
 * ============================================================================ */

static void test_init_null_pointer(void** state) {
    (void)state;
    PWM_Status_t status = PWM_init(NULL, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    assert_int_equal(status, PWM_ERROR_NULL_POINTER);
}

static void test_init_null_frequency(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_Status_t status = PWM_init(&pwm, 0, TEST_TIMER_RELOAD_MAX);
    assert_int_equal(status, PWM_ERROR);
}

static void test_init_defaults_and_clear(void** state) {
    (void)state;
    PWM_t pwm;

    // Initialize with some non-zero data to verify clearing
    memset(&pwm, 0xFF, sizeof(pwm));

    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Verify internal state is cleared
    assert_int_equal(pwm._timerAutoReload, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(pwm._freqMultiplier, 1, 1e-5); // 1000000 / 1000000 = 1
    assert_int_equal(pwm._updatedChannels, 0);

    // Verify channels are cleared
    for (uint8_t ii = 0; ii < PWM_MAX_CHANNELS; ii++) {
        assert_int_equal(pwm.channels[ii], 0);
        assert_int_equal(pwm._riseTimer[ii], 0);
        assert_int_equal(pwm._fallTimer[ii], 0);
        assert_int_equal(pwm._pulseCounter[ii], 0);
    }

#if PWM_ENABLE_STATS
    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 0);
#endif
}

static void test_init_frequency_multiplier_calculation(void** state) {
    (void)state;
    PWM_t pwm;

    // Test 1MHz timer (most common)
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(pwm._freqMultiplier, 1, 1e-5);

    // Test 2MHz timer
    PWM_init(&pwm, TEST_TIMER_FREQ_2MHZ, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(pwm._freqMultiplier, 0.5, 1e-5);

    // Test 500kHz timer
    PWM_init(&pwm, 500000U, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(pwm._freqMultiplier, 2, 1e-5);

    // Test 100kHz timer
    PWM_init(&pwm, 100000U, TEST_TIMER_RELOAD_MAX);
    assert_float_equal(pwm._freqMultiplier, 10, 1e-5);
}

#if PWM_ENABLE_FRESHNESS_CHECK
static void test_timestamp_callback_setting(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test setting callback
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);

    // Test with NULL state
    PWM_setTimestampCallback(NULL, test_getTimestamp_ms); // Should not crash
}
#endif

/* ============================================================================
 * PACKET PROCESSING TESTS
 * ============================================================================ */

static void test_processPacket_null_pointer(void** state) {
    (void)state;
    PWM_Status_t status = PWM_processPacket(NULL, 0, 1, 100);
    assert_int_equal(status, PWM_ERROR_NULL_POINTER);
}

static void test_processPacket_single_channel_rising_edges(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    uint8_t channel = 0;

    // Send rising edges
    for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
        PWM_Status_t status = PWM_processPacket(&pwm, channel, 1, 1000);
        assert_int_equal(status, PWM_WAIT);
        assert_int_equal(pwm._riseTimer[channel], 1000 * (ii + 1));
        assert_int_equal(pwm._pulseCounter[channel], ii + 1);
    }
}

static void test_processPacket_single_channel_falling_edges(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    uint8_t channel = 0;

    // Send falling edges
    for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
        PWM_Status_t status = PWM_processPacket(&pwm, channel, 0, 1500);
        assert_int_equal(status, PWM_WAIT);
        assert_int_equal(pwm._fallTimer[channel], 1500 * (ii + 1));
        assert_int_equal(pwm._pulseCounter[channel], ii + 1);
    }
}

static void test_processPacket_complete_single_channel_no_overflow(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

#if PWM_ENABLE_FRESHNESS_CHECK
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);
    mock_timestamp = 1000;
#endif

    uint8_t channel = 0;

    // Send alternating rising and falling edges
    for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
        PWM_processPacket(&pwm, channel, 1, 1000); // Rising
        PWM_processPacket(&pwm, channel, 0, 2500); // Falling (1500us pulse width)
    }

    // Channel should be updated with pulse width: (2500*SAMPLES - 1000*SAMPLES) / SAMPLES = 1500us
    assert_int_equal(pwm.channels[channel], 1500);
    assert_int_equal(pwm._riseTimer[channel], 0);
    assert_int_equal(pwm._fallTimer[channel], 0);
    assert_int_equal(pwm._pulseCounter[channel], 0);
}

static void test_processPacket_complete_single_channel_with_overflow(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_1000);

    uint8_t channel = 0;
    uint16_t counter = 950;

    // Send alternating rising and falling edges with timer overflow
    // Rise at 950, Fall at 100 (wrapped around) = pulse width ~151
    for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
        PWM_processPacket(&pwm, channel, 1, counter); // Rising
        counter += 100;
        if (counter > TEST_TIMER_RELOAD_1000) {
            counter = counter - (TEST_TIMER_RELOAD_1000 + 1);
        }
        PWM_processPacket(&pwm, channel, 0, counter); // Falling (overflow)
        counter += 100;
        if (counter > TEST_TIMER_RELOAD_1000) {
            counter = counter - (TEST_TIMER_RELOAD_1000 + 1);
        }
    }

    // Expected: 100us
    assert_int_equal(pwm.channels[channel], 100);
}

static void test_processPacket_multiple_channels_complete_frame(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

#if PWM_ENABLE_FRESHNESS_CHECK
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);
    mock_timestamp = 500;
#endif

    uint16_t pulse_widths[PWM_MAX_CHANNELS] = {1000, 1200, 1500, 1800};

    // Process all channels
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_Status_t status = PWM_processPacket(&pwm, ch, 0, 1000 + pulse_widths[ch]);

            if (ch == PWM_MAX_CHANNELS - 1 && ii == PWM_SAMPLES_NUM - 1) {
                assert_int_equal(status, PWM_OK);
            } else {
                assert_int_equal(status, PWM_WAIT);
            }
        }
        assert_int_equal(pwm.channels[ch], pulse_widths[ch]);
    }

#if PWM_ENABLE_STATS
    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 1);
#endif

#if PWM_ENABLE_FRESHNESS_CHECK
    assert_int_equal(pwm._packet_time, 500);
#endif
}

static void test_processPacket_partial_channels_updated(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Complete only first 4 channels
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS / 2; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_Status_t status = PWM_processPacket(&pwm, ch, 0, 2500);
            assert_int_equal(status, PWM_WAIT);
        }
    }

    // Verify only completed channels have updated flag set
    uint8_t expected_mask = (1 << (PWM_MAX_CHANNELS / 2)) - 1;
    assert_int_equal(pwm._updatedChannels, expected_mask);
}

static void test_processPacket_frequency_multiplier_effect(void** state) {
    (void)state;
    PWM_t pwm;

    // Test with 500kHz timer (multiplier = 2)
    PWM_init(&pwm, 500000U, TEST_TIMER_RELOAD_MAX);

    uint8_t channel = 0;

    // Send samples with 750 ticks pulse width
    for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
        PWM_processPacket(&pwm, channel, 1, 1000);
        PWM_processPacket(&pwm, channel, 0, 1750);
    }

    // Channel value should be: 750 ticks * 2 multiplier = 1500us
    assert_int_equal(pwm.channels[channel], 1500);
}

/* ============================================================================
 * STATISTICS TESTS
 * ============================================================================ */

#if PWM_ENABLE_STATS
static void test_stats_null_arguments(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    uint32_t packets;

    // Test NULL PWM
    PWM_getStats(NULL, &packets); // Should not crash

    // Test NULL packets pointer
    PWM_getStats(&pwm, NULL); // Should not crash
}

static void test_stats_packet_counting(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 0);

    // Complete one full frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 1);

    // Complete another frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 2);
}

static void test_stats_reset(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Complete some frames
    for (int frame = 0; frame < 3; frame++) {
        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
            for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
                PWM_processPacket(&pwm, ch, 1, 1000);
                PWM_processPacket(&pwm, ch, 0, 2500);
            }
        }
    }

    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 3);

    // Reset stats
    PWM_resetStats(&pwm);
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 0);

    // Test reset with NULL
    PWM_resetStats(NULL); // Should not crash
}
#endif

/* ============================================================================
 * FRESHNESS CHECK TESTS
 * ============================================================================ */

#if PWM_ENABLE_FRESHNESS_CHECK
static void test_freshness_null_arguments(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test NULL PWM
    assert_false(PWM_isPacketFresh(NULL, 100));

    // Test NULL callback
    assert_false(PWM_isPacketFresh(&pwm, 100));
}

static void test_freshness_tracking(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete a frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    // Should be fresh immediately
    assert_true(PWM_isPacketFresh(&pwm, 0));
    assert_true(PWM_isPacketFresh(&pwm, 100));

    // Advance time
    mock_timestamp = 1050;
    assert_true(PWM_isPacketFresh(&pwm, 100)); // Still fresh (50ms old)
    assert_false(PWM_isPacketFresh(&pwm, 30)); // Too old

    // Advance time further
    mock_timestamp = 1200;
    assert_false(PWM_isPacketFresh(&pwm, 100)); // Now stale (200ms old)
}

static void test_freshness_multiple_packets(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete first frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    // Advance time
    mock_timestamp = 1100;

    // Complete second frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    mock_timestamp += 5;
    // Should use latest timestamp
    assert_true(PWM_isPacketFresh(&pwm, 10)); // Fresh (5ms old)
    assert_false(PWM_isPacketFresh(&pwm, 0)); // Not fresh
}

static void test_freshness_edge_case_max_age(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);

    mock_timestamp = 1000;

    // Complete frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    // Advance time to exactly max_age
    mock_timestamp = 1100;
    assert_true(PWM_isPacketFresh(&pwm, 100)); // Exactly at limit (100ms old)

    // Advance one more ms
    mock_timestamp = 1101;
    assert_false(PWM_isPacketFresh(&pwm, 100)); // Now too old (101ms old)
}

static void test_freshness_timestamp_wraparound(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);

    // Set timestamp near max
    mock_timestamp = UINT32_MAX - 50;

    // Complete frame
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 2500);
        }
    }

    // Wrap around
    mock_timestamp = 50; // Wrapped

    // Should handle wraparound correctly (100ms elapsed)
    assert_true(PWM_isPacketFresh(&pwm, 150));
    assert_false(PWM_isPacketFresh(&pwm, 50));
}
#endif

/* ============================================================================
 * INTEGRATION TESTS
 * ============================================================================ */

static void test_complete_pwm_frame_sequence(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

#if PWM_ENABLE_FRESHNESS_CHECK
    PWM_setTimestampCallback(&pwm, test_getTimestamp_ms);
    mock_timestamp = 0;
#endif

    uint16_t channel_values[PWM_MAX_CHANNELS] = {1000, 1200, 1500, 1800};

    for (int frame = 0; frame < 3; frame++) {
        // Process all channels
        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
            for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
                PWM_processPacket(&pwm, ch, 1, 1000);
                PWM_Status_t status = PWM_processPacket(&pwm, ch, 0, 1000 + channel_values[ch]);

                if (ch == PWM_MAX_CHANNELS - 1 && ii == PWM_SAMPLES_NUM - 1) {
                    assert_int_equal(status, PWM_OK);
                } else {
                    assert_int_equal(status, PWM_WAIT);
                }
            }
        }

        // Verify channel values
        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
            assert_int_equal(pwm.channels[ch], channel_values[ch]);
        }

#if PWM_ENABLE_FRESHNESS_CHECK
        mock_timestamp += 20; // Advance 20ms per frame
        if (frame < 2) {      // Only check freshness for first 2 frames
            assert_true(PWM_isPacketFresh(&pwm, 50));
        }
#endif
    }

#if PWM_ENABLE_STATS
    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 3);
#endif
}

static void test_variable_pulse_widths(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Test with varying pulse widths (typical RC range: 1000-2000us)
    uint16_t test_pulses[] = {1000, 1100, 1200, 1500, 1800, 1900, 2000, 1500};

    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, 1000);
            PWM_processPacket(&pwm, ch, 0, 1000 + test_pulses[ch]);
        }
        assert_int_equal(pwm.channels[ch], test_pulses[ch]);
    }
}

static void test_rapid_frame_processing(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, TEST_TIMER_RELOAD_MAX);

    // Process 100 frames rapidly
    for (int frame = 0; frame < 100; frame++) {
        for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
            for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
                PWM_processPacket(&pwm, ch, 1, 1000);
                PWM_processPacket(&pwm, ch, 0, 2500);
            }
        }
    }

#if PWM_ENABLE_STATS
    uint32_t packets;
    PWM_getStats(&pwm, &packets);
    assert_int_equal(packets, 100);
#endif
}

static void test_timer_overflow_multiple_channels(void** state) {
    (void)state;
    PWM_t pwm;
    PWM_init(&pwm, TEST_TIMER_FREQ_1MHZ, 1000);
    uint16_t counter = 950;

    // Test overflow on multiple channels
    for (uint8_t ch = 0; ch < PWM_MAX_CHANNELS; ch++) {
        for (uint8_t ii = 0; ii < PWM_SAMPLES_NUM; ii++) {
            PWM_processPacket(&pwm, ch, 1, counter); // Rising near max
            counter += 100;
            if (counter > TEST_TIMER_RELOAD_1000) {
                counter = counter - (TEST_TIMER_RELOAD_1000 + 1);
            }
            PWM_processPacket(&pwm, ch, 0, counter); // Falling after overflow
            counter += 100;
            if (counter > TEST_TIMER_RELOAD_1000) {
                counter = counter - (TEST_TIMER_RELOAD_1000 + 1);
            }
        }
        // Just verify it doesn't crash and completes
        assert_int_equal(pwm.channels[ch], 100);
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
#if PWM_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_timestamp_callback_setting),
#endif

        /* PACKET PROCESSING */
        cmocka_unit_test(test_processPacket_null_pointer),
        cmocka_unit_test(test_processPacket_single_channel_rising_edges),
        cmocka_unit_test(test_processPacket_single_channel_falling_edges),
        cmocka_unit_test(test_processPacket_complete_single_channel_no_overflow),
        cmocka_unit_test(test_processPacket_complete_single_channel_with_overflow),
        cmocka_unit_test(test_processPacket_multiple_channels_complete_frame),
        cmocka_unit_test(test_processPacket_partial_channels_updated),
        cmocka_unit_test(test_processPacket_frequency_multiplier_effect),

/* STATISTICS */
#if PWM_ENABLE_STATS
        cmocka_unit_test(test_stats_null_arguments),
        cmocka_unit_test(test_stats_packet_counting),
        cmocka_unit_test(test_stats_reset),
#endif

/* FRESHNESS CHECK */
#if PWM_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_freshness_null_arguments),
        cmocka_unit_test(test_freshness_tracking),
        cmocka_unit_test(test_freshness_multiple_packets),
        cmocka_unit_test(test_freshness_edge_case_max_age),
        cmocka_unit_test(test_freshness_timestamp_wraparound),
#endif

        /* INTEGRATION */
        cmocka_unit_test(test_complete_pwm_frame_sequence),
        cmocka_unit_test(test_variable_pulse_widths),
        cmocka_unit_test(test_rapid_frame_processing),
        cmocka_unit_test(test_timer_overflow_multiple_channels),
    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}