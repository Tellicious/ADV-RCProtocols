/* BEGIN Header */
/**
 ******************************************************************************
 * \file            test_SymaX.c
 * \author          Andrea Vivani
 * \brief           SymaX protocol decoder/encoder test suite
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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmocka.h>

#include "SymaX.h"

/* Test Constants and Helpers */
#define SYMAX_PREBIND_PACKET_EXPECTED                                                                                                                          \
    { 0xF9, 0x96, 0x82, 0x1B, 0x20, 0x08, 0x08, 0xF2, 0x7D, 0xEF }

#if SYMAX_ENABLE_FRESHNESS_CHECK
static uint32_t mock_timestamp = 12345;

static uint32_t test_getTimestamp_ms(void) { return mock_timestamp; }
#endif

/* Helper: calculate checksum matching SymaX_calcChecksum */
static uint8_t test_calc_checksum(const uint8_t* data) {
    uint8_t sum = data[0];
    for (uint8_t i = 1; i < SYMAX_PACKET_SIZE - 1; ++i) {
        sum ^= data[i];
    }
    return sum + 0x55;
}

#ifdef SYMAX_CONFIG_RX
/* Helper: create valid bind packet */
static void create_bind_packet(uint8_t* packet, const uint8_t* address) {
    packet[0] = address[4]; // Reversed order
    packet[1] = address[3];
    packet[2] = address[2];
    packet[3] = address[1];
    packet[4] = address[0];
    packet[5] = 0xAA;
    packet[6] = 0xAA;
    packet[7] = 0xAA;
    packet[8] = 0x00;
    packet[9] = test_calc_checksum(packet);
}

/* Helper: create valid data packet */
static void create_data_packet(uint8_t* packet, int8_t thr, int8_t ele, int8_t rud, int8_t ail) {
    packet[0] = thr;
    packet[1] = (ele < 0) ? 0x80 | (-ele) : ele;
    packet[2] = (rud < 0) ? 0x80 | (-rud) : rud;
    packet[3] = (ail < 0) ? 0x80 | (-ail) : ail;
    packet[4] = 0x00; // flags
    packet[5] = 0xC0; // rate control
    packet[6] = 0x00; // more flags
    packet[7] = 0x00; // more flags
    packet[8] = 0x00;
    packet[9] = test_calc_checksum(packet);
}
#endif

/* ============================================================================
 * INITIALIZATION AND CONFIGURATION TESTS
 * ============================================================================ */

static void test_init_null_pointer(void** state) {
    (void)state;
    SymaX_init(NULL); // Should not crash
}

static void test_init_defaults_and_clear(void** state) {
    (void)state;
    SymaX_t st;

    // Initialize with some non-zero data to verify clearing
    memset(&st, 0xFF, sizeof(st));

    SymaX_init(&st);

    // Verify phase initialization
#if SYMAX_SKIP_PREBIND
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
#else
    assert_int_equal(st.link.phase, SYMAX_PREBIND);
#endif

    // Verify defaults
    assert_int_equal(st.link.current_channel_idx, 0);
    assert_int_equal(st.link.packet_count, 0);

    // Verify bind address is set
    uint8_t addr[5];
    SymaX_getAddress(&st, addr);
    assert_int_equal(addr[0], 0xAB);
    assert_int_equal(addr[1], 0xAC);
    assert_int_equal(addr[2], 0xAD);
    assert_int_equal(addr[3], 0xAE);
    assert_int_equal(addr[4], 0xAF);

    // Verify bind channels are set
    uint8_t expected_channels[] = {0x4B, 0x30, 0x40, 0x20};
    for (int i = 0; i < 4; i++) {
        assert_int_equal(st.link.rf_channels[i], expected_channels[i]);
    }

    // Verify channel data is cleared
    assert_int_equal(st.channelsData.thr, 0);
    assert_int_equal(st.channelsData.ele, 0);
    assert_int_equal(st.channelsData.rud, 0);
    assert_int_equal(st.channelsData.ail, 0);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.packets_processed, 0);
    assert_int_equal(stats.bind_packets, 0);
    assert_int_equal(stats.data_packets, 0);
    assert_int_equal(stats.checksum_failures, 0);
#endif
}

#if SYMAX_ENABLE_FRESHNESS_CHECK
static void test_timestamp_callback_setting(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test setting callback
    SymaX_setTimestampCallback(&st, test_getTimestamp_ms);

    // Test with NULL state
    SymaX_setTimestampCallback(NULL, test_getTimestamp_ms); // Should not crash
}
#endif

/* ============================================================================
 * GETTER/SETTER API TESTS
 * ============================================================================ */

static void test_getCurrentChannel(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test valid state
    uint8_t channel = SymaX_getCurrentChannel(&st);
#if SYMAX_SKIP_PREBIND
    assert_int_equal(channel, st.link.rf_channels[0]); // Should match first channel
#else
    assert_int_equal(channel, 0x8); // Should return 0x8 in prebind mode
#endif

    // Test different channel indices
    st.link.current_channel_idx = 1;
    channel = SymaX_getCurrentChannel(&st);
    if (st.link.phase != SYMAX_PREBIND) {
        assert_int_equal(channel, st.link.rf_channels[1]);
    }

    st.link.current_channel_idx = 2;
    channel = SymaX_getCurrentChannel(&st);
    if (st.link.phase != SYMAX_PREBIND) {
        assert_int_equal(channel, st.link.rf_channels[2]);
    }

    st.link.current_channel_idx = 3;
    channel = SymaX_getCurrentChannel(&st);
    if (st.link.phase != SYMAX_PREBIND) {
        assert_int_equal(channel, st.link.rf_channels[3]);
    }
}

static void test_startBinding(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

#if SYMAX_SKIP_PREBIND
    SymaX_startBinding(&st);
    assert_true(st.link.phase == SYMAX_BIND_IN_PROGRESS);
#else
    SymaX_startBinding(&st);
    assert_true(st.link.phase == SYMAX_PREBIND);
#endif
}

static void test_isBound(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Initially not bound
    assert_false(SymaX_isBound(&st));

    // Test all phases
    st.link.phase = SYMAX_PREBIND;
    assert_false(SymaX_isBound(&st));

    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    assert_false(SymaX_isBound(&st));

    st.link.phase = SYMAX_WAIT_FIRST_PACKET;
    assert_true(SymaX_isBound(&st));

    st.link.phase = SYMAX_DATA;
    assert_true(SymaX_isBound(&st));

    // Test NULL state
    assert_false(SymaX_isBound(NULL));
}

static void test_getAddress_macro(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    uint8_t addr[5];
    SymaX_getAddress(&st, addr);

    // Verify it copies the address correctly
    assert_memory_equal(addr, st.link.rf_address, 5);

    // Test with modified address
    st.link.rf_address[0] = 0x12;
    st.link.rf_address[1] = 0x34;
    st.link.rf_address[2] = 0x56;
    st.link.rf_address[3] = 0x78;
    st.link.rf_address[4] = 0x9A;

    SymaX_getAddress(&st, addr);
    assert_int_equal(addr[0], 0x12);
    assert_int_equal(addr[1], 0x34);
    assert_int_equal(addr[2], 0x56);
    assert_int_equal(addr[3], 0x78);
    assert_int_equal(addr[4], 0x9A);
}

#if SYMAX_ENABLE_STATS
static void test_stats_getter(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.packets_processed, 0);
    assert_int_equal(stats.bind_packets, 0);
    assert_int_equal(stats.data_packets, 0);
    assert_int_equal(stats.checksum_failures, 0);

    // Test NULL arguments - should not crash
    SymaX_getStats(NULL, &stats);
    SymaX_getStats(&st, NULL);
}
#endif

/* ============================================================================
 * RF CHANNEL CALCULATION TESTS (SymaX_calcRCChannels coverage)
 * ============================================================================ */

#ifdef SYMAX_CONFIG_TX
static void test_calc_rc_channels_range_0_to_0x0F(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test various addresses in range [0, 0x0F] (uses startChans_1)
    for (uint8_t addr = 0; addr <= 0x0F; addr++) {
        if (addr == 6) {
            continue; // Skip special case, tested separately
        }

        st.link.rf_address[0] = addr;
        st.link.phase = SYMAX_BIND_IN_PROGRESS;
        st.link.packet_count = SYMAX_BIND_PACKETS;

        uint8_t packet[SYMAX_PACKET_SIZE];
        SymaX_buildPacket(&st, packet);

        uint8_t expected_base[] = {0x0a, 0x1A, 0x2a, 0x3A};
        for (int i = 0; i < 4; i++) {
            assert_int_equal(st.link.rf_channels[i], expected_base[i] + addr);
        }
    }
}

static void test_calc_rc_channels_special_case_address_6(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test address == 6 special case (becomes 7)
    st.link.rf_address[0] = 0x06;
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    st.link.packet_count = SYMAX_BIND_PACKETS;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Should use 7 instead of 6
    uint8_t expected_base[] = {0x0a, 0x1A, 0x2a, 0x3A};
    for (int i = 0; i < 4; i++) {
        assert_int_equal(st.link.rf_channels[i], expected_base[i] + 7);
    }
}

static void test_calc_rc_channels_range_0x10_to_0x17(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test all addresses in range [0x10, 0x17] except 0x16 (uses startChans_2)
    for (uint8_t addr = 0x10; addr <= 0x17; addr++) {
        if (addr == 0x16) {
            continue; // Skip special case
        }

        st.link.rf_address[0] = addr;
        st.link.phase = SYMAX_BIND_IN_PROGRESS;
        st.link.packet_count = SYMAX_BIND_PACKETS;

        uint8_t packet[SYMAX_PACKET_SIZE];
        SymaX_buildPacket(&st, packet);

        uint8_t expected_base[] = {0x2a, 0x0A, 0x42, 0x22};
        for (int i = 0; i < 4; i++) {
            assert_int_equal(st.link.rf_channels[i], expected_base[i] + (addr & 0x07));
        }
    }
}

static void test_calc_rc_channels_special_case_address_0x16(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test address == 0x16 special case
    st.link.rf_address[0] = 0x16;
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    st.link.packet_count = SYMAX_BIND_PACKETS;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    uint8_t expected_base[] = {0x2a, 0x0A, 0x42, 0x22};
    uint8_t addr_offset = 0x16 & 0x07;

    // First two channels get +1 additional
    assert_int_equal(st.link.rf_channels[0], expected_base[0] + addr_offset + 1);
    assert_int_equal(st.link.rf_channels[1], expected_base[1] + addr_offset + 1);
    assert_int_equal(st.link.rf_channels[2], expected_base[2] + addr_offset);
    assert_int_equal(st.link.rf_channels[3], expected_base[3] + addr_offset);
}

static void test_calc_rc_channels_range_0x18_to_0x1D(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test all addresses in range [0x18, 0x1D] (uses startChans_3)
    for (uint8_t addr = 0x18; addr <= 0x1D; addr++) {
        st.link.rf_address[0] = addr;
        st.link.phase = SYMAX_BIND_IN_PROGRESS;
        st.link.packet_count = SYMAX_BIND_PACKETS;

        uint8_t packet[SYMAX_PACKET_SIZE];
        SymaX_buildPacket(&st, packet);

        uint8_t expected_base[] = {0x1a, 0x3A, 0x12, 0x32};
        for (int i = 0; i < 4; i++) {
            assert_int_equal(st.link.rf_channels[i], expected_base[i] + (addr & 0x07));
        }
    }
}

static void test_calc_rc_channels_special_case_address_0x1E(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test address == 0x1e special case
    st.link.rf_address[0] = 0x1E;
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    st.link.packet_count = SYMAX_BIND_PACKETS;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Should use fixed pattern 0x38184121
    assert_int_equal(st.link.rf_channels[0], 0x21);
    assert_int_equal(st.link.rf_channels[1], 0x41);
    assert_int_equal(st.link.rf_channels[2], 0x18);
    assert_int_equal(st.link.rf_channels[3], 0x38);
}

static void test_calc_rc_channels_special_case_address_0x1F(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test address == 0x1f special case
    st.link.rf_address[0] = 0x1F;
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    st.link.packet_count = SYMAX_BIND_PACKETS;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Should use fixed pattern 0x39194121
    assert_int_equal(st.link.rf_channels[0], 0x21);
    assert_int_equal(st.link.rf_channels[1], 0x41);
    assert_int_equal(st.link.rf_channels[2], 0x19);
    assert_int_equal(st.link.rf_channels[3], 0x39);
}

/* ============================================================================
 * TX PACKET BUILDING TESTS
 * ============================================================================ */

static void test_buildPacket_null_arguments(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    uint8_t packet[SYMAX_PACKET_SIZE];

    // Test NULL state
    assert_int_equal(SymaX_buildPacket(NULL, packet), SYMAX_ERROR_NULL_POINTER);

    // Test NULL packet
    assert_int_equal(SymaX_buildPacket(&st, NULL), SYMAX_ERROR_NULL_POINTER);
}

#if !SYMAX_SKIP_PREBIND
static void test_buildPacket_prebind_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    uint8_t packet[SYMAX_PACKET_SIZE];
    uint8_t expected[] = SYMAX_PREBIND_PACKET_EXPECTED;

    // Build prebind packet
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);

    // Verify packet content
    assert_memory_equal(packet, expected, SYMAX_PACKET_SIZE);

    // Verify channel not hopping during PREBIND phase
    uint8_t initial_channel = SymaX_getCurrentChannel(&st);

    for (int i = 0; i < SYMAX_PREBIND_PACKETS - 2; i++) {
        SymaX_buildPacket(&st, packet);
        assert_int_equal(SymaX_getCurrentChannel(&st), initial_channel); // Should stay same
    }
}

static void test_buildPacket_prebind_to_bind_transition(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Send SYMAX_PREBIND_PACKETS - 1
    for (int i = 0; i < SYMAX_PREBIND_PACKETS - 1; i++) {
        assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);
        assert_int_equal(st.link.phase, SYMAX_PREBIND);
    }

    // Next packet should transition to BIND_IN_PROGRESS
    assert_int_equal(SymaX_getCurrentChannel(&st), 8);
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
    assert_int_equal(st.link.packet_count, 0);        // Reset counter
    assert_int_equal(st.link.current_channel_idx, 0); // Reset channel
    assert_int_equal(SymaX_getCurrentChannel(&st), st.link.rf_channels[0]);
}
#endif

static void test_buildPacket_bind_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

#if SYMAX_SKIP_PREBIND
    // Already in BIND_IN_PROGRESS
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
#else
    // Force to bind phase
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
#endif

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Build bind packet
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);

    // Verify bind packet structure
    assert_int_equal(packet[5], 0xAA);
    assert_int_equal(packet[6], 0xAA);
    assert_int_equal(packet[7], 0xAA);
    assert_int_equal(packet[8], 0x00);
    assert_int_equal(packet[9], test_calc_checksum(packet));

    // Verify address is not default bind address (should be generated)
    uint8_t addr[5];
    SymaX_getAddress(&st, addr);
    assert_int_not_equal(addr[0], 0xAB); // Should have changed from default
    assert_int_equal(addr[4], 0xA2);     // Should always be 0xa2
}

static void test_buildPacket_bind_with_non_default_address(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Set non-default address to test the condition branch
    st.link.rf_address[0] = 0x12;
    st.link.rf_address[1] = 0x34;
    st.link.rf_address[2] = 0x56;
    st.link.rf_address[3] = 0x78;
    st.link.rf_address[4] = 0x9A;
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Address should remain unchanged (not regenerated)
    assert_int_equal(st.link.rf_address[0], 0x12);
    assert_int_equal(st.link.rf_address[1], 0x34);
    assert_int_equal(st.link.rf_address[2], 0x56);
    assert_int_equal(st.link.rf_address[3], 0x78);
    assert_int_equal(st.link.rf_address[4], 0x9A);

    // Packet should contain the reversed address
    assert_int_equal(packet[0], 0x9A);
    assert_int_equal(packet[1], 0x78);
    assert_int_equal(packet[2], 0x56);
    assert_int_equal(packet[3], 0x34);
    assert_int_equal(packet[4], 0x12);
}

static void test_buildPacket_bind_to_data_transition(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Send exactly SYMAX_BIND_PACKETS
    for (int i = 0; i < SYMAX_BIND_PACKETS - 1; i++) {
        assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);
        assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
    }

    // Next packet should transition to DATA
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_BIND_COMPLETE);
    assert_int_equal(st.link.phase, SYMAX_DATA);
    assert_int_equal(st.link.packet_count, 0);        // Reset counter
    assert_int_equal(st.link.current_channel_idx, 0); // Reset channel
}

static void test_buildPacket_data_phase_basic(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Set some test RC data
    st.channelsData.thr = 50;
    st.channelsData.ele = -30;
    st.channelsData.rud = 70;
    st.channelsData.ail = -90;

    uint8_t packet[SYMAX_PACKET_SIZE] = {0};
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_SUCCESS);

    // Verify basic structure
    assert_int_equal(packet[5], 0xC0); // Rate control always high
    assert_int_equal(packet[8], 0x00); // Reserved byte
    assert_int_equal(packet[9], test_calc_checksum(packet));

    // Verify throttle encoding
    assert_int_equal(packet[0], 50); // Positive throttle

    // Verify other channels (normal encoding)
    assert_int_equal(packet[1], 0x80 | 30); // Negative ele -> sign bit + abs value
    assert_int_equal(packet[2], 70);        // Positive rud -> just value
    assert_int_equal(packet[3], 0x80 | 90); // Negative ail -> sign bit + abs value
}

static void test_buildPacket_channel_conversion(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Test all edge cases for SymaX_convertRCChannels
    struct {
        int8_t input;
        uint8_t expected;
    } test_cases[] = {
        {0, 0x00},                           // Zero
        {1, 0x01},                           // Positive small
        {127, 0x7F},                         // Positive max
        {-1, 0x80 | 0x01},                   // Negative small
        {-127, 0x80 | 0x7F},                 // Negative max
        {SYMAX_CHANNEL_MIN - 1, 0x80 | 127}, // Under min (clamped)
        {SYMAX_CHANNEL_MAX, 0x7F},           // At max
        {SYMAX_CHANNEL_MIN, 0x80 | 127},     // At min
    };

    for (size_t i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
        uint8_t packet[SYMAX_PACKET_SIZE];

        // Test each channel
        st.channelsData.ele = test_cases[i].input;
        st.channelsData.rud = 0;
        st.channelsData.ail = 0;

        SymaX_buildPacket(&st, packet);
        assert_int_equal(packet[1], test_cases[i].expected);

        st.channelsData.ele = 0;
        st.channelsData.rud = test_cases[i].input;
        st.channelsData.ail = 0;

        SymaX_buildPacket(&st, packet);
        assert_int_equal(packet[2], test_cases[i].expected);

        st.channelsData.ele = 0;
        st.channelsData.rud = 0;
        st.channelsData.ail = test_cases[i].input;

        SymaX_buildPacket(&st, packet);
        assert_int_equal(packet[3], test_cases[i].expected);
    }
}

#if SYMAX_ENABLE_FLAGS
static void test_buildPacket_flags_encoding(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Set all flags
    st.flags.video = 1;
    st.flags.picture = 1;
    st.flags.flip = 1;
    st.flags.headless = 0; // Not headless so xtrm_rates can be set
    st.flags.xtrm_rates = 1;

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Verify flag encoding in correct bytes
    assert_int_equal(packet[4] & 0x80, 0x80); // video flag
    assert_int_equal(packet[4] & 0x40, 0x40); // picture flag
    assert_int_equal(packet[6] & 0x40, 0x40); // flip flag
    assert_int_equal(packet[7] & 0x80, 0x00); // headless flag off

    // Test xtrm_rates gets disabled when headless is enabled
    st.flags.headless = 1;
    SymaX_buildPacket(&st, packet);
    assert_int_equal(st.flags.xtrm_rates, 0); // Should be automatically disabled
    assert_int_equal(packet[7] & 0x80, 0x80); // headless flag on

    // Test no flags set
    st.flags.video = 0;
    st.flags.picture = 0;
    st.flags.flip = 0;
    st.flags.headless = 0;
    st.flags.xtrm_rates = 0;
    SymaX_buildPacket(&st, packet);

    assert_int_equal(packet[4] & 0x80, 0x00);
    assert_int_equal(packet[4] & 0x40, 0x00);
    assert_int_equal(packet[6] & 0x40, 0x00);
    assert_int_equal(packet[7] & 0x80, 0x00);
}

#if SYMAX_ENABLE_TRIM_DATA
static void test_buildPacket_trim_data(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Test with xtrm_rates enabled
    st.flags.xtrm_rates = 1;
    st.flags.headless = 0; // Must be off for xtrm_rates

    // Set channel values that will need trim data
    st.channelsData.thr = 0;    // Throttle doesn't use trim
    st.channelsData.ele = 100;  // Will need trim extension
    st.channelsData.rud = -100; // Will need trim extension
    st.channelsData.ail = 80;   // Will need trim extension

    uint8_t packet[SYMAX_PACKET_SIZE];
    SymaX_buildPacket(&st, packet);

    // Verify trim data is encoded in bytes 5-7
    assert_true((packet[5] & 0x3F) != 0); // Elevator trim
    assert_true((packet[6] & 0x3F) != 0); // Rudder trim
    assert_true((packet[7] & 0x3F) != 0); // Aileron trim

    // Test with xtrm_rates disabled
    st.flags.xtrm_rates = 0;
    SymaX_buildPacket(&st, packet);

    // Verify no trim data is encoded
    assert_int_equal(packet[5] & 0x3F, 0); // No elevator trim
    assert_int_equal(packet[6] & 0x3F, 0); // No rudder trim
    assert_int_equal(packet[7] & 0x3F, 0); // No aileron trim

    // Test trim data is not encoded when headless mode is active
    st.flags.xtrm_rates = 1;
    st.flags.headless = 1;
    SymaX_buildPacket(&st, packet);

    // Verify no trim data due to headless mode
    assert_int_equal(packet[5] & 0x3F, 0);
    assert_int_equal(packet[6] & 0x3F, 0);
    assert_int_equal(packet[7] & 0x3F, 0);
}
#endif
#endif

static void test_buildPacket_frequency_hopping(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    uint8_t packet[SYMAX_PACKET_SIZE];
    uint8_t initial_channel = SymaX_getCurrentChannel(&st);

    // Send packets and verify hopping
    for (int i = 0; i < SYMAX_HOP_DATA_RATE - 1; i++) {
        SymaX_buildPacket(&st, packet);
        assert_int_equal(SymaX_getCurrentChannel(&st), initial_channel); // Should stay same
    }

    // Next packet should hop
    SymaX_buildPacket(&st, packet);
    assert_int_not_equal(SymaX_getCurrentChannel(&st), initial_channel);

    // Verify packet count resets
    assert_int_equal(st.link.packet_count, 0);
}

static void test_buildPacket_invalid_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = 99; // Invalid phase

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Should not crash, returns SYMAX_WAIT but no action
    assert_int_equal(SymaX_buildPacket(&st, packet), SYMAX_WAIT);
}
#endif

/* ============================================================================
 * RX PACKET PROCESSING TESTS
 * ============================================================================ */

#ifdef SYMAX_CONFIG_RX
static void test_processPacket_null_arguments(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    uint8_t packet[SYMAX_PACKET_SIZE] = {0};

    // Test NULL state
    assert_int_equal(SymaX_processPacket(NULL, packet), SYMAX_ERROR_NULL_POINTER);

    // Test NULL packet
    assert_int_equal(SymaX_processPacket(&st, NULL), SYMAX_ERROR_INVALID_PACKET);
}

static void test_processPacket_checksum_validation(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA; // Set to DATA phase for processing

    uint8_t packet[SYMAX_PACKET_SIZE] = {0};

    // Test invalid checksum
    packet[9] = 0x00; // Wrong checksum
    assert_int_equal(SymaX_processPacket(&st, packet), SYMAX_ERROR_CHECKSUM_FAIL);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.checksum_failures, 1);
#endif

    // Test valid checksum
    packet[9] = test_calc_checksum(packet);
    assert_int_equal(SymaX_processPacket(&st, packet), SYMAX_SUCCESS);
}

#if !SYMAX_SKIP_PREBIND
static void test_processPacket_prebind_checksum_bypass(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // In PREBIND phase, checksum should be bypassed
    uint8_t prebind_packet[] = SYMAX_PREBIND_PACKET_EXPECTED;

    // Corrupt the checksum - should still work in PREBIND
    prebind_packet[9] = 0x00; // Wrong checksum
    assert_int_equal(SymaX_processPacket(&st, prebind_packet), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_PREBIND);
}

static void test_processPacket_prebind_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    uint8_t prebind_packet[] = SYMAX_PREBIND_PACKET_EXPECTED;

    assert_int_equal(st.link.phase, SYMAX_PREBIND);
    assert_int_equal(SymaX_processPacket(&st, prebind_packet), SYMAX_WAIT);

    // Should transition to BIND_IN_PROGRESS
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
    assert_int_equal(st.link.current_channel_idx, 0);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.packets_processed, 1);
#endif
}

static void test_processPacket_prebind_invalid(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Invalid prebind packet (different content)
    uint8_t invalid_prebind[SYMAX_PACKET_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00};
    invalid_prebind[9] = test_calc_checksum(invalid_prebind);

    assert_int_equal(SymaX_processPacket(&st, invalid_prebind), SYMAX_WAIT);
    // Should remain in PREBIND phase
    assert_int_equal(st.link.phase, SYMAX_PREBIND);
}
#endif

static void test_processPacket_bind_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    uint8_t test_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t bind_packet[SYMAX_PACKET_SIZE];
    create_bind_packet(bind_packet, test_address);

    assert_int_equal(SymaX_processPacket(&st, bind_packet), SYMAX_BIND_COMPLETE);

    // Should transition to WAIT_FIRST_PACKET
    assert_int_equal(st.link.phase, SYMAX_WAIT_FIRST_PACKET);
    assert_int_equal(st.link.current_channel_idx, 0);

    // Verify address was extracted correctly (reversed)
    uint8_t extracted_addr[5];
    SymaX_getAddress(&st, extracted_addr);
    assert_memory_equal(extracted_addr, test_address, 5);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.bind_packets, 1);
    assert_int_equal(stats.packets_processed, 1);
#endif
}

static void test_processPacket_bind_invalid(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    // Invalid bind packet (wrong signature bytes)
    uint8_t invalid_bind[SYMAX_PACKET_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xBB, 0xAA, 0xAA, 0x00, 0x00};
    invalid_bind[9] = test_calc_checksum(invalid_bind);

    assert_int_equal(SymaX_processPacket(&st, invalid_bind), SYMAX_WAIT);
    // Should remain in BIND_IN_PROGRESS phase
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);

    // Test other invalid signature combinations
    uint8_t invalid_bind2[SYMAX_PACKET_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xAA, 0xBB, 0xAA, 0x00, 0x00};
    invalid_bind2[9] = test_calc_checksum(invalid_bind2);
    assert_int_equal(SymaX_processPacket(&st, invalid_bind2), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);

    uint8_t invalid_bind3[SYMAX_PACKET_SIZE] = {0x01, 0x02, 0x03, 0x04, 0x05, 0xAA, 0xAA, 0xBB, 0x00, 0x00};
    invalid_bind3[9] = test_calc_checksum(invalid_bind3);
    assert_int_equal(SymaX_processPacket(&st, invalid_bind3), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
}

static void test_processPacket_wait_first_packet_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_WAIT_FIRST_PACKET;

    // Create data packet (not bind packet)
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 10, 20, 30, 40);

    assert_int_equal(SymaX_processPacket(&st, data_packet), SYMAX_WAIT);

    // Should transition to DATA phase
    assert_int_equal(st.link.phase, SYMAX_DATA);
    assert_int_equal(st.link.packet_count, 1);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.data_packets, 1);
    assert_int_equal(stats.packets_processed, 1);
#endif

    // Test that bind packet in this phase doesn't transition
    uint8_t test_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t bind_packet[SYMAX_PACKET_SIZE];
    create_bind_packet(bind_packet, test_address);

    st.link.phase = SYMAX_WAIT_FIRST_PACKET; // Reset
    assert_int_equal(SymaX_processPacket(&st, bind_packet), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_WAIT_FIRST_PACKET); // Should stay
}

static void test_processPacket_data_phase_basic(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 50, -60, 70, -80);

    assert_int_equal(SymaX_processPacket(&st, data_packet), SYMAX_SUCCESS);

    // Verify channel data was extracted
    assert_int_equal(st.channelsData.thr, 50);
    assert_int_equal(st.channelsData.ele, -60);
    assert_int_equal(st.channelsData.rud, 70);
    assert_int_equal(st.channelsData.ail, -80);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.data_packets, 1);
    assert_int_equal(stats.packets_processed, 1);
#endif
}

static void test_processPacket_data_sign_bit_handling(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Create packet with specific sign bit patterns
    uint8_t data_packet[SYMAX_PACKET_SIZE] = {0x8A, // thr: value 138
                                              0x14, // ele: no sign bit, value 20 -> should be 20
                                              0x9E, // rud: sign bit set, value 30 -> should be -30
                                              0x28, // ail: no sign bit, value 40 -> should be 40
                                              0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    data_packet[9] = test_calc_checksum(data_packet);

    assert_int_equal(SymaX_processPacket(&st, data_packet), SYMAX_SUCCESS);

    assert_int_equal(st.channelsData.thr, 138);
    assert_int_equal(st.channelsData.ele, 20);  // No sign bit -> positive
    assert_int_equal(st.channelsData.rud, -30); // Sign bit -> negative
    assert_int_equal(st.channelsData.ail, 40);  // No sign bit -> positive

    // Test edge case: sign bit set with value 0
    uint8_t edge_packet[SYMAX_PACKET_SIZE] = {0x00, 0x80, 0x80, 0x80, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    edge_packet[9] = test_calc_checksum(edge_packet);

    SymaX_processPacket(&st, edge_packet);
    assert_int_equal(st.channelsData.ele, 0); // -0 should be 0
    assert_int_equal(st.channelsData.rud, 0); // -0 should be 0
    assert_int_equal(st.channelsData.ail, 0); // -0 should be 0
}

static void test_processPacket_data_boundary_values(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Test maximum positive values
    uint8_t max_packet[SYMAX_PACKET_SIZE] = {0xFF, // Max positive (255)
                                             0x7F, // Max positive (127)
                                             0x7F, // Max positive (127)
                                             0x7F, // Max positive (127)
                                             0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    max_packet[9] = test_calc_checksum(max_packet);

    SymaX_processPacket(&st, max_packet);
    assert_int_equal(st.channelsData.thr, 255);
    assert_int_equal(st.channelsData.ele, 127);
    assert_int_equal(st.channelsData.rud, 127);
    assert_int_equal(st.channelsData.ail, 127);

    // Test maximum negative values
    uint8_t min_packet[SYMAX_PACKET_SIZE] = {0x00, // Min (0)
                                             0xFF, // Max negative (-127)
                                             0xFF, // Max negative (-127)
                                             0xFF, // Max negative (-127)
                                             0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    min_packet[9] = test_calc_checksum(min_packet);

    SymaX_processPacket(&st, min_packet);
    assert_int_equal(st.channelsData.thr, 0);
    assert_int_equal(st.channelsData.ele, -127);
    assert_int_equal(st.channelsData.rud, -127);
    assert_int_equal(st.channelsData.ail, -127);
}

#if SYMAX_ENABLE_FLAGS
static void test_processPacket_flags_extraction(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Create packet with all flags set except headless
    uint8_t flags_packet[SYMAX_PACKET_SIZE] = {0x00,        0x00, 0x00, 0x00, // Channel data
                                               0x80 | 0x40,                   // video | picture flags
                                               0xC0 | 0x3F,                   // Rate control + trim data
                                               0x40 | 0x3F,                   // flip flag + trim data
                                               0x00 | 0x3F,                   // no headless + trim data
                                               0x00,        0x00};
    flags_packet[9] = test_calc_checksum(flags_packet);

    assert_int_equal(SymaX_processPacket(&st, flags_packet), SYMAX_SUCCESS);

    assert_int_equal(st.flags.video, 1);
    assert_int_equal(st.flags.picture, 1);
    assert_int_equal(st.flags.flip, 1);
    assert_int_equal(st.flags.headless, 0);
    assert_int_equal(st.flags.xtrm_rates, 1); // Should be set due to trim data

    // Test headless mode disables xtrm_rates
    flags_packet[7] = 0x80 | 0x3F; // Set headless + trim data
    flags_packet[9] = test_calc_checksum(flags_packet);

    SymaX_processPacket(&st, flags_packet);
    assert_int_equal(st.flags.headless, 1);
    assert_int_equal(st.flags.xtrm_rates, 0); // Should be disabled by headless

    // Test no flags set
    uint8_t no_flags_packet[SYMAX_PACKET_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    no_flags_packet[9] = test_calc_checksum(no_flags_packet);

    SymaX_processPacket(&st, no_flags_packet);
    assert_int_equal(st.flags.video, 0);
    assert_int_equal(st.flags.picture, 0);
    assert_int_equal(st.flags.flip, 0);
    assert_int_equal(st.flags.headless, 0);
    assert_int_equal(st.flags.xtrm_rates, 0);
}

static void test_processPacket_xtrm_rates_calculation(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Test xtrm_rates calculation logic: sum of (packet[5] & 0x3F) + (packet[6] & 0x3F) + (packet[7] & 0x3F) != 0

    // Case 1: All trim values are 0 -> xtrm_rates should be 0
    uint8_t packet1[SYMAX_PACKET_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00};
    packet1[9] = test_calc_checksum(packet1);

    SymaX_processPacket(&st, packet1);
    assert_int_equal(st.flags.xtrm_rates, 0);

    // Case 2: Only one trim value non-zero -> xtrm_rates should be 1
    uint8_t packet2[SYMAX_PACKET_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 | 0x01, 0x00, 0x00, 0x00, 0x00};
    packet2[9] = test_calc_checksum(packet2);

    SymaX_processPacket(&st, packet2);
    assert_int_equal(st.flags.xtrm_rates, 1);

    // Case 3: Multiple trim values non-zero -> xtrm_rates should be 1
    uint8_t packet3[SYMAX_PACKET_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 | 0x05, 0x03, 0x07, 0x00, 0x00};
    packet3[9] = test_calc_checksum(packet3);

    SymaX_processPacket(&st, packet3);
    assert_int_equal(st.flags.xtrm_rates, 1);

    // Case 4: Headless mode overrides xtrm_rates
    uint8_t packet4[SYMAX_PACKET_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0 | 0x05, 0x03, 0x80 | 0x07, 0x00, 0x00};
    packet4[9] = test_calc_checksum(packet4);

    SymaX_processPacket(&st, packet4);
    assert_int_equal(st.flags.headless, 1);
    assert_int_equal(st.flags.xtrm_rates, 0); // Should be 0 due to headless
}

#if SYMAX_ENABLE_TRIM_DATA
static void test_processPacket_trim_data(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Create packet with channel data and trim extensions
    uint8_t packet[SYMAX_PACKET_SIZE] = {0x00,        // throttle = 0
                                         0x40,        // elevator = 64
                                         0x80 | 0x40, // rudder = -64
                                         0x20,        // aileron = 32
                                         0x00,        // no flags
                                         0xC0 | 0x10, // trim elevator +16
                                         0x00 | 0x10, // trim rudder +16
                                         0x00 | 0x10, // trim aileron +16
                                         0x00,        0x00};
    packet[9] = test_calc_checksum(packet);

    assert_int_equal(SymaX_processPacket(&st, packet), SYMAX_SUCCESS);

    // Verify channels with trim extension
    assert_int_equal(st.channelsData.thr, 0);   // No trim for throttle
    assert_int_equal(st.channelsData.ele, 80);  // 64 + 16
    assert_int_equal(st.channelsData.rud, -48); // -64 + 16
    assert_int_equal(st.channelsData.ail, 48);  // 32 + 16

    // Test negative trim values
    packet[5] = 0xC0 | 0x30; // trim = -16 (bit 5 set)
    packet[6] = 0x00 | 0x30; // trim = -16
    packet[7] = 0x00 | 0x30; // trim = -16
    packet[9] = test_calc_checksum(packet);

    SymaX_processPacket(&st, packet);
    assert_int_equal(st.channelsData.ele, 48);  // 64 - 16
    assert_int_equal(st.channelsData.rud, -80); // -64 - 16
    assert_int_equal(st.channelsData.ail, 16);  // 32 - 16

    // Test trim data clamping at upper limit
    packet[1] = 0x7C;        // elevator = 124
    packet[5] = 0xC0 | 0x1F; // trim = +31
    packet[9] = test_calc_checksum(packet);

    SymaX_processPacket(&st, packet);
    assert_int_equal(st.channelsData.ele, SYMAX_CHANNEL_MAX); // Should clamp at max

    // Test trim data clamping at lower limit
    packet[1] = 0x80 | 0x7C; // elevator = -124
    packet[5] = 0xC0 | 0x2F; // trim = -15
    packet[9] = test_calc_checksum(packet);

    SymaX_processPacket(&st, packet);
    assert_int_equal(st.channelsData.ele, SYMAX_CHANNEL_MIN); // Should clamp at min

    // Test trim data ignored when xtrm_rates is 0 (no trim data in packet)
    packet[5] = 0xC0; // No trim data
    packet[6] = 0x00;
    packet[7] = 0x00;
    packet[9] = test_calc_checksum(packet);

    SymaX_processPacket(&st, packet);
    assert_int_equal(st.channelsData.ele, -124); // Original value without trim
}
#endif
#endif

static void test_processPacket_frequency_hopping(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 0, 0, 0, 0);

    uint8_t initial_channel = SymaX_getCurrentChannel(&st);

    // Send packets and verify hopping
    for (int i = 0; i < SYMAX_HOP_DATA_RATE - 1; i++) {
        SymaX_processPacket(&st, data_packet);
        assert_int_equal(SymaX_getCurrentChannel(&st), initial_channel); // Should stay same
    }

    // Next packet should hop
    SymaX_processPacket(&st, data_packet);
    assert_int_not_equal(SymaX_getCurrentChannel(&st), initial_channel);

    // Verify packet count resets
    assert_int_equal(st.link.packet_count, 0);
}

static void test_processPacket_invalid_phase(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = 99; // Invalid phase

    uint8_t packet[SYMAX_PACKET_SIZE] = {0};
    packet[9] = test_calc_checksum(packet);

    // Should not crash, returns WAIT but no action
    assert_int_equal(SymaX_processPacket(&st, packet), SYMAX_WAIT);
}
#endif

/* ============================================================================
 * FLAGS & TRIM TESTS
 * ============================================================================ */

#if SYMAX_ENABLE_FLAGS
static void test_flags_union_access(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test individual flag setting
    st.flags.flip = 1;
    st.flags.picture = 1;
    st.flags.video = 1;
    st.flags.headless = 1;
    st.flags.xtrm_rates = 1;

    // Check that the flags register reflects the changes
    assert_int_not_equal(st.flags.flags, 0);

    // Test clearing via flags register
    st.flags.flags = 0;
    assert_int_equal(st.flags.flip, 0);
    assert_int_equal(st.flags.picture, 0);
    assert_int_equal(st.flags.video, 0);
    assert_int_equal(st.flags.headless, 0);
    assert_int_equal(st.flags.xtrm_rates, 0);

    // Test setting via flags register
    st.flags.flags = 0xFF;
    assert_int_equal(st.flags.flip, 1);
    assert_int_equal(st.flags.picture, 1);
    assert_int_equal(st.flags.video, 1);
    assert_int_equal(st.flags.headless, 1);
    assert_int_equal(st.flags.xtrm_rates, 1);
}
#endif

/* ============================================================================
 * PACKET FRESHNESS TESTS
 * ============================================================================ */

#if SYMAX_ENABLE_FRESHNESS_CHECK
static void test_packet_freshness_tracking(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    SymaX_setTimestampCallback(&st, test_getTimestamp_ms);

    // Initially no packets should be fresh
    assert_false(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 100));

    // Process a data packet
    st.link.phase = SYMAX_DATA;
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 0, 0, 0, 0);
    SymaX_processPacket(&st, data_packet);

    // Should be fresh immediately
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 0));
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 100));

    // Advance time
    mock_timestamp += 50;
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 100)); // Still fresh
    assert_false(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 30)); // Too old

    // Advance time further
    mock_timestamp += 100;
    assert_false(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 100)); // Now stale
}

static void test_packet_freshness_different_types(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    SymaX_setTimestampCallback(&st, test_getTimestamp_ms);

#if !SYMAX_SKIP_PREBIND
    // Test prebind packet freshness
    uint8_t prebind_packet[] = SYMAX_PREBIND_PACKET_EXPECTED;
    SymaX_processPacket(&st, prebind_packet);
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_PREBIND, 0));
    assert_false(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_BIND, 0)); // Different type
    assert_false(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 0)); // Different type
#endif

    // Test bind packet freshness
    uint8_t test_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t bind_packet[SYMAX_PACKET_SIZE];
    create_bind_packet(bind_packet, test_address);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
    SymaX_processPacket(&st, bind_packet);
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_BIND, 0));

    // Test data packet freshness
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 0, 0, 0, 0);
    st.link.phase = SYMAX_DATA;
    SymaX_processPacket(&st, data_packet);
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 0));
}

static void test_packet_freshness_invalid_arguments(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    SymaX_setTimestampCallback(&st, test_getTimestamp_ms);

    // Test invalid packet type
    assert_false(SymaX_isPacketFresh(&st, SYMAX_TRACKED_FRAME_TYPES, 10));
    assert_false(SymaX_isPacketFresh(&st, 255, 10));

    // Test NULL state
    assert_false(SymaX_isPacketFresh(NULL, SYMAX_PACKET_TYPE_DATA, 10));

    // Test no callback set
    SymaX_t no_callback;
    SymaX_init(&no_callback);
    assert_false(SymaX_isPacketFresh(&no_callback, SYMAX_PACKET_TYPE_DATA, 10));
}

static void test_packet_freshness_edge_cases(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    SymaX_setTimestampCallback(&st, test_getTimestamp_ms);

    st.link.phase = SYMAX_DATA;
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 0, 0, 0, 0);
    SymaX_processPacket(&st, data_packet); // No packet processed yet

    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 0));

    mock_timestamp = UINT32_MAX - 10;
    SymaX_processPacket(&st, data_packet);

    mock_timestamp = 10; // Wrapped around
    assert_true(SymaX_isPacketFresh(&st, SYMAX_PACKET_TYPE_DATA, 30));
}
#endif

/* ============================================================================
 * STATISTICS TESTS
 * ============================================================================ */

#if SYMAX_ENABLE_STATS
static void test_stats_initialization(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);

    assert_int_equal(stats.packets_processed, 0);
    assert_int_equal(stats.bind_packets, 0);
    assert_int_equal(stats.data_packets, 0);
    assert_int_equal(stats.checksum_failures, 0);
}

#ifdef SYMAX_CONFIG_TX
static void test_stats_bind_packet_counting(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    // Build bind packets (TX side stats)
    uint8_t packet[SYMAX_PACKET_SIZE];
    for (int i = 0; i < 5; i++) {
        SymaX_buildPacket(&st, packet);
    }

    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.bind_packets, 5);
    assert_int_equal(stats.packets_processed, 5);
    assert_int_equal(stats.data_packets, 0);
}

static void test_stats_data_packet_counting(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Build data packets (TX side stats)
    uint8_t packet[SYMAX_PACKET_SIZE];
    for (int i = 0; i < 10; i++) {
        SymaX_buildPacket(&st, packet);
    }

    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.data_packets, 10);
    assert_int_equal(stats.packets_processed, 10);
    assert_int_equal(stats.bind_packets, 0);
}
#endif

#ifdef SYMAX_CONFIG_RX
static void test_stats_checksum_failure_counting(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    st.link.phase = SYMAX_DATA; // Set to DATA phase for processing
    // Create packets with bad checksums
    uint8_t bad_packet[SYMAX_PACKET_SIZE] = {0};

    for (int i = 0; i < 3; i++) {
        SymaX_processPacket(&st, bad_packet);
    }

    SymaX_Stats_t stats;
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.checksum_failures, 3);
    assert_int_equal(stats.packets_processed, 0); // Bad packets don't count as processed
}

static void test_stats_mixed_operations(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    SymaX_Stats_t stats;

    //reset with NULL, it should not crash
    SymaX_resetStats(NULL);

    // Process some bind packets (RX side)
    uint8_t test_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t bind_packet[SYMAX_PACKET_SIZE];
    create_bind_packet(bind_packet, test_address);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    for (int i = 0; i < 2; i++) {
        SymaX_processPacket(&st, bind_packet);
    }

    // Process some data packets (RX side)
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 0, 0, 0, 0);
    st.link.phase = SYMAX_DATA;

    for (int i = 0; i < 5; i++) {
        SymaX_processPacket(&st, data_packet);
    }

    // Process some bad packets
    uint8_t bad_packet[SYMAX_PACKET_SIZE] = {0};
    bad_packet[9] = 0x00; // Wrong checksum

    for (int i = 0; i < 3; i++) {
        SymaX_processPacket(&st, bad_packet);
    }

    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.bind_packets, 1); // 2 bind packets processed, but only 1 unique bind
    assert_int_equal(stats.data_packets, 5);
    assert_int_equal(stats.checksum_failures, 3);
    assert_int_equal(stats.packets_processed, 6); // Only good packets

    SymaX_resetStats(&st);
    SymaX_getStats(&st, &stats);
    assert_int_equal(stats.bind_packets, 0);
    assert_int_equal(stats.data_packets, 0);
    assert_int_equal(stats.checksum_failures, 0);
    assert_int_equal(stats.packets_processed, 0);
}
#endif
#endif

/* ============================================================================
 * ERROR RESILIENCE AND EDGE CASE TESTS
 * ============================================================================ */

#ifdef SYMAX_CONFIG_RX
static void test_sustained_corruption_resilience(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test processing many corrupted packets doesn't crash or corrupt state
    uint8_t bad_packet[SYMAX_PACKET_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

    for (int i = 0; i < 1000; i++) {
        SymaX_processPacket(&st, bad_packet);
        bad_packet[0] = (bad_packet[0] + 1) & 0xFF; // Vary the corruption
    }

    // State should still be valid
#if SYMAX_SKIP_PREBIND
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
#else
    assert_int_equal(st.link.phase, SYMAX_PREBIND);
#endif
    assert_int_equal(st.link.current_channel_idx, 0);

#if !SYMAX_SKIP_PREBIND
    // Should still be able to process valid packets
    uint8_t good_packet[] = SYMAX_PREBIND_PACKET_EXPECTED;
    assert_int_equal(SymaX_processPacket(&st, good_packet), SYMAX_WAIT);
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
#endif
}

static void test_phase_consistency_after_errors(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Transition through phases with intermittent errors
    uint8_t bad_packet[SYMAX_PACKET_SIZE] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x00};

#if !SYMAX_SKIP_PREBIND
    // Try to process prebind with errors
    uint8_t prebind_packet[] = SYMAX_PREBIND_PACKET_EXPECTED;
    SymaX_processPacket(&st, bad_packet); // Error
    assert_int_equal(st.link.phase, SYMAX_PREBIND);

    SymaX_processPacket(&st, prebind_packet); // Success
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);
#else
    st.link.phase = SYMAX_BIND_IN_PROGRESS;
#endif

    // Try to bind with errors
    uint8_t test_address[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
    uint8_t bind_packet[SYMAX_PACKET_SIZE];
    create_bind_packet(bind_packet, test_address);

    SymaX_processPacket(&st, bad_packet); // Error
    assert_int_equal(st.link.phase, SYMAX_BIND_IN_PROGRESS);

    assert_int_equal(SymaX_processPacket(&st, bind_packet), SYMAX_BIND_COMPLETE);
    assert_int_equal(st.link.phase, SYMAX_WAIT_FIRST_PACKET);

    // Test data phase with errors
    uint8_t data_packet[SYMAX_PACKET_SIZE];
    create_data_packet(data_packet, 10, 20, 30, 40);

    SymaX_processPacket(&st, bad_packet); // Error
    assert_int_equal(st.link.phase, SYMAX_WAIT_FIRST_PACKET);

    SymaX_processPacket(&st, data_packet); // Success
    assert_int_equal(st.link.phase, SYMAX_DATA);
}
#endif

#ifdef SYMAX_CONFIG_TX
static void test_channel_wraparound(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;
    st.link.current_channel_idx = 3; // Last channel

    uint8_t packet[SYMAX_PACKET_SIZE];
    uint8_t last_channel = SymaX_getCurrentChannel(&st);

    // Trigger hop from last channel
    for (int i = 0; i < SYMAX_HOP_DATA_RATE; i++) {
        SymaX_buildPacket(&st, packet);
    }

    // Should wrap around to first channel
    assert_int_equal(st.link.current_channel_idx, 0);
    assert_int_not_equal(SymaX_getCurrentChannel(&st), last_channel);
    assert_int_equal(SymaX_getCurrentChannel(&st), st.link.rf_channels[0]);
}

static void test_large_packet_counts(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Test large number of packets (potential overflow scenarios)
    for (int i = 0; i < 10000; i++) {
        SymaX_buildPacket(&st, packet);
    }

    // State should still be valid
    assert_int_equal(st.link.phase, SYMAX_DATA);
    assert_true(st.link.current_channel_idx < 4);            // Should be valid channel index
    assert_true(st.link.packet_count < SYMAX_HOP_DATA_RATE); // Should be valid count
}

#if !SYMAX_SKIP_PREBIND
static void test_prebind_packet_hopping_timing(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    uint8_t initial_channel = SymaX_getCurrentChannel(&st);

    // Test not hopping in prebind phase
    for (int i = 1; i < SYMAX_PREBIND_PACKETS - 2; i++) {
        uint8_t packet[SYMAX_PACKET_SIZE];
        SymaX_buildPacket(&st, packet);
        assert_int_equal(SymaX_getCurrentChannel(&st), initial_channel);
        assert_int_equal(st.link.current_channel_idx, 0);
    }
}
#endif
#endif

/* ============================================================================
 * CHECKSUM CALCULATION TESTS
 * ============================================================================ */

static void test_checksum_calculation_edge_cases(void** state) {
    (void)state;

    // Test all-zero packet
    uint8_t zero_packet[SYMAX_PACKET_SIZE] = {0};
    uint8_t expected_checksum = 0x55; // 0 + 0x55
    assert_int_equal(test_calc_checksum(zero_packet), expected_checksum);

    // Test all-0xFF packet (except checksum byte)
    uint8_t ff_packet[SYMAX_PACKET_SIZE];
    memset(ff_packet, 0xFF, SYMAX_PACKET_SIZE - 1);
    ff_packet[SYMAX_PACKET_SIZE - 1] = 0; // Don't include checksum byte

    uint8_t expected_ff = 0xFF; // Start with 0xFF
    for (int i = 1; i < SYMAX_PACKET_SIZE - 1; i++) {
        expected_ff ^= 0xFF;
    }
    expected_ff += 0x55;
    assert_int_equal(test_calc_checksum(ff_packet), expected_ff);

    // Test alternating pattern
    uint8_t alt_packet[SYMAX_PACKET_SIZE] = {0};
    for (int i = 0; i < SYMAX_PACKET_SIZE - 1; i++) {
        alt_packet[i] = (i % 2) ? 0xAA : 0x55;
    }
    uint8_t calc_checksum = test_calc_checksum(alt_packet);
    assert_true(calc_checksum != 0); // Should produce valid checksum
}

/* ============================================================================
 * UNION/STRUCT ACCESS PATTERN TESTS
 * ============================================================================ */

static void test_channel_data_union_access(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);

    // Test both union access patterns work correctly
    st.channelsData.thr = 129;
    st.channelsData.ele = -50;
    st.channelsData.rud = 75;
    st.channelsData.ail = -25;

    // Access via array notation
    assert_int_equal((uint8_t)st.channelsData.channels[0], 129); // thr (uint8_t, but accessed as int8_t)
    assert_int_equal(st.channelsData.channels[1], -50);          // ele
    assert_int_equal(st.channelsData.channels[2], 75);           // rud
    assert_int_equal(st.channelsData.channels[3], -25);          // ail

    // Modify via array and check named access
    st.channelsData.channels[1] = 60;
    st.channelsData.channels[2] = -80;

    assert_int_equal(st.channelsData.ele, 60);
    assert_int_equal(st.channelsData.rud, -80);
}

#ifdef SYMAX_CONFIG_RX
static void test_bind_packet_address_extraction_edge_cases(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_BIND_IN_PROGRESS;

    // Test with all possible byte values in address
    uint8_t test_addresses[][5] = {
        {0x00, 0x00, 0x00, 0x00, 0x00}, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF}, {0x01, 0x23, 0x45, 0x67, 0x89}, {0xFE, 0xDC, 0xBA, 0x98, 0x76}};

    for (size_t i = 0; i < sizeof(test_addresses) / sizeof(test_addresses[0]); i++) {
        uint8_t bind_packet[SYMAX_PACKET_SIZE];
        create_bind_packet(bind_packet, test_addresses[i]);

        SymaX_t st_test;
        SymaX_init(&st_test);
        st_test.link.phase = SYMAX_BIND_IN_PROGRESS;

        SymaX_processPacket(&st_test, bind_packet);

        uint8_t extracted_addr[5];
        SymaX_getAddress(&st_test, extracted_addr);
        assert_memory_equal(extracted_addr, test_addresses[i], 5);
    }
}

static void test_data_packet_channel_extraction_comprehensive(void** state) {
    (void)state;
    SymaX_t st;
    SymaX_init(&st);
    st.link.phase = SYMAX_DATA;

    // Test all combinations of sign bits
    struct {
        uint8_t raw_bytes[4];
        int16_t expected[4];
    } test_cases[] = {{{0x00, 0x00, 0x00, 0x00}, {0, 0, 0, 0}},
                      {{0x7F, 0x7F, 0x7F, 0x7F}, {127, 127, 127, 127}},
                      {{0xFF, 0xFF, 0xFF, 0xFF}, {255, -127, -127, -127}}, // thr is uint8_t, others are signed
                      {{0x80, 0x80, 0x80, 0x80}, {128, 0, 0, 0}},          // Sign bit with zero value
                      {{0x81, 0x81, 0x81, 0x81}, {129, -1, -1, -1}},
                      {{0x55, 0xAA, 0x33, 0xCC}, {85, -42, 51, -76}}};

    for (size_t i = 0; i < sizeof(test_cases) / sizeof(test_cases[0]); i++) {
        uint8_t packet[SYMAX_PACKET_SIZE] = {0};
        memcpy(packet, test_cases[i].raw_bytes, 4);
        packet[5] = 0xC0;
        packet[9] = test_calc_checksum(packet);

        SymaX_processPacket(&st, packet);

        assert_int_equal(st.channelsData.thr, test_cases[i].expected[0]);
        assert_int_equal(st.channelsData.ele, test_cases[i].expected[1]);
        assert_int_equal(st.channelsData.rud, test_cases[i].expected[2]);
        assert_int_equal(st.channelsData.ail, test_cases[i].expected[3]);
    }
}
#endif

/* ============================================================================
 * INTEGRATION TESTS (End-to-End)
 * ============================================================================ */

/* ============================================================================
 * INTEGRATION AND ROUND-TRIP TESTS
 * ============================================================================ */
#if defined(SYMAX_CONFIG_TX) && defined(SYMAX_CONFIG_RX)
static void test_complete_tx_rx_binding_flow(void** state) {
    (void)state;
    SymaX_t tx, rx;
    SymaX_init(&tx);
    SymaX_init(&rx);

    uint8_t packet[SYMAX_PACKET_SIZE];

#if !SYMAX_SKIP_PREBIND
    // Phase 1: TX sends prebind packets, RX processes them
    while (tx.link.phase == SYMAX_PREBIND) {
        SymaX_buildPacket(&tx, packet);
        if (rx.link.phase == SYMAX_PREBIND) {
            SymaX_processPacket(&rx, packet);
        }
    }

    // Both should be in bind phase now
    assert_int_equal(tx.link.phase, SYMAX_BIND_IN_PROGRESS);
    assert_int_equal(rx.link.phase, SYMAX_BIND_IN_PROGRESS);
#endif

    // Phase 2: TX sends bind packets, RX processes them
    SymaX_Status_t rx_status = SYMAX_SUCCESS;
    while (tx.link.phase == SYMAX_BIND_IN_PROGRESS && rx_status != SYMAX_BIND_COMPLETE) {
        SymaX_buildPacket(&tx, packet);
        rx_status = SymaX_processPacket(&rx, packet);
    }

    // RX should complete binding
    assert_int_equal(rx_status, SYMAX_BIND_COMPLETE);
    assert_int_equal(rx.link.phase, SYMAX_WAIT_FIRST_PACKET);

    // Continue TX until it transitions to data
    while (tx.link.phase == SYMAX_BIND_IN_PROGRESS) {
        SymaX_buildPacket(&tx, packet);
    }
    assert_int_equal(tx.link.phase, SYMAX_DATA);

    // Transition RX to SYMAX_DATA
    SymaX_buildPacket(&tx, packet);
    SymaX_processPacket(&rx, packet);
    assert_int_equal(rx.link.phase, SYMAX_DATA);

    // Both should have same address
    uint8_t tx_addr[5], rx_addr[5];
    SymaX_getAddress(&tx, tx_addr);
    SymaX_getAddress(&rx, rx_addr);
    assert_memory_equal(tx_addr, rx_addr, 5);

    // Phase 3: Test data exchange
    tx.channelsData.thr = 42;
    tx.channelsData.ele = -42;
    tx.channelsData.rud = 84;
    tx.channelsData.ail = -84;

    SymaX_buildPacket(&tx, packet);
    SymaX_processPacket(&rx, packet);

    assert_int_equal(rx.channelsData.thr, 42);
    assert_int_equal(rx.channelsData.ele, -42);
    assert_int_equal(rx.channelsData.rud, 84);
    assert_int_equal(rx.channelsData.ail, -84);
}

static void test_encode_decode_roundtrip_comprehensive(void** state) {
    (void)state;

    uint8_t test_values_thr[] = {
        0, 1, 50, 127, 255, 255, 127, 63, 64, 2, 0 // Various values including edge cases
    };

    int8_t test_values[] = {
        0, 1, -1, 50, -50, SYMAX_CHANNEL_MIN, SYMAX_CHANNEL_MAX, SYMAX_CHANNEL_MIN - 1, SYMAX_CHANNEL_MAX - 1, 63, -63, 64, -64 // Powers of 2 and around them
    };

    for (size_t i = 0; i < sizeof(test_values) / sizeof(test_values[0]); i++) {
        SymaX_t tx, rx;
        SymaX_init(&tx);
        SymaX_init(&rx);

        // Set both to data phase
        tx.link.phase = SYMAX_DATA;
        rx.link.phase = SYMAX_DATA;

        tx.channelsData.thr = test_values_thr[i % (sizeof(test_values_thr) / sizeof(test_values_thr[0]))];
        tx.channelsData.ele = test_values[i];
        tx.channelsData.rud = test_values[i];
        tx.channelsData.ail = test_values[i];

        uint8_t packet[SYMAX_PACKET_SIZE];
        assert_int_equal(SymaX_buildPacket(&tx, packet), SYMAX_SUCCESS);
        assert_int_equal(SymaX_processPacket(&rx, packet), SYMAX_SUCCESS);

        assert_int_equal(rx.channelsData.thr, tx.channelsData.thr);
        if (test_values[i] < SYMAX_CHANNEL_MIN) {
            assert_int_equal(rx.channelsData.ele, SYMAX_CHANNEL_MIN);
            assert_int_equal(rx.channelsData.rud, SYMAX_CHANNEL_MIN);
            assert_int_equal(rx.channelsData.ail, SYMAX_CHANNEL_MIN);
        } else {
            assert_int_equal(rx.channelsData.ele, test_values[i]);
            assert_int_equal(rx.channelsData.rud, test_values[i]);
            assert_int_equal(rx.channelsData.ail, test_values[i]);
        }
    }
}

#if SYMAX_ENABLE_FLAGS
static void test_flags_roundtrip(void** state) {
    (void)state;
    SymaX_t tx, rx;
    SymaX_init(&tx);
    SymaX_init(&rx);

    tx.link.phase = SYMAX_DATA;
    rx.link.phase = SYMAX_DATA;

    // Test all combinations of flags
    for (int video = 0; video <= 1; video++) {
        for (int picture = 0; picture <= 1; picture++) {
            for (int flip = 0; flip <= 1; flip++) {
                for (int headless = 0; headless <= 1; headless++) {
                    tx.flags.video = video;
                    tx.flags.picture = picture;
                    tx.flags.flip = flip;
                    tx.flags.headless = headless;

                    uint8_t packet[SYMAX_PACKET_SIZE];
                    SymaX_buildPacket(&tx, packet);
                    SymaX_processPacket(&rx, packet);

                    assert_int_equal(rx.flags.video, video);
                    assert_int_equal(rx.flags.picture, picture);
                    assert_int_equal(rx.flags.flip, flip);
                    assert_int_equal(rx.flags.headless, headless);
                }
            }
        }
    }
}
#endif

static void test_frequency_hopping_synchronization(void** state) {
    (void)state;
    SymaX_t tx, rx;
    SymaX_init(&tx);
    SymaX_init(&rx);

    // Perform complete binding to sync addresses and channels
    tx.link.phase = SYMAX_BIND_IN_PROGRESS;
    rx.link.phase = SYMAX_BIND_IN_PROGRESS;

    uint8_t packet[SYMAX_PACKET_SIZE];

    // Complete bind process
    SymaX_buildPacket(&tx, packet);                 // Generate TX address
    create_bind_packet(packet, tx.link.rf_address); // Create proper bind packet
    SymaX_processPacket(&rx, packet);               // RX processes bind

    // Transition both to data phase
    while (tx.link.phase != SYMAX_DATA) {
        SymaX_buildPacket(&tx, packet);
    }
    assert_int_equal(tx.link.packet_count, 0);
    assert_int_equal(rx.link.packet_count, 0);
    rx.link.phase = SYMAX_WAIT_FIRST_PACKET;

    // Test that both hop on same schedule
    for (int cycle = 0; cycle < 3; cycle++) {
        uint8_t tx_channel = SymaX_getCurrentChannel(&tx);
        uint8_t rx_channel = SymaX_getCurrentChannel(&rx);
        assert_int_equal(tx_channel, rx_channel);

        // Send SYMAX_HOP_DATA_RATE packets
        for (int i = 0; i < SYMAX_HOP_DATA_RATE; i++) {
            // Should stay on same channel
            assert_int_equal(SymaX_getCurrentChannel(&tx), tx_channel);
            assert_int_equal(SymaX_getCurrentChannel(&rx), rx_channel);

            SymaX_buildPacket(&tx, packet);
            SymaX_processPacket(&rx, packet);
        }

        uint8_t new_tx_channel = SymaX_getCurrentChannel(&tx);
        uint8_t new_rx_channel = SymaX_getCurrentChannel(&rx);

        assert_int_not_equal(new_tx_channel, tx_channel); // Should have hopped
        assert_int_equal(new_tx_channel, new_rx_channel); // Should be same
    }
}
#endif

/* ============================================================================
 * NEW COMPREHENSIVE INTEGRATION TEST
 * ============================================================================ */

#if defined(SYMAX_CONFIG_TX) && defined(SYMAX_CONFIG_RX)
static void test_complete_protocol_stress_test(void** state) {
    (void)state;
    SymaX_t tx, rx;
    SymaX_init(&tx);
    SymaX_init(&rx);

    uint8_t packet[SYMAX_PACKET_SIZE];

#if SYMAX_ENABLE_FRESHNESS_CHECK
    SymaX_setTimestampCallback(&rx, test_getTimestamp_ms);
#endif

    // Complete binding process
#if !SYMAX_SKIP_PREBIND
    // Prebind phase
    while (tx.link.phase == SYMAX_PREBIND) {
        SymaX_buildPacket(&tx, packet);
        if (rx.link.phase == SYMAX_PREBIND) {
            SymaX_processPacket(&rx, packet);
        }
    }
#endif

    // Bind phase
    while (tx.link.phase == SYMAX_BIND_IN_PROGRESS && rx.link.phase != SYMAX_WAIT_FIRST_PACKET) {
        SymaX_buildPacket(&tx, packet);
        SymaX_processPacket(&rx, packet);
    }

    // Transition to data phase
    while (tx.link.phase != SYMAX_DATA) {
        SymaX_buildPacket(&tx, packet);
    }

    SymaX_buildPacket(&tx, packet);
    SymaX_processPacket(&rx, packet);

    // Stress test data transmission with various patterns
    int8_t test_patterns[][4] = {{0, 0, 0, 0}, {127, 127, 127, 127}, {-127, -127, -127, -127}, {50, -50, 75, -75}, {1, -1, 2, -2}};

    for (int cycle = 0; cycle < 100; cycle++) {
        int pattern_idx = cycle % (sizeof(test_patterns) / sizeof(test_patterns[0]));

        tx.channelsData.thr = abs(test_patterns[pattern_idx][0]);
        tx.channelsData.ele = test_patterns[pattern_idx][1];
        tx.channelsData.rud = test_patterns[pattern_idx][2];
        tx.channelsData.ail = test_patterns[pattern_idx][3];

#if SYMAX_ENABLE_FLAGS
        tx.flags.flip = cycle % 2;
        tx.flags.video = (cycle / 2) % 2;
        tx.flags.picture = (cycle / 4) % 2;
        tx.flags.headless = (cycle / 8) % 2;
#endif

        SymaX_buildPacket(&tx, packet);
        assert_int_equal(SymaX_processPacket(&rx, packet), SYMAX_SUCCESS);

        // Verify data integrity
        assert_int_equal(rx.channelsData.thr, abs(test_patterns[pattern_idx][0]));
        assert_int_equal(rx.channelsData.ele, test_patterns[pattern_idx][1]);
        assert_int_equal(rx.channelsData.rud, test_patterns[pattern_idx][2]);
        assert_int_equal(rx.channelsData.ail, test_patterns[pattern_idx][3]);

#if SYMAX_ENABLE_FLAGS
        assert_int_equal(rx.flags.flip, cycle % 2);
        assert_int_equal(rx.flags.video, (cycle / 2) % 2);
        assert_int_equal(rx.flags.picture, (cycle / 4) % 2);
        assert_int_equal(rx.flags.headless, (cycle / 8) % 2);
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
        assert_true(SymaX_isPacketFresh(&rx, SYMAX_PACKET_TYPE_DATA, 1000));
        mock_timestamp++; // Advance time slightly
#endif
    }

    // Verify frequency hopping occurred
    assert_true(rx.link.packet_count < SYMAX_HOP_DATA_RATE);
    assert_true(tx.link.packet_count < SYMAX_HOP_DATA_RATE);

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t tx_stats, rx_stats;
    SymaX_getStats(&tx, &tx_stats);
    SymaX_getStats(&rx, &rx_stats);

    assert_true(tx_stats.packets_processed > 0);
    assert_true(rx_stats.packets_processed > 0);
    assert_int_equal(rx_stats.checksum_failures, 0); // Should have no failures in clean test
#endif
}
#endif

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================ */

int main(void) {
    const struct CMUnitTest tests[] = {
        /* INITIALIZATION AND CONFIGURATION */
        cmocka_unit_test(test_init_null_pointer),
        cmocka_unit_test(test_init_defaults_and_clear),
#if SYMAX_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_timestamp_callback_setting),
#endif

        /* GETTER/SETTER API TESTS */
        cmocka_unit_test(test_getCurrentChannel),
        cmocka_unit_test(test_startBinding),
        cmocka_unit_test(test_isBound),
        cmocka_unit_test(test_getAddress_macro),
#if SYMAX_ENABLE_STATS
        cmocka_unit_test(test_stats_getter),
#endif

/* RF CHANNEL CALCULATION */
#ifdef SYMAX_CONFIG_TX
        cmocka_unit_test(test_calc_rc_channels_range_0_to_0x0F),
        cmocka_unit_test(test_calc_rc_channels_special_case_address_6),
        cmocka_unit_test(test_calc_rc_channels_range_0x10_to_0x17),
        cmocka_unit_test(test_calc_rc_channels_special_case_address_0x16),
        cmocka_unit_test(test_calc_rc_channels_range_0x18_to_0x1D),
        cmocka_unit_test(test_calc_rc_channels_special_case_address_0x1E),
        cmocka_unit_test(test_calc_rc_channels_special_case_address_0x1F),

        /* TX PACKET BUILDING */
        cmocka_unit_test(test_buildPacket_null_arguments),
#if !SYMAX_SKIP_PREBIND
        cmocka_unit_test(test_buildPacket_prebind_phase),
        cmocka_unit_test(test_buildPacket_prebind_to_bind_transition),
#endif
        cmocka_unit_test(test_buildPacket_bind_phase),
        cmocka_unit_test(test_buildPacket_bind_with_non_default_address),
        cmocka_unit_test(test_buildPacket_bind_to_data_transition),
        cmocka_unit_test(test_buildPacket_data_phase_basic),
        cmocka_unit_test(test_buildPacket_channel_conversion),
#if SYMAX_ENABLE_FLAGS
        cmocka_unit_test(test_buildPacket_flags_encoding),
#if SYMAX_ENABLE_TRIM_DATA
        cmocka_unit_test(test_buildPacket_trim_data),
#endif
#endif
        cmocka_unit_test(test_buildPacket_frequency_hopping),
        cmocka_unit_test(test_buildPacket_invalid_phase),
#endif

/* RX PACKET PROCESSING */
#ifdef SYMAX_CONFIG_RX
        cmocka_unit_test(test_processPacket_null_arguments),
        cmocka_unit_test(test_processPacket_checksum_validation),
#if !SYMAX_SKIP_PREBIND
        cmocka_unit_test(test_processPacket_prebind_checksum_bypass),
        cmocka_unit_test(test_processPacket_prebind_phase),
        cmocka_unit_test(test_processPacket_prebind_invalid),
#endif
        cmocka_unit_test(test_processPacket_bind_phase),
        cmocka_unit_test(test_processPacket_bind_invalid),
        cmocka_unit_test(test_processPacket_wait_first_packet_phase),
        cmocka_unit_test(test_processPacket_data_phase_basic),
        cmocka_unit_test(test_processPacket_data_sign_bit_handling),
        cmocka_unit_test(test_processPacket_data_boundary_values),
#if SYMAX_ENABLE_FLAGS
        cmocka_unit_test(test_processPacket_flags_extraction),
        cmocka_unit_test(test_processPacket_xtrm_rates_calculation),
#if SYMAX_ENABLE_TRIM_DATA
        cmocka_unit_test(test_processPacket_trim_data),
#endif
#endif
        cmocka_unit_test(test_processPacket_frequency_hopping),
        cmocka_unit_test(test_processPacket_invalid_phase),
#endif

/* FLAGS & TRIM */
#if SYMAX_ENABLE_FLAGS
        cmocka_unit_test(test_flags_union_access),
#endif
/* PACKET FRESHNESS */
#if SYMAX_ENABLE_FRESHNESS_CHECK
        cmocka_unit_test(test_packet_freshness_tracking),
        cmocka_unit_test(test_packet_freshness_different_types),
        cmocka_unit_test(test_packet_freshness_invalid_arguments),
        cmocka_unit_test(test_packet_freshness_edge_cases),
#endif

/* STATISTICS */
#if SYMAX_ENABLE_STATS
        cmocka_unit_test(test_stats_initialization),
#ifdef SYMAX_CONFIG_TX
        cmocka_unit_test(test_stats_bind_packet_counting),
        cmocka_unit_test(test_stats_data_packet_counting),
#endif
#ifdef SYMAX_CONFIG_RX
        cmocka_unit_test(test_stats_checksum_failure_counting),
        cmocka_unit_test(test_stats_mixed_operations),
#endif
#endif

/* ERROR RESILIENCE AND EDGE CASES */
#ifdef SYMAX_CONFIG_RX
        cmocka_unit_test(test_sustained_corruption_resilience),
        cmocka_unit_test(test_phase_consistency_after_errors),
        cmocka_unit_test(test_bind_packet_address_extraction_edge_cases),
        cmocka_unit_test(test_data_packet_channel_extraction_comprehensive),
#endif
#ifdef SYMAX_CONFIG_TX
        cmocka_unit_test(test_channel_wraparound),
        cmocka_unit_test(test_large_packet_counts),
#if !SYMAX_SKIP_PREBIND
        cmocka_unit_test(test_prebind_packet_hopping_timing),
#endif
#endif

        /* UTILITY AND EDGE CASE TESTS */
        cmocka_unit_test(test_checksum_calculation_edge_cases),
        cmocka_unit_test(test_channel_data_union_access),
#if SYMAX_ENABLE_FLAGS
#endif
    /* INTEGRATION AND ROUND-TRIP */
#if defined(SYMAX_CONFIG_TX) && defined(SYMAX_CONFIG_RX)
        cmocka_unit_test(test_complete_tx_rx_binding_flow),
        cmocka_unit_test(test_encode_decode_roundtrip_comprehensive),
#if SYMAX_ENABLE_FLAGS
        cmocka_unit_test(test_flags_roundtrip),
#endif
        cmocka_unit_test(test_frequency_hopping_synchronization),
        cmocka_unit_test(test_complete_protocol_stress_test),
#endif

    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}
