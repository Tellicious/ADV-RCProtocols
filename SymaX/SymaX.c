/* BEGIN Header */
/**
 ******************************************************************************
 * \file            SymaX.c
 * \author          Andrea Vivani
 * \brief           SymaX protocol decoder/encoder
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

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "SymaX.h"

/* Macros --------------------------------------------------------------------*/

// Flags
#define SYMAX_FLAG_FLIP     0x40
#define SYMAX_FLAG_PICTURE  0x40
#define SYMAX_FLAG_VIDEO    0x80
#define SYMAX_FLAG_HEADLESS 0x80

/* Private Function Prototypes -----------------------------------------------*/

static uint8_t SymaX_calcChecksum(const uint8_t* data);
static void SymaX_calcRCChannels(SymaX_t* SymaX, uint8_t address);

#ifdef SYMAX_CONFIG_TX
static uint8_t SymaX_convertRCChannels(int8_t channel);
static void SymaX_calcTXAddr(SymaX_t* SymaX);
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
static void SymaX_updateTimestamp(SymaX_t* SymaX, uint8_t packet_type);
#endif

/* Functions -----------------------------------------------------------------*/

void SymaX_init(SymaX_t* SymaX) {
    const uint8_t SymaX_BindAddress[5] = {0xAB, 0xAC, 0xAD, 0xAE, 0xAF};
    const uint8_t SymaX_BindChannels[4] = {0x4B, 0x30, 0x40, 0x20};

    if (!SymaX) {
        return;
    }
    memset(SymaX, 0x00, sizeof(*SymaX));

    // Initialize to appropriate starting SymaX based on configuration
#if SYMAX_SKIP_PREBIND
    SymaX->link.phase = SYMAX_BIND_IN_PROGRESS;
#else
    SymaX->link.phase = SYMAX_PREBIND;
#endif

    // Initialize with bind channels
    SymaX->link.current_channel_idx = 0;
    memcpy(SymaX->link.rf_channels, SymaX_BindChannels, 4);
    memcpy(SymaX->link.rf_address, SymaX_BindAddress, 5);
}

#if SYMAX_ENABLE_FRESHNESS_CHECK
void SymaX_setTimestampCallback(SymaX_t* SymaX, uint32_t (*getTimestamp_ms)(void)) {
    if (SymaX) {
        SymaX->getTimestamp_ms = getTimestamp_ms;
    }
}
#endif

#ifdef SYMAX_CONFIG_TX
SymaX_Status_t SymaX_buildPacket(SymaX_t* SymaX, uint8_t* packet) {
    if (!SymaX || !packet) {
        return SYMAX_ERROR_NULL_POINTER;
    }

#if !SYMAX_SKIP_PREBIND
    uint8_t _SymaX_PrebindPacket[10] = {0xF9, 0x96, 0x82, 0x1B, 0x20, 0x08, 0x08, 0xF2, 0x7D, 0xEF};
#endif

    switch (SymaX->link.phase) {
#if !SYMAX_SKIP_PREBIND
        case SYMAX_PREBIND:
            memcpy(packet, _SymaX_PrebindPacket, SYMAX_PACKET_SIZE);

            // Send a finite number of bind packets
            if (++SymaX->link.packet_count >= SYMAX_PREBIND_PACKETS) {
                SymaX->link.packet_count = 0;
                // Transition to SYMAX_BIND_IN_PROGRESS phase
                SymaX->link.phase = SYMAX_BIND_IN_PROGRESS;
            }
            break;
#endif
        case SYMAX_BIND_IN_PROGRESS:
            if (SymaX->link.rf_address[0] == 0xAB && SymaX->link.rf_address[1] == 0xAC && SymaX->link.rf_address[2] == 0xAD && SymaX->link.rf_address[3] == 0xAE
                && SymaX->link.rf_address[4] == 0xAF) {
                SymaX_calcTXAddr(SymaX);
            }
            packet[0] = SymaX->link.rf_address[4];
            packet[1] = SymaX->link.rf_address[3];
            packet[2] = SymaX->link.rf_address[2];
            packet[3] = SymaX->link.rf_address[1];
            packet[4] = SymaX->link.rf_address[0];
            packet[5] = 0xAA;
            packet[6] = 0xAA;
            packet[7] = 0xAA;
            packet[8] = 0x00;
            packet[9] = SymaX_calcChecksum(packet);

#if SYMAX_ENABLE_STATS
            SymaX->stats.bind_packets++;
            SymaX->stats.packets_processed++;
#endif

            // Change channel every SYMAX_HOP_DATA_RATE packets
            if (++SymaX->link.packet_count % SYMAX_HOP_DATA_RATE == 0) {
                SymaX->link.current_channel_idx = (SymaX->link.current_channel_idx + 1) % 4;
            }

            // Send a finite number of bind packets
            if (SymaX->link.packet_count >= SYMAX_BIND_PACKETS) {
                SymaX->link.packet_count = 0;
                SymaX->link.current_channel_idx = 0;
                SymaX_calcRCChannels(SymaX, SymaX->link.rf_address[0]);
                // Transition to SYMAX_DATA phase
                SymaX->link.phase = SYMAX_DATA;
                return SYMAX_BIND_COMPLETE;
            }
            break;

        case SYMAX_DATA:
            packet[0] = SymaX->channelsData.thr;
            packet[1] = SymaX_convertRCChannels(SymaX->channelsData.ele);
            packet[2] = SymaX_convertRCChannels(SymaX->channelsData.rud);
            packet[3] = SymaX_convertRCChannels(SymaX->channelsData.ail);
            packet[5] = 0xC0; // always high rates (bit 7 is rate control)
#if SYMAX_ENABLE_FLAGS
            packet[4] = (SymaX->flags.video ? SYMAX_FLAG_VIDEO : 0x00) | (SymaX->flags.picture ? SYMAX_FLAG_PICTURE : 0x00);
            packet[6] = (SymaX->flags.flip ? SYMAX_FLAG_FLIP : 0x00);
            packet[7] = (SymaX->flags.headless ? SYMAX_FLAG_HEADLESS : 0x00);
            if (SymaX->flags.headless) {
                SymaX->flags.xtrm_rates = 0; // Extended rates not used in headless mode
            }
#else
            packet[4] = 0;
            packet[6] = 0;
            packet[7] = 0;
#endif
#if SYMAX_ENABLE_TRIM_DATA
            if (SymaX->flags.xtrm_rates) { // use trims to extend controls
                packet[5] |= (packet[1] >> 2);
                packet[6] |= (packet[2] >> 2);
                packet[7] |= (packet[3] >> 2);
            }
#endif
            packet[8] = 0x00;
            packet[9] = SymaX_calcChecksum(packet);

#if SYMAX_ENABLE_STATS
            SymaX->stats.data_packets++;
            SymaX->stats.packets_processed++;
#endif

            // Change channel every SYMAX_HOP_DATA_RATE packets
            if (++SymaX->link.packet_count >= SYMAX_HOP_DATA_RATE) {
                SymaX->link.packet_count = 0;
                SymaX->link.current_channel_idx = (SymaX->link.current_channel_idx + 1) % 4;
            }
            return SYMAX_SUCCESS;
            break;

        default: break;
    }

    return SYMAX_WAIT;
}
#endif

#ifdef SYMAX_CONFIG_RX
SymaX_Status_t SymaX_processPacket(SymaX_t* SymaX, const uint8_t* packet) {
    if (!SymaX) {
        return SYMAX_ERROR_NULL_POINTER;
    }
    if (!packet) {
        return SYMAX_ERROR_INVALID_PACKET;
    }

    // Validate checksum first
    if (SymaX->link.phase != SYMAX_PREBIND && SymaX_calcChecksum(packet) != packet[9]) {
#if SYMAX_ENABLE_STATS
        SymaX->stats.checksum_failures++;
#endif
        return SYMAX_ERROR_CHECKSUM_FAIL;
    }
#if !SYMAX_SKIP_PREBIND
    uint8_t _SymaX_PrebindPacket[10] = {0xF9, 0x96, 0x82, 0x1B, 0x20, 0x08, 0x08, 0xF2, 0x7D, 0xEF};
#endif

    switch (SymaX->link.phase) {
#if !SYMAX_SKIP_PREBIND
        case SYMAX_PREBIND:
            // Check for prebind packet signature
            if (memcmp(packet, _SymaX_PrebindPacket, SYMAX_PACKET_SIZE) == 0) {
#if SYMAX_ENABLE_FRESHNESS_CHECK
                SymaX_updateTimestamp(SymaX, SYMAX_PACKET_TYPE_PREBIND);
#endif
                SymaX->link.current_channel_idx = 0;
                SymaX->link.phase = SYMAX_BIND_IN_PROGRESS;
#if SYMAX_ENABLE_STATS
                SymaX->stats.packets_processed++;
#endif
            }
            break;
#endif
        case SYMAX_BIND_IN_PROGRESS:
            // Check for bind packet
            if ((packet[5] == 0xAA && packet[6] == 0xAA && packet[7] == 0xAA)) {
#if SYMAX_ENABLE_FRESHNESS_CHECK
                SymaX_updateTimestamp(SymaX, SYMAX_PACKET_TYPE_BIND);
#endif
                // Extract TX ID from bind packet (bytes 0-4)
                for (uint8_t i = 0; i < 5; i++) {
                    SymaX->link.rf_address[i] = packet[4 - i];
                }

                // Calculate frequency hopping channels based on TX ID
                SymaX_calcRCChannels(SymaX, SymaX->link.rf_address[0]);
                SymaX->link.current_channel_idx = 0;
                SymaX->link.phase = SYMAX_WAIT_FIRST_PACKET;
#if SYMAX_ENABLE_STATS
                SymaX->stats.bind_packets++;
                SymaX->stats.packets_processed++;
#endif
                return SYMAX_BIND_COMPLETE;
            }
            break;
        case SYMAX_WAIT_FIRST_PACKET:
            // Process data packet and check if we should move to data mode
            if (packet[5] != 0xAA || packet[6] != 0xAA || packet[7] != 0xAA) {
#if SYMAX_ENABLE_FRESHNESS_CHECK
                SymaX_updateTimestamp(SymaX, SYMAX_PACKET_TYPE_DATA);
#endif
                SymaX->link.packet_count = 1;
                SymaX->link.phase = SYMAX_DATA;
#if SYMAX_ENABLE_STATS
                SymaX->stats.data_packets++;
                SymaX->stats.packets_processed++;

#endif
            }
            break;
        case SYMAX_DATA:
// Regular data packet processing
#if SYMAX_ENABLE_FRESHNESS_CHECK
            SymaX_updateTimestamp(SymaX, SYMAX_PACKET_TYPE_DATA);
#endif
            // 0–3: channels (Throttle, Elev, Rudder, Aile)
            SymaX->channelsData.thr = packet[0];
            for (uint8_t ii = 1; ii < 4; ii++) {
                SymaX->channelsData.channels[ii] = ((packet[ii] & 0x80) ? -(int8_t)(packet[ii] & 0x7F) : (int8_t)(packet[ii] & 0x7F));
            }

#if SYMAX_ENABLE_FLAGS
            // byte 4: feature flags
            SymaX->flags.video = (packet[4] & SYMAX_FLAG_VIDEO) != 0;
            SymaX->flags.picture = (packet[4] & SYMAX_FLAG_PICTURE) != 0;

            // auto-flip is bit 6 of byte 6
            SymaX->flags.flip = (packet[6] & SYMAX_FLAG_FLIP) != 0;

            // headless is bit 7 of byte 7
            SymaX->flags.headless = (packet[7] & SYMAX_FLAG_HEADLESS) != 0;
            SymaX->flags.xtrm_rates = SymaX->flags.headless ? 0 : (((packet[5] & 0x3F) + (packet[6] & 0X3F) + (packet[7] & 0X3F)) != 0);
#endif

#if SYMAX_ENABLE_TRIM_DATA
            // 5–7: trims, used to extend controls (Elev, Rud, Aile)
            if (SymaX->flags.xtrm_rates) {
                for (uint8_t ii = 1, jj = 5; ii < 4; ii++, jj++) {
                    int16_t val = SymaX->channelsData.channels[ii] + ((packet[jj] & 0x20) ? -(int8_t)(packet[jj] & 0x1F) : (int8_t)(packet[jj] & 0x1F));
                    // Clamp to int8_t range
                    if (val > SYMAX_CHANNEL_MAX) {
                        val = SYMAX_CHANNEL_MAX;
                    } else if (val < SYMAX_CHANNEL_MIN) {
                        val = SYMAX_CHANNEL_MIN;
                    }
                    SymaX->channelsData.channels[ii] = (int8_t)val;
                }
            }
#endif

#if SYMAX_ENABLE_STATS
            SymaX->stats.data_packets++;
            SymaX->stats.packets_processed++;
#endif
            // Change channel every SYMAX_HOP_DATA_RATE packets
            if (++SymaX->link.packet_count >= SYMAX_HOP_DATA_RATE) {
                SymaX->link.packet_count = 0;
                SymaX->link.current_channel_idx = (SymaX->link.current_channel_idx + 1) % 4;
            }
            return SYMAX_SUCCESS;
        default:
            // Invalid phase, do nothing
            break;
    }
    return SYMAX_WAIT; // No error, but no action taken
}
#endif

uint8_t SymaX_isBound(const SymaX_t* SymaX) {
    if (!SymaX) {
        return 0;
    }
    return (SymaX->link.phase == SYMAX_DATA || SymaX->link.phase == SYMAX_WAIT_FIRST_PACKET);
}

#if SYMAX_ENABLE_STATS
void SymaX_getStats(const SymaX_t* SymaX, SymaX_Stats_t* stats) {
    if (!SymaX || !stats) {
        return;
    }
    *stats = SymaX->stats;
}

void SymaX_resetStats(SymaX_t* SymaX) {
    if (!SymaX) {
        return;
    }
    memset(&SymaX->stats, 0x00, sizeof(SymaX->stats));
}
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
uint8_t SymaX_isPacketFresh(const SymaX_t* SymaX, uint8_t packet_type, uint32_t max_age_ms) {
    if (!SymaX || !SymaX->getTimestamp_ms) {
        return 0;
    }

    // Calculate packet age
    if (packet_type >= SYMAX_TRACKED_FRAME_TYPES) {
        // Invalid packet type
        return 0;
    }
    return (SymaX->getTimestamp_ms() - SymaX->_packet_times[packet_type]) <= max_age_ms;
}
#endif

/* Private Functions ---------------------------------------------------------*/

static uint8_t SymaX_calcChecksum(const uint8_t* data) {
    // Checksum implementation
    uint8_t sum = data[0];
    for (uint8_t i = 1; i < SYMAX_PACKET_SIZE - 1; i++) {
        sum ^= data[i];
    }
    return sum + 0x55;
}

static void SymaX_calcRCChannels(SymaX_t* SymaX, uint8_t address) {
    static const uint8_t startChans_1[] = {0x0a, 0x1A, 0x2a, 0x3A};
    static const uint8_t startChans_2[] = {0x2a, 0x0A, 0x42, 0x22};
    static const uint8_t startChans_3[] = {0x1a, 0x3A, 0x12, 0x32};

    uint8_t laddress = address & 0x1F;
    uint8_t ii;
    uint32_t* pchans = (uint32_t*)SymaX->link.rf_channels; // avoid compiler warning

    if (laddress < 0x10) {
        if (laddress == 6) {
            laddress = 7;
        }
        for (ii = 0; ii < 4; ii++) {
            SymaX->link.rf_channels[ii] = startChans_1[ii] + laddress;
        }
    } else if (laddress < 0x18) {
        for (ii = 0; ii < 4; ii++) {
            SymaX->link.rf_channels[ii] = startChans_2[ii] + (laddress & 0x07);
        }
        if (laddress == 0x16) {
            SymaX->link.rf_channels[0] += 1;
            SymaX->link.rf_channels[1] += 1;
        }
    } else if (laddress < 0x1e) {
        for (ii = 0; ii < 4; ii++) {
            SymaX->link.rf_channels[ii] = startChans_3[ii] + (laddress & 0x07);
        }
    } else if (laddress == 0x1e) {
        *pchans = 0x38184121;
    } else {
        *pchans = 0x39194121;
    }
}

#ifdef SYMAX_CONFIG_TX
static uint8_t SymaX_convertRCChannels(int8_t channel) {
    if (channel < SYMAX_CHANNEL_MIN) {
        channel = SYMAX_CHANNEL_MIN;
    }
    return (uint8_t)((channel < 0 ? 0x80 : 0) | ((channel < 0) ? -(uint8_t)(channel) : (channel)));
}

static void MCU_SerialNumber(uint8_t* var, int len) {
// Every STM32 should have 12 bytes long unique id at 0x1FFFF7E8
#ifdef SYMAX_TEST_ID
    int l = len > 12 ? 12 : len;
    for (int i = 0; i < l; i++) {
        var[i] = 0xA0 + (i % 10); // Test ID, replace with actual unique ID
    }
#else
    const uint8_t* stm32id = (uint8_t*)0x1FFFF7E8;
    int l = len > 12 ? 12 : len;
    for (int i = 0; i < l; i++) {
        var[i] = *stm32id++;
    }
    while (l < len) {
        var[l++] = 0x00;
    }
#endif
}

// Linear feedback shift register with 32-bit Xilinx polinomial x^32 + x^22 + x^2 + x + 1
static const uint32_t LFSR_FEEDBACK = 0x80200003ul;
static const uint32_t LFSR_INTAP = 32 - 1;

static void update_lfsr(uint32_t* lfsr, uint8_t b) {
    for (int i = 0; i < 8; ++i) {
        *lfsr = (*lfsr >> 1) ^ ((-(*lfsr & 1u) & LFSR_FEEDBACK) ^ ~((uint32_t)(b & 1) << LFSR_INTAP));
        b >>= 1;
    }
}

static void SymaX_calcTXAddr(SymaX_t* SymaX) {
    uint32_t lfsr = 0xB2C54A2FUL;

    uint8_t var[12] = {0};
    MCU_SerialNumber(var, 12);
    for (int i = 0; i < 12; ++i) {
        update_lfsr(&lfsr, var[i]);
    }

    // Pump zero bytes for LFSR to diverge more
    for (uint8_t i = 0; i < sizeof(lfsr); ++i) {
        update_lfsr(&lfsr, 0);
    }

    SymaX->link.rf_address[4] = 0xa2;
    for (uint8_t i = 0; i < sizeof(SymaX->link.rf_address) - 1; ++i) {
        SymaX->link.rf_address[i] = lfsr & 0xff;
        update_lfsr(&lfsr, i);
    }
}
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
static void SymaX_updateTimestamp(SymaX_t* SymaX, uint8_t packet_type) {
    if (!SymaX->getTimestamp_ms) {
        return;
    }

    SymaX->_packet_times[packet_type] = SymaX->getTimestamp_ms();
}
#endif
