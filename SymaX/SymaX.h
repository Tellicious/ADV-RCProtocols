/* BEGIN Header */
/**
 ******************************************************************************
 * \file            SymaX.h
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

#ifndef __SYMAX_H__
#define __SYMAX_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>

/* Configuration Options -----------------------------------------------------*/
// Configuration validation - ensure at least one mode is selected
#if !defined(SYMAX_CONFIG_RX) && !defined(SYMAX_CONFIG_TX)
#define SYMAX_CONFIG_RX
#endif

#ifdef SYMAX_CONFIG_TX
#undef SYMAX_ENABLE_FRESHNESS_CHECK
#define SYMAX_ENABLE_FRESHNESS_CHECK 0
#endif

#ifndef SYMAX_ENABLE_FLAGS
#define SYMAX_ENABLE_FLAGS 1
#endif

#ifndef SYMAX_ENABLE_TRIM_DATA
#define SYMAX_ENABLE_TRIM_DATA 0
#endif

#if SYMAX_ENABLE_TRIM_DATA
#undef SYMAX_ENABLE_FLAGS
#define SYMAX_ENABLE_FLAGS 1
#endif

#ifndef SYMAX_ENABLE_STATS
#define SYMAX_ENABLE_STATS 0
#endif

#ifndef SYMAX_ENABLE_FRESHNESS_CHECK
#define SYMAX_ENABLE_FRESHNESS_CHECK 0
#endif

#ifndef SYMAX_SKIP_PREBIND
#define SYMAX_SKIP_PREBIND 0
#endif

/* SymaX Protocol Constants --------------------------------------------------*/

#define SYMAX_PREBIND_PACKETS     300 // Packets sent by TX during pre-bind phase
#define SYMAX_BIND_PACKETS        345 // Packets sent by TX during binding phase
#define SYMAX_HOP_DATA_RATE       2   // Packets per channel before hop
#define SYMAX_PACKET_SIZE         10  // Size of SymaX packets

// Protocol phases
#define SYMAX_PREBIND             0
#define SYMAX_BIND_IN_PROGRESS    1
#define SYMAX_WAIT_FIRST_PACKET   2
#define SYMAX_DATA                3

// Packet types for freshness tracking
#define SYMAX_TRACKED_FRAME_TYPES 3
#define SYMAX_PACKET_TYPE_PREBIND 0
#define SYMAX_PACKET_TYPE_BIND    1
#define SYMAX_PACKET_TYPE_DATA    2

// Channel limits
#define SYMAX_CHANNEL_MAX         127  // Maximum channel value
#define SYMAX_CHANNEL_MIN         -127 // Minimum channel value

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return values
 */
typedef enum { SYMAX_OK = 0, SYMAX_BIND_COMPLETE, SYMAX_WAIT, SYMAX_ERROR_NULL_POINTER, SYMAX_ERROR_INVALID_PACKET, SYMAX_ERROR_CHECKSUM_FAIL } SymaX_Status_t;

/**
 * RC channels struct
 */
typedef union {

    struct {
        uint8_t thr;
        int8_t ele, rud, ail;
    };

    int8_t channels[4];
} SymaX_ChannelData_t;

/**
 * Flags struct
 */
#if SYMAX_ENABLE_FLAGS
typedef union {
    struct {
        uint8_t flip       : 1;
        uint8_t picture    : 1;
        uint8_t video      : 1;
        uint8_t headless   : 1;
        uint8_t xtrm_rates : 1; // Extended rates
        uint8_t __padding  : 3;
    };

    uint8_t flags;
} SymaX_Flags_t;
#endif

/**
 * Link data struct
 */
typedef struct {
    uint8_t rf_address[5];       // Transmitter ID (5 bytes)
    uint8_t rf_channels[4];      // Frequency hopping channels
    uint8_t phase;               // Compatible with RC.c phases
    uint8_t current_channel_idx; // Current channel index (0-3)
#ifdef SYMAX_CONFIG_TX
    uint16_t packet_count; // Packets on current channel
#else
    uint8_t packet_count; // Packets on current channel
#endif
} SymaX_LinkData_t;

/**
 * Statistics struct
 */
#if SYMAX_ENABLE_STATS
typedef struct {
    uint16_t packets_processed;
    uint16_t bind_packets;
    uint16_t data_packets;
    uint8_t checksum_failures;
} SymaX_Stats_t;
#endif

/**
 * SymaX struct
 */
typedef struct {

    SymaX_ChannelData_t channelsData;

#if SYMAX_ENABLE_FLAGS
    SymaX_Flags_t flags;
#endif

    SymaX_LinkData_t link;

#if SYMAX_ENABLE_STATS
    SymaX_Stats_t stats;
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
    uint32_t (*getTimestamp_ms)(void);
    uint32_t _packet_times[SYMAX_TRACKED_FRAME_TYPES];
#endif
} SymaX_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Initialize SymaX decoder
 * 
 * \param[in]       SymaX: SymaX decoder
 */
void SymaX_init(SymaX_t* SymaX);

#if SYMAX_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Set timestamp callback for packet timestamping
 * 
 * \param[in]       SymaX: SymaX decoder
 * \param[in]       getTimestamp_ms: Function pointer to get current timestamp in milliseconds
 */
void SymaX_setTimestampCallback(SymaX_t* SymaX, uint32_t (*getTimestamp_ms)(void));
#endif

#ifdef SYMAX_CONFIG_TX
/**
 * \brief           Build a SymaX data packet
 * 
 * \param[in]       SymaX: SymaX decoder
 * \param[out]      packet: Output packet buffer (10 bytes)
 * 
 * \return          SYMAX_OK if packet is valid, SYMAX_BIND_COMPLETE if it is needed to change channel and address, SYMAX_WAIT if binding is in progress, an error otherwise
 */
SymaX_Status_t SymaX_buildPacket(SymaX_t* SymaX, uint8_t* packet);
#endif

#ifdef SYMAX_CONFIG_RX
/**
 * \brief           Process received RF packet
 * 
 * \param[in]       SymaX: SymaX decoder
 * \param[in]       packet: RF packet data (10 bytes)
 *
 * \return          SYMAX_OK if packet is valid, SYMAX_BIND_COMPLETE if it is needed to change channel and address, SYMAX_WAIT if binding is in progress, an error otherwise
 */
SymaX_Status_t SymaX_processPacket(SymaX_t* SymaX, const uint8_t* packet);
#endif

/**
 * \brief Check if binding is complete
 * 
 * \param[in] SymaX : SymaX structure
 * 
 * \return 1 if bound, 0 if not bound 
 */
uint8_t SymaX_isBound(const SymaX_t* SymaX);

/**
 * \brief           Get current RF address
 * 
 * \param[in]       SymaX: SymaX structure
 * \param[out]      address: Output buffer for RF address (5 bytes)
 * 
 * \return          SYMAX_OK if address is valid, SYMAX_ERROR_NULL_POINTER if SymaX or address is NULL
 */
#define SymaX_getAddress(SymaX, address) memcpy(address, (SymaX)->link.rf_address, 5)

/**
 * \brief           Get current RF channel
 * 
 * \param[in]       SymaX: SymaX structure
 * 
 * \return          Current RF channel number
 */
#if SYMAX_SKIP_PREBIND
#define SymaX_getCurrentChannel(SymaX) ((SymaX)->link.rf_channels[(SymaX)->link.current_channel_idx])
#else
#define SymaX_getCurrentChannel(SymaX) (((SymaX)->link.phase == SYMAX_PREBIND) ? (0x8) : ((SymaX)->link.rf_channels[(SymaX)->link.current_channel_idx]))
#endif

#if SYMAX_ENABLE_STATS
/**
 * \brief           Get statistics about the decoder
 * 
 * \param[in]       SymaX: SymaX decoder
 * \param[out]      stats: Output statistics structure
 */
void SymaX_getStats(const SymaX_t* SymaX, SymaX_Stats_t* stats);

/**
 * \brief           Reset statistics counters
 * 
 * \param[out]      SymaX: SymaX decoder
 */
void SymaX_resetStats(SymaX_t* SymaX);
#endif

#if SYMAX_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Check if a packet type was received recently
 * 
 * \param[in]       SymaX: SymaX decoder
 * \param[in]       packet_type: Packet type to check (0=prebind, 1=bind, 2=data)
 * \param[in]       max_age_ms: Maximum age in milliseconds
 * 
 * \return          1 if packet is fresh, 0 if stale or never received
 */
uint8_t SymaX_isPacketFresh(const SymaX_t* SymaX, uint8_t packet_type, uint32_t max_age_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SYMAX_H__ */
