/* BEGIN Header */
/**
 ******************************************************************************
 * \file            PPM.h
 * \author          Andrea Vivani
 * \brief           PPM protocol decoder
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

#ifndef __PPM_H__
#define __PPM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>

/* Configuration Options -----------------------------------------------------*/

#ifndef PPM_ENABLE_STATS
#define PPM_ENABLE_STATS 0
#endif

#ifndef PPM_ENABLE_FRESHNESS_CHECK
#define PPM_ENABLE_FRESHNESS_CHECK 0
#endif

#ifndef PPM_MAX_CHANNELS
#define PPM_MAX_CHANNELS 8U
#endif

#ifndef PPM_PACKET_END_US
#define PPM_PACKET_END_US 2500U
#endif

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return values
 */
typedef enum {
    PPM_OK = 0,             /** Operation completed successfully */
    PPM_ERROR_NULL_POINTER, /** Null pointer provided */
    PPM_WAIT,               /** Data reception in progress */
    PPM_ERROR,              /** Invalid packet length or format */
} PPM_Status_t;

/**
 * PPM struct
 */
typedef struct {
    uint16_t channels[PPM_MAX_CHANNELS]; /** Raw PPM channel values, typically ~1000–2000 range (µs equivalent). */

#if PPM_ENABLE_STATS
    uint32_t packets_total; /** Total packets parsed */
#endif
#if PPM_ENABLE_FRESHNESS_CHECK
    uint32_t (*getTimestamp_ms)(void);
    uint32_t _packet_time;
#endif

    float _freqMultiplier;
    uint16_t _timerAutoReload, _packetEndTicks, _lastTimerCounter;
    uint8_t _currentChannel;

} PPM_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Initialize PPM decoder
 * 
 * \param[in]       PPM: PPM decoder
 * \param[in]       timerFrequency: Timer frequency in Hz
 * \param[in]       timerAutoReload: Timer auto-reload value
 * 
 * \return          PPM_OK if initialization succesful, PPM_ERROR_NULL_POINTER if pointer is invalid, PPM_ERROR if timerFrequency in invalid
 */
PPM_Status_t PPM_init(PPM_t* PPM, uint32_t timerFrequency, uint16_t timerAutoReload);

#if PPM_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Set timestamp callback for packet timestamping
 * 
 * \param[in]       PPM: PPM decoder
 * \param[in]       getTimestamp_ms: Function pointer to get current timestamp in milliseconds
 */
void PPM_setTimestampCallback(PPM_t* PPM, uint32_t (*getTimestamp_ms)(void));
#endif

/**
 * \brief           Process received RF packet
 * 
 * \param[in]       PPM: PPM decoder
 * \param[in]       timerCounter: Current timer counter value at edge detection
 *
 * \return          PPM_OK if packet is valid, PPM_WAIT if reception is in progress, an error otherwise
 */
PPM_Status_t PPM_processPacket(PPM_t* PPM, uint16_t timerCounter);

#if PPM_ENABLE_STATS
/**
 * \brief           Get statistics about the decoder
 * 
 * \param[in]       PPM: PPM decoder
 * \param[out]      packetsTotal: Number of total packets received
 */
void PPM_getStats(const PPM_t* PPM, uint32_t* packetsTotal);

/**
 * \brief           Reset statistics counters
 * 
 * \param[out]      PPM: PPM decoder
 */
void PPM_resetStats(PPM_t* PPM);
#endif

#if PPM_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Check if a packet type was received recently
 * 
 * \param[in]       PPM: PPM decoder
 * \param[in]       max_age_ms: Maximum age in milliseconds
 * 
 * \return          1 if packet is fresh, 0 if stale or never received
 */
uint8_t PPM_isPacketFresh(const PPM_t* PPM, uint32_t max_age_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PPM_H__ */
