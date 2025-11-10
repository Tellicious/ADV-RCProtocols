/* BEGIN Header */
/**
 ******************************************************************************
 * \file            PWM.h
 * \author          Andrea Vivani
 * \brief           PWM protocol decoder
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

#ifndef __PWM_H__
#define __PWM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stddef.h>
#include <stdint.h>

/* Configuration Options -----------------------------------------------------*/

#ifndef PWM_ENABLE_STATS
#define PWM_ENABLE_STATS 0
#endif

#ifndef PWM_ENABLE_FRESHNESS_CHECK
#define PWM_ENABLE_FRESHNESS_CHECK 0
#endif

#ifndef PWM_MAX_CHANNELS
#define PWM_MAX_CHANNELS 4U
#endif

#ifndef PWM_SAMPLES_NUM
#define PWM_SAMPLES_NUM 2U
#endif

/* Typedefs ------------------------------------------------------------------*/

/**
 * Return values
 */
typedef enum {
    PWM_OK = 0,             /** Operation completed successfully */
    PWM_ERROR_NULL_POINTER, /** Null pointer provided */
    PWM_WAIT,               /** Data reception in progress */
    PWM_ERROR,              /** Invalid packet length or format */
} PWM_Status_t;

/**
 * PWM struct
 */
typedef struct {
    uint16_t channels[PWM_MAX_CHANNELS]; /** Raw PWM channel values, typically ~1000–2000 range (µs equivalent). */

#if PWM_ENABLE_STATS
    uint32_t packets_total; /** Total packets parsed */
#endif
#if PWM_ENABLE_FRESHNESS_CHECK
    uint32_t (*getTimestamp_ms)(void);
    uint32_t _packet_time;
#endif

    uint8_t _pulseCounter[PWM_MAX_CHANNELS];
    uint16_t _timerAutoReload, _lastTimerCounter, _updatedChannels;
    uint16_t _riseTimer[PWM_MAX_CHANNELS], _fallTimer[PWM_MAX_CHANNELS];
    float _freqMultiplier;
} PWM_t;

/* Function prototypes -------------------------------------------------------*/

/**
 * \brief           Initialize PWM decoder
 * 
 * \param[in]       PWM: PWM decoder
 * \param[in]       timerFrequency: Timer frequency in Hz
 * \param[in]       timerAutoReload: Timer auto-reload value
 * 
 * \return          PWM_OK if initialization succesful, PWM_ERROR_NULL_POINTER if pointer is invalid, PWM_ERROR if timerFrequency in invalid
 */
PWM_Status_t PWM_init(PWM_t* PWM, uint32_t timerFrequency, uint16_t timerAutoReload);

#if PWM_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Set timestamp callback for packet timestamping
 * 
 * \param[in]       PWM: PWM decoder
 * \param[in]       getTimestamp_ms: Function pointer to get current timestamp in milliseconds
 */
void PWM_setTimestampCallback(PWM_t* PWM, uint32_t (*getTimestamp_ms)(void));
#endif

/**
 * \brief           Process received RF packet
 * 
 * \param[in]       PWM: PWM decoder
 * \param[in]       channel: Current channel
 * \param[in]       rising: 1 if rising edge, 0 if falling edge
 * \param[in]       timerCounter: Current timer counter value at edge detection
 *
 * \return          PWM_OK if packet is valid, PWM_WAIT if reception is in progress, an error otherwise
 */
PWM_Status_t PWM_processPacket(PWM_t* PWM, uint8_t channel, uint8_t rising, uint16_t timerCounter);

#if PWM_ENABLE_STATS
/**
 * \brief           Get statistics about the decoder
 * 
 * \param[in]       PWM: PWM decoder
 * \param[out]      packetsTotal: Number of total packets received
 */
void PWM_getStats(const PWM_t* PWM, uint32_t* packetsTotal);

/**
 * \brief           Reset statistics counters
 * 
 * \param[out]      PWM: PWM decoder
 */
void PWM_resetStats(PWM_t* PWM);
#endif

#if PWM_ENABLE_FRESHNESS_CHECK
/**
 * \brief           Check if a packet type was received recently
 * 
 * \param[in]       PWM: PWM decoder
 * \param[in]       max_age_ms: Maximum age in milliseconds
 * 
 * \return          1 if packet is fresh, 0 if stale or never received
 */
uint8_t PWM_isPacketFresh(const PWM_t* PWM, uint32_t max_age_ms);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __PWM_H__ */
