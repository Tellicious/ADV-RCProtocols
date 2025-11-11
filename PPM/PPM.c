/* BEGIN Header */
/**
 ******************************************************************************
 * \file            PPM.c
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

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "PPM.h"

/* Macros --------------------------------------------------------------------*/

/* Private Function Prototypes -----------------------------------------------*/

#if PPM_ENABLE_FRESHNESS_CHECK
static void PPM_updateTimestamp(PPM_t* PPM);
#endif

/* Functions -----------------------------------------------------------------*/

PPM_Status_t PPM_init(PPM_t* PPM, uint32_t timerFrequency, uint16_t timerAutoReload) {

    if (!PPM) {
        return PPM_ERROR_NULL_POINTER;
    }

    if (!timerFrequency) {
        return PPM_ERROR;
    }
    memset(PPM, 0x00, sizeof(*PPM));

    PPM->_timerAutoReload = timerAutoReload;
    PPM->_freqMultiplier = 1000000.f / timerFrequency;
    PPM->_packetEndTicks = PPM_PACKET_END_US / PPM->_freqMultiplier;

    return PPM_SUCCESS;
}

#if PPM_ENABLE_FRESHNESS_CHECK
void PPM_setTimestampCallback(PPM_t* PPM, uint32_t (*getTimestamp_ms)(void)) {
    if (PPM) {
        PPM->getTimestamp_ms = getTimestamp_ms;
    }
}
#endif

PPM_Status_t PPM_processPacket(PPM_t* PPM, uint16_t timerCounter) {
    if (!PPM) {
        return PPM_ERROR_NULL_POINTER;
    }
    volatile int32_t pulseLength = timerCounter - PPM->_lastTimerCounter;

    if (timerCounter < PPM->_lastTimerCounter) {
        pulseLength += PPM->_timerAutoReload + 1;
    }
    PPM->_lastTimerCounter = timerCounter;

    if (pulseLength > PPM->_packetEndTicks) {
        if (PPM->_currentChannel < PPM_MAX_CHANNELS) {
            PPM->_currentChannel = 0;
#if PPM_ENABLE_STATS
            PPM->packets_total++;
#endif
#if PPM_ENABLE_FRESHNESS_CHECK
            PPM_updateTimestamp(PPM);
#endif
            return PPM_SUCCESS;
        }
        PPM->_currentChannel = 0;
        return PPM_WAIT;
    } else if (PPM->_currentChannel >= PPM_MAX_CHANNELS) {
        PPM->_currentChannel++;
        return PPM_WAIT;
    }

    PPM->channels[PPM->_currentChannel] = pulseLength * PPM->_freqMultiplier;
    PPM->_currentChannel++;

    if (PPM->_currentChannel == PPM_MAX_CHANNELS) {
#if PPM_ENABLE_STATS
        PPM->packets_total++;
#endif
#if PPM_ENABLE_FRESHNESS_CHECK
        PPM_updateTimestamp(PPM);
#endif
        return PPM_SUCCESS;
    }

    return PPM_WAIT;
}

#if PPM_ENABLE_STATS
void PPM_getStats(const PPM_t* PPM, uint32_t* packetsTotal) {
    if (!PPM || !packetsTotal) {
        return;
    }
    *packetsTotal = PPM->packets_total;
}

void PPM_resetStats(PPM_t* PPM) {
    if (!PPM) {
        return;
    }
    PPM->packets_total = 0;
}
#endif

#if PPM_ENABLE_FRESHNESS_CHECK
uint8_t PPM_isPacketFresh(const PPM_t* PPM, uint32_t max_age_ms) {
    if (!PPM || !PPM->getTimestamp_ms) {
        return 0;
    }

    return (PPM->getTimestamp_ms() - PPM->_packet_time) <= max_age_ms;
}
#endif

#if PPM_ENABLE_FRESHNESS_CHECK
static void PPM_updateTimestamp(PPM_t* PPM) {
    if (!PPM->getTimestamp_ms) {
        return;
    }

    PPM->_packet_time = PPM->getTimestamp_ms();
}
#endif