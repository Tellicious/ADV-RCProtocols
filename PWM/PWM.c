/* BEGIN Header */
/**
 ******************************************************************************
 * \file            PWM.c
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

/* Includes ------------------------------------------------------------------*/

#include <string.h>
#include "PWM.h"

/* Macros --------------------------------------------------------------------*/

/* Private Function Prototypes -----------------------------------------------*/

#if PWM_ENABLE_FRESHNESS_CHECK
static void PWM_updateTimestamp(PWM_t* PWM);
#endif

/* Functions -----------------------------------------------------------------*/

PWM_Status_t PWM_init(PWM_t* PWM, uint32_t timerFrequency, uint16_t timerAutoReload) {

    if (!PWM) {
        return PWM_ERROR_NULL_POINTER;
    }

    if (!timerFrequency) {
        return PWM_ERROR;
    }
    memset(PWM, 0x00, sizeof(*PWM));

    PWM->_timerAutoReload = timerAutoReload;
    PWM->_freqMultiplier = 1000000.f / timerFrequency;

    return PWM_OK;
}

#if PWM_ENABLE_FRESHNESS_CHECK
void PWM_setTimestampCallback(PWM_t* PWM, uint32_t (*getTimestamp_ms)(void)) {
    if (PWM) {
        PWM->getTimestamp_ms = getTimestamp_ms;
    }
}
#endif

PWM_Status_t PWM_processPacket(PWM_t* PWM, uint8_t channel, uint8_t rising, uint16_t timerCounter) {
    if (!PWM) {
        return PWM_ERROR_NULL_POINTER;
    }

    if (rising == 1U) {
        PWM->_riseTimer[channel] += timerCounter;
    } else {
        PWM->_fallTimer[channel] += timerCounter;
    }
    PWM->_pulseCounter[channel]++;

    if (PWM->_pulseCounter[channel] >= PWM_SAMPLES_NUM * 2U) {
        if (PWM->_fallTimer[channel] > PWM->_riseTimer[channel]) {
            PWM->channels[channel] = (PWM->_fallTimer[channel] - PWM->_riseTimer[channel]) * PWM->_freqMultiplier / PWM_SAMPLES_NUM;
        } else {
            PWM->channels[channel] =
                (PWM->_fallTimer[channel] - PWM->_riseTimer[channel] + PWM->_timerAutoReload + 1U) * PWM->_freqMultiplier / PWM_SAMPLES_NUM;
        }
        PWM->_riseTimer[channel] = 0;
        PWM->_fallTimer[channel] = 0;
        PWM->_pulseCounter[channel] = 0;
        PWM->_updatedChannels |= (1U << channel);
    }

    if (PWM->_updatedChannels == ((1 << PWM_MAX_CHANNELS) - 1U)) {
        PWM->_updatedChannels = 0;
#if PWM_ENABLE_STATS
        PWM->packets_total++;
#endif
#if PWM_ENABLE_FRESHNESS_CHECK
        PWM_updateTimestamp(PWM);
#endif
        return PWM_OK;
    }
    return PWM_WAIT;
}

#if PWM_ENABLE_STATS
void PWM_getStats(const PWM_t* PWM, uint32_t* packetsTotal) {
    if (!PWM || !packetsTotal) {
        return;
    }
    *packetsTotal = PWM->packets_total;
}

void PWM_resetStats(PWM_t* PWM) {
    if (!PWM) {
        return;
    }
    PWM->packets_total = 0;
}
#endif

#if PWM_ENABLE_FRESHNESS_CHECK
uint8_t PWM_isPacketFresh(const PWM_t* PWM, uint32_t max_age_ms) {
    if (!PWM || !PWM->getTimestamp_ms) {
        return 0;
    }

    return (PWM->getTimestamp_ms() - PWM->_packet_time) <= max_age_ms;
}
#endif

#if PWM_ENABLE_FRESHNESS_CHECK
static void PWM_updateTimestamp(PWM_t* PWM) {
    if (!PWM->getTimestamp_ms) {
        return;
    }

    PWM->_packet_time = PWM->getTimestamp_ms();
}
#endif