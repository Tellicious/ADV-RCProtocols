/* BEGIN Header */
/**
 ******************************************************************************
 * \file            iBus.c
 * \author          Andrea Vivani
 * \brief           FlySky iBUS (AFHDS 2A) – protocol decoder
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

#include "iBus.h"

/* Private Function Prototypes -----------------------------------------------*/

static inline uint16_t iBus_calcChecksum(const uint8_t* buf, uint8_t len);
#if IBUS_ENABLE_FRESHNESS_CHECK
static inline void iBus_updateTimestamp(iBus_t* iBus, uint8_t frame_type);
#endif
#if IBUS_ENABLE_TELEMETRY
static uint8_t iBus_getSensorSize(uint8_t type);
#endif

/* Functions -----------------------------------------------------------------*/
void iBus_init(iBus_t* iBus) {
    if (!iBus) {
        return;
    }
    memset(iBus, 0x00, sizeof(*iBus));
#if IBUS_ENABLE_TELEMETRY
    iBus->sensorCount = 1; // Reserve slot 0 for internal voltage
    iBus->sensors[0].type = IBUS_MEAS_TYPE_INTERNAL_VOLTAGE;
    iBus->sensors[0].size = iBus_getSensorSize(IBUS_MEAS_TYPE_INTERNAL_VOLTAGE);
    iBus->sensors[0].value = 0;
#endif
}

#if IBUS_ENABLE_FRESHNESS_CHECK
void iBus_setTimestampCallback(iBus_t* iBus, uint32_t (*getTimestamp_ms)(void)) {
    if (!iBus) {
        return;
    }
    iBus->getTimestamp_ms = getTimestamp_ms;
}
#endif

#if IBUS_ENABLE_TELEMETRY
iBus_Status_t iBus_registerSensor(iBus_t* iBus, const iBus_SensorType_t sensor) {
    if (!iBus) {
        return IBUS_ERROR_NULL_POINTER;
    }
    if (iBus->sensorCount >= IBUS_TEL_MAX_SENSORS) {
        return IBUS_ERROR_BUFFER_OVERFLOW;
    }

    for (uint8_t i = 0; i < iBus->sensorCount; i++) {
        if (iBus->sensors[i].type == sensor) {
            return IBUS_ERROR_SENSOR_EXISTS; /* already registered */
        }
    }

    // Register if not set
    iBus->sensors[iBus->sensorCount].type = sensor;
    iBus->sensors[iBus->sensorCount].size = iBus_getSensorSize(sensor); // Auto-assign

#if IBUS_ENABLE_FRESHNESS_CHECK
    if (iBus->getTimestamp_ms) {
        iBus->sensors[iBus->sensorCount].updated_ms = iBus->getTimestamp_ms();
    }
#endif

    iBus->sensorCount++;

    return IBUS_OK;
}
#endif /* IBUS_ENABLE_TELEMETRY */

iBus_Status_t iBus_processFrame(iBus_t* iBus, const uint8_t* frame) {
    if (!iBus || !frame) {
        return IBUS_ERROR_NULL_POINTER;
    }

#if IBUS_ENABLE_STATS && !IBUS_ENABLE_TELEMETRY
    iBus->stats.frames_total++;
#endif

    if (frame[0] != IBUS_SERVO_HDR0 || frame[1] != IBUS_SERVO_HDR1) {
        return IBUS_ERROR_INVALID_FRAME;
    }

    if (iBus_calcChecksum(frame, (uint8_t)(IBUS_SERVO_FRAME_LEN - 2)) != (uint16_t)(frame[IBUS_SERVO_FRAME_LEN - 2] | ((uint16_t)frame[IBUS_SERVO_FRAME_LEN - 1] << 8))) {
#if IBUS_ENABLE_STATS
        iBus->stats.frames_bad_crc++;
#endif
        return IBUS_ERROR_CHECKSUM_FAIL;
    }

    for (uint8_t ch = 0, i = 2; ch < 14 && ch < IBUS_MAX_CHANNELS; ch++, i += 2) {
        iBus->channels[ch] = (uint16_t)(frame[i] | ((uint16_t)frame[i + 1] << 8));
    }
#if IBUS_ENABLE_FRESHNESS_CHECK
    /* freshness update for RC */
    iBus_updateTimestamp(iBus, IBUS_FRAME_TYPE_RC);
#endif /* IBUS_ENABLE_FRESHNESS_CHECK */

#if IBUS_ENABLE_STATS
    iBus->stats.servo_frames_ok++;
#endif
    return IBUS_OK;
}

#if IBUS_ENABLE_TELEMETRY
iBus_Status_t iBus_handleTelemetryFromISR(iBus_t* iBus, const uint8_t* rx, uint8_t* tx, uint8_t* tx_len) {
    if (!iBus || !rx || !tx || !tx_len) {
        return IBUS_ERROR_NULL_POINTER;
    }

#if IBUS_ENABLE_STATS && IBUS_ENABLE_TELEMETRY
    iBus->stats.frames_total++;
#endif

    if (rx[0] != 0x04 && rx[0] != 0x06) {
        // This is not a telemetry frame
        return IBUS_TEL_NO_REPLY;
    }
    uint16_t csum = iBus_calcChecksum(rx, (uint8_t)(rx[0] - 2));
    if (csum != (uint16_t)(rx[rx[0] - 2] | ((uint16_t)rx[rx[0] - 1] << 8))) {
#if IBUS_ENABLE_STATS
        iBus->stats.frames_bad_crc++;
#endif
        return IBUS_ERROR_CHECKSUM_FAIL;
    }

    const uint8_t cmd = (uint8_t)(rx[1] & IBUS_TEL_CMD_MASK);
    const uint8_t addr = (uint8_t)(rx[1] & IBUS_TEL_ADDR_MASK);

    if (addr >= iBus->sensorCount) {
#if IBUS_ENABLE_STATS
        iBus->stats.polls_unknown_addr++;
#endif
        return IBUS_ERROR_SENSOR_NOT_FOUND; /* do not reply */
    }

    iBus_Sensor_t* s = &iBus->sensors[addr];
    uint8_t* p = tx;

    switch (cmd) {
        case IBUS_TEL_CMD_DISCOVER:
#if IBUS_ENABLE_STATS
            iBus->stats.polls_rx++;
#endif
#if IBUS_ENABLE_FRESHNESS_CHECK
            iBus_updateTimestamp(iBus, IBUS_FRAME_TYPE_TEL);
#endif
            memcpy_s(p, 4, rx, 4);
            *tx_len = 4; // Discovery reply is always 4 bytes
            return IBUS_TEL_REPLY_READY;
            break;
        case IBUS_TEL_CMD_TYPE:
#if IBUS_ENABLE_STATS
            iBus->stats.polls_rx++;
#endif
#if IBUS_ENABLE_FRESHNESS_CHECK
            iBus_updateTimestamp(iBus, IBUS_FRAME_TYPE_TEL);
#endif
            *p++ = 0x06;  /* 2B payload: type, size */
            *p++ = rx[1]; /* Command + address */
            *p++ = s->type;
            *p++ = s->size;
            csum = iBus_calcChecksum(tx, 4);
            *p++ = (uint8_t)(csum & 0xFF);
            *p++ = (uint8_t)(csum >> 8);
            *tx_len = 6;
            return IBUS_TEL_REPLY_READY;
            break;
        case IBUS_TEL_CMD_MEAS:
#if IBUS_ENABLE_STATS
            iBus->stats.polls_rx++;
#endif
#if IBUS_ENABLE_FRESHNESS_CHECK
            iBus_updateTimestamp(iBus, IBUS_FRAME_TYPE_TEL);
#endif
            *p++ = (uint8_t)(4U + s->size); /* Length of reply */
            *p++ = rx[1];                   /* Command + address */
            *p++ = (uint8_t)(s->value & 0xFF);
            *p++ = (uint8_t)((s->value >> 8) & 0xFF);
            if (s->size == 4U) {
                *p++ = (uint8_t)((s->value >> 16) & 0xFF);
                *p++ = (uint8_t)((s->value >> 24) & 0xFF);
            }
            csum = iBus_calcChecksum(tx, s->size + 2);
            *p++ = (uint8_t)(csum & 0xFF);
            *p++ = (uint8_t)(csum >> 8);
            *tx_len = s->size + 4;
            return IBUS_TEL_REPLY_READY;
            break;
        default: break;
    }
    return IBUS_TEL_NO_REPLY; /* unknown command */
}
#endif /* IBUS_ENABLE_TELEMETRY */

#if IBUS_ENABLE_STATS
void iBus_getStats(const iBus_t* iBus, iBus_Stats_t* stats) {
    if (!iBus || !stats) {
        return;
    }
    *stats = iBus->stats;
}

void iBus_resetStats(iBus_t* iBus) {
    if (!iBus) {
        return;
    }
    memset(&iBus->stats, 0x00, sizeof(iBus->stats));
}
#endif

#if IBUS_ENABLE_FRESHNESS_CHECK
uint8_t iBus_isFrameFresh(const iBus_t* iBus, uint8_t frame_type, uint32_t max_age_ms) {
    if (!iBus || !iBus->getTimestamp_ms) {
        return 0;
    }
    if (frame_type >= IBUS_TRACKED_FRAME_TYPES) {
        return 0;
    }

    return (iBus->getTimestamp_ms() - iBus->_packet_times[frame_type]) <= max_age_ms;
}
#endif

/* Private Functions ---------------------------------------------------------*/

static inline uint16_t iBus_calcChecksum(const uint8_t* buf, uint8_t len) {
    uint16_t s = 0xFFFFU;
    for (uint8_t i = 0; i < len; ++i) {
        s -= buf[i];
    }
    return s;
}

#if IBUS_ENABLE_FRESHNESS_CHECK
static inline void iBus_updateTimestamp(iBus_t* iBus, uint8_t frame_type) {

    if (iBus && iBus->getTimestamp_ms && frame_type < IBUS_TRACKED_FRAME_TYPES) {
        iBus->_packet_times[frame_type] = iBus->getTimestamp_ms();
    }
}
#endif /* IBUS_ENABLE_FRESHNESS_CHECK */

#if IBUS_ENABLE_TELEMETRY
static uint8_t iBus_getSensorSize(uint8_t type) {
    switch (type) {
        /* 4-byte defaults */
        case IBUS_MEAS_TYPE_GPS_LAT:
        case IBUS_MEAS_TYPE_GPS_LON: return 4;
        /* 2-byte defaults */
        default: return 2;
    }
}
#endif /* IBUS_ENABLE_TELEMETRY */
