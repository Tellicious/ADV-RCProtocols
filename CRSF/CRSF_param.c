// crsf_param_2b.h — single-header encoder/decoder for CRSF 0x2B "Parameter Settings (Entry)" frames
//
// Usage:
//   #define CRSF_PARAM2B_IMPLEMENTATION
//   #include "crsf_param_2b.h"
//
// This header EXPECTS payloads that begin at **parent**:
//   [parent][type|hidden][name...][type-specific payload]
// I.e., you have already stripped CRSF framing, type=0x2B, length, and any
// preceding `[index][chunks_remaining]` fields used by other transports.
//
// The decoder returns a normalized structure with a tagged union for the
// per-type fields. The encoder builds a payload from the same structure.
//
// Assumptions (matching de-facto behavior across stacks):
// - Multi-byte numbers are big-endian on the wire.
// - Strings are ASCII, NULL-terminated.
// - SELECT options are semicolon-separated ("Off;On;Auto").
// - FLOAT is fixed-point: int32 values with `precision` decimal places and `step`.
// - FOLDER may include an optional child-index list terminated by 0xFF. Decoder accepts both.
// - STRING may optionally include a max_len u8 after the value.
// - COMMAND carries: step u8, timeout u8 (10ms ticks), status string.
// - Hidden bit is the MSB of the type byte.
// - Decoder is zero-copy: string pointers reference the caller's buffer.
//
// MIT License
//
#ifndef CRSF_PARAM_2B_H
#define CRSF_PARAM_2B_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_MAX_PARAM_STRING_LENGTH    20
#define CRSF_MAX_PARAM_SETTINGS_PAYLOAD 56

// ---- Types ---------------------------------------------------------------

typedef enum {
    CRSF_PARAM_INT8 = 0x00,
    CRSF_PARAM_UINT8 = 0x01,
    CRSF_PARAM_INT16 = 0x02,
    CRSF_PARAM_UINT16 = 0x03,
    CRSF_PARAM_INT32 = 0x04,
    CRSF_PARAM_UINT32 = 0x05,
    CRSF_PARAM_INT64 = 0x06,
    CRSF_PARAM_UINT64 = 0x07,
    CRSF_PARAM_FLOAT = 0x08,
    CRSF_PARAM_SELECT = 0x09,
    CRSF_PARAM_STRING = 0x0A,
    CRSF_PARAM_FOLDER = 0x0B,
    CRSF_PARAM_INFO = 0x0C,
    CRSF_PARAM_COMMAND = 0x0D,
} CRSF_ParamType_t;

typedef enum {
    READY = 0,               //--> feedback
    START = 1,               //<-- input
    PROGRESS = 2,            //--> feedback
    CONFIRMATION_NEEDED = 3, //--> feedback
    CONFIRM = 4,             //<-- input
    CANCEL = 5,              //<-- input
    POLL = 6                 //<-- input
} CRSF_ParamCommandStatus_t;

/**
 * Parameter inner payload
 */
typedef union {
    struct {
        int64_t cur, min, max;
        char units[5];
    } i; // int-like

    struct {
        int32_t value, min, max, def;
        uint8_t precision;
        int32_t step;
        char units[5];
    } f; // fixed-point float

    struct {
        char options[CRSF_MAX_PARAM_STRING_LENGTH];
        uint8_t value, hasOptData, min, max, def;
        char units[5];
    } sel; // select

    struct {
        char value[CRSF_MAX_PARAM_STRING_LENGTH];
        uint8_t has_max_len;
        uint8_t max_len;
    } str; // string

    struct {
        uint8_t children[CRSF_MAX_PARAM_STRING_LENGTH];
        uint8_t child_count;
    } folder; // folder

    struct {
        char text[CRSF_MAX_PARAM_STRING_LENGTH];
    } info; // info

    struct {
        CRSF_ParamCommandStatus_t status;
        uint8_t timeout; // ms * 100
        char info[CRSF_MAX_PARAM_STRING_LENGTH];
    } cmd; // command
} CRSF_ParamEntry_t;

// ---- Error codes ---------------------------------------------------------

typedef enum {
    CRSF_OK = 0,
    CRSF_ERROR_TYPE_LENGTH = -1,   // not enough bytes
    CRSF_P2B_ERR_BADSTRING = -2,   // missing NULL terminator
    CRSF_ERROR_INVALID_FRAME = -3, // unknown type
} CRSF_Status_t;

// ---- Helpers: big-endian read/write -------------------------------------

static inline uint16_t CRSF_unpackBE16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

static inline uint32_t CRSF_unpackBE32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

static inline uint64_t CRSF_unpackBE64(const uint8_t* p) { return ((uint64_t)CRSF_unpackBE32(p) << 32) | CRSF_unpackBE32(p + 4); }

static inline void CRSF_packBE16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void CRSF_packBE32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline void CRSF_packBE64(uint8_t* p, uint64_t v) {
    CRSF_packBE32(p, (uint32_t)(v >> 32));
    CRSF_packBE32(p + 4, (uint32_t)v);
}

static inline uint8_t CRSF_unpackString(const uint8_t* payload, char* string, const uint8_t maxStringLength, const uint8_t maxPayloadLength) {
    uint8_t strLen = strnlen((char*)payload, maxPayloadLength) + 1U;
    strncpy(string, (char*)payload, ((strLen > maxStringLength) ? maxStringLength : strLen) - 1U);
    string[maxStringLength - 1U] = '\0';
    return strLen;
}

static inline uint8_t CRSF_packString(uint8_t* payload, const char* string, const uint8_t maxStringLength, const uint8_t maxPayloadLength) {
    uint8_t strLen = strnlen(string, maxStringLength - 1U);
    if ((strLen + 1U) > maxPayloadLength) {
        return CRSF_ERROR_TYPE_LENGTH;
    }
    memcpy(payload, string, strLen);
    payload[strLen] = '\0';
    return strLen + 1U;
}

// ---- Decoder -------------------------------------------------------------
// Decodes a 0x2B entry payload that starts at [parent]. On success, returns CRSF_OK and fills *out.
// Pointers in *out refer to 'payload' memory; keep it alive while using the struct.

static CRSF_Status_t CRSF_decodeParamEntry(CRSF_ParamType_t type, CRSF_ParamEntry_t* out, const uint8_t* payload, uint8_t length) {
    if (length < 3U) {
        return CRSF_ERROR_TYPE_LENGTH; // need at least parent + type + one name byte
    }
    uint8_t off = 0;
    switch (type) {
        case CRSF_PARAM_INT8:
        case CRSF_PARAM_UINT8:
        case CRSF_PARAM_INT16:
        case CRSF_PARAM_UINT16:
        case CRSF_PARAM_INT32:
        case CRSF_PARAM_UINT32:
        case CRSF_PARAM_INT64:
        case CRSF_PARAM_UINT64: //deprecated
            break;
        case CRSF_PARAM_FLOAT: {
            /*out->f.parent = parent;
            out->f.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->f.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->f.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;*/

            if (length < off + 4 * 4 + 1 + 4) {
                return CRSF_ERROR_TYPE_LENGTH; // cur,min,max,def,prec,step
            }
            out->f.value = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.min = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.max = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.def = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.precision = payload[off++];
            out->f.step = CRSF_unpackBE32(payload + off);
            off += 4;
            off += CRSF_unpackString(payload + off, out->f.units, 5U, length - off);
            //TODO remove
            //uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            //strncpy(out->f.units, (char*)(payload + off), ((strLen > 5U) ? 5U : strLen) - 1U);
            //out->f.units[4] = '\0';
            break;
        }

        case CRSF_PARAM_SELECT: {
            if (length < off + 2U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            off += CRSF_unpackString(payload + off, out->sel.options, CRSF_MAX_PARAM_STRING_LENGTH, length - off);
            out->sel.value = payload[off++];
            out->sel.hasOptData = 0U;
            if (off + 3U < length) {
                out->sel.hasOptData = 1U;
                out->sel.min = payload[off++];
                out->sel.max = payload[off++];
                out->sel.def = payload[off++];
                off += CRSF_unpackString(payload + off, out->sel.units, 5U, length - off);
            }
            //TODO remove
            //uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            //strncpy(out->sel.options, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            //out->sel.options[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            //off += strLen;
            //strLen = strlen((char*)(payload + off)) + 1U;
            //strncpy(out->sel.units, (char*)(payload + off), ((strLen > 5U) ? 5U : strLen) - 1U);
            //out->sel.units[4] = '\0';
            break;
        }

        case CRSF_PARAM_STRING: {
            //TODO remove
            // uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            // strncpy(out->str.value, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            // out->str.value[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            // off += strLen;
            off += CRSF_unpackString(payload + off, out->str.value, CRSF_MAX_PARAM_STRING_LENGTH, length - off);
            if (off < length) {
                out->str.has_max_len = true;
                out->str.max_len = payload[off++];
            } else {
                out->str.has_max_len = false;
                out->str.max_len = 0;
            }
            break;
        }

        case CRSF_PARAM_FOLDER: {
            out->folder.child_count = 0;
            if (off < length) {
                uint8_t rem = length - off;
                for (uint8_t ii = 0; ii < rem; ii++) {
                    if (payload[off] != 0xFF) {
                        break;
                    }
                    out->folder.children[ii] = payload[off++];
                    out->folder.child_count++;
                }
            }
            break;
        }

        case CRSF_PARAM_INFO: {
            off += CRSF_unpackString(payload + off, out->info.text, CRSF_MAX_PARAM_STRING_LENGTH, length - off);
            break;
        }

        case CRSF_PARAM_COMMAND: {
            if (length < off + 2U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->cmd.status = (CRSF_ParamCommandStatus_t)payload[off++];
            out->cmd.timeout = payload[off++];
            off += CRSF_unpackString(payload + off, out->cmd.info, CRSF_MAX_PARAM_STRING_LENGTH, length - off);
            //TODO remove
            //uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            //strncpy(out->cmd.info, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            //out->cmd.info[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            break;
        }

        default: return CRSF_ERROR_INVALID_FRAME;
    }
    return CRSF_OK;
}

// ---- Encoder -------------------------------------------------------------
// Encodes a 0x2B entry payload that starts at [parent] into payload (capacity out_cap).
// Returns number of bytes written, or a negative error code.

static CRSF_Status_t CRSF_encodeParamEntry(CRSF_ParamType_t type, const CRSF_ParamEntry_t* in, uint8_t* payload, uint8_t* length) {
    uint8_t off = 0;

    switch (type) {
        case CRSF_PARAM_INT8:
        case CRSF_PARAM_UINT8:
        case CRSF_PARAM_INT16:
        case CRSF_PARAM_UINT16:
        case CRSF_PARAM_INT32:
        case CRSF_PARAM_UINT32:
        case CRSF_PARAM_INT64:
        case CRSF_PARAM_UINT64: // deprecated
            break;

        case CRSF_PARAM_FLOAT: {
            CRSF_packBE32(payload + off, in->f.value);
            off += 4;
            CRSF_packBE32(payload + off, in->f.min);
            off += 4;
            CRSF_packBE32(payload + off, in->f.max);
            off += 4;
            CRSF_packBE32(payload + off, in->f.def);
            off += 4;
            payload[off++] = in->f.precision;
            CRSF_packBE32(payload + off, in->f.step);
            off += 4;
            off += CRSF_packString(payload + off, in->f.units, 5U, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            break;
        }

        case CRSF_PARAM_SELECT: {
            off += CRSF_packString(payload + off, in->sel.options, CRSF_MAX_PARAM_STRING_LENGTH, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            payload[off++] = in->sel.value;
            if ((in->sel.hasOptData == 1U) && (off + 3U * sizeof(uint8_t) + CRSF_MIN_STRING_LENGTH <= CRSF_MAX_PARAM_SETTINGS_PAYLOAD)) {
                payload[off++] = in->sel.min;
                payload[off++] = in->sel.max;
                payload[off++] = in->sel.def;
                off += CRSF_packString(payload + off, in->sel.units, 5U, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            }
            break;
        }

        case CRSF_PARAM_STRING: {
            off += CRSF_packString(payload + off, in->str.value, CRSF_MAX_PARAM_STRING_LENGTH, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            //TODO remove
            //uint8_t strLen = strnlen(in->str.value, CRSF_MAX_PARAM_STRING_LENGTH);
            //if ((strLen + 1U) > CRSF_MAX_PARAM_SETTINGS_PAYLOAD) {
            //    return CRSF_ERROR_TYPE_LENGTH;
            //}
            //memcpy(payload + off, in->str.value, strLen);
            //off += strLen;
            //payload[off++] = '\0';
            if (in->str.has_max_len) {
                payload[off++] = in->str.max_len;
            }
            break;
        }

        case CRSF_PARAM_FOLDER: {
            if ((in->folder.child_count + 1U) > CRSF_MAX_PARAM_SETTINGS_PAYLOAD) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            memcpy(payload + off, in->folder.children, in->folder.child_count);
            off += in->folder.child_count;
            payload[off++] = 0xFF;
            break;
        }

        case CRSF_PARAM_INFO: {
            off += CRSF_packString(payload + off, in->info.text, CRSF_MAX_PARAM_STRING_LENGTH, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            //TODO remove
            //uint8_t strLen = strnlen(in->info.text, CRSF_MAX_PARAM_STRING_LENGTH);
            //if ((strLen + 1U) > CRSF_MAX_PARAM_SETTINGS_PAYLOAD) {
            //    return CRSF_ERROR_TYPE_LENGTH;
            //}
            //memcpy(payload + off, in->info.text, strLen);
            //off += strLen;
            //payload[off++] = '\0';
            break;
        }

        case CRSF_PARAM_COMMAND: {
            payload[off++] = (uint8_t)in->cmd.status;
            payload[off++] = in->cmd.timeout;
            off += CRSF_packString(payload + off, in->cmd.info, CRSF_MAX_PARAM_STRING_LENGTH, CRSF_MAX_PARAM_SETTINGS_PAYLOAD - off);
            //TODO remove
            //uint8_t strLen = strnlen(in->cmd.info, CRSF_MAX_PARAM_STRING_LENGTH);
            //if ((strLen + 1U) > CRSF_MAX_PARAM_SETTINGS_PAYLOAD) {
            //    return CRSF_ERROR_TYPE_LENGTH;
            //}
            //memcpy(payload + off, in->cmd.info, strLen);
            //off += strLen;
            //payload[off++] = '\0';
            break;
        }

        default: return CRSF_ERROR_INVALID_FRAME;
    }

    *length += off;
    return CRSF_OK;
}

#ifdef __cplusplus
}
#endif

#endif // CRSF_PARAM_2B_H
