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

#define CRSF_MAX_PARAM_STRING_LENGTH 20

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
 * Command inner payload
 */
typedef union {
    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        int64_t cur, min, max;
        char units[5];
    } i; // int-like

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        int32_t cur, min, max;
        uint8_t precision;
        uint32_t step;
        char units[5];
    } f; // fixed-point float

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        uint8_t current_index;
        char options[CRSF_MAX_PARAM_STRING_LENGTH];
        char units[5];
    } sel; // select

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        char value[CRSF_MAX_PARAM_STRING_LENGTH];
        uint8_t has_max_len;
        uint8_t max_len;
    } str; // string

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        uint8_t children[CRSF_MAX_PARAM_STRING_LENGTH];
        uint8_t child_count;
    } folder; // folder

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
        char text[CRSF_MAX_PARAM_STRING_LENGTH];
    } info; // info

    struct {
        uint8_t hidden;                          // from high bit of type byte
        uint8_t parent;                          // parent field index; 0xFF for root
        char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
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
} crsf_p2b_rc;

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

static inline int crsf_find_nul(const uint8_t* buf, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (buf[i] == 0) {
            return (int)i;
        }
    }
    return -1;
}

// ---- Decoder -------------------------------------------------------------
// Decodes a 0x2B entry payload that starts at [parent]. On success, returns CRSF_OK and fills *out.
// Pointers in *out refer to 'payload' memory; keep it alive while using the struct.

static inline crsf_p2b_rc crsf_param2b_decode_from_parent(const uint8_t* payload, uint8_t len, CRSF_ParamEntry_t* out) {
    if (len < 3U) {
        return CRSF_ERROR_TYPE_LENGTH; // need at least parent + type + one name byte
    }

    uint8_t off = 0;
    uint8_t parent = payload[off++];
    CRSF_ParamType_t type = (CRSF_ParamType_t)(payload[off] & 0x7F); //do not increment here

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
            out->f.parent = parent;
            out->f.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->f.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->f.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;

            if (len < off + 4 * 3 + 1 + 4) {
                return CRSF_ERROR_TYPE_LENGTH; // cur,min,max,prec,step
            }
            out->f.cur = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.min = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.max = CRSF_unpackBE32(payload + off);
            off += 4;
            out->f.precision = payload[off++];
            out->f.step = CRSF_unpackBE32(payload + off);
            off += 4;
            strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->f.units, (char*)(payload + off), ((strLen > 5U) ? 5U : strLen) - 1U);
            out->f.units[4] = '\0';
            break;
        }

        case CRSF_PARAM_SELECT: {
            out->sel.parent = parent;
            out->sel.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->sel.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->sel.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            if (len < off + 1U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->sel.current_index = payload[off++];
            strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->sel.options, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->sel.options[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->sel.units, (char*)(payload + off), ((strLen > 5U) ? 5U : strLen) - 1U);
            out->sel.units[4] = '\0';
            break;
        }

        case CRSF_PARAM_STRING: {
            out->str.parent = parent;
            out->str.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->str.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->str.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->str.value, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->str.value[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            if (off < len) {
                out->str.has_max_len = true;
                out->str.max_len = payload[off++];
            } else {
                out->str.has_max_len = false;
                out->str.max_len = 0;
            }
            break;
        }

        case CRSF_PARAM_FOLDER: {
            out->folder.parent = parent;
            out->folder.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->folder.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->folder.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            out->folder.child_count = 0;
            if (len > off) {
                uint8_t rem = len - off;
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
            out->info.parent = parent;
            out->info.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->info.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->info.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->info.text, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->info.text[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            break;
        }

        case CRSF_PARAM_COMMAND: {
            out->cmd.parent = parent;
            out->cmd.hidden = (payload[off++] & 0x80) != 0;
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->cmd.name, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->cmd.name[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            off += strLen;
            if (len < off + 2U) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            out->cmd.status = (CRSF_ParamCommandStatus_t)payload[off++];
            out->cmd.timeout = payload[off++];
            uint8_t strLen = strlen((char*)(payload + off)) + 1U;
            strncpy(out->cmd.info, (char*)(payload + off), ((strLen > CRSF_MAX_PARAM_STRING_LENGTH) ? CRSF_MAX_PARAM_STRING_LENGTH : strLen) - 1U);
            out->cmd.info[CRSF_MAX_PARAM_STRING_LENGTH - 1U] = '\0';
            break;
        }

        default: return CRSF_ERROR_INVALID_FRAME;
    }
    return CRSF_OK;
}

// ---- Encoder -------------------------------------------------------------
// Encodes a 0x2B entry payload that starts at [parent] into outb (capacity out_cap).
// Returns number of bytes written, or a negative error code.

static inline int crsf_param2b_encode_from_parent(uint8_t* outb, size_t out_cap, const CRSF_ParamEntry_t* in) {
    size_t off = 0;
    if (out_cap < 2) {
        return CRSF_ERROR_TYPE_LENGTH; // need at least parent+type
    }
    outb[off++] = in->parent;
    outb[off++] = (uint8_t)((in->hidden ? 0x80 : 0) | (in->type & 0x7F));

    // name
    const char* name = in->name ? in->name : "";
    for (size_t i = 0; name[i]; ++i) {
        if (off >= out_cap) {
            return CRSF_ERROR_TYPE_LENGTH;
        }
        outb[off++] = (uint8_t)name[i];
    }
    if (off >= out_cap) {
        return CRSF_ERROR_TYPE_LENGTH;
    }
    outb[off++] = 0;

    switch (in->type) {
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
            if (off + 4 * 3 + 1 + 4 > out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            CRSF_packBE32(outb + off, (uint32_t)in->v.f.cur);
            off += 4;
            CRSF_packBE32(outb + off, (uint32_t)in->v.f.min);
            off += 4;
            CRSF_packBE32(outb + off, (uint32_t)in->v.f.max);
            off += 4;
            outb[off++] = in->v.f.precision;
            CRSF_packBE32(outb + off, in->v.f.step);
            off += 4;
            const char* u = in->v.f.units ? in->v.f.units : "";
            for (size_t i = 0; u[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)u[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_SELECT: {
            if (off + 1 > out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = in->v.sel.current_index;
            const char* opts = in->v.sel.options ? in->v.sel.options : "";
            for (size_t i = 0; opts[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)opts[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            const char* uu = in->v.sel.units ? in->v.sel.units : "";
            for (size_t i = 0; uu[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)uu[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_STRING: {
            const char* s = in->v.str.value ? in->v.str.value : "";
            for (size_t i = 0; s[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)s[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            if (in->v.str.has_max_len) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = in->v.str.max_len;
            }
            break;
        }

        case CRSF_PARAM_FOLDER: {
            const char* n = in->v.folder.name ? in->v.folder.name : "";
            for (size_t i = 0; n[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)n[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            if (in->v.folder.children && in->v.folder.child_count > 0) {
                if (off + in->v.folder.child_count + 1 > out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                for (size_t k = 0; k < in->v.folder.child_count; ++k) {
                    outb[off++] = in->v.folder.children[k];
                }
                outb[off++] = 0xFF; // terminator
            }
            break;
        }

        case CRSF_PARAM_INFO: {
            const char* t = in->v.info.text ? in->v.info.text : "";
            for (size_t i = 0; t[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)t[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_COMMAND: {
            if (off + 2 > out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = in->v.cmd.step;
            outb[off++] = in->v.cmd.timeout;
            const char* s = in->v.cmd.status ? in->v.cmd.status : "";
            for (size_t i = 0; s[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_ERROR_TYPE_LENGTH;
                }
                outb[off++] = (uint8_t)s[i];
            }
            if (off >= out_cap) {
                return CRSF_ERROR_TYPE_LENGTH;
            }
            outb[off++] = 0;
            break;
        }

        default: return CRSF_ERROR_INVALID_FRAME;
    }

    return (int)off;
}

#ifdef __cplusplus
}
#endif

#endif // CRSF_PARAM_2B_H
