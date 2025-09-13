// crsf_param_2b.h — single-header encoder/decoder for CRSF 0x2B "Parameter Settings (Entry)" frames
//
// Usage:
//   #define CRSF_PARAM2B_IMPLEMENTATION
//   #include "crsf_param_2b.h"
//
// This header EXPECTS payloads that begin at **parent**:
//   [parent][type|hidden][label...][type-specific payload]
// I.e., you have already stripped CRSF framing, type=0x2B, length, and any
// preceding `[index][chunks_remaining]` fields used by other transports.
//
// The decoder returns a normalized structure with a tagged union for the
// per-type fields. The encoder builds a payload from the same structure.
//
// Assumptions (matching de-facto behavior across stacks):
// - Multi-byte numbers are big-endian on the wire.
// - Strings are ASCII, NUL-terminated.
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

// ---- Types ---------------------------------------------------------------

typedef enum {
    CRSF_PARAM_T_INT8 = 0x00,
    CRSF_PARAM_T_UINT8 = 0x01,
    CRSF_PARAM_T_INT16 = 0x02,
    CRSF_PARAM_T_UINT16 = 0x03,
    CRSF_PARAM_T_INT32 = 0x04,
    CRSF_PARAM_T_UINT32 = 0x05,
    CRSF_PARAM_T_INT64 = 0x06,
    CRSF_PARAM_T_UINT64 = 0x07,
    CRSF_PARAM_T_FLOAT = 0x08,
    CRSF_PARAM_T_SELECT = 0x09,
    CRSF_PARAM_T_STRING = 0x0A,
    CRSF_PARAM_T_FOLDER = 0x0B,
    CRSF_PARAM_T_INFO = 0x0C,
    CRSF_PARAM_T_CMD = 0x0D,
} crsf_param_type_t;

// Unified parameter entry (normalized)

typedef struct {
    uint8_t parent;         // parent field index; 0xFF for root
    bool hidden;            // from high bit of type byte
    crsf_param_type_t type; // low 7 bits of type byte
    const char* label;      // NUL-terminated string in input buffer

    union {
        struct {
            int64_t cur, min, max;
            const char* units;
        } i; // int-like

        struct {
            int32_t cur, min, max;
            uint8_t precision;
            uint32_t step;
            const char* units;
        } f; // fixed-point float

        struct {
            uint8_t current_index;
            const char* options;
            const char* units;
        } sel; // select

        struct {
            const char* value;
            bool has_max_len;
            uint8_t max_len;
        } str; // string

        struct {
            const char* name;
            const uint8_t* children;
            size_t child_count;
        } folder; // folder

        struct {
            const char* text;
        } info; // info

        struct {
            uint8_t step;
            uint8_t timeout10ms;
            const char* status;
        } cmd; // command
    } v;
} crsf_param_entry_t;

// ---- Error codes ---------------------------------------------------------

typedef enum {
    CRSF_P2B_OK = 0,
    CRSF_P2B_ERR_SHORT = -1,     // not enough bytes
    CRSF_P2B_ERR_BADSTRING = -2, // missing NUL terminator
    CRSF_P2B_ERR_BADTYPE = -3,   // unknown type
} crsf_p2b_rc;

// ---- Helpers: big-endian read/write -------------------------------------

static inline uint16_t crsf_be16(const uint8_t* p) { return (uint16_t)((p[0] << 8) | p[1]); }

static inline uint32_t crsf_be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

static inline uint64_t crsf_be64(const uint8_t* p) { return ((uint64_t)crsf_be32(p) << 32) | crsf_be32(p + 4); }

static inline void crsf_wbe16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void crsf_wbe32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline void crsf_wbe64(uint8_t* p, uint64_t v) {
    crsf_wbe32(p, (uint32_t)(v >> 32));
    crsf_wbe32(p + 4, (uint32_t)v);
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
// Decodes a 0x2B entry payload that starts at [parent]. On success, returns CRSF_P2B_OK and fills *out.
// Pointers in *out refer to 'payload' memory; keep it alive while using the struct.

static inline crsf_p2b_rc crsf_param2b_decode_from_parent(const uint8_t* payload, size_t len, crsf_param_entry_t* out) {
    if (len < 3) {
        return CRSF_P2B_ERR_SHORT; // need at least parent + type + one label byte
    }

    size_t off = 0;
    out->parent = payload[off++];
    uint8_t type_h = payload[off++];
    out->hidden = (type_h & 0x80) != 0;
    out->type = (crsf_param_type_t)(type_h & 0x7F);

    if (off >= len) {
        return CRSF_P2B_ERR_SHORT;
    }
    int lz = crsf_find_nul(payload + off, len - off);
    if (lz < 0) {
        return CRSF_P2B_ERR_BADSTRING;
    }
    out->label = (const char*)(payload + off);
    off += (size_t)lz + 1; // skip label NUL

    switch (out->type) {
        case CRSF_PARAM_T_INT8:
        case CRSF_PARAM_T_UINT8:
        case CRSF_PARAM_T_INT16:
        case CRSF_PARAM_T_UINT16:
        case CRSF_PARAM_T_INT32:
        case CRSF_PARAM_T_UINT32:
        case CRSF_PARAM_T_INT64:
        case CRSF_PARAM_T_UINT64: {
            size_t w = (out->type == CRSF_PARAM_T_INT8 || out->type == CRSF_PARAM_T_UINT8)     ? 1
                       : (out->type == CRSF_PARAM_T_INT16 || out->type == CRSF_PARAM_T_UINT16) ? 2
                       : (out->type == CRSF_PARAM_T_INT32 || out->type == CRSF_PARAM_T_UINT32) ? 4
                                                                                               : 8;
            size_t need = w * 3; // cur,min,max
            if (off + need > len) {
                return CRSF_P2B_ERR_SHORT;
            }
            const uint8_t* p = payload + off;
            int64_t cur = 0, min = 0, max = 0;
            bool is_signed =
                (out->type == CRSF_PARAM_T_INT8 || out->type == CRSF_PARAM_T_INT16 || out->type == CRSF_PARAM_T_INT32 || out->type == CRSF_PARAM_T_INT64);
            switch (w) {
                case 1:
                    cur = is_signed ? (int8_t)p[0] : (uint8_t)p[0];
                    min = is_signed ? (int8_t)p[1] : (uint8_t)p[1];
                    max = is_signed ? (int8_t)p[2] : (uint8_t)p[2];
                    break;
                case 2:
                    cur = is_signed ? (int16_t)crsf_be16(p) : (uint16_t)crsf_be16(p);
                    min = is_signed ? (int16_t)crsf_be16(p + 2) : (uint16_t)crsf_be16(p + 2);
                    max = is_signed ? (int16_t)crsf_be16(p + 4) : (uint16_t)crsf_be16(p + 4);
                    break;
                case 4:
                    cur = is_signed ? (int32_t)crsf_be32(p) : (uint32_t)crsf_be32(p);
                    min = is_signed ? (int32_t)crsf_be32(p + 4) : (uint32_t)crsf_be32(p + 4);
                    max = is_signed ? (int32_t)crsf_be32(p + 8) : (uint32_t)crsf_be32(p + 8);
                    break;
                case 8:
                    cur = is_signed ? (int64_t)crsf_be64(p) : (uint64_t)crsf_be64(p);
                    min = is_signed ? (int64_t)crsf_be64(p + 8) : (uint64_t)crsf_be64(p + 8);
                    max = is_signed ? (int64_t)crsf_be64(p + 16) : (uint64_t)crsf_be64(p + 16);
                    break;
            }
            out->v.i.cur = cur;
            out->v.i.min = min;
            out->v.i.max = max;
            off += need;
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING; // expect units NUL
            }
            int uz = crsf_find_nul(payload + off, len - off);
            if (uz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.i.units = (const char*)(payload + off);
            off += (size_t)uz + 1;
            break;
        }

        case CRSF_PARAM_T_FLOAT: {
            if (off + 4 * 3 + 1 + 4 > len) {
                return CRSF_P2B_ERR_SHORT; // cur,min,max,prec,step
            }
            out->v.f.cur = (int32_t)crsf_be32(payload + off);
            off += 4;
            out->v.f.min = (int32_t)crsf_be32(payload + off);
            off += 4;
            out->v.f.max = (int32_t)crsf_be32(payload + off);
            off += 4;
            out->v.f.precision = payload[off++];
            out->v.f.step = crsf_be32(payload + off);
            off += 4;
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int uz = crsf_find_nul(payload + off, len - off);
            if (uz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.f.units = (const char*)(payload + off);
            off += (size_t)uz + 1;
            break;
        }

        case CRSF_PARAM_T_SELECT: {
            if (off + 1 > len) {
                return CRSF_P2B_ERR_SHORT;
            }
            out->v.sel.current_index = payload[off++];
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int oz = crsf_find_nul(payload + off, len - off);
            if (oz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.sel.options = (const char*)(payload + off);
            off += (size_t)oz + 1;
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int uz = crsf_find_nul(payload + off, len - off);
            if (uz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.sel.units = (const char*)(payload + off);
            off += (size_t)uz + 1;
            break;
        }

        case CRSF_PARAM_T_STRING: {
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int sz = crsf_find_nul(payload + off, len - off);
            if (sz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.str.value = (const char*)(payload + off);
            off += (size_t)sz + 1;
            if (off < len) {
                out->v.str.has_max_len = true;
                out->v.str.max_len = payload[off++];
            } else {
                out->v.str.has_max_len = false;
                out->v.str.max_len = 0;
            }
            break;
        }

        case CRSF_PARAM_T_FOLDER: {
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int nz = crsf_find_nul(payload + off, len - off);
            if (nz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.folder.name = (const char*)(payload + off);
            off += (size_t)nz + 1;
            if (off < len) {
                const uint8_t* p = payload + off;
                size_t cnt = 0;
                while (off < len && payload[off] != 0xFF) {
                    ++off;
                    ++cnt;
                }
                if (off < len && payload[off] == 0xFF) {
                    out->v.folder.children = p;
                    out->v.folder.child_count = cnt;
                    ++off;
                } else {
                    out->v.folder.children = NULL;
                    out->v.folder.child_count = 0;
                }
            } else {
                out->v.folder.children = NULL;
                out->v.folder.child_count = 0;
            }
            break;
        }

        case CRSF_PARAM_T_INFO: {
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int tz = crsf_find_nul(payload + off, len - off);
            if (tz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.info.text = (const char*)(payload + off);
            off += (size_t)tz + 1;
            break;
        }

        case CRSF_PARAM_T_CMD: {
            if (off + 2 > len) {
                return CRSF_P2B_ERR_SHORT;
            }
            out->v.cmd.step = payload[off++];
            out->v.cmd.timeout10ms = payload[off++];
            if (off >= len) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            int sz = crsf_find_nul(payload + off, len - off);
            if (sz < 0) {
                return CRSF_P2B_ERR_BADSTRING;
            }
            out->v.cmd.status = (const char*)(payload + off);
            off += (size_t)sz + 1;
            break;
        }

        default: return CRSF_P2B_ERR_BADTYPE;
    }

    (void)off; // consumed length available if needed by caller
    return CRSF_P2B_OK;
}

// ---- Encoder -------------------------------------------------------------
// Encodes a 0x2B entry payload that starts at [parent] into outb (capacity out_cap).
// Returns number of bytes written, or a negative error code.

static inline int crsf_param2b_encode_from_parent(uint8_t* outb, size_t out_cap, const crsf_param_entry_t* in) {
    size_t off = 0;
    if (out_cap < 2) {
        return CRSF_P2B_ERR_SHORT; // need at least parent+type
    }
    outb[off++] = in->parent;
    outb[off++] = (uint8_t)((in->hidden ? 0x80 : 0) | (in->type & 0x7F));

    // label
    const char* label = in->label ? in->label : "";
    for (size_t i = 0; label[i]; ++i) {
        if (off >= out_cap) {
            return CRSF_P2B_ERR_SHORT;
        }
        outb[off++] = (uint8_t)label[i];
    }
    if (off >= out_cap) {
        return CRSF_P2B_ERR_SHORT;
    }
    outb[off++] = 0;

    switch (in->type) {
        case CRSF_PARAM_T_INT8:
        case CRSF_PARAM_T_UINT8:
        case CRSF_PARAM_T_INT16:
        case CRSF_PARAM_T_UINT16:
        case CRSF_PARAM_T_INT32:
        case CRSF_PARAM_T_UINT32:
        case CRSF_PARAM_T_INT64:
        case CRSF_PARAM_T_UINT64: {
            size_t w = (in->type == CRSF_PARAM_T_INT8 || in->type == CRSF_PARAM_T_UINT8)     ? 1
                       : (in->type == CRSF_PARAM_T_INT16 || in->type == CRSF_PARAM_T_UINT16) ? 2
                       : (in->type == CRSF_PARAM_T_INT32 || in->type == CRSF_PARAM_T_UINT32) ? 4
                                                                                             : 8;
            if (off + w * 3 > out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            uint8_t* p = outb + off;
            switch (w) {
                case 1:
                    p[0] = (uint8_t)in->v.i.cur;
                    p[1] = (uint8_t)in->v.i.min;
                    p[2] = (uint8_t)in->v.i.max;
                    break;
                case 2:
                    crsf_wbe16(p, (uint16_t)in->v.i.cur);
                    crsf_wbe16(p + 2, (uint16_t)in->v.i.min);
                    crsf_wbe16(p + 4, (uint16_t)in->v.i.max);
                    break;
                case 4:
                    crsf_wbe32(p, (uint32_t)in->v.i.cur);
                    crsf_wbe32(p + 4, (uint32_t)in->v.i.min);
                    crsf_wbe32(p + 8, (uint32_t)in->v.i.max);
                    break;
                case 8:
                    crsf_wbe64(p, (uint64_t)in->v.i.cur);
                    crsf_wbe64(p + 8, (uint64_t)in->v.i.min);
                    crsf_wbe64(p + 16, (uint64_t)in->v.i.max);
                    break;
            }
            off += w * 3;
            const char* u = in->v.i.units ? in->v.i.units : "";
            for (size_t i = 0; u[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)u[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_T_FLOAT: {
            if (off + 4 * 3 + 1 + 4 > out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            crsf_wbe32(outb + off, (uint32_t)in->v.f.cur);
            off += 4;
            crsf_wbe32(outb + off, (uint32_t)in->v.f.min);
            off += 4;
            crsf_wbe32(outb + off, (uint32_t)in->v.f.max);
            off += 4;
            outb[off++] = in->v.f.precision;
            crsf_wbe32(outb + off, in->v.f.step);
            off += 4;
            const char* u = in->v.f.units ? in->v.f.units : "";
            for (size_t i = 0; u[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)u[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_T_SELECT: {
            if (off + 1 > out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = in->v.sel.current_index;
            const char* opts = in->v.sel.options ? in->v.sel.options : "";
            for (size_t i = 0; opts[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)opts[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            const char* uu = in->v.sel.units ? in->v.sel.units : "";
            for (size_t i = 0; uu[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)uu[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_T_STRING: {
            const char* s = in->v.str.value ? in->v.str.value : "";
            for (size_t i = 0; s[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)s[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            if (in->v.str.has_max_len) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = in->v.str.max_len;
            }
            break;
        }

        case CRSF_PARAM_T_FOLDER: {
            const char* n = in->v.folder.name ? in->v.folder.name : "";
            for (size_t i = 0; n[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)n[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            if (in->v.folder.children && in->v.folder.child_count > 0) {
                if (off + in->v.folder.child_count + 1 > out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                for (size_t k = 0; k < in->v.folder.child_count; ++k) {
                    outb[off++] = in->v.folder.children[k];
                }
                outb[off++] = 0xFF; // terminator
            }
            break;
        }

        case CRSF_PARAM_T_INFO: {
            const char* t = in->v.info.text ? in->v.info.text : "";
            for (size_t i = 0; t[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)t[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            break;
        }

        case CRSF_PARAM_T_CMD: {
            if (off + 2 > out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = in->v.cmd.step;
            outb[off++] = in->v.cmd.timeout10ms;
            const char* s = in->v.cmd.status ? in->v.cmd.status : "";
            for (size_t i = 0; s[i]; ++i) {
                if (off >= out_cap) {
                    return CRSF_P2B_ERR_SHORT;
                }
                outb[off++] = (uint8_t)s[i];
            }
            if (off >= out_cap) {
                return CRSF_P2B_ERR_SHORT;
            }
            outb[off++] = 0;
            break;
        }

        default: return CRSF_P2B_ERR_BADTYPE;
    }

    return (int)off;
}

#ifdef __cplusplus
}
#endif

#endif // CRSF_PARAM_2B_H
