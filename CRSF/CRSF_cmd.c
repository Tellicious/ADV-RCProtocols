// crsf_cmd_32.h — payload-only encoder/decoder for CRSF 0x32 “Direct Commands”
//
// Spec source: tbs-fpv/tbs-crsf-spec/crsf.md — sections “0x32 Direct Commands” and
// the child pages (Command ACK, FC Commands, Bluetooth Command, OSD Commands,
// VTX Commands, LED, General, Crossfire, Flow Control Frame, Screen Command).
// This header supports the **payload part only**, i.e. bytes after Command_ID.
// Assumptions satisfied by the caller:
//   - Frame CRC (0xD5) and optional Command_CRC8 (poly 0xBA) already verified.
//   - Command_ID (the 2nd-level identifier after 0x32) is already extracted.
//
// Usage:
//   #define CRSF_CMD32_IMPLEMENTATION
//   #include "crsf_cmd_32.h"
//
// Decoder:
//   crsf32_rc crsf32_decode_payload(uint8_t command_id, const uint8_t *payload, size_t len,
//                                   crsf32_packet_t *out);
// Encoder:
//   int        crsf32_encode_payload(uint8_t *out, size_t cap, const crsf32_packet_t *in);
//
// Notes:
// - Big-endian for multi-byte integers (per CRSF spec).
// - Strings are ASCII NUL-terminated; optional strings may be empty ("\0").
// - Where the spec marks fields as “packed” bitfields, helpers here pack them as
//   a 24-bit big-endian bit stream in the documented order (e.g. H(9),S(7),V(8)).
// - Unknown / reserved commands are preserved as opaque raw payloads.
// - The names of enums mirror the spec headings and subcommand labels.
//
// MIT License

#ifndef CRSF_CMD_32_H
#define CRSF_CMD_32_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// Command_ID (0x32.<Command_ID>) values — mirror spec section names
// ---------------------------------------------------------------------------

typedef enum {
    CRSF32_CMDID_COMMAND_ACK = 0xFF, // 0x32.0xFF Command ACK
    CRSF32_CMDID_FC = 0x01,          // 0x32.0x01 FC Commands
    CRSF32_CMDID_BLUETOOTH = 0x03,   // 0x32.0x03 Bluetooth Command
    CRSF32_CMDID_OSD = 0x05,         // 0x32.0x05 OSD Commands
    CRSF32_CMDID_VTX = 0x08,         // 0x32.0x08 VTX Commands
    CRSF32_CMDID_LED = 0x09,         // 0x32.0x09 LED
    CRSF32_CMDID_GENERAL = 0x0A,     // 0x32.0x0A General
    CRSF32_CMDID_CROSSFIRE = 0x10,   // 0x32.0x10 Crossfire
    CRSF32_CMDID_RESERVED_12 = 0x12, // 0x32.0x12 Reserved
    CRSF32_CMDID_FLOW_CTRL = 0x20,   // 0x32.0x20 Flow Control Frame
    CRSF32_CMDID_SCREEN = 0x22       // 0x32.0x22 Screen Command
} CRSF_CommandID_t;

// ---------------------------------------------------------------------------
// Subcommand enums (per Command_ID)
// ---------------------------------------------------------------------------

typedef enum {                     // 0x32.0x01 FC Commands
    CRSF32_FC_FORCE_DISARM = 0x01, // Force Disarm
    CRSF32_FC_SCALE_CHANNEL = 0x02 // Scale Channel (payload unspecified in spec)
} CRSF_CommandFC_subCMD_t;

typedef enum {               // 0x32.0x03 Bluetooth Command
    CRSF32_BT_RESET = 0x01,  // Reset
    CRSF32_BT_ENABLE = 0x02, // Enable (uint8 Enable: 0/1)
    CRSF32_BT_ECHO = 0x64    // Echo
} CRSF_CommandBT_subCMD_t;

typedef enum {                     // 0x32.0x05 OSD Commands
    CRSF32_OSD_SEND_BUTTONS = 0x01 // Send Buttons (uint8 Buttons bitwise)
} CRSF_CommandOSD_subCMD_t;

typedef enum {                               // 0x32.0x08 VTX Commands
    CRSF32_VTX_CHANGE_CHANNEL = 0x01,        // DISCONTINUED
    CRSF32_VTX_SET_FREQUENCY = 0x02,         // uint16 Frequency (MHz in range 5000–6000)
    CRSF32_VTX_CHANGE_POWER = 0x03,          // DISCONTINUED (moved to 0x08)
    CRSF32_VTX_ENABLE_PITMODE_ON_PUP = 0x04, // packed pitmode byte
    CRSF32_VTX_POWER_UP_FROM_PITMODE = 0x05, // bare command
    CRSF32_VTX_SET_DYNAMIC_POWER = 0x06,     // uint8 Power (dBm)
    CRSF32_VTX_SET_POWER = 0x08              // uint8 Power (dBm)
} CRSF_CommandVTX_subCMD_t;

typedef enum { // 0x32.0x09 LED
    CRSF32_LED_SET_TO_DEFAULT = 0x01,
    CRSF32_LED_OVERRIDE_COLOR = 0x02, // packed HSV
    CRSF32_LED_OVERRIDE_PULSE = 0x03, // uint16 duration + HSV(start) + HSV(stop)
    CRSF32_LED_OVERRIDE_BLINK = 0x04, // uint16 interval + HSV(start) + HSV(stop)
    CRSF32_LED_OVERRIDE_SHIFT = 0x05  // uint16 interval + HSV
} CRSF_CommandLED_subCMD_t;

typedef enum { // 0x32.0x0A General
    // 0x04 and 0x61 are reserved in the spec
    CRSF32_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL = 0x70,         // uint8 port_id + uint32 proposed_baudrate
    CRSF32_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE = 0x71 // uint8 port_id + bool response
} CRSF_CommandGen_subCMD_t;

typedef enum { // 0x32.0x10 Crossfire
    CRSF32_CF_SET_RX_BIND_MODE = 0x01,
    CRSF32_CF_CANCEL_BIND_MODE = 0x02,
    CRSF32_CF_SET_BIND_ID = 0x03,         // payload not detailed by spec
    CRSF32_CF_MODEL_SELECTION = 0x05,     // uint8 Model Number
    CRSF32_CF_CURRENT_MODEL_QUERY = 0x06, // query
    CRSF32_CF_CURRENT_MODEL_REPLY = 0x07  // uint8 Model Number
    // 0x08, 0x09 reserved
} CRSF_CommandCF_subCMD_t;

typedef enum {                     // 0x32.0x20 Flow Control Frame
    CRSF32_FLOW_SUBSCRIBE = 0x01,  // uint8 Frame type + uint16 Max interval time (ms)
    CRSF32_FLOW_UNSUBSCRIBE = 0x02 // uint8 Frame type
} CRSF_CommandFlow_subCMD_t;

typedef enum { // 0x32.0x22 Screen Command
    CRSF32_SCREEN_POPUP_MESSAGE_START = 0x01,
    CRSF32_SCREEN_SELECTION_RETURN = 0x02
    // 0x03, 0x04 reserved
} CRSF_CommandScreen_subCMD_t;

// ---------------------------------------------------------------------------
// Data structures for decoded packets
// ---------------------------------------------------------------------------

typedef enum {
    CRSF32_KIND_UNKNOWN = 0,
    CRSF32_KIND_COMMAND_ACK,
    // FC
    CRSF32_KIND_FC_FORCE_DISARM,
    CRSF32_KIND_FC_SCALE_CHANNEL,
    // Bluetooth
    CRSF32_KIND_BT_RESET,
    CRSF32_KIND_BT_ENABLE,
    CRSF32_KIND_BT_ECHO,
    // OSD
    CRSF32_KIND_OSD_SEND_BUTTONS,
    // VTX
    CRSF32_KIND_VTX_CHANGE_CHANNEL,
    CRSF32_KIND_VTX_SET_FREQUENCY,
    CRSF32_KIND_VTX_CHANGE_POWER,
    CRSF32_KIND_VTX_ENABLE_PITMODE_ON_PUP,
    CRSF32_KIND_VTX_POWER_UP_FROM_PITMODE,
    CRSF32_KIND_VTX_SET_DYNAMIC_POWER,
    CRSF32_KIND_VTX_SET_POWER,
    // LED
    CRSF32_KIND_LED_SET_TO_DEFAULT,
    CRSF32_KIND_LED_OVERRIDE_COLOR,
    CRSF32_KIND_LED_OVERRIDE_PULSE,
    CRSF32_KIND_LED_OVERRIDE_BLINK,
    CRSF32_KIND_LED_OVERRIDE_SHIFT,
    // General
    CRSF32_KIND_GEN_PROTOCOL_SPEED_PROPOSAL,
    CRSF32_KIND_GEN_PROTOCOL_SPEED_RESPONSE,
    // Crossfire
    CRSF32_KIND_CF_SET_RX_BIND_MODE,
    CRSF32_KIND_CF_CANCEL_BIND_MODE,
    CRSF32_KIND_CF_SET_BIND_ID,
    CRSF32_KIND_CF_MODEL_SELECTION,
    CRSF32_KIND_CF_CURRENT_MODEL_QUERY,
    CRSF32_KIND_CF_CURRENT_MODEL_REPLY,
    // Flow control
    CRSF32_KIND_FLOW_SUBSCRIBE,
    CRSF32_KIND_FLOW_UNSUBSCRIBE,
    // Screen
    CRSF32_KIND_SCREEN_POPUP_MESSAGE_START,
    CRSF32_KIND_SCREEN_SELECTION_RETURN
} crsf32_kind_t;

// 24-bit packed HSV helper: H(9), S(7), V(8) — sequential big-endian bits
// Implementation: value = (H & 0x1FF)<<15 | (S & 0x7F)<<8 | (V);
// Bytes: out[0]=value>>16, out[1]=value>>8, out[2]=value.
static inline void CRSF_packHSV(uint16_t H, uint8_t S, uint8_t V, uint8_t out[3]) {
    uint32_t v = ((uint32_t)(H & 0x1FF) << 15) | ((uint32_t)(S & 0x7F) << 8) | (uint32_t)V;
    out[0] = (uint8_t)(v >> 16);
    out[1] = (uint8_t)(v >> 8);
    out[2] = (uint8_t)v;
}

static inline void CRSF_unpackHSV(const uint8_t in[3], uint16_t* H, uint8_t* S, uint8_t* V) {
    uint32_t v = ((uint32_t)in[0] << 16) | ((uint32_t)in[1] << 8) | in[2];
    if (H) {
        *H = (uint16_t)((v >> 15) & 0x1FF);
    }
    if (S) {
        *S = (uint8_t)((v >> 8) & 0x7F);
    }
    if (V) {
        *V = (uint8_t)(v & 0xFF);
    }
}

// Big-endian helpers
static inline uint16_t crsf32_be16(const uint8_t* p) { return (uint16_t)((((uint16_t)p[0]) << 8) | p[1]); }

static inline uint32_t crsf32_be32(const uint8_t* p) { return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; }

static inline void crsf32_wbe16(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void crsf32_wbe32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline int crsf32_find_nul(const uint8_t* buf, size_t len) {
    size_t i;
    for (i = 0; i < len; ++i) {
        if (!buf[i]) {
            return (int)i;
        }
    }
    return -1;
}

// Command ACK payload
typedef struct {
    uint8_t Command_ID;      // echoed realm
    uint8_t SubCommand_ID;   // echoed subcommand
    uint8_t Action;          // 1=already took action; 0=no function/invalid
    const char* Information; // optional NUL-terminated string (may be empty)
} CRSF_CommandACK_t;

// Screen Command: 0x32.0x22.0x01 Pop-up Message Start
typedef struct {
    const char* Header;           // NUL-terminated
    const char* Info_message;     // NUL-terminated
    uint8_t Max_timeout_interval; // seconds
    bool Close_button_option;     // 0/1

    struct {
        bool present;              // true if selectionText not empty
        const char* selectionText; // NUL-terminated; if empty, this whole struct is absent
        uint8_t value;
        uint8_t minValue;
        uint8_t maxValue;
        uint8_t defaultValue;
        const char* unit; // NUL-terminated
    } add_data;

    bool has_possible_values;
    const char* possible_values; // semicolon-separated NUL-terminated
} CRSF_CommandScreen_PopupStart_t;

// Screen Command: 0x32.0x22.0x02 Selection Return Value
typedef struct {
    uint8_t value; // selected value
    bool response; // true(Process) / false(Cancel)
} CRSF_CommandScreen_SelectionReturn_t;

// VTX pitmode control byte (0x32.0x08 subcmd 0x04)
typedef struct {
    uint8_t PitMode;         // 0=OFF, 1=ON (1 bit effective)
    uint8_t pitmode_control; // 0=Off,1=On,2=Arm,3=Failsafe (2 bits)
    uint8_t pitmode_switch;  // 0=Ch5,1=Ch5 Inv,...,15=Ch12 Inv (4 bits)
} crsf32_vtx_pitmode_cfg_t;

// Unified decoded packet
typedef struct {
    uint8_t command_id; // realm (e.g., CRSF32_CMDID_VTX)
    uint8_t subcommand; // subcommand within realm
    crsf32_kind_t kind; // normalized tag

    union {
        CRSF_CommandACK_t ACK; // 0xFF

        // FC
        struct { /* empty */
        } fc_force_disarm;

        struct {
            const uint8_t* bytes;
            size_t len;
        } fc_scale_channel; // payload unspecified

        // Bluetooth
        struct { /* empty */
        } bt_reset;

        struct {
            uint8_t Enable;
        } bt_enable;

        struct { /* empty */
        } bt_echo;

        // OSD
        struct {
            uint8_t Buttons;
        } OSD;

        // VTX
        struct {              /* empty */
        } vtx_change_channel; // discontinued

        struct {
            uint16_t FrequencyMHz;
        } vtx_set_frequency;

        struct {            /* empty */
        } vtx_change_power; // discontinued

        struct {
            crsf32_vtx_pitmode_cfg_t cfg;
        } vtx_enable_pitmode_on_pup;

        struct { /* empty */
        } vtx_power_up_from_pitmode;

        struct {
            uint8_t Power_dBm;
        } vtx_set_dynamic_power;

        struct {
            uint8_t Power_dBm;
        } vtx_set_power;

        // LED
        struct { /* empty */
        } led_set_to_default;

        struct {
            uint16_t H;
            uint8_t S;
            uint8_t V;
        } led_override_color;

        struct {
            uint16_t duration_ms;
            uint16_t H_start;
            uint8_t S_start;
            uint8_t V_start;
            uint16_t H_stop;
            uint8_t S_stop;
            uint8_t V_stop;
        } led_override_pulse;

        struct {
            uint16_t interval_ms;
            uint16_t H_start;
            uint8_t S_start;
            uint8_t V_start;
            uint16_t H_stop;
            uint8_t S_stop;
            uint8_t V_stop;
        } led_override_blink;

        struct {
            uint16_t interval_ms;
            uint16_t H;
            uint8_t S;
            uint8_t V;
        } led_override_shift;

        // General
        struct {
            uint8_t port_id;
            uint32_t proposed_baudrate;
        } gen_protocol_speed_proposal;

        struct {
            uint8_t port_id;
            bool response;
        } gen_protocol_speed_response;

        // Crossfire
        struct { /* empty */
        } cf_set_rx_bind_mode;

        struct { /* empty */
        } cf_cancel_bind_mode;

        struct {
            const uint8_t* bytes;
            size_t len;
        } cf_set_bind_id; // unspecified

        struct {
            uint8_t Model_Number;
        } cf_model_selection;

        struct { /* empty */
        } cf_current_model_query;

        struct {
            uint8_t Model_Number;
        } cf_current_model_reply;

        // Flow control
        struct {
            uint8_t Frame_type;
            uint16_t Max_interval_time_ms;
        } flow_subscribe;

        struct {
            uint8_t Frame_type;
        } flow_unsubscribe;

        // Screen
        CRSF_CommandScreen_PopupStart_t screen_popup_message_start;
        CRSF_CommandScreen_SelectionReturn_t screen_selection_return;

        // Fallback
        struct {
            const uint8_t* bytes;
            size_t len;
        } raw;
    } v;
} crsf32_packet_t;

// ---------------------------------------------------------------------------
// Return codes
// ---------------------------------------------------------------------------

typedef enum { CRSF32_OK = 0, CRSF32_ERR_SHORT = -1, CRSF32_ERR_BADSTRING = -2 } crsf32_rc;

// ---------------------------------------------------------------------------
// Internal helpers (pure C replacements for former C++ lambdas)
// ---------------------------------------------------------------------------

static inline crsf32_rc crsf32_read_cstr(const uint8_t** q, size_t* rem, const char** out_s) {
    int z;
    if (!q || !*q || !rem || !out_s) {
        return CRSF32_ERR_BADSTRING;
    }
    z = crsf32_find_nul(*q, *rem);
    if (z < 0) {
        return CRSF32_ERR_BADSTRING;
    }
    *out_s = (const char*)(*q);
    *q += (size_t)z + 1;
    *rem -= (size_t)z + 1;
    return CRSF32_OK;
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

static inline crsf32_rc crsf32_decode_payload(uint8_t command_id, const uint8_t* p, size_t len, crsf32_packet_t* out) {
    const uint8_t* q = p;
    size_t rem = len;

    if (!out) {
        return CRSF32_ERR_SHORT;
    }
    out->command_id = command_id;
    out->subcommand = 0;
    out->kind = CRSF32_KIND_UNKNOWN;
    if (!p && len) {
        return CRSF32_ERR_SHORT;
    }

    switch (command_id) {
        case CRSF32_CMDID_COMMAND_ACK: {
            if (rem < 3) {
                return CRSF32_ERR_SHORT; // Command_ID, SubCommand_ID, Action
            }
            out->subcommand = 0x00; // N/A
            out->kind = CRSF32_KIND_COMMAND_ACK;
            out->v.ACK.Command_ID = q[0];
            out->v.ACK.SubCommand_ID = q[1];
            out->v.ACK.Action = q[2];
            q += 3;
            rem -= 3;
            if (rem) {
                const char* s;
                crsf32_rc rc = crsf32_read_cstr(&q, &rem, &s);
                if (rc != CRSF32_OK) {
                    return rc;
                }
                out->v.ACK.Information = s;
            } else {
                out->v.ACK.Information = ""; // empty allowed
            }
            return CRSF32_OK;
        }

        case CRSF32_CMDID_FC: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_FC_FORCE_DISARM: out->kind = CRSF32_KIND_FC_FORCE_DISARM; return CRSF32_OK;
                case CRSF32_FC_SCALE_CHANNEL:
                    out->kind = CRSF32_KIND_FC_SCALE_CHANNEL;
                    out->v.fc_scale_channel.bytes = q;
                    out->v.fc_scale_channel.len = rem;
                    return CRSF32_OK; // unspecified
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_BLUETOOTH: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_BT_RESET: out->kind = CRSF32_KIND_BT_RESET; return CRSF32_OK;
                case CRSF32_BT_ENABLE:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_BT_ENABLE;
                    out->v.bt_enable.Enable = q[0];
                    return CRSF32_OK;
                case CRSF32_BT_ECHO: out->kind = CRSF32_KIND_BT_ECHO; return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_OSD: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            if (sub == CRSF32_OSD_SEND_BUTTONS) {
                if (rem < 1) {
                    return CRSF32_ERR_SHORT;
                }
                out->kind = CRSF32_KIND_OSD_SEND_BUTTONS;
                out->v.OSD.buttons = q[0];
                return CRSF32_OK;
            }
            out->v.raw.bytes = q - 1;
            out->v.raw.len = rem + 1;
            return CRSF32_OK;
        }

        case CRSF32_CMDID_VTX: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_VTX_CHANGE_CHANNEL:
                    out->kind = CRSF32_KIND_VTX_CHANGE_CHANNEL;
                    /* empty payload */
                    return CRSF32_OK;
                case CRSF32_VTX_SET_FREQUENCY:
                    if (rem < 2) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_VTX_SET_FREQUENCY;
                    out->v.vtx_set_frequency.FrequencyMHz = crsf32_be16(q);
                    return CRSF32_OK;
                case CRSF32_VTX_CHANGE_POWER: out->kind = CRSF32_KIND_VTX_CHANGE_POWER; return CRSF32_OK;
                case CRSF32_VTX_ENABLE_PITMODE_ON_PUP:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_VTX_ENABLE_PITMODE_ON_PUP;
                    {
                        uint8_t b = q[0];
                        out->v.vtx_enable_pitmode_on_pup.cfg.PitMode = (uint8_t)((b >> 7) & 1);
                        out->v.vtx_enable_pitmode_on_pup.cfg.pitmode_control = (uint8_t)((b >> 5) & 0x03);
                        out->v.vtx_enable_pitmode_on_pup.cfg.pitmode_switch = (uint8_t)((b >> 1) & 0x0F);
                    }
                    return CRSF32_OK;
                case CRSF32_VTX_POWER_UP_FROM_PITMODE: out->kind = CRSF32_KIND_VTX_POWER_UP_FROM_PITMODE; return CRSF32_OK;
                case CRSF32_VTX_SET_DYNAMIC_POWER:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_VTX_SET_DYNAMIC_POWER;
                    out->v.vtx_set_dynamic_power.Power_dBm = q[0];
                    return CRSF32_OK;
                case CRSF32_VTX_SET_POWER:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_VTX_SET_POWER;
                    out->v.vtx_set_power.Power_dBm = q[0];
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_LED: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_LED_SET_TO_DEFAULT: out->kind = CRSF32_KIND_LED_SET_TO_DEFAULT; return CRSF32_OK;
                case CRSF32_LED_OVERRIDE_COLOR:
                    if (rem < 3) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_LED_OVERRIDE_COLOR;
                    {
                        uint16_t H;
                        uint8_t S;
                        uint8_t V;
                        CRSF_unpackHSV(q, &H, &S, &V);
                        out->v.led_override_color.H = H;
                        out->v.led_override_color.S = S;
                        out->v.led_override_color.V = V;
                    }
                    return CRSF32_OK;
                case CRSF32_LED_OVERRIDE_PULSE:
                    if (rem < (size_t)(2 + 3 + 3)) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_LED_OVERRIDE_PULSE;
                    out->v.led_override_pulse.duration_ms = crsf32_be16(q);
                    q += 2;
                    CRSF_unpackHSV(q, &out->v.led_override_pulse.H_start, &out->v.led_override_pulse.S_start, &out->v.led_override_pulse.V_start);
                    q += 3;
                    CRSF_unpackHSV(q, &out->v.led_override_pulse.H_stop, &out->v.led_override_pulse.S_stop, &out->v.led_override_pulse.V_stop);
                    return CRSF32_OK;
                case CRSF32_LED_OVERRIDE_BLINK:
                    if (rem < (size_t)(2 + 3 + 3)) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_LED_OVERRIDE_BLINK;
                    out->v.led_override_blink.interval_ms = crsf32_be16(q);
                    q += 2;
                    CRSF_unpackHSV(q, &out->v.led_override_blink.H_start, &out->v.led_override_blink.S_start, &out->v.led_override_blink.V_start);
                    q += 3;
                    CRSF_unpackHSV(q, &out->v.led_override_blink.H_stop, &out->v.led_override_blink.S_stop, &out->v.led_override_blink.V_stop);
                    return CRSF32_OK;
                case CRSF32_LED_OVERRIDE_SHIFT:
                    if (rem < (size_t)(2 + 3)) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_LED_OVERRIDE_SHIFT;
                    out->v.led_override_shift.interval_ms = crsf32_be16(q);
                    q += 2;
                    CRSF_unpackHSV(q, &out->v.led_override_shift.H, &out->v.led_override_shift.S, &out->v.led_override_shift.V);
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_GENERAL: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL:
                    if (rem < (size_t)(1 + 4)) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_GEN_PROTOCOL_SPEED_PROPOSAL;
                    out->v.gen_protocol_speed_proposal.port_id = q[0];
                    out->v.gen_protocol_speed_proposal.proposed_baudrate = crsf32_be32(q + 1);
                    return CRSF32_OK;
                case CRSF32_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE:
                    if (rem < 2) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_GEN_PROTOCOL_SPEED_RESPONSE;
                    out->v.gen_protocol_speed_response.port_id = q[0];
                    out->v.gen_protocol_speed_response.response = (q[1] != 0);
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_CROSSFIRE: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_CF_SET_RX_BIND_MODE: out->kind = CRSF32_KIND_CF_SET_RX_BIND_MODE; return CRSF32_OK;
                case CRSF32_CF_CANCEL_BIND_MODE: out->kind = CRSF32_KIND_CF_CANCEL_BIND_MODE; return CRSF32_OK;
                case CRSF32_CF_SET_BIND_ID:
                    out->kind = CRSF32_KIND_CF_SET_BIND_ID;
                    out->v.cf_set_bind_id.bytes = q;
                    out->v.cf_set_bind_id.len = rem;
                    return CRSF32_OK; // not detailed
                case CRSF32_CF_MODEL_SELECTION:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_CF_MODEL_SELECTION;
                    out->v.cf_model_selection.Model_Number = q[0];
                    return CRSF32_OK;
                case CRSF32_CF_CURRENT_MODEL_QUERY: out->kind = CRSF32_KIND_CF_CURRENT_MODEL_QUERY; return CRSF32_OK;
                case CRSF32_CF_CURRENT_MODEL_REPLY:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_CF_CURRENT_MODEL_REPLY;
                    out->v.cf_current_model_reply.Model_Number = q[0];
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_FLOW_CTRL: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_FLOW_SUBSCRIBE:
                    if (rem < (size_t)(1 + 2)) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_FLOW_SUBSCRIBE;
                    out->v.flow_subscribe.Frame_type = q[0];
                    out->v.flow_subscribe.Max_interval_time_ms = crsf32_be16(q + 1);
                    return CRSF32_OK;
                case CRSF32_FLOW_UNSUBSCRIBE:
                    if (rem < 1) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_FLOW_UNSUBSCRIBE;
                    out->v.flow_unsubscribe.Frame_type = q[0];
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        case CRSF32_CMDID_SCREEN: {
            uint8_t sub;
            if (rem < 1) {
                return CRSF32_ERR_SHORT;
            }
            sub = q[0];
            q++;
            rem--;
            out->subcommand = sub;
            switch (sub) {
                case CRSF32_SCREEN_POPUP_MESSAGE_START: {
                    const char *hdr, *info;
                    crsf32_rc rc;
                    out->kind = CRSF32_KIND_SCREEN_POPUP_MESSAGE_START;
                    rc = crsf32_read_cstr(&q, &rem, &hdr);
                    if (rc != CRSF32_OK) {
                        return rc;
                    }
                    rc = crsf32_read_cstr(&q, &rem, &info);
                    if (rc != CRSF32_OK) {
                        return rc;
                    }
                    if (rem < 2) {
                        return CRSF32_ERR_SHORT; // Max_timeout_interval + Close_button_option
                    }
                    out->v.screen_popup_message_start.Header = hdr;
                    out->v.screen_popup_message_start.Info_message = info;
                    out->v.screen_popup_message_start.Max_timeout_interval = q[0];
                    out->v.screen_popup_message_start.Close_button_option = (q[1] != 0);
                    q += 2;
                    rem -= 2;

                    // optional add_data
                    out->v.screen_popup_message_start.add_data.present = false;
                    out->v.screen_popup_message_start.has_possible_values = false;
                    if (rem > 0) {
                        const char* sel;
                        rc = crsf32_read_cstr(&q, &rem, &sel);
                        if (rc != CRSF32_OK) {
                            return rc;
                        }
                        if (sel[0] != '\0') {
                            out->v.screen_popup_message_start.add_data.present = true;
                            out->v.screen_popup_message_start.add_data.selectionText = sel;
                            if (rem < 4) {
                                return CRSF32_ERR_SHORT; // value,min,max,default
                            }
                            out->v.screen_popup_message_start.add_data.value = q[0];
                            out->v.screen_popup_message_start.add_data.minValue = q[1];
                            out->v.screen_popup_message_start.add_data.maxValue = q[2];
                            out->v.screen_popup_message_start.add_data.defaultValue = q[3];
                            q += 4;
                            rem -= 4;
                            {
                                const char* unit;
                                rc = crsf32_read_cstr(&q, &rem, &unit);
                                if (rc != CRSF32_OK) {
                                    return rc;
                                }
                                out->v.screen_popup_message_start.add_data.unit = unit;
                            }
                        }
                        // optional possible_values (semicolon-separated)
                        if (rem > 0) {
                            const char* pv;
                            rc = crsf32_read_cstr(&q, &rem, &pv);
                            if (rc == CRSF32_OK) {
                                out->v.screen_popup_message_start.has_possible_values = true;
                                out->v.screen_popup_message_start.possible_values = pv;
                            }
                        }
                    }
                    return CRSF32_OK;
                }
                case CRSF32_SCREEN_SELECTION_RETURN:
                    if (rem < 2) {
                        return CRSF32_ERR_SHORT;
                    }
                    out->kind = CRSF32_KIND_SCREEN_SELECTION_RETURN;
                    out->v.screen_selection_return.value = q[0];
                    out->v.screen_selection_return.response = (q[1] != 0);
                    return CRSF32_OK;
                default:
                    out->v.raw.bytes = q - 1;
                    out->v.raw.len = rem + 1;
                    return CRSF32_OK;
            }
        }

        default:
            // unknown realm: keep raw
            out->v.raw.bytes = p;
            out->v.raw.len = len;
            return CRSF32_OK;
    }
}

// ---------------------------------------------------------------------------
// Encoder (mirrors the decoded structure)
// ---------------------------------------------------------------------------

static inline int crsf32_encode_payload(uint8_t* out, size_t cap, const crsf32_packet_t* in) {
    size_t off = 0;
    if (!out || !in) {
        return CRSF32_ERR_SHORT;
    }

    // write subcommand or ACK header depending on realm
    switch (in->command_id) {
        case CRSF32_CMDID_COMMAND_ACK: {
            const char* info;
            const char* s;
            if (cap < 3) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->v.ACK.Command_ID;
            out[off++] = in->v.ACK.SubCommand_ID;
            out[off++] = in->v.ACK.Action;
            info = in->v.ACK.Information ? in->v.ACK.Information : "";
            for (s = info; *s; ++s) {
                if (off >= cap) {
                    return CRSF32_ERR_SHORT;
                }
                out[off++] = (uint8_t)*s;
            }
            if (off >= cap) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = 0; // NUL
            break;
        }

        case CRSF32_CMDID_FC: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_FC_FORCE_DISARM: break;
                case CRSF32_KIND_FC_SCALE_CHANNEL:
                    if (in->v.fc_scale_channel.bytes && in->v.fc_scale_channel.len) {
                        size_t i;
                        if (off + in->v.fc_scale_channel.len > cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        for (i = 0; i < in->v.fc_scale_channel.len; ++i) {
                            out[off++] = in->v.fc_scale_channel.bytes[i];
                        }
                    }
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_BLUETOOTH: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_BT_RESET: break;
                case CRSF32_KIND_BT_ENABLE:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.bt_enable.Enable;
                    break;
                case CRSF32_KIND_BT_ECHO: break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_OSD: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            if (in->kind == CRSF32_KIND_OSD_SEND_BUTTONS) {
                if (off + 1 > cap) {
                    return CRSF32_ERR_SHORT;
                }
                out[off++] = in->v.OSD.buttons;
            }
            break;
        }

        case CRSF32_CMDID_VTX: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_VTX_CHANGE_CHANNEL: break;
                case CRSF32_KIND_VTX_SET_FREQUENCY:
                    if (off + 2 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    crsf32_wbe16(out + off, in->v.vtx_set_frequency.FrequencyMHz);
                    off += 2;
                    break;
                case CRSF32_KIND_VTX_CHANGE_POWER: break;
                case CRSF32_KIND_VTX_ENABLE_PITMODE_ON_PUP:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    {
                        uint8_t b = (uint8_t)(((in->v.vtx_enable_pitmode_on_pup.cfg.PitMode & 1) << 7)
                                              | ((in->v.vtx_enable_pitmode_on_pup.cfg.pitmode_control & 3) << 5)
                                              | ((in->v.vtx_enable_pitmode_on_pup.cfg.pitmode_switch & 0x0F) << 1));
                        out[off++] = b;
                    }
                    break;
                case CRSF32_KIND_VTX_POWER_UP_FROM_PITMODE: break;
                case CRSF32_KIND_VTX_SET_DYNAMIC_POWER:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.vtx_set_dynamic_power.Power_dBm;
                    break;
                case CRSF32_KIND_VTX_SET_POWER:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.vtx_set_power.power_dBm; // NOTE: case sensitive fix not needed; keep as in original
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_LED: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_LED_SET_TO_DEFAULT: break;
                case CRSF32_KIND_LED_OVERRIDE_COLOR:
                    if (off + 3 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    {
                        uint8_t b[3];
                        CRSF_packHSV(in->v.led_override_color.H, in->v.led_override_color.S, in->v.led_override_color.V, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                    }
                    break;
                case CRSF32_KIND_LED_OVERRIDE_PULSE:
                    if (off + 2 + 3 + 3 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    crsf32_wbe16(out + off, in->v.led_override_pulse.duration_ms);
                    off += 2;
                    {
                        uint8_t b[3];
                        CRSF_packHSV(in->v.led_override_pulse.H_start, in->v.led_override_pulse.S_start, in->v.led_override_pulse.V_start, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                        CRSF_packHSV(in->v.led_override_pulse.H_stop, in->v.led_override_pulse.S_stop, in->v.led_override_pulse.V_stop, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                    }
                    break;
                case CRSF32_KIND_LED_OVERRIDE_BLINK:
                    if (off + 2 + 3 + 3 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    crsf32_wbe16(out + off, in->v.led_override_blink.interval_ms);
                    off += 2;
                    {
                        uint8_t b[3];
                        CRSF_packHSV(in->v.led_override_blink.H_start, in->v.led_override_blink.S_start, in->v.led_override_blink.V_start, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                        CRSF_packHSV(in->v.led_override_blink.H_stop, in->v.led_override_blink.S_stop, in->v.led_override_blink.V_stop, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                    }
                    break;
                case CRSF32_KIND_LED_OVERRIDE_SHIFT:
                    if (off + 2 + 3 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    crsf32_wbe16(out + off, in->v.led_override_shift.interval_ms);
                    off += 2;
                    {
                        uint8_t b[3];
                        CRSF_packHSV(in->v.led_override_shift.H, in->v.led_override_shift.S, in->v.led_override_shift.V, b);
                        out[off++] = b[0];
                        out[off++] = b[1];
                        out[off++] = b[2];
                    }
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_GENERAL: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_GEN_PROTOCOL_SPEED_PROPOSAL:
                    if (off + 1 + 4 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.gen_protocol_speed_proposal.port_id;
                    crsf32_wbe32(out + off, in->v.gen_protocol_speed_proposal.proposed_baudrate);
                    off += 4;
                    break;
                case CRSF32_KIND_GEN_PROTOCOL_SPEED_RESPONSE:
                    if (off + 2 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.gen_protocol_speed_response.port_id;
                    out[off++] = (uint8_t)(in->v.gen_protocol_speed_response.response ? 1 : 0);
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_CROSSFIRE: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_CF_SET_RX_BIND_MODE:
                case CRSF32_KIND_CF_CANCEL_BIND_MODE:
                case CRSF32_KIND_CF_CURRENT_MODEL_QUERY: break;
                case CRSF32_KIND_CF_SET_BIND_ID:
                    if (in->v.cf_set_bind_id.bytes && in->v.cf_set_bind_id.len) {
                        size_t i;
                        if (off + in->v.cf_set_bind_id.len > cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        for (i = 0; i < in->v.cf_set_bind_id.len; ++i) {
                            out[off++] = in->v.cf_set_bind_id.bytes[i];
                        }
                    }
                    break;
                case CRSF32_KIND_CF_MODEL_SELECTION:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.cf_model_selection.Model_Number;
                    break;
                case CRSF32_KIND_CF_CURRENT_MODEL_REPLY:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.cf_current_model_reply.Model_Number;
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_FLOW_CTRL: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_FLOW_SUBSCRIBE:
                    if (off + 1 + 2 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.flow_subscribe.Frame_type;
                    crsf32_wbe16(out + off, in->v.flow_subscribe.Max_interval_time_ms);
                    off += 2;
                    break;
                case CRSF32_KIND_FLOW_UNSUBSCRIBE:
                    if (off + 1 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.flow_unsubscribe.Frame_type;
                    break;
                default: break;
            }
            break;
        }

        case CRSF32_CMDID_SCREEN: {
            if (cap < 1) {
                return CRSF32_ERR_SHORT;
            }
            out[off++] = in->subcommand;
            switch (in->kind) {
                case CRSF32_KIND_SCREEN_POPUP_MESSAGE_START: {
                    const char* h = in->v.screen_popup_message_start.Header ? in->v.screen_popup_message_start.Header : "";
                    const char* im = in->v.screen_popup_message_start.Info_message ? in->v.screen_popup_message_start.Info_message : "";
                    const char* s;
                    for (s = h; *s; ++s) {
                        if (off >= cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = (uint8_t)*s;
                    }
                    if (off >= cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = 0;
                    for (s = im; *s; ++s) {
                        if (off >= cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = (uint8_t)*s;
                    }
                    if (off >= cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = 0;
                    if (off + 2 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.screen_popup_message_start.Max_timeout_interval;
                    out[off++] = (uint8_t)(in->v.screen_popup_message_start.Close_button_option ? 1 : 0);
                    if (in->v.screen_popup_message_start.add_data.present) {
                        const char* sel =
                            in->v.screen_popup_message_start.add_data.selectionText ? in->v.screen_popup_message_start.add_data.selectionText : "";
                        for (s = sel; *s; ++s) {
                            if (off >= cap) {
                                return CRSF32_ERR_SHORT;
                            }
                            out[off++] = (uint8_t)*s;
                        }
                        if (off >= cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = 0;
                        if (off + 4 > cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = in->v.screen_popup_message_start.add_data.value;
                        out[off++] = in->v.screen_popup_message_start.add_data.minValue;
                        out[off++] = in->v.screen_popup_message_start.add_data.maxValue;
                        out[off++] = in->v.screen_popup_message_start.add_data.defaultValue;
                        {
                            const char* unit = in->v.screen_popup_message_start.add_data.unit ? in->v.screen_popup_message_start.add_data.unit : "";
                            for (s = unit; *s; ++s) {
                                if (off >= cap) {
                                    return CRSF32_ERR_SHORT;
                                }
                                out[off++] = (uint8_t)*s;
                            }
                            if (off >= cap) {
                                return CRSF32_ERR_SHORT;
                            }
                            out[off++] = 0;
                        }
                    } else {
                        // write empty selectionText (single NUL) to signal absence
                        if (off >= cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = 0;
                    }
                    if (in->v.screen_popup_message_start.has_possible_values) {
                        const char* pv = in->v.screen_popup_message_start.possible_values ? in->v.screen_popup_message_start.possible_values : "";
                        for (s = pv; *s; ++s) {
                            if (off >= cap) {
                                return CRSF32_ERR_SHORT;
                            }
                            out[off++] = (uint8_t)*s;
                        }
                        if (off >= cap) {
                            return CRSF32_ERR_SHORT;
                        }
                        out[off++] = 0;
                    }
                    break;
                }
                case CRSF32_KIND_SCREEN_SELECTION_RETURN:
                    if (off + 2 > cap) {
                        return CRSF32_ERR_SHORT;
                    }
                    out[off++] = in->v.screen_selection_return.value;
                    out[off++] = (uint8_t)(in->v.screen_selection_return.response ? 1 : 0);
                    break;
                default: break;
            }
            break;
        }

        default: {
            // unknown realm: just dump raw payload present in v.raw if provided
            if (in->v.raw.bytes && in->v.raw.len) {
                size_t i;
                if (in->v.raw.len > cap) {
                    return CRSF32_ERR_SHORT;
                }
                for (i = 0; i < in->v.raw.len; ++i) {
                    out[off++] = in->v.raw.bytes[i];
                }
            }
            break;
        }
    }

    return (int)off;
}

#ifdef __cplusplus
}
#endif

#endif // CRSF_CMD_32_H

#ifdef CRSF_CMD32_IMPLEMENTATION
// header-only — all functions are static inline above
#endif
