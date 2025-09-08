#include <stdint.h>
#include <string.h>

// CRSF 0x32 Sub-command enums
typedef enum {
    CRSF_SUBCMD_COMMAND_ACK = 0xFF,
    CRSF_SUBCMD_FC_COMMANDS = 0x01,
    CRSF_SUBCMD_BLUETOOTH = 0x03,
    CRSF_SUBCMD_OSD = 0x05,
    CRSF_SUBCMD_VTX = 0x08,
    CRSF_SUBCMD_LED = 0x09,
    CRSF_SUBCMD_GENERAL = 0x0A,
    CRSF_SUBCMD_CROSSFIRE = 0x10,
    CRSF_SUBCMD_RESERVED = 0x12,
    CRSF_SUBCMD_FLOW_CONTROL = 0x20,
    CRSF_SUBCMD_SCREEN = 0x22
} CRSF_Subcommand_t;

// FC Commands (0x32.0x01)
typedef enum { FC_CMD_FORCE_DISARM = 0x01, FC_CMD_SCALE_CHANNEL = 0x02 } fc_command_t;

// Bluetooth Commands (0x32.0x03)
typedef enum { BT_CMD_RESET = 0x01, BT_CMD_ENABLE = 0x02, BT_CMD_ECHO = 0x64 } bluetooth_command_t;

// OSD Commands (0x32.0x05)
typedef enum { OSD_CMD_SEND_BUTTONS = 0x01 } osd_command_t;

// VTX Commands (0x32.0x08)
typedef enum {
    VTX_CMD_CHANGE_CHANNEL = 0x01, // DISCONTINUED
    VTX_CMD_SET_FREQUENCY = 0x02,
    VTX_CMD_CHANGE_POWER = 0x03, // DISCONTINUED
    VTX_CMD_ENABLE_PITMODE = 0x04,
    VTX_CMD_POWERUP_FROM_PITMODE = 0x05,
    VTX_CMD_SET_DYNAMIC_POWER = 0x06,
    VTX_CMD_SET_POWER = 0x08
} vtx_command_t;

// LED Commands (0x32.0x09)
typedef enum { LED_CMD_SET_DEFAULT = 0x01, LED_CMD_OVERRIDE_COLOR = 0x02, LED_CMD_OVERRIDE_PULSE = 0x03, LED_CMD_OVERRIDE_BLINK = 0x04, LED_CMD_OVERRIDE_SHIFT = 0x05 } led_command_t;

// General Commands (0x32.0x0A)
typedef enum { GENERAL_CMD_PROTOCOL_SPEED_PROPOSAL = 0x70, GENERAL_CMD_PROTOCOL_SPEED_RESPONSE = 0x71 } general_command_t;

// Crossfire Commands (0x32.0x10)
typedef enum {
    CF_CMD_SET_BIND_MODE = 0x01,
    CF_CMD_CANCEL_BIND = 0x02,
    CF_CMD_SET_BIND_ID = 0x03,
    CF_CMD_MODEL_SELECTION = 0x05,
    CF_CMD_QUERY_MODEL_SELECTION = 0x06,
    CF_CMD_REPLY_MODEL_SELECTION = 0x07
} crossfire_command_t;

// Flow Control Commands (0x32.0x20)
typedef enum { FLOW_CMD_SUBSCRIBE = 0x01, FLOW_CMD_UNSUBSCRIBE = 0x02 } flow_control_command_t;

// Screen Commands (0x32.0x22)
typedef enum { SCREEN_CMD_POPUP_START = 0x01, SCREEN_CMD_SELECTION_RETURN = 0x02 } screen_command_t;

// Action enum for Command ACK
typedef enum { ACK_ACTION_NO_FUNCTION = 0, ACK_ACTION_TAKEN = 1 } ack_action_t;

// OSD Button definitions
typedef enum { OSD_BTN_RIGHT = (1 << 3), OSD_BTN_LEFT = (1 << 4), OSD_BTN_DOWN = (1 << 5), OSD_BTN_UP = (1 << 6), OSD_BTN_ENTER = (1 << 7) } osd_buttons_t;

// PitMode Control enum
typedef enum { PITMODE_CTRL_OFF = 0, PITMODE_CTRL_ON = 1, PITMODE_CTRL_ARM = 2, PITMODE_CTRL_FAILSAFE = 3 } pitmode_control_t;

// PitMode Switch enum (0=Ch5, 1=Ch5 Inv, ..., 15=Ch12 Inv)
typedef enum {
    PITMODE_SWITCH_CH5 = 0,
    PITMODE_SWITCH_CH5_INV = 1,
    PITMODE_SWITCH_CH6 = 2,
    PITMODE_SWITCH_CH6_INV = 3,
    PITMODE_SWITCH_CH7 = 4,
    PITMODE_SWITCH_CH7_INV = 5,
    PITMODE_SWITCH_CH8 = 6,
    PITMODE_SWITCH_CH8_INV = 7,
    PITMODE_SWITCH_CH9 = 8,
    PITMODE_SWITCH_CH9_INV = 9,
    PITMODE_SWITCH_CH10 = 10,
    PITMODE_SWITCH_CH10_INV = 11,
    PITMODE_SWITCH_CH11 = 12,
    PITMODE_SWITCH_CH11_INV = 13,
    PITMODE_SWITCH_CH12 = 14,
    PITMODE_SWITCH_CH12_INV = 15
} pitmode_switch_t;

// ===============================================
// SUB-PAYLOAD STRUCTURES
// ===============================================
#pragma pack(push, 1)

// 0x32.0xFF - Command ACK
typedef struct {
    uint8_t command_id;
    uint8_t subcommand_id;
    uint8_t action;       // ack_action_t
    char information[50]; // Null-terminated string //TODO check max length
} crsf_command_ack_t;

// 0x32.0x01 - FC Commands (no additional payload for basic commands)
typedef struct {
    uint8_t fc_command; // fc_command_t
} crsf_fc_command_t;

// 0x32.0x03.0x02 - Bluetooth Enable
typedef struct {
    uint8_t bt_command; // bluetooth_command_t
    uint8_t enable;     // 0 = disable, 1 = enable
} crsf_bluetooth_enable_t;

// 0x32.0x03 - Bluetooth Commands (for commands without payload)
typedef struct {
    uint8_t bt_command; // bluetooth_command_t
} crsf_bluetooth_command_t;

// 0x32.0x05.0x01 - OSD Send Buttons
typedef struct {
    uint8_t osd_command; // osd_command_t
    uint8_t buttons;     // Bitwise combination of osd_buttons_t
} crsf_osd_buttons_t;

// 0x32.0x08.0x01 - VTX Change Channel (DISCONTINUED)
typedef struct {
    uint8_t vtx_command; // vtx_command_t
    uint8_t channel;
} crsf_vtx_change_channel_t;

// 0x32.0x08.0x02 - VTX Set Frequency
typedef struct {
    uint8_t vtx_command; // vtx_command_t
    uint16_t frequency;  // 5000-6000 MHz
} crsf_vtx_set_frequency_t;

// 0x32.0x08.0x04 - VTX Enable PitMode
typedef struct {
    uint8_t vtx_command;         // vtx_command_t
    uint8_t pitmode         : 1; // 0 = OFF, 1 = ON
    uint8_t pitmode_control : 2; // pitmode_control_t
    uint8_t pitmode_switch  : 4; // pitmode_switch_t
    uint8_t reserved        : 1;
} crsf_vtx_pitmode_t;

// 0x32.0x08.0x05 - VTX Power up from PitMode (bare command)
typedef struct {
    uint8_t vtx_command; // vtx_command_t
} crsf_vtx_powerup_pitmode_t;

// 0x32.0x08.0x06 - VTX Set Dynamic Power
typedef struct {
    uint8_t vtx_command; // vtx_command_t
    uint8_t power_dbm;   // Power in dBm (0dBm = PitMode)
} crsf_vtx_dynamic_power_t;

// 0x32.0x08.0x08 - VTX Set Power
typedef struct {
    uint8_t vtx_command; // vtx_command_t
    uint8_t power_dbm;   // Power in dBm (0dBm = PitMode)
} crsf_vtx_set_power_t;

// 0x32.0x09.0x01 - LED Set Default
typedef struct {
    uint8_t led_command; // led_command_t
} crsf_led_default_t;

// 0x32.0x09.0x02 - LED Override Color (packed)
typedef struct {
    uint8_t led_command; // led_command_t
    uint16_t h : 9;      // Hue 0-359°
    uint8_t s  : 7;      // Saturation 0-100%
    uint8_t v  : 8;      // Value 0-100%
} crsf_led_color_t;

// 0x32.0x09.0x03 - LED Override Pulse (packed)
typedef struct {
    uint8_t led_command;  // led_command_t
    uint16_t duration;    // Milliseconds
    uint16_t h_start : 9; // Start Hue 0-359°
    uint8_t s_start  : 7; // Start Saturation 0-100%
    uint8_t v_start  : 8; // Start Value 0-100%
    uint16_t h_stop  : 9; // Stop Hue 0-359°
    uint8_t s_stop   : 7; // Stop Saturation 0-100%
    uint8_t v_stop   : 8; // Stop Value 0-100%
} crsf_led_pulse_t;

// 0x32.0x09.0x04 - LED Override Blink (packed)
typedef struct {
    uint8_t led_command; // led_command_t
    uint16_t interval;
    uint16_t h_start : 9; // Start Hue 0-359°
    uint8_t s_start  : 7; // Start Saturation 0-100%
    uint8_t v_start  : 8; // Start Value 0-100%
    uint16_t h_stop  : 9; // Stop Hue 0-359°
    uint8_t s_stop   : 7; // Stop Saturation 0-100%
    uint8_t v_stop   : 8; // Stop Value 0-100%
} crsf_led_blink_t;

// 0x32.0x09.0x05 - LED Override Shift (packed)
typedef struct {
    uint8_t led_command; // led_command_t
    uint16_t interval;
    uint16_t h : 9; // Hue 0-359°
    uint8_t s  : 7; // Saturation 0-100%
    uint8_t v  : 8; // Value 0-100%
} crsf_led_shift_t;

// 0x32.0x0A.0x70 - Protocol Speed Proposal
typedef struct {
    uint8_t general_command; // general_command_t
    uint8_t port_id;
    uint32_t proposed_baudrate;
} crsf_protocol_speed_proposal_t;

// 0x32.0x0A.0x71 - Protocol Speed Response
typedef struct {
    uint8_t general_command; // general_command_t
    uint8_t port_id;
    uint8_t response; // 1 = accepted, 0 = rejected
} crsf_protocol_speed_response_t;

// 0x32.0x10 - Crossfire Commands (most are bare commands)
typedef struct {
    uint8_t cf_command; // crossfire_command_t
} crsf_crossfire_command_t;

// 0x32.0x10.0x05 - Model Selection
typedef struct {
    uint8_t cf_command; // crossfire_command_t
    uint8_t model_number;
} crsf_model_selection_t;

// 0x32.0x10.0x07 - Reply Model Selection
typedef struct {
    uint8_t cf_command; // crossfire_command_t
    uint8_t model_number;
} crsf_model_selection_reply_t;

// 0x32.0x20.0x01 - Flow Control Subscribe
typedef struct {
    uint8_t flow_command; // flow_control_command_t
    uint8_t frame_type;
    uint16_t max_interval_ms;
} crsf_flow_subscribe_t;

// 0x32.0x20.0x02 - Flow Control Unsubscribe
typedef struct {
    uint8_t flow_command; // flow_control_command_t
    uint8_t frame_type;
} crsf_flow_unsubscribe_t;

// Additional data structure for popup (when selectionText is not empty)
typedef struct {
    // First byte determines if this struct exists - if it's null, only 1 byte total
    char selection_text[32]; // Null-terminated string, max practical size //TODO check max length
    uint8_t value;
    uint8_t min_value;
    uint8_t max_value;
    uint8_t default_value;
    char unit[16]; // Null-terminated string of measurement unit //TODO check max length
} crsf_popup_add_data_t;

// 0x32.0x22.0x01 - Screen Pop-up Message Start (Complete Structure)
typedef struct {
    uint8_t screen_command;         // screen_command_t (SCREEN_CMD_POPUP_START)
    char header[64];                // Null-terminated string //TODO check max length
    char info_message[128];         // Null-terminated string //TODO check max length
    uint8_t max_timeout_interval;   // Time in seconds
    uint8_t close_button_option;    // bool: 1 = show close button, 0 = no close button
    crsf_popup_add_data_t add_data; // Optional additional data (check if selection_text[0] != 0)
    char possible_values[128];      // Optional semicolon-separated values, null-terminated
} crsf_screen_popup_start_t;

// 0x32.0x22.0x02 - Screen Selection Return Value
typedef struct {
    uint8_t screen_command; // screen_command_t
    uint8_t value;
    uint8_t response; // 1 = Process, 0 = Cancel
} crsf_screen_selection_return_t;

#pragma pack(pop)

// ===============================================
// UNPACK FUNCTION
// ===============================================

// Helper function to get expected payload length for fixed-length commands
uint8_t crsf_get_expected_length(uint8_t subcommand, uint8_t sub_subcommand) {
    switch (subcommand) {
        case CRSF_SUBCMD_FC_COMMANDS: return 2; // 1 subcmd + 1 fc_command
        case CRSF_SUBCMD_BLUETOOTH:
            if (sub_subcommand == BT_CMD_ENABLE) {
                return 3; // 1 subcmd + 1 bt_cmd + 1 enable
            }
            return 2; // 1 subcmd + 1 bt_cmd
        case CRSF_SUBCMD_OSD:
            if (sub_subcommand == OSD_CMD_SEND_BUTTONS) {
                return 3; // 1 subcmd + 1 osd_cmd + 1 buttons
            }
            return 2;
        case CRSF_SUBCMD_VTX:
            switch (sub_subcommand) {
                case VTX_CMD_CHANGE_CHANNEL: return 3;
                case VTX_CMD_SET_FREQUENCY: return 4;
                case VTX_CMD_ENABLE_PITMODE: return 3;
                case VTX_CMD_POWERUP_FROM_PITMODE: return 2;
                case VTX_CMD_SET_DYNAMIC_POWER: return 3;
                case VTX_CMD_SET_POWER: return 3;
                default: return 0; // Variable or unknown
            }
        case CRSF_SUBCMD_LED:
            switch (sub_subcommand) {
                case LED_CMD_SET_DEFAULT: return 2;
                case LED_CMD_OVERRIDE_COLOR: return 5;
                case LED_CMD_OVERRIDE_PULSE: return 10;
                case LED_CMD_OVERRIDE_BLINK: return 10;
                case LED_CMD_OVERRIDE_SHIFT: return 7;
                default: return 0;
            }
        case CRSF_SUBCMD_GENERAL:
            switch (sub_subcommand) {
                case GENERAL_CMD_PROTOCOL_SPEED_PROPOSAL: return 7;
                case GENERAL_CMD_PROTOCOL_SPEED_RESPONSE: return 4;
                default: return 0;
            }
        case CRSF_SUBCMD_CROSSFIRE:
            switch (sub_subcommand) {
                case CF_CMD_MODEL_SELECTION: return 3;
                case CF_CMD_REPLY_MODEL_SELECTION: return 3;
                default: return 2; // Most are bare commands
            }
        case CRSF_SUBCMD_FLOW_CONTROL:
            switch (sub_subcommand) {
                case FLOW_CMD_SUBSCRIBE: return 5;
                case FLOW_CMD_UNSUBSCRIBE: return 3;
                default: return 0;
            }
        default: return 0; // Variable length or unknown
    }
}

// Helper function to copy string and advance offset
uint8_t parse_string(const uint8_t* data, uint8_t start_offset, char* dest, uint8_t max_len) {
    const char* src = (const char*)&data[start_offset];
    uint8_t len = strlen(src);

    // Copy string with length limit
    strncpy(dest, src, max_len - 1);
    dest[max_len - 1] = '\0'; // Ensure null termination

    // Return new offset (past the null terminator)
    return start_offset + len + 1U;
}

// Alternative unpack function that auto-detects boundaries
void crsf_unpack_0x32_payload_auto(const uint8_t* payload, void* output_struct) {
    if (!payload || !output_struct) {
        return;
    }

    uint8_t subcommand = payload[0];
    const uint8_t* data = &payload[1];

    switch (subcommand) {
        case CRSF_SUBCMD_COMMAND_ACK: {
            crsf_command_ack_t* ack = (crsf_command_ack_t*)output_struct;
            ack->command_id = data[0];
            ack->subcommand_id = data[1];
            ack->action = data[2];

            // Information string starts at offset 3 and is null-terminated
            const char* info_str = (const char*)&data[3];
            strncpy(ack->information, info_str, sizeof(ack->information) - 1);
            ack->information[sizeof(ack->information) - 1] = '\0';
            break;
        }

        case CRSF_SUBCMD_FC_COMMANDS: {
            crsf_fc_command_t* fc_cmd = (crsf_fc_command_t*)output_struct;
            fc_cmd->fc_command = data[0];
            break;
        }

        case CRSF_SUBCMD_BLUETOOTH: {
            uint8_t bt_cmd = data[0];
            if (bt_cmd == BT_CMD_ENABLE) {
                crsf_bluetooth_enable_t* bt_enable = (crsf_bluetooth_enable_t*)output_struct;
                bt_enable->bt_command = bt_cmd;
                bt_enable->enable = data[1];
            } else {
                crsf_bluetooth_command_t* bt_basic = (crsf_bluetooth_command_t*)output_struct;
                bt_basic->bt_command = bt_cmd;
            }
            break;
        }

        case CRSF_SUBCMD_OSD: {
            uint8_t osd_cmd = data[0];
            if (osd_cmd == OSD_CMD_SEND_BUTTONS) {
                crsf_osd_buttons_t* osd_btn = (crsf_osd_buttons_t*)output_struct;
                osd_btn->osd_command = osd_cmd;
                osd_btn->buttons = data[1];
            }
            break;
        }

        case CRSF_SUBCMD_VTX: {
            uint8_t vtx_cmd = data[0];
            switch (vtx_cmd) {
                case VTX_CMD_CHANGE_CHANNEL: {
                    crsf_vtx_change_channel_t* vtx_ch = (crsf_vtx_change_channel_t*)output_struct;
                    vtx_ch->vtx_command = vtx_cmd;
                    vtx_ch->channel = data[1];
                    break;
                }
                case VTX_CMD_SET_FREQUENCY: {
                    crsf_vtx_set_frequency_t* vtx_freq = (crsf_vtx_set_frequency_t*)output_struct;
                    vtx_freq->vtx_command = vtx_cmd;
                    vtx_freq->frequency = (data[2] << 8) | data[1]; // Little endian
                    break;
                }
                case VTX_CMD_ENABLE_PITMODE: {
                    crsf_vtx_pitmode_t* vtx_pit = (crsf_vtx_pitmode_t*)output_struct;
                    vtx_pit->vtx_command = vtx_cmd;
                    uint8_t packed_data = data[1];
                    vtx_pit->pitmode = packed_data & 0x01;
                    vtx_pit->pitmode_control = (packed_data >> 1) & 0x03;
                    vtx_pit->pitmode_switch = (packed_data >> 3) & 0x0F;
                    vtx_pit->reserved = 0;
                    break;
                }
                case VTX_CMD_POWERUP_FROM_PITMODE: {
                    crsf_vtx_powerup_pitmode_t* vtx_powerup = (crsf_vtx_powerup_pitmode_t*)output_struct;
                    vtx_powerup->vtx_command = vtx_cmd;
                    break;
                }
                case VTX_CMD_SET_DYNAMIC_POWER: {
                    crsf_vtx_dynamic_power_t* vtx_dyn = (crsf_vtx_dynamic_power_t*)output_struct;
                    vtx_dyn->vtx_command = vtx_cmd;
                    vtx_dyn->power_dbm = data[1];
                    break;
                }
                case VTX_CMD_SET_POWER: {
                    crsf_vtx_set_power_t* vtx_pwr = (crsf_vtx_set_power_t*)output_struct;
                    vtx_pwr->vtx_command = vtx_cmd;
                    vtx_pwr->power_dbm = data[1];
                    break;
                }
            }
            break;
        }

        case CRSF_SUBCMD_LED: {
            uint8_t led_cmd = data[0];
            switch (led_cmd) {
                case LED_CMD_SET_DEFAULT: {
                    crsf_led_default_t* led_def = (crsf_led_default_t*)output_struct;
                    led_def->led_command = led_cmd;
                    break;
                }
                case LED_CMD_OVERRIDE_COLOR: {
                    crsf_led_color_t* led_color = (crsf_led_color_t*)output_struct;
                    led_color->led_command = led_cmd;
                    // Unpack 24 bits: 9H + 7S + 8V
                    uint32_t packed = (data[3] << 16) | (data[2] << 8) | data[1];
                    led_color->h = packed & 0x1FF;        // 9 bits
                    led_color->s = (packed >> 9) & 0x7F;  // 7 bits
                    led_color->v = (packed >> 16) & 0xFF; // 8 bits
                    break;
                }
                case LED_CMD_OVERRIDE_PULSE: {
                    crsf_led_pulse_t* led_pulse = (crsf_led_pulse_t*)output_struct;
                    led_pulse->led_command = led_cmd;
                    led_pulse->duration = (data[2] << 8) | data[1];

                    // Unpack start color (24 bits)
                    uint32_t start_packed = (data[5] << 16) | (data[4] << 8) | data[3];
                    led_pulse->h_start = start_packed & 0x1FF;
                    led_pulse->s_start = (start_packed >> 9) & 0x7F;
                    led_pulse->v_start = (start_packed >> 16) & 0xFF;

                    // Unpack stop color (24 bits)
                    uint32_t stop_packed = (data[8] << 16) | (data[7] << 8) | data[6];
                    led_pulse->h_stop = stop_packed & 0x1FF;
                    led_pulse->s_stop = (stop_packed >> 9) & 0x7F;
                    led_pulse->v_stop = (stop_packed >> 16) & 0xFF;
                    break;
                }
                case LED_CMD_OVERRIDE_BLINK: {
                    crsf_led_blink_t* led_blink = (crsf_led_blink_t*)output_struct;
                    led_blink->led_command = led_cmd;
                    led_blink->interval = (data[2] << 8) | data[1];

                    // Unpack start color (24 bits)
                    uint32_t start_packed = (data[5] << 16) | (data[4] << 8) | data[3];
                    led_blink->h_start = start_packed & 0x1FF;
                    led_blink->s_start = (start_packed >> 9) & 0x7F;
                    led_blink->v_start = (start_packed >> 16) & 0xFF;

                    // Unpack stop color (24 bits)
                    uint32_t stop_packed = (data[8] << 16) | (data[7] << 8) | data[6];
                    led_blink->h_stop = stop_packed & 0x1FF;
                    led_blink->s_stop = (stop_packed >> 9) & 0x7F;
                    led_blink->v_stop = (stop_packed >> 16) & 0xFF;
                    break;
                }
                case LED_CMD_OVERRIDE_SHIFT: {
                    crsf_led_shift_t* led_shift = (crsf_led_shift_t*)output_struct;
                    led_shift->led_command = led_cmd;
                    led_shift->interval = (data[2] << 8) | data[1];

                    // Unpack color (24 bits)
                    uint32_t packed = (data[5] << 16) | (data[4] << 8) | data[3];
                    led_shift->h = packed & 0x1FF;
                    led_shift->s = (packed >> 9) & 0x7F;
                    led_shift->v = (packed >> 16) & 0xFF;
                    break;
                }
            }
            break;
        }

        case CRSF_SUBCMD_GENERAL: {
            uint8_t gen_cmd = data[0];
            switch (gen_cmd) {
                case GENERAL_CMD_PROTOCOL_SPEED_PROPOSAL: {
                    crsf_protocol_speed_proposal_t* speed_prop = (crsf_protocol_speed_proposal_t*)output_struct;
                    speed_prop->general_command = gen_cmd;
                    speed_prop->port_id = data[1];
                    speed_prop->proposed_baudrate = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
                    break;
                }
                case GENERAL_CMD_PROTOCOL_SPEED_RESPONSE: {
                    crsf_protocol_speed_response_t* speed_resp = (crsf_protocol_speed_response_t*)output_struct;
                    speed_resp->general_command = gen_cmd;
                    speed_resp->port_id = data[1];
                    speed_resp->response = data[2];
                    break;
                }
            }
            break;
        }

        case CRSF_SUBCMD_CROSSFIRE: {
            uint8_t cf_cmd = data[0];
            switch (cf_cmd) {
                case CF_CMD_MODEL_SELECTION: {
                    crsf_model_selection_t* model_sel = (crsf_model_selection_t*)output_struct;
                    model_sel->cf_command = cf_cmd;
                    model_sel->model_number = data[1];
                    break;
                }
                case CF_CMD_REPLY_MODEL_SELECTION: {
                    crsf_model_selection_reply_t* model_reply = (crsf_model_selection_reply_t*)output_struct;
                    model_reply->cf_command = cf_cmd;
                    model_reply->model_number = data[1];
                    break;
                }
                default: {
                    // Basic crossfire command without additional data
                    crsf_crossfire_command_t* cf_basic = (crsf_crossfire_command_t*)output_struct;
                    cf_basic->cf_command = cf_cmd;
                    break;
                }
            }
            break;
        }

        case CRSF_SUBCMD_FLOW_CONTROL: {
            uint8_t flow_cmd = data[0];
            switch (flow_cmd) {
                case FLOW_CMD_SUBSCRIBE: {
                    crsf_flow_subscribe_t* flow_sub = (crsf_flow_subscribe_t*)output_struct;
                    flow_sub->flow_command = flow_cmd;
                    flow_sub->frame_type = data[1];
                    flow_sub->max_interval_ms = (data[3] << 8) | data[2];
                    break;
                }
                case FLOW_CMD_UNSUBSCRIBE: {
                    crsf_flow_unsubscribe_t* flow_unsub = (crsf_flow_unsubscribe_t*)output_struct;
                    flow_unsub->flow_command = flow_cmd;
                    flow_unsub->frame_type = data[1];
                    break;
                }
            }
            break;
        }

        case CRSF_SUBCMD_SCREEN: {
            uint8_t screen_cmd = data[0];
            switch (screen_cmd) {
                case SCREEN_CMD_POPUP_START: {
                    crsf_screen_popup_start_t* popup = (crsf_screen_popup_start_t*)output_struct;
                    popup->screen_command = screen_cmd;

                    // Parse variable-length strings automatically using null terminators
                    uint8_t offset = 1;

                    // Parse header string (auto-detect end)
                    offset = parse_string(data, offset, popup->header, sizeof(popup->header));

                    // Parse info message string
                    offset = parse_string(data, offset, popup->info_message, sizeof(popup->info_message));

                    // Parse timeout and close button option (fixed size)
                    popup->max_timeout_interval = data[offset++];
                    popup->close_button_option = data[offset++];

                    // Parse optional add_data selection text
                    uint8_t sel_start = offset;
                    offset = parse_string(data, offset, popup->add_data.selection_text, sizeof(popup->add_data.selection_text));

                    // If selection text exists (not just null), parse additional fields
                    if (popup->add_data.selection_text[0] != '\0') {
                        popup->add_data.value = data[offset++];
                        popup->add_data.min_value = data[offset++];
                        popup->add_data.max_value = data[offset++];
                        popup->add_data.default_value = data[offset++];

                        // Parse unit string
                        offset = parse_string(data, offset, popup->add_data.unit, sizeof(popup->add_data.unit));
                    }

                    // Parse possible values string (auto-detect end)
                    parse_string(data, offset, popup->possible_values, sizeof(popup->possible_values));
                    break;
                }
                case SCREEN_CMD_SELECTION_RETURN: {
                    crsf_screen_selection_return_t* sel_ret = (crsf_screen_selection_return_t*)output_struct;
                    sel_ret->screen_command = screen_cmd;
                    sel_ret->value = data[1];
                    sel_ret->response = data[2];
                    break;
                }
            }
            break;
        }

        default:
            // Unknown subcommand - do nothing
            break;
    }
}

// Complete unpack function with length parameter (recommended for safety)
void crsf_unpack_0x32_payload(const uint8_t* payload, uint8_t payload_length, void* output_struct) {
    if (!payload || !output_struct || payload_length == 0) {
        return;
    }

    uint8_t subcommand = payload[0];
    const uint8_t* data = &payload[1];
    uint8_t data_length = payload_length - 1;

    switch (subcommand) {
        case CRSF_SUBCMD_COMMAND_ACK: {
            crsf_command_ack_t* ack = (crsf_command_ack_t*)output_struct;
            if (data_length >= 3) {
                ack->command_id = data[0];
                ack->subcommand_id = data[1];
                ack->action = data[2];
                // Copy remaining data as information string
                uint8_t info_len = data_length - 3;
                if (info_len > 0) {
                    memcpy(ack->information, &data[3], info_len);
                    // Ensure null termination
                    if (info_len < sizeof(((crsf_command_ack_t*)0)->information)) {
                        ack->information[info_len] = '\0';
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_FC_COMMANDS: {
            crsf_fc_command_t* fc_cmd = (crsf_fc_command_t*)output_struct;
            if (data_length >= 1) {
                fc_cmd->fc_command = data[0];
            }
            break;
        }

        case CRSF_SUBCMD_BLUETOOTH: {
            if (data_length >= 1) {
                uint8_t bt_cmd = data[0];
                if (bt_cmd == BT_CMD_ENABLE && data_length >= 2) {
                    crsf_bluetooth_enable_t* bt_enable = (crsf_bluetooth_enable_t*)output_struct;
                    bt_enable->bt_command = bt_cmd;
                    bt_enable->enable = data[1];
                } else {
                    crsf_bluetooth_command_t* bt_basic = (crsf_bluetooth_command_t*)output_struct;
                    bt_basic->bt_command = bt_cmd;
                }
            }
            break;
        }

        case CRSF_SUBCMD_OSD: {
            if (data_length >= 1) {
                uint8_t osd_cmd = data[0];
                if (osd_cmd == OSD_CMD_SEND_BUTTONS && data_length >= 2) {
                    crsf_osd_buttons_t* osd_btn = (crsf_osd_buttons_t*)output_struct;
                    osd_btn->osd_command = osd_cmd;
                    osd_btn->buttons = data[1];
                }
            }
            break;
        }

        case CRSF_SUBCMD_VTX: {
            if (data_length >= 1) {
                uint8_t vtx_cmd = data[0];
                switch (vtx_cmd) {
                    case VTX_CMD_CHANGE_CHANNEL: {
                        if (data_length >= 2) {
                            crsf_vtx_change_channel_t* vtx_ch = (crsf_vtx_change_channel_t*)output_struct;
                            vtx_ch->vtx_command = vtx_cmd;
                            vtx_ch->channel = data[1];
                        }
                        break;
                    }
                    case VTX_CMD_SET_FREQUENCY: {
                        if (data_length >= 3) {
                            crsf_vtx_set_frequency_t* vtx_freq = (crsf_vtx_set_frequency_t*)output_struct;
                            vtx_freq->vtx_command = vtx_cmd;
                            vtx_freq->frequency = (data[2] << 8) | data[1]; // Little endian
                        }
                        break;
                    }
                    case VTX_CMD_ENABLE_PITMODE: {
                        if (data_length >= 2) {
                            crsf_vtx_pitmode_t* vtx_pit = (crsf_vtx_pitmode_t*)output_struct;
                            vtx_pit->vtx_command = vtx_cmd;
                            uint8_t packed_data = data[1];
                            vtx_pit->pitmode = packed_data & 0x01;
                            vtx_pit->pitmode_control = (packed_data >> 1) & 0x03;
                            vtx_pit->pitmode_switch = (packed_data >> 3) & 0x0F;
                            vtx_pit->reserved = 0;
                        }
                        break;
                    }
                    case VTX_CMD_POWERUP_FROM_PITMODE: {
                        crsf_vtx_powerup_pitmode_t* vtx_powerup = (crsf_vtx_powerup_pitmode_t*)output_struct;
                        vtx_powerup->vtx_command = vtx_cmd;
                        break;
                    }
                    case VTX_CMD_SET_DYNAMIC_POWER: {
                        if (data_length >= 2) {
                            crsf_vtx_dynamic_power_t* vtx_dyn = (crsf_vtx_dynamic_power_t*)output_struct;
                            vtx_dyn->vtx_command = vtx_cmd;
                            vtx_dyn->power_dbm = data[1];
                        }
                        break;
                    }
                    case VTX_CMD_SET_POWER: {
                        if (data_length >= 2) {
                            crsf_vtx_set_power_t* vtx_pwr = (crsf_vtx_set_power_t*)output_struct;
                            vtx_pwr->vtx_command = vtx_cmd;
                            vtx_pwr->power_dbm = data[1];
                        }
                        break;
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_LED: {
            if (data_length >= 1) {
                uint8_t led_cmd = data[0];
                switch (led_cmd) {
                    case LED_CMD_SET_DEFAULT: {
                        crsf_led_default_t* led_def = (crsf_led_default_t*)output_struct;
                        led_def->led_command = led_cmd;
                        break;
                    }
                    case LED_CMD_OVERRIDE_COLOR: {
                        if (data_length >= 4) { // 1 + 3 bytes packed data
                            crsf_led_color_t* led_color = (crsf_led_color_t*)output_struct;
                            led_color->led_command = led_cmd;
                            // Unpack 24 bits: 9H + 7S + 8V
                            uint32_t packed = (data[3] << 16) | (data[2] << 8) | data[1];
                            led_color->h = packed & 0x1FF;        // 9 bits
                            led_color->s = (packed >> 9) & 0x7F;  // 7 bits
                            led_color->v = (packed >> 16) & 0xFF; // 8 bits
                        }
                        break;
                    }
                    case LED_CMD_OVERRIDE_PULSE: {
                        if (data_length >= 9) { // 1 + 2 + 6 bytes packed data
                            crsf_led_pulse_t* led_pulse = (crsf_led_pulse_t*)output_struct;
                            led_pulse->led_command = led_cmd;
                            led_pulse->duration = (data[2] << 8) | data[1];

                            // Unpack start color (24 bits)
                            uint32_t start_packed = (data[5] << 16) | (data[4] << 8) | data[3];
                            led_pulse->h_start = start_packed & 0x1FF;
                            led_pulse->s_start = (start_packed >> 9) & 0x7F;
                            led_pulse->v_start = (start_packed >> 16) & 0xFF;

                            // Unpack stop color (24 bits)
                            uint32_t stop_packed = (data[8] << 16) | (data[7] << 8) | data[6];
                            led_pulse->h_stop = stop_packed & 0x1FF;
                            led_pulse->s_stop = (stop_packed >> 9) & 0x7F;
                            led_pulse->v_stop = (stop_packed >> 16) & 0xFF;
                        }
                        break;
                    }
                    case LED_CMD_OVERRIDE_BLINK: {
                        if (data_length >= 9) { // 1 + 2 + 6 bytes packed data
                            crsf_led_blink_t* led_blink = (crsf_led_blink_t*)output_struct;
                            led_blink->led_command = led_cmd;
                            led_blink->interval = (data[2] << 8) | data[1];

                            // Unpack start color (24 bits)
                            uint32_t start_packed = (data[5] << 16) | (data[4] << 8) | data[3];
                            led_blink->h_start = start_packed & 0x1FF;
                            led_blink->s_start = (start_packed >> 9) & 0x7F;
                            led_blink->v_start = (start_packed >> 16) & 0xFF;

                            // Unpack stop color (24 bits)
                            uint32_t stop_packed = (data[8] << 16) | (data[7] << 8) | data[6];
                            led_blink->h_stop = stop_packed & 0x1FF;
                            led_blink->s_stop = (stop_packed >> 9) & 0x7F;
                            led_blink->v_stop = (stop_packed >> 16) & 0xFF;
                        }
                        break;
                    }
                    case LED_CMD_OVERRIDE_SHIFT: {
                        if (data_length >= 6) { // 1 + 2 + 3 bytes packed data
                            crsf_led_shift_t* led_shift = (crsf_led_shift_t*)output_struct;
                            led_shift->led_command = led_cmd;
                            led_shift->interval = (data[2] << 8) | data[1];

                            // Unpack color (24 bits)
                            uint32_t packed = (data[5] << 16) | (data[4] << 8) | data[3];
                            led_shift->h = packed & 0x1FF;
                            led_shift->s = (packed >> 9) & 0x7F;
                            led_shift->v = (packed >> 16) & 0xFF;
                        }
                        break;
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_GENERAL: {
            if (data_length >= 1) {
                uint8_t gen_cmd = data[0];
                switch (gen_cmd) {
                    case GENERAL_CMD_PROTOCOL_SPEED_PROPOSAL: {
                        if (data_length >= 6) {
                            crsf_protocol_speed_proposal_t* speed_prop = (crsf_protocol_speed_proposal_t*)output_struct;
                            speed_prop->general_command = gen_cmd;
                            speed_prop->port_id = data[1];
                            speed_prop->proposed_baudrate = (data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2];
                        }
                        break;
                    }
                    case GENERAL_CMD_PROTOCOL_SPEED_RESPONSE: {
                        if (data_length >= 3) {
                            crsf_protocol_speed_response_t* speed_resp = (crsf_protocol_speed_response_t*)output_struct;
                            speed_resp->general_command = gen_cmd;
                            speed_resp->port_id = data[1];
                            speed_resp->response = data[2];
                        }
                        break;
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_CROSSFIRE: {
            if (data_length >= 1) {
                uint8_t cf_cmd = data[0];
                switch (cf_cmd) {
                    case CF_CMD_MODEL_SELECTION: {
                        if (data_length >= 2) {
                            crsf_model_selection_t* model_sel = (crsf_model_selection_t*)output_struct;
                            model_sel->cf_command = cf_cmd;
                            model_sel->model_number = data[1];
                        }
                        break;
                    }
                    case CF_CMD_REPLY_MODEL_SELECTION: {
                        if (data_length >= 2) {
                            crsf_model_selection_reply_t* model_reply = (crsf_model_selection_reply_t*)output_struct;
                            model_reply->cf_command = cf_cmd;
                            model_reply->model_number = data[1];
                        }
                        break;
                    }
                    default: {
                        // Basic crossfire command without additional data
                        crsf_crossfire_command_t* cf_basic = (crsf_crossfire_command_t*)output_struct;
                        cf_basic->cf_command = cf_cmd;
                        break;
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_FLOW_CONTROL: {
            if (data_length >= 1) {
                uint8_t flow_cmd = data[0];
                switch (flow_cmd) {
                    case FLOW_CMD_SUBSCRIBE: {
                        if (data_length >= 4) {
                            crsf_flow_subscribe_t* flow_sub = (crsf_flow_subscribe_t*)output_struct;
                            flow_sub->flow_command = flow_cmd;
                            flow_sub->frame_type = data[1];
                            flow_sub->max_interval_ms = (data[3] << 8) | data[2];
                        }
                        break;
                    }
                    case FLOW_CMD_UNSUBSCRIBE: {
                        if (data_length >= 2) {
                            crsf_flow_unsubscribe_t* flow_unsub = (crsf_flow_unsubscribe_t*)output_struct;
                            flow_unsub->flow_command = flow_cmd;
                            flow_unsub->frame_type = data[1];
                        }
                        break;
                    }
                }
            }
            break;
        }

        case CRSF_SUBCMD_SCREEN: {
            if (data_length >= 1) {
                uint8_t screen_cmd = data[0];
                switch (screen_cmd) {
                    case SCREEN_CMD_POPUP_START: {
                        crsf_screen_popup_start_t* popup = (crsf_screen_popup_start_t*)output_struct;
                        popup->screen_command = screen_cmd;

                        // Parse variable-length strings and fields
                        uint8_t offset = 1;

                        // Parse header string
                        uint8_t header_len = strnlen((const char*)&data[offset], sizeof(popup->header) - 1);
                        strncpy(popup->header, (const char*)&data[offset], header_len);
                        popup->header[header_len] = '\0';
                        offset += header_len + 1;

                        // Parse info message string
                        if (offset < data_length) {
                            uint8_t info_len = strnlen((const char*)&data[offset], sizeof(popup->info_message) - 1);
                            strncpy(popup->info_message, (const char*)&data[offset], info_len);
                            popup->info_message[info_len] = '\0';
                            offset += info_len + 1;
                        }

                        // Parse timeout and close button option
                        if (offset + 1 < data_length) {
                            popup->max_timeout_interval = data[offset++];
                            popup->close_button_option = data[offset++];
                        }

                        // Parse optional add_data
                        if (offset < data_length) {
                            uint8_t sel_len = strnlen((const char*)&data[offset], sizeof(popup->add_data.selection_text) - 1);
                            strncpy(popup->add_data.selection_text, (const char*)&data[offset], sel_len);
                            popup->add_data.selection_text[sel_len] = '\0';
                            offset += sel_len + 1;

                            // If selection text exists, parse additional fields
                            if (sel_len > 0 && offset + 4 < data_length) {
                                popup->add_data.value = data[offset++];
                                popup->add_data.min_value = data[offset++];
                                popup->add_data.max_value = data[offset++];
                                popup->add_data.default_value = data[offset++];

                                // Parse unit string
                                if (offset < data_length) {
                                    uint8_t unit_len = strnlen((const char*)&data[offset], sizeof(popup->add_data.unit) - 1);
                                    strncpy(popup->add_data.unit, (const char*)&data[offset], unit_len);
                                    popup->add_data.unit[unit_len] = '\0';
                                    offset += unit_len + 1;
                                }
                            }
                        }

                        // Parse possible values string
                        if (offset < data_length) {
                            uint8_t pv_len = strnlen((const char*)&data[offset], sizeof(popup->possible_values) - 1);
                            strncpy(popup->possible_values, (const char*)&data[offset], pv_len);
                            popup->possible_values[pv_len] = '\0';
                        }
                        break;
                    }
                    case SCREEN_CMD_SELECTION_RETURN: {
                        if (data_length >= 3) {
                            crsf_screen_selection_return_t* sel_ret = (crsf_screen_selection_return_t*)output_struct;
                            sel_ret->screen_command = screen_cmd;
                            sel_ret->value = data[1];
                            sel_ret->response = data[2];
                        }
                        break;
                    }
                }
            }
            break;
        }

        default:
            // Unknown subcommand - do nothing
            break;
    }
}