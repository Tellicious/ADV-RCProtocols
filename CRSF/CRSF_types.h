/* BEGIN Header */
/**
 ******************************************************************************
 * \file            CRSF_types.h
 * \author          Andrea Vivani
 * \brief           CRSF protocol-specific types
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

#ifndef __CRSF_TYPES_H__
#define __CRSF_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Configuration -------------------------------------------------------------*/

#define CRSF_MAX_RPM_VALUES         19 /* Max RPM values */
#define CRSF_MAX_TEMPERATURE_VALUES 20 /* Max temperature values */
#define CRSF_MAX_VOLTAGE_VALUES     29 /* Max voltage values */
#define CRSF_RC_CHANNELS            16 /* Number of RC channels */

#ifndef CRSF_MAX_FLIGHT_MODE_NAME_LEN
#define CRSF_MAX_FLIGHT_MODE_NAME_LEN 16 /* Max flight mode name length */
#endif

#ifndef CRSF_MAX_DEVICE_NAME_LEN
#define CRSF_MAX_DEVICE_NAME_LEN 16 /* Max device name length */
#endif

#define CRSF_MAX_PARAM_SETTINGS_PAYLOAD 56 /* Max parameter settings payload */

#ifndef CRSF_MAX_PARAM_DATA_LEN
#define CRSF_MAX_PARAM_DATA_LEN 32 /* Max parameter value length */
#endif

#ifndef CRSF_MAX_COMMAND_PAYLOAD
#define CRSF_MAX_COMMAND_PAYLOAD 56 /* Max command payload */
#endif

#ifndef CRSF_MAX_COMMAND_PAYLOAD_STRINGS
#define CRSF_MAX_COMMAND_PAYLOAD_STRINGS 20 /* Max size of command payload strings */
#endif

#if CRSF_MAX_COMMAND_PAYLOAD_STRINGS > CRSF_MAX_COMMAND_PAYLOAD - 3
#undef CRSF_MAX_COMMAND_PAYLOAD_STRINGS
#define CRSF_MAX_COMMAND_PAYLOAD_STRINGS CRSF_MAX_COMMAND_PAYLOAD - 3
#endif

#ifndef CRSF_MAX_MAVLINK_PAYLOAD
#define CRSF_MAX_MAVLINK_PAYLOAD 58 /* Max MAVLink payload */
#endif

#ifndef CRSF_MAX_PARAM_STRING_LENGTH
#define CRSF_MAX_PARAM_STRING_LENGTH 20 /* Max parameter string length */
#endif

#ifndef CRSF_MAX_PARAM_SETTINGS_PAYLOAD
#define CRSF_MAX_PARAM_SETTINGS_PAYLOAD 56 /* Max parameter settings payload */
#endif

#ifndef CRSF_MIN_STRING_LENGTH
#define CRSF_MIN_STRING_LENGTH 2 /* Min string length */
#endif

/* Characteristics -----------------------------------------------------------*/

#define CRSF_MIN_FRAME_LEN   4U  // Including addr byte and Frame Length
#define CRSF_MAX_FRAME_LEN   64U // Including addr byte and Frame Length
#define CRSF_STD_HDR_SIZE    3U  /* addr,len,type */
#define CRSF_CRC_SIZE        1U
#define CRSF_MAX_PAYLOAD_LEN (CRSF_MAX_FRAME_LEN - CRSF_CRC_SIZE - CRSF_STD_HDR_SIZE)

/* Typedefs ------------------------------------------------------------------*/

/**
 * Addresses
 */
typedef enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
    CRSF_ADDRESS_ELRS_LUA = 0xEF
} CRSF_Address_t;

/**
 * Frame types
 */
typedef enum {
    /* Telemetry (standard header) */
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_GPS_TIME = 0x03,
    CRSF_FRAMETYPE_GPS_EXTENDED = 0x06,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BAROALT_VSPEED = 0x09,
    CRSF_FRAMETYPE_AIRSPEED = 0x0A,
    CRSF_FRAMETYPE_HEARTBEAT = 0x0B,
    CRSF_FRAMETYPE_RPM = 0x0C,
    CRSF_FRAMETYPE_TEMPERATURE = 0x0D,
    CRSF_FRAMETYPE_VOLTAGES = 0x0E,
    CRSF_FRAMETYPE_DISCONTINUED = 0x0F,
    CRSF_FRAMETYPE_VTX = 0x10,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS = 0x17,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED_UNUSED = 0x18, //Unused
    CRSF_FRAMETYPE_RESERVED_CROSSFIRE_1 = 0x19,      //Reserved
    CRSF_FRAMETYPE_RESERVED_CROSSFIRE_2 = 0x1A,      //Reserved
    CRSF_FRAMETYPE_RESERVED_CROSSFIRE_3 = 0x1B,      //Reserved
    CRSF_FRAMETYPE_LINK_STATISTICS_RX = 0X1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX = 0X1D,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_MAVLINK_FC = 0x1F,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    CRSF_FRAMETYPE_ESP_NOW_MESSAGES = 0X22,
    CRSF_FRAMETYPE_RESERVED = 0X27, //Reserved

    /* Extended header frames (TBS) 0x28-0x96 */
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    /* Direct Command */
    CRSF_FRAMETYPE_COMMAND = 0x32,

    /* Other */
    CRSF_FRAMETYPE_LOGGING = 0x34,    //Not implemented
    CRSF_FRAMETYPE_RESERVED_2 = 0x36, //Reserved
    CRSF_FRAMETYPE_RESERVED_3 = 0x38, //Reserved
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,   //Not implemented
    CRSF_FRAMETYPE_GAME = 0x3C,       //Not implemented
    CRSF_FRAMETYPE_RESERVED_4 = 0x3E, //Reserved
    CRSF_FRAMETYPE_RESERVED_5 = 0x40, //Reserved

    /* MSP/KISS chunked */
    CRSF_FRAMETYPE_KISS_REQ = 0x78,  //Not implemented
    CRSF_FRAMETYPE_KISS_RESP = 0x79, //Not implemented
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   //Not implemented
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  //Not implemented
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
    /* Ardupilot frames */
    CRSF_FRAMETYPE_ARDUPILOT_LEGACY = 0x7F, //Reserved
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,   //Reserved

    /* mLRS reserved */
    CRSF_FRAMETYPE_MLRS_RESERVED_1 = 0x81, //Reserved
    CRSF_FRAMETYPE_MLRS_RESERVED_2 = 0x82, //Reserved

    /* MAVLink */
    CRSF_FRAMETYPE_MAVLINK_ENVELOPE = 0xAA,
    CRSF_FRAMETYPE_MAVLINK_STATUS = 0xAC,
} CRSF_FrameType_t;

/**
 * Command types
 */
typedef enum {
    CRSF_CMDID_COMMAND_ACK = 0xFF, // 0x32.0xFF Command ACK
    CRSF_CMDID_FC = 0x01,          // 0x32.0x01 FC Commands
    CRSF_CMDID_BLUETOOTH = 0x03,   // 0x32.0x03 Bluetooth Command
    CRSF_CMDID_OSD = 0x05,         // 0x32.0x05 OSD Commands
    CRSF_CMDID_VTX = 0x08,         // 0x32.0x08 VTX Commands
    CRSF_CMDID_LED = 0x09,         // 0x32.0x09 LED
    CRSF_CMDID_GENERAL = 0x0A,     // 0x32.0x0A General
    CRSF_CMDID_CROSSFIRE = 0x10,   // 0x32.0x10 Crossfire
    CRSF_CMDID_RESERVED_12 = 0x12, // 0x32.0x12 Reserved
    CRSF_CMDID_FLOW_CTRL = 0x20,   // 0x32.0x20 Flow Control Frame
    CRSF_CMDID_SCREEN = 0x22       // 0x32.0x22 Screen Command
} CRSF_CommandID_t;

/**
 * Parameter types for device configuration
 */
typedef enum {
    CRSF_PARAM_UINT8 = 0,  // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_INT8 = 1,   // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_UINT16 = 2, // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_INT16 = 3,  // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_UINT32 = 4, // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_INT32 = 5,  // Deprecated, use CRSF_FLOAT
    CRSF_PARAM_FLOAT = 8,
    CRSF_PARAM_TEXT_SELECTION = 9,
    CRSF_PARAM_STRING = 10,
    CRSF_PARAM_FOLDER = 11,
    CRSF_PARAM_INFO = 12,
    CRSF_PARAM_COMMAND = 13,
    CRSF_PARAM_OUT_OF_RANGE = 127
} CRSF_ParamType_t;

/**
 * 0x32.0x01 FC Commands
 */
typedef enum {
    CRSF_CMD_FC_FORCE_DISARM = 0x01, // Force Disarm
    CRSF_CMD_FC_SCALE_CHANNEL = 0x02 // Scale Channel (payload unspecified in spec)
} CRSF_CommandFC_subCMD_t;

/**
 * 0x32.0x03 Bluetooth Command
 */
typedef enum {
    CRSF_CMD_BT_RESET = 0x01,  // Reset
    CRSF_CMD_BT_ENABLE = 0x02, // Enable (uint8 Enable: 0/1)
    CRSF_CMD_BT_ECHO = 0x64    // Echo
} CRSF_CommandBT_subCMD_t;

/**
 * 0x32.0x05 OSD Commands
 */
typedef enum {
    CRSF_CMD_OSD_SEND_BUTTONS = 0x01 // Send Buttons (uint8 Buttons bitwise)
} CRSF_CommandOSD_subCMD_t;

/**
 * 0x32.0x08 VTX Commands
 */
typedef enum {
    CRSF_CMD_VTX_CHANGE_CHANNEL = 0x01,        // DISCONTINUED
    CRSF_CMD_VTX_SET_FREQUENCY = 0x02,         // uint16 Frequency (MHz in range 5000–6000)
    CRSF_CMD_VTX_CHANGE_POWER = 0x03,          // DISCONTINUED (moved to 0x08)
    CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP = 0x04, // packed pitmode byte
    CRSF_CMD_VTX_POWER_UP_FROM_PITMODE = 0x05, // bare command
    CRSF_CMD_VTX_SET_DYNAMIC_POWER = 0x06,     // uint8 Power (dBm)
    CRSF_CMD_VTX_SET_POWER = 0x08              // uint8 Power (dBm)
} CRSF_CommandVTX_subCMD_t;

/**
 * 0x32.0x09 LED
 */
typedef enum {
    CRSF_CMD_LED_SET_TO_DEFAULT = 0x01,
    CRSF_CMD_LED_OVERRIDE_COLOR = 0x02, // packed HSV
    CRSF_CMD_LED_OVERRIDE_PULSE = 0x03, // uint16 duration + HSV(start) + HSV(stop)
    CRSF_CMD_LED_OVERRIDE_BLINK = 0x04, // uint16 interval + HSV(start) + HSV(stop)
    CRSF_CMD_LED_OVERRIDE_SHIFT = 0x05  // uint16 interval + HSV
} CRSF_CommandLED_subCMD_t;

/**
 * 0x32.0x0A General
 */
typedef enum {
    // 0x04 and 0x61 are reserved in the spec
    CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL = 0x70,         // uint8 port_id + uint32 proposed_baudrate
    CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE = 0x71 // uint8 port_id + uint8_t response
} CRSF_CommandGen_subCMD_t;

/**
 * 0x32.0x10 Crossfire
 */
typedef enum {
    CRSF_CMD_CF_SET_RX_BIND_MODE = 0x01,
    CRSF_CMD_CF_CANCEL_BIND_MODE = 0x02,
    CRSF_CMD_CF_SET_BIND_ID = 0x03,         // payload not detailed by spec
    CRSF_CMD_CF_MODEL_SELECTION = 0x05,     // uint8 Model Number
    CRSF_CMD_CF_CURRENT_MODEL_QUERY = 0x06, // query
    CRSF_CMD_CF_CURRENT_MODEL_REPLY = 0x07  // uint8 Model Number
    // 0x08, 0x09 reserved
} CRSF_CommandCF_subCMD_t;

/**
 * 0x32.0x20 Flow Control Frame
 */
typedef enum {
    CRSF_CMD_FLOW_SUBSCRIBE = 0x01,  // uint8 Frame type + uint16 Max interval time (ms)
    CRSF_CMD_FLOW_UNSUBSCRIBE = 0x02 // uint8 Frame type
} CRSF_CommandFlow_subCMD_t;

/**
 * 0x32.0x22 Screen Command
 */
typedef enum {
    CRSF_CMD_SCREEN_POPUP_MESSAGE_START = 0x01,
    CRSF_CMD_SCREEN_SELECTION_RETURN = 0x02
    // 0x03, 0x04 reserved
} CRSF_CommandScreen_subCMD_t;

/* Payloads ------------------------------------------------------------------*/

#pragma pack(push, 1)

/**
 * CRSF_FRAMETYPE_GPS payload
 */
typedef struct {
    int32_t latitude;     // degree / 10`000`000
    int32_t longitude;    // degree / 10`000`000
    uint16_t groundspeed; // km/h / 100
    uint16_t heading;     // degree / 100
    uint16_t altitude;    // meter - 1000m offset
    uint8_t satellites;   // # of sats in view
} CRSF_GPS_t;

/**
 * CRSF_FRAMETYPE_GPS_TIME payload
 */
typedef struct {
    int16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} CRSF_GPS_Time_t;

/**
 * CRSF_FRAMETYPE_GPS_EXTENDED payload
 */
typedef struct {
    uint8_t fix_type;      // Current GPS fix quality
    int16_t n_speed;       // Northward (north = positive) Speed [cm/sec]
    int16_t e_speed;       // Eastward (east = positive) Speed [cm/sec]
    int16_t v_speed;       // Vertical (up = positive) Speed [cm/sec]
    int16_t h_speed_acc;   // Horizontal Speed accuracy cm/sec
    int16_t track_acc;     // Heading accuracy in degrees scaled with 1e-1 degrees times 10)
    int16_t alt_ellipsoid; // Meters Height above GPS Ellipsoid (not MSL)
    int16_t h_acc;         // horizontal accuracy in cm
    int16_t v_acc;         // vertical accuracy in cm
    uint8_t reserved;
    uint8_t hDOP; // Horizontal dilution of precision,Dimensionless in nits of.1.
    uint8_t vDOP; // vertical dilution of precision, Dimensionless in nits of .1.
} CRSF_GPS_Ext_t;

/**
 * CRSF_FRAMETYPE_VARIO payload
 */
typedef struct {
    int16_t v_speed; // Vertical speed cm/s
} CRSF_Vario_t;

/**
 * CRSF_FRAMETYPE_BATTERY payload
 */
typedef struct {
    int16_t voltage; // Voltage (LSB = 0.1 V)
    int16_t current; // Current (LSB = 0.1 A)
    //uint24_t capacity_used; // Capacity used (mAh) - as per specs
    uint32_t capacity_used; // Capacity used (mAh)
    uint8_t remaining;      // Battery remaining (percent)
} CRSF_Battery_t;

/**
 * CRSF_FRAMETYPE_BAROALT_VSPEED payload
 */
/** Payload sent
typedef struct {
    uint16_t altitude_packed;     // Altitude above start (calibration) point
                                  // Altitude value depends on MSB (bit 15):
                                  //   MSB = 0: altitude is in decimeters - 10000dm offset (so 0 represents -1000m; 10000 represents 0m (starting altitude); 0x7fff represents 2276.7m);
                                  //   MSB = 1: altitude is in meters. Without any offset.
    int8_t vertical_speed_packed; // vertical speed is represented in cm/s with logarithmic scale
}
CRSF_BaroAlt_VS_t;*/
typedef struct {
    int32_t altitude;       // Altitude above start (calibration) point in dm
    int16_t vertical_speed; // Vertical speed in cm/s
} CRSF_BaroAlt_VS_t;

/** 
 * CRSF_FRAMETYPE_AIRSPEED payload
 */
typedef struct {
    uint16_t speed; // Airspeed in 0.1 * km/h (hectometers/h)
} CRSF_Airspeed_t;

/** 
 * CRSF_FRAMETYPE_HEARTBEAT payload
 */
typedef struct {
    int16_t origin_address; // Origin Device address
} CRSF_Heartbeat_t;

/** 
 * CRSF_FRAMETYPE_RPM payload
 */
typedef struct {
    uint8_t rpm_source_id; // Identifies the source of the RPM data (e.g., 0 = Motor 1, 1 = Motor 2, etc.)
    //int24_t rpm_value[CRSF_MAX_RPM_VALUES]; // 1 - 19 RPM values with negative ones representing the motor spinning in reverse - payload sent
    int32_t rpm_value[CRSF_MAX_RPM_VALUES]; // 1 - 19 RPM values with negative ones representing the motor spinning in reverse
} CRSF_RPM_t;

/** 
 * CRSF_FRAMETYPE_TEMPERATURE payload
 */
typedef struct {
    uint8_t temp_source_id; // Identifies the source of the temperature data (e.g., 0 = FC including all ESCs, 1 = Ambient, etc.)
    int16_t
        temperature[CRSF_MAX_TEMPERATURE_VALUES]; // up to 20 temperature values in deci-degree (tenths of a degree) Celsius (e.g., 250 = 25.0°C, -50 = -5.0°C)
} CRSF_Temperature_t;

/** 
 * CRSF_FRAMETYPE_VOLTAGES payload
 */
typedef struct {
    uint8_t Voltage_source_id;                        // source of the voltages
    uint16_t Voltage_values[CRSF_MAX_VOLTAGE_VALUES]; // Up to 29 voltages in millivolts (e.g. 3.850V = 3850)
} CRSF_Voltages_t;

/**
 * CRSF_FRAMETYPE_VTX payload.
 */
typedef struct {
    uint8_t origin_address;
    uint8_t power_dBm;           // VTX power in dBm
    uint16_t frequency_MHz;      // VTX frequency in MHz
    uint8_t pit_mode        : 1; // 0=Off, 1=On
    uint8_t pitmode_control : 2; // 0=Off, 1=On, 2=Switch, 3=Failsafe
    uint8_t pitmode_switch  : 4; // 0=Ch5, 1=Ch5 Inv, … , 15=Ch12 Inv
} CRSF_VTX_t;

/**
 * CRSF_FRAMETYPE_LINK_STATISTICS payload
 */
typedef struct {
    uint8_t up_rssi_ant1;      // Uplink RSSI Antenna 1 (dBm * -1)
    uint8_t up_rssi_ant2;      // Uplink RSSI Antenna 2 (dBm * -1)
    uint8_t up_link_quality;   // Uplink Package success rate / Link quality (%)
    int8_t up_snr;             // Uplink SNR (dB)
    uint8_t active_antenna;    // number of currently best antenna
    uint8_t rf_profile;        // enum {4fps = 0 , 50fps, 150fps}
    uint8_t up_rf_power;       // enum {0mW = 0, 10mW, 25mW, 100mW,
                               // 500mW, 1000mW, 2000mW, 250mW, 50mW}
    uint8_t down_rssi;         // Downlink RSSI (dBm * -1)
    uint8_t down_link_quality; // Downlink Package success rate / Link quality (%)
    int8_t down_snr;           // Downlink SNR (dB)
} CRSF_LinkStatistics_t;

/**
 * CRSF_FRAMETYPE_CHANNELS_PACKED payload
 */
typedef union {
    struct {
        uint16_t channel_01;
        uint16_t channel_02;
        uint16_t channel_03;
        uint16_t channel_04;
        uint16_t channel_05;
        uint16_t channel_06;
        uint16_t channel_07;
        uint16_t channel_08;
        uint16_t channel_09;
        uint16_t channel_10;
        uint16_t channel_11;
        uint16_t channel_12;
        uint16_t channel_13;
        uint16_t channel_14;
        uint16_t channel_15;
        uint16_t channel_16;
    };

    uint16_t channels[CRSF_RC_CHANNELS]; // Range 1000-2000
} CRSF_RC_Channels_t;

/**
 * CRSF_FRAMETYPE_SUBSET_RC_CHANNELS payload - in revision
 */
// #define PACK_TX(x)      ((x - 3750) * 8 / 25 + 993)
//#define UNPACK_RX(x, S) (x * S + 988)
// S = 1.0 for 10-bit, S = 0.5 for 11-bit, S = 0.25 for 12-bit, S = 0.125 for 13-bit
/*typedef struct {
    uint8_t starting_channel    : 5;        // which channel number is the first one in the frame
    uint8_t res_configuration   : 2;        // configuration for the RC data resolution
                                            // (10 - 13 bits)
    uint8_t digital_switch_flag : 1;        // configuration bit for digital channel
    uint16_t channel[] : resolution;        // variable amount of channels
                                            // (with variable resolution based on the
                                            // res_configuration)
                                            // based on the frame size
    uint16_t digital_switch_channel[] : 10; // digital switch channel
} CRSF_RC_ChannelsSubset_t;*/

/**
 * CRSF_FRAMETYPE_LINK_STATISTICS_RX payload
 */
typedef struct {

    uint8_t rssi_db;      // RSSI (dBm * -1)
    uint8_t rssi_percent; // RSSI in percent
    uint8_t link_quality; // Package success rate / Link quality (%)
    int8_t snr;           // SNR (dB)
    uint8_t rf_power_db;  // rf power in dBm
} CRSF_LinkStatisticsRX_t;

/**
 * CRSF_FRAMETYPE_LINK_STATISTICS_TX payload
 */
typedef struct {
    uint8_t rssi_db;      // RSSI (dBm * -1)
    uint8_t rssi_percent; // RSSI in percent
    uint8_t link_quality; // Package success rate / Link quality (%)
    int8_t snr;           // SNR (dB)
    uint8_t rf_power_db;  // rf power in dBm
    uint8_t fps;          // rf frames per second (fps / 10)
} CRSF_LinkStatisticsTX_t;

/**
 * CRSF_FRAMETYPE_ATTITUDE payload
 */
typedef struct {
    int16_t pitch; // Pitch angle (LSB = 100 µrad)
    int16_t roll;  // Roll angle  (LSB = 100 µrad)
    int16_t yaw;   // Yaw angle   (LSB = 100 µrad)
} CRSF_Attitude_t;

/**
 * CRSF_FRAMETYPE_MAVLINK_FC payload
 */
typedef struct {
    int16_t airspeed;
    uint8_t base_mode;      // vehicle mode flags, defined in MAV_MODE_FLAG enum
    uint32_t custom_mode;   // autopilot-specific flags
    uint8_t autopilot_type; // FC type; defined in MAV_AUTOPILOT enum
    uint8_t firmware_type;  // vehicle type; defined in MAV_TYPE enum
} CRSF_MAVLinkFC_t;

/**
 * CRSF_FRAMETYPE_FLIGHT_MODE payload
 */
typedef struct {
    char flight_mode[CRSF_MAX_FLIGHT_MODE_NAME_LEN]; // Null-terminated string
} CRSF_FlightMode_t;

/**
 * CRSF_FRAMETYPE_ESP_NOW_MESSAGES payload
 */
typedef struct {
    uint8_t VAL1;       // Used for Seat Position of the Pilot
    uint8_t VAL2;       // Used for the Current Pilots Lap
    char VAL3[15];      // 15 characters for the lap time current/split
    char VAL4[15];      // 15 characters for the lap time current/split
    char FREE_TEXT[20]; // Free text of 20 character at the bottom of the screen
} CRSF_ESPNowMessages_t;

/**
 * CRSF_FRAMETYPE_DEVICE_PING payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
} CRSF_Ping_t;

/**
 * CRSF_FRAMETYPE_DEVICE_INFO payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
    char Device_name[CRSF_MAX_DEVICE_NAME_LEN]; // Null-terminated string
    uint32_t Serial_number;
    uint32_t Hardware_ID;
    uint32_t Firmware_ID;
    uint8_t Parameters_total; // Total amount of parameters
    uint8_t Parameter_version_number;
} CRSF_DeviceInfo_t;

/**
 * Command parameter status
*/
typedef enum {
    READY = 0,               //--> feedback
    START = 1,               //<-- input
    PROGRESS = 2,            //--> feedback
    CONFIRMATION_NEEDED = 3, //--> feedback
    CONFIRM = 4,             //<-- input
    CANCEL = 5,              //<-- input
    POLL = 6                 //<-- input
} CRSF_ParamEntryCommandStatus_t;

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
        uint8_t children[CRSF_MAX_PARAM_SETTINGS_PAYLOAD - 1];
        uint8_t childrenCnt;
    } folder; // folder

    struct {
        char text[CRSF_MAX_PARAM_STRING_LENGTH];
    } info; // info

    struct {
        CRSF_ParamEntryCommandStatus_t status;
        uint8_t timeout; // ms * 100
        char info[CRSF_MAX_PARAM_STRING_LENGTH];
    } cmd; // command
} CRSF_ParamEntry_t;

/**
 * CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
    uint8_t Parameter_number;           // Starting from 0
    uint8_t Parameter_chunks_remaining; // Chunks remaining count        uint8_t hidden;                          // from high bit of type byte
    uint8_t parent;                     // parent field index; 0xFF for root

    union {
        struct {
            uint8_t v      : 7;
            uint8_t hidden : 1;
        };

        uint8_t byte;
    } type;

    char name[CRSF_MAX_PARAM_STRING_LENGTH]; // NULL-terminated string in input buffer
    CRSF_ParamEntry_t payload;               // inner payload
} CRSF_ParamSettingsEntry_t;

/**
 * CRSF_FRAMETYPE_PARAMETER_READ payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
    uint8_t Parameter_number;
    uint8_t Parameter_chunk_number; // Chunk number to request, starts with 0
} CRSF_ParamRead_t;

/**
 * CRSF_FRAMETYPE_PARAMETER_WRITE payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
    uint8_t Parameter_number;
    uint8_t Data[CRSF_MAX_PARAM_DATA_LEN];
} CRSF_ParamWrite_t;

/**
 * Command inner payload
 */
typedef union {
    struct {
        uint8_t Command_ID;                              // echoed realm
        uint8_t SubCommand_ID;                           // echoed subcommand
        uint8_t Action;                                  // 1=already took action; 0=no function/invalid
        char Information[CRSF_MAX_COMMAND_PAYLOAD - 3U]; // optional NULL-terminated string (may be empty)
    } ACK;                                               // 0xFF

    // FC
    struct {
        CRSF_CommandFC_subCMD_t subCommand;
    } FC;

    // Bluetooth
    struct {
        CRSF_CommandBT_subCMD_t subCommand;
        uint8_t Enable;
    } Bluetooth;

    // OSD
    struct {
        CRSF_CommandOSD_subCMD_t subCommand;

        union {
            struct {
                uint8_t enter    : 1;
                uint8_t up       : 1;
                uint8_t down     : 1;
                uint8_t left     : 1;
                uint8_t right    : 1;
                uint8_t _padding : 3;
            };

            uint8_t buttons;
        };
    } OSD;

    // VTX
    struct {
        CRSF_CommandVTX_subCMD_t subCommand;

        union {
            uint16_t FrequencyMHz;

            union {
                struct {
                    uint8_t PitMode         : 1; //(0 = OFF, 1 = ON)
                    uint8_t pitmode_control : 2; //(0=Off, 1=On, 2=Arm, 3=Failsafe)
                    uint8_t pitmode_switch  : 4; //(0=Ch5, 1=Ch5 Inv, … , 15=Ch12 Inv)
                };

                uint8_t pitModeCfg;
            };

            uint8_t Power_dBm;
        };
    } VTX;

    // LED
    struct {
        CRSF_CommandLED_subCMD_t subCommand;

        union {

            struct {
                uint16_t H;
                uint8_t S;
                uint8_t V;
            } overrideColor;

            struct {
                uint16_t duration_ms;
                uint16_t H_start;
                uint8_t S_start;
                uint8_t V_start;
                uint16_t H_stop;
                uint8_t S_stop;
                uint8_t V_stop;
            } overridePulse;

            struct {
                uint16_t interval_ms;
                uint16_t H_start;
                uint8_t S_start;
                uint8_t V_start;
                uint16_t H_stop;
                uint8_t S_stop;
                uint8_t V_stop;
            } overrideBlink;

            struct {
                uint16_t interval_ms;
                uint16_t H;
                uint8_t S;
                uint8_t V;
            } overrideShift;
        };
    } LED;

    // General
    struct {
        CRSF_CommandGen_subCMD_t subCommand;

        union {
            struct {

                uint8_t port_id;
                uint32_t proposed_baudrate;
            } protocolSpeedProposal;

            struct {
                uint8_t port_id;
                uint8_t response;
            } protocolSpeedResponse;
        };
    } general;

    // Crossfire
    struct {
        CRSF_CommandCF_subCMD_t subCommand;

        union {
            struct {
                uint8_t bytes[CRSF_MAX_COMMAND_PAYLOAD - 1U];
                uint8_t len;
            } setBindId; // unspecified

            uint8_t Model_Number;
        };
    } crossfire;

    // Flow control
    struct {
        CRSF_CommandFlow_subCMD_t subCommand;
        uint8_t Frame_type;
        uint16_t Max_interval_time_ms;
    } flow;

    // Screen
    struct {
        CRSF_CommandScreen_subCMD_t subCommand;

        union {
            struct {
                char Header[CRSF_MAX_COMMAND_PAYLOAD_STRINGS];       // NULL-terminated
                char Info_message[CRSF_MAX_COMMAND_PAYLOAD_STRINGS]; // NULL-terminated
                uint8_t Max_timeout_interval;                        // seconds
                uint8_t Close_button_option;                         // 0/1

                struct {
                    uint8_t present;                                      // true if selectionText not empty
                    char selectionText[CRSF_MAX_COMMAND_PAYLOAD_STRINGS]; // NULL-terminated; if empty, this whole struct is absent
                    uint8_t value;
                    uint8_t minValue;
                    uint8_t maxValue;
                    uint8_t defaultValue;
                    char unit[5]; // NULL-terminated
                } add_data;

                uint8_t has_possible_values;
                char possible_values[CRSF_MAX_COMMAND_PAYLOAD_STRINGS]; // semicolon-separated NULL-terminated
            } popupMessageStart;

            struct {
                uint8_t value;    // selected value
                uint8_t response; // true(Process) / false(Cancel)
            } selectionReturn;
        };
    } screen;
} CRSF_CommandPayload_t;

/**
 * CRSF_FRAMETYPE_COMMAND payload
 */
typedef struct {
    uint8_t dest_address;
    uint8_t origin_address;
    CRSF_CommandID_t Command_ID;
    CRSF_CommandPayload_t payload; // Depending on Command ID
    // uint8_t Command_CRC8;                      // 8 bit CRC POLYNOM = 0xBA
} CRSF_Command_t;

/**
 * CRSF_FRAMETYPE_MAVLINK_ENVELOPE payload
 */
typedef struct {
    uint8_t total_chunks  : 4;              // total count of chunks
    uint8_t current_chunk : 4;              // current chunk number
    uint8_t data_size;                      // size of data (max 58)
    uint8_t data[CRSF_MAX_MAVLINK_PAYLOAD]; // data array (58 bytes max)
} CRSF_MAVLinkEnv_t;

/**
 * CRSF_FRAMETYPE_MAVLINK_STATUS payload
 */
typedef struct {
    uint32_t sensor_present;
    uint32_t sensor_enabled;
    uint32_t sensor_health;
} CRSF_MAVLinkStat_t;

#pragma pack(pop)

/* Byte Conversion -----------------------------------------------------------*/

#if defined(__GNUC__) || defined(__clang__)
#define BSWAP32(x) __builtin_bswap32(x)
#define BSWAP16(x) __builtin_bswap16(x)
#elif defined(_MSC_VER)
#include <stdlib.h>
#define BSWAP32(x) _byteswap_ulong(x)
#define BSWAP16(x) _byteswap_ushort(x)
#else
#define BSWAP32(x) ((((x) & 0xFF000000U) >> 24) | (((x) & 0x00FF0000U) >> 8) | (((x) & 0x0000FF00U) << 8) | (((x) & 0x000000FFU) << 24))
#define BSWAP16(x) ((((x) & 0xFF00U) >> 8) | (((x) & 0x00FFU) << 8))
#endif

#if !defined(__linux__)
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#define HTOBE16(VAL) (VAL)
#define HTOBE32(VAL) (VAL)
#define BE16TOH(VAL) (VAL)
#define BE32TOH(VAL) (VAL)
#else
#define HTOBE16(VAL) BSWAP16(VAL)
#define HTOBE32(VAL) BSWAP32(VAL)
#define BE16TOH(VAL) BSWAP16(VAL)
#define BE32TOH(VAL) BSWAP32(VAL)
#endif
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CRSF_TYPES_H__ */
