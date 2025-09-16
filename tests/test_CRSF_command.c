/* BEGIN: test_CRSF_command_0x32.c (C version)
 * Tests for 0x32 Direct Command frame: build (TX), parse (RX), roundtrip (TX+RX).
 * Covers ALL sub-types + nested variants with:
 *  - one minimal/no-optional
 *  - one with add_data
 *  - one with possible_values
 *  - one maximal case (for screen popup)
 */

#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cmocka.h>

#include "CRSF.h"

/* ----------------------------- Helpers ---------------------------------- */

static uint8_t _crc_step(uint8_t crc, uint8_t byte, uint8_t poly) {
    crc ^= byte;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ poly) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t calc_checksum(const uint8_t* data, uint8_t len, uint8_t poly) {
    uint8_t c = 0;
    for (uint8_t i = 0; i < len; i++) {
        c = _crc_step(c, data[i], poly);
    }
    return c;
}

/* Golden builder for Direct Command (0x32)
 * Frame: [addr][len][0x32][dest][origin][cmd_id][payload...][inner_crc(0xBA)][outer_crc(0xD5)]
 */
static void make_cmd_frame(uint8_t bus_addr, uint8_t dest, uint8_t origin, uint8_t cmd_id, const uint8_t* pl, uint8_t pl_len, uint8_t* out, uint8_t* out_len) {
    uint8_t* pay = out + CRSF_STD_HDR_SIZE;
    uint8_t off = 0;

    out[0] = bus_addr;
    out[2] = CRSF_FRAMETYPE_COMMAND;

    pay[off++] = dest;
    pay[off++] = origin;
    pay[off++] = cmd_id;

    if (pl && pl_len) {
        memcpy(pay + off, pl, pl_len);
        off = (uint8_t)(off + pl_len);
    }

    /* inner (0xBA) over [type..last payload] */
    pay[off++] = calc_checksum(out + 2U, (uint8_t)(1U + 2U + 1U + pl_len), 0xBAU);

    /* outer (0xD5) */
    {
        uint8_t len_plus_type = (uint8_t)(1U + off);
        out[1] = (uint8_t)(len_plus_type + CRSF_CRC_SIZE);
        out[CRSF_STD_HDR_SIZE + off] = calc_checksum(out + 2U, len_plus_type, 0xD5U);
        *out_len = (uint8_t)(CRSF_STD_HDR_SIZE + off + CRSF_CRC_SIZE);
    }
}

static void pack_cstr(uint8_t** pp, const char* s) {
    size_t n = strlen(s);
    memcpy(*pp, s, n);
    *pp += n;
    **pp = '\0';
    *pp += 1;
}

/* arbitrary addresses */
enum { BUS = CRSF_ADDRESS_FLIGHT_CONTROLLER, DEST = 0xE1, ORIG = 0xE2 };

/* ------------------------- 0xFF COMMAND ACK ----------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_ack_min(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t b[64], bl = 0, g[64], gl = 0, p[3];
    CRSF_init(&s);
    s.Command.dest_address = DEST;
    s.Command.origin_address = ORIG;
    s.Command.Command_ID = CRSF_CMDID_COMMAND_ACK;
    s.Command.payload.ACK.Command_ID = CRSF_CMDID_FC;
    s.Command.payload.ACK.SubCommand_ID = CRSF_CMD_FC_FORCE_DISARM;
    s.Command.payload.ACK.Action = 1;
    s.Command.payload.ACK.Information[0] = '\0';
    assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    p[0] = CRSF_CMDID_FC;
    p[1] = CRSF_CMD_FC_FORCE_DISARM;
    p[2] = 1;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_COMMAND_ACK, p, 3, g, &gl);
    assert_int_equal(bl, gl);
    assert_memory_equal(b, g, bl);
}

static void test_build_ack_with_info(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t built[64], bl = 0;
    uint8_t gold[64], gl = 0;

    const char* info = "All good"; /* includes terminating NULL in golden */
    uint8_t p[3 + 8];              /* 3 fixed + "All good" + '\0' = 3 + 8 */

    CRSF_init(&s);
    s.Command.dest_address = 0xE1;
    s.Command.origin_address = 0xE2;
    s.Command.Command_ID = CRSF_CMDID_COMMAND_ACK;
    s.Command.payload.ACK.Command_ID = CRSF_CMDID_FC;
    s.Command.payload.ACK.SubCommand_ID = CRSF_CMD_FC_FORCE_DISARM;
    s.Command.payload.ACK.Action = 1;
    strcpy(s.Command.payload.ACK.Information, info);

    assert_int_equal(CRSF_buildFrame(&s, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 0, built, &bl), CRSF_OK);

    /* golden */
    p[0] = CRSF_CMDID_FC;
    p[1] = CRSF_CMD_FC_FORCE_DISARM;
    p[2] = 1;
    memcpy(&p[3], info, 9); /* "All good" (8) + '\0' */
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_COMMAND_ACK, p, 3 + 9, gold, &gl);

    assert_int_equal(bl, gl);
    assert_memory_equal(built, gold, bl);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_ack_min(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[64], gl = 0, p[3];
    CRSF_FrameType_t t = 0;
    p[0] = CRSF_CMDID_LED;
    p[1] = CRSF_CMD_LED_SET_TO_DEFAULT;
    p[2] = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_COMMAND_ACK, p, 3, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal(s.Command.payload.ACK.Command_ID, CRSF_CMDID_LED);
    assert_int_equal(s.Command.payload.ACK.SubCommand_ID, CRSF_CMD_LED_SET_TO_DEFAULT);
    assert_int_equal(s.Command.payload.ACK.Action, 0);
}

static void test_parse_ack_with_info(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t frame[64], fl = 0;
    CRSF_FrameType_t t = 0;
    const char* info = "All good";

    uint8_t p[3 + 8];
    p[0] = CRSF_CMDID_LED;
    p[1] = CRSF_CMD_LED_SET_TO_DEFAULT;
    p[2] = 0;
    memcpy(&p[3], info, 9); /* includes NULL */

    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_COMMAND_ACK, p, 3 + 9, frame, &fl);

    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_OK);
    assert_int_equal(s.Command.payload.ACK.Command_ID, CRSF_CMDID_LED);
    assert_int_equal(s.Command.payload.ACK.SubCommand_ID, CRSF_CMD_LED_SET_TO_DEFAULT);
    assert_int_equal(s.Command.payload.ACK.Action, 0);
    assert_string_equal(s.Command.payload.ACK.Information, info);
}

static void test_parse_ack_invalid_len(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[64], gl = 0, p[2];
    CRSF_FrameType_t t = 0;
    p[0] = CRSF_CMDID_LED;
    p[1] = CRSF_CMD_LED_OVERRIDE_COLOR; /* missing Action */
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_COMMAND_ACK, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_ack(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[64], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_COMMAND_ACK;
    tx.Command.payload.ACK.Command_ID = CRSF_CMDID_BLUETOOTH;
    tx.Command.payload.ACK.SubCommand_ID = CRSF_CMD_BT_ENABLE;
    tx.Command.payload.ACK.Action = 1;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.ACK.Command_ID, CRSF_CMDID_BLUETOOTH);
    assert_int_equal(rx.Command.payload.ACK.SubCommand_ID, CRSF_CMD_BT_ENABLE);
    assert_int_equal(rx.Command.payload.ACK.Action, 1);
}

static void test_roundtrip_ack_with_info(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t buf[64], bl = 0;
    CRSF_FrameType_t t = 0;

    CRSF_init(&tx);
    tx.Command.dest_address = 0xE1;
    tx.Command.origin_address = 0xE2;
    tx.Command.Command_ID = CRSF_CMDID_COMMAND_ACK;
    tx.Command.payload.ACK.Command_ID = CRSF_CMDID_BLUETOOTH;
    tx.Command.payload.ACK.SubCommand_ID = CRSF_CMD_BT_ENABLE;
    tx.Command.payload.ACK.Action = 1;
    strcpy(tx.Command.payload.ACK.Information, "OK");

    assert_int_equal(CRSF_buildFrame(&tx, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 0, buf, &bl), CRSF_OK);

    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, buf, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.ACK.Command_ID, CRSF_CMDID_BLUETOOTH);
    assert_int_equal(rx.Command.payload.ACK.SubCommand_ID, CRSF_CMD_BT_ENABLE);
    assert_int_equal(rx.Command.payload.ACK.Action, 1);
    assert_string_equal(rx.Command.payload.ACK.Information, "OK");
}
#endif

/* ------------------------------- 0x01 FC -------------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_fc_force_disarm(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {CRSF_CMD_FC_FORCE_DISARM};
    CRSF_init(&s);
    s.Command.dest_address = DEST;
    s.Command.origin_address = ORIG;
    s.Command.Command_ID = CRSF_CMDID_FC;
    s.Command.payload.FC.subCommand = CRSF_CMD_FC_FORCE_DISARM;
    assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FC, p, 1, g, &gl);
    assert_int_equal(bl, gl);
    assert_memory_equal(b, g, bl);
}

/* default subcommand: unknown value (encode adds only subcmd) */
static void test_build_fc_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
    CRSF_init(&s);
    s.Command.dest_address = DEST;
    s.Command.origin_address = ORIG;
    s.Command.Command_ID = CRSF_CMDID_FC;
    s.Command.payload.FC.subCommand = (CRSF_CommandFC_subCMD_t)0x7E;
    assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FC, p, 1, g, &gl);
    assert_int_equal(bl, gl);
    assert_memory_equal(b, g, bl);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_fc_scale_channel(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_FC_SCALE_CHANNEL};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FC, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal(s.Command.payload.FC.subCommand, CRSF_CMD_FC_SCALE_CHANNEL);
}

static void test_parse_fc_invalid_len_lt1(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0;
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FC, NULL, 0, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

/* default: unknown subcmd accepted by decoder (no extra bytes) */
static void test_parse_fc_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FC, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.FC.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_fc_scale_channel(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_FC;
    tx.Command.payload.FC.subCommand = CRSF_CMD_FC_SCALE_CHANNEL;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.FC.subCommand, CRSF_CMD_FC_SCALE_CHANNEL);
}
#endif

/* ---------------------------- 0x03 BLUETOOTH ---------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_bt_all(void** state) {
    (void)state;
    /* RESET */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {CRSF_CMD_BT_RESET};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_BLUETOOTH;
        s.Command.payload.Bluetooth.subCommand = CRSF_CMD_BT_RESET;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* ENABLE=1 */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_BT_ENABLE, 1};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_BLUETOOTH;
        s.Command.payload.Bluetooth.subCommand = CRSF_CMD_BT_ENABLE;
        s.Command.payload.Bluetooth.Enable = 1;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* ECHO */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {CRSF_CMD_BT_ECHO};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_BLUETOOTH;
        s.Command.payload.Bluetooth.subCommand = CRSF_CMD_BT_ECHO;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_BLUETOOTH;
        s.Command.payload.Bluetooth.subCommand = (CRSF_CommandBT_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_bt_enable_and_reset(void** state) {
    (void)state;
    /* ENABLE=0 */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_BT_ENABLE, 0};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 2, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.Bluetooth.subCommand, CRSF_CMD_BT_ENABLE);
        assert_int_equal(s.Command.payload.Bluetooth.Enable, 0);
    }
    /* RESET */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_BT_RESET};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, p, 1, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.Bluetooth.subCommand, CRSF_CMD_BT_RESET);
    }
}

static void test_parse_bt_invalid_len_missing_subcmd(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0;
    CRSF_FrameType_t t = 0;
    /* decoder requires length>=1 (subcmd) */
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_BLUETOOTH, NULL, 0, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_bt_enable(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_BLUETOOTH;
    tx.Command.payload.Bluetooth.subCommand = CRSF_CMD_BT_ENABLE;
    tx.Command.payload.Bluetooth.Enable = 1;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.Bluetooth.subCommand, CRSF_CMD_BT_ENABLE);
    assert_int_equal(rx.Command.payload.Bluetooth.Enable, 1);
}
#endif

/* -------------------------------- 0x05 OSD ------------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_osd_buttons_and_default(void** state) {
    (void)state;
    /* buttons */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_OSD_SEND_BUTTONS, (uint8_t)((1U << 0) | (1U << 4))};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_OSD;
        s.Command.payload.OSD.subCommand = CRSF_CMD_OSD_SEND_BUTTONS;
        s.Command.payload.OSD.buttons = p[1];
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_OSD, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_OSD;
        s.Command.payload.OSD.subCommand = (CRSF_CommandOSD_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_OSD, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_osd_buttons(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_OSD_SEND_BUTTONS, (uint8_t)((1U << 1) | (1U << 3))};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_OSD, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal(s.Command.payload.OSD.buttons, p[1]);
}

static void test_parse_osd_invalid_len_lt2(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_OSD_SEND_BUTTONS};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_OSD, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_osd_buttons(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_OSD;
    tx.Command.payload.OSD.subCommand = CRSF_CMD_OSD_SEND_BUTTONS;
    tx.Command.payload.OSD.enter = 1;
    tx.Command.payload.OSD.down = 1;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.OSD.enter, 1);
    assert_int_equal(rx.Command.payload.OSD.down, 1);
}
#endif

/* -------------------------------- 0x08 VTX ------------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_vtx_variants_and_default(void** state) {
    (void)state;
    /* SET_FREQUENCY 5800 */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[3] = {CRSF_CMD_VTX_SET_FREQUENCY, (uint8_t)(5800 >> 8), (uint8_t)5800};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_VTX;
        s.Command.payload.VTX.subCommand = CRSF_CMD_VTX_SET_FREQUENCY;
        s.Command.payload.VTX.FrequencyMHz = 5800;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 3, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* SET_POWER 14 */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_VTX_SET_POWER, 14};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_VTX;
        s.Command.payload.VTX.subCommand = CRSF_CMD_VTX_SET_POWER;
        s.Command.payload.VTX.Power_dBm = 14;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* ENABLE_PITMODE_ON_PUP (packed) */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP, (uint8_t)((1U) | (2U << 1) | (7U << 3))};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_VTX;
        s.Command.payload.VTX.subCommand = CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP;
        s.Command.payload.VTX.PitMode = 1;
        s.Command.payload.VTX.pitmode_control = 2;
        s.Command.payload.VTX.pitmode_switch = 7;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_VTX;
        s.Command.payload.VTX.subCommand = (CRSF_CommandVTX_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_vtx_freq_and_power(void** state) {
    (void)state;
    /* freq */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[3] = {CRSF_CMD_VTX_SET_FREQUENCY, (uint8_t)(5325 >> 8), (uint8_t)5325};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 3, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.VTX.FrequencyMHz, 5325);
    }
    /* power */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_VTX_SET_POWER, 10};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 2, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.VTX.Power_dBm, 10);
    }
}

/* invalids for all guarded checks */
/* VTX: CRSF.c:1203 (length < 1) */
static void test_parse_vtx_invalid_len_len0(void** state) {
    (void)state;
    uint8_t frame[32], fl = 0;
    /* Build raw frame with Command ID = VTX, but zero bytes of command payload */
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_VTX, NULL, 0, frame, &fl);
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_vtx_invalid_len_set_freq_lt2(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_VTX_SET_FREQUENCY};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_vtx_invalid_len_pitmode_lt1(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 1, g, &gl); /* missing config byte */
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_vtx_invalid_len_power_lt1(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_VTX_SET_POWER};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 1, g, &gl); /* missing power byte */
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

/* default unknown subcmd parses OK */
static void test_parse_vtx_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_VTX, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.VTX.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_vtx_pitmode(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_VTX;
    tx.Command.payload.VTX.subCommand = CRSF_CMD_VTX_ENABLE_PITMODE_ON_PUP;
    tx.Command.payload.VTX.PitMode = 1;
    tx.Command.payload.VTX.pitmode_control = 3;
    tx.Command.payload.VTX.pitmode_switch = 15;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.VTX.PitMode, 1);
    assert_int_equal(rx.Command.payload.VTX.pitmode_control, 3);
    assert_int_equal(rx.Command.payload.VTX.pitmode_switch, 15);
}
#endif

/* -------------------------------- 0x09 LED ------------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_led_color_shift_pulse_blink_and_default(void** state) {
    (void)state;
    /* SET_TO_DEFAULT */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[1] = {CRSF_CMD_LED_SET_TO_DEFAULT};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = CRSF_CMD_LED_SET_TO_DEFAULT;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* OVERRIDE_COLOR H=300,S=100,V=200 */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[4];
        uint32_t v = ((uint32_t)(300 & 0x1FF) << 15) | ((uint32_t)(100 & 0x7F) << 8) | 200;
        p[0] = CRSF_CMD_LED_OVERRIDE_COLOR;
        p[1] = (uint8_t)(v >> 16);
        p[2] = (uint8_t)(v >> 8);
        p[3] = (uint8_t)v;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = CRSF_CMD_LED_OVERRIDE_COLOR;
        s.Command.payload.LED.overrideColor.H = 300;
        s.Command.payload.LED.overrideColor.S = 100;
        s.Command.payload.LED.overrideColor.V = 200;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 4, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* OVERRIDE_SHIFT 333ms, H=511,S=127,V=255 */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[6];
        uint32_t v = ((uint32_t)511 << 15) | ((uint32_t)127 << 8) | 255;
        p[0] = CRSF_CMD_LED_OVERRIDE_SHIFT;
        p[1] = (uint8_t)(333 >> 8);
        p[2] = (uint8_t)333;
        p[3] = (uint8_t)(v >> 16);
        p[4] = (uint8_t)(v >> 8);
        p[5] = (uint8_t)v;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = CRSF_CMD_LED_OVERRIDE_SHIFT;
        s.Command.payload.LED.overrideShift.interval_ms = 333;
        s.Command.payload.LED.overrideShift.H = 511;
        s.Command.payload.LED.overrideShift.S = 127;
        s.Command.payload.LED.overrideShift.V = 255;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 6, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* OVERRIDE_PULSE 250ms + HSV pairs */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[9];
        uint32_t a = ((uint32_t)1 << 15) | ((uint32_t)2 << 8) | 3, z = ((uint32_t)4 << 15) | ((uint32_t)5 << 8) | 6;
        p[0] = CRSF_CMD_LED_OVERRIDE_PULSE;
        p[1] = (uint8_t)(250 >> 8);
        p[2] = (uint8_t)250;
        p[3] = (uint8_t)(a >> 16);
        p[4] = (uint8_t)(a >> 8);
        p[5] = (uint8_t)a;
        p[6] = (uint8_t)(z >> 16);
        p[7] = (uint8_t)(z >> 8);
        p[8] = (uint8_t)z;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = CRSF_CMD_LED_OVERRIDE_PULSE;
        s.Command.payload.LED.overridePulse.duration_ms = 250;
        s.Command.payload.LED.overridePulse.H_start = 1;
        s.Command.payload.LED.overridePulse.S_start = 2;
        s.Command.payload.LED.overridePulse.V_start = 3;
        s.Command.payload.LED.overridePulse.H_stop = 4;
        s.Command.payload.LED.overridePulse.S_stop = 5;
        s.Command.payload.LED.overridePulse.V_stop = 6;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 9, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* OVERRIDE_BLINK 1200ms + HSV pairs */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[9];
        uint32_t a = ((uint32_t)100 << 15) | ((uint32_t)40 << 8) | 10, z = ((uint32_t)255 << 15) | ((uint32_t)100 << 8) | 200;
        p[0] = CRSF_CMD_LED_OVERRIDE_BLINK;
        p[1] = (uint8_t)(1200 >> 8);
        p[2] = (uint8_t)1200;
        p[3] = (uint8_t)(a >> 16);
        p[4] = (uint8_t)(a >> 8);
        p[5] = (uint8_t)a;
        p[6] = (uint8_t)(z >> 16);
        p[7] = (uint8_t)(z >> 8);
        p[8] = (uint8_t)z;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = CRSF_CMD_LED_OVERRIDE_BLINK;
        s.Command.payload.LED.overrideBlink.interval_ms = 1200;
        s.Command.payload.LED.overrideBlink.H_start = 100;
        s.Command.payload.LED.overrideBlink.S_start = 40;
        s.Command.payload.LED.overrideBlink.V_start = 10;
        s.Command.payload.LED.overrideBlink.H_stop = 255;
        s.Command.payload.LED.overrideBlink.S_stop = 100;
        s.Command.payload.LED.overrideBlink.V_stop = 200;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 9, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_LED;
        s.Command.payload.LED.subCommand = (CRSF_CommandLED_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_led_color_shift_blink(void** state) {
    (void)state;
    /* COLOR */
    {
        CRSF_t s;
        uint8_t g[64], gl = 0, p[4];
        uint32_t v = ((uint32_t)321 << 15) | ((uint32_t)55 << 8) | 123;
        CRSF_FrameType_t t = 0;
        p[0] = CRSF_CMD_LED_OVERRIDE_COLOR;
        p[1] = (uint8_t)(v >> 16);
        p[2] = (uint8_t)(v >> 8);
        p[3] = (uint8_t)v;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 4, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.LED.overrideColor.H, 321);
        assert_int_equal(s.Command.payload.LED.overrideColor.S, 55);
        assert_int_equal(s.Command.payload.LED.overrideColor.V, 123);
    }
    /* SHIFT */
    {
        CRSF_t s;
        uint8_t g[64], gl = 0, p[6];
        uint32_t v = ((uint32_t)100 << 15) | ((uint32_t)20 << 8) | 5;
        CRSF_FrameType_t t = 0;
        p[0] = CRSF_CMD_LED_OVERRIDE_SHIFT;
        p[1] = (uint8_t)(200 >> 8);
        p[2] = (uint8_t)200;
        p[3] = (uint8_t)(v >> 16);
        p[4] = (uint8_t)(v >> 8);
        p[5] = (uint8_t)v;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 6, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.LED.overrideShift.interval_ms, 200);
        assert_int_equal(s.Command.payload.LED.overrideShift.H, 100);
        assert_int_equal(s.Command.payload.LED.overrideShift.S, 20);
        assert_int_equal(s.Command.payload.LED.overrideShift.V, 5);
    }
    /* BLINK */
    {
        CRSF_t s;
        uint8_t g[64], gl = 0, p[9];
        uint32_t a = ((uint32_t)100 << 15) | ((uint32_t)40 << 8) | 10, z = ((uint32_t)255 << 15) | ((uint32_t)100 << 8) | 200;
        CRSF_FrameType_t t = 0;
        p[0] = CRSF_CMD_LED_OVERRIDE_BLINK;
        p[1] = (uint8_t)(1200 >> 8);
        p[2] = (uint8_t)1200;
        p[3] = (uint8_t)(a >> 16);
        p[4] = (uint8_t)(a >> 8);
        p[5] = (uint8_t)a;
        p[6] = (uint8_t)(z >> 16);
        p[7] = (uint8_t)(z >> 8);
        p[8] = (uint8_t)z;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 9, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.LED.overrideBlink.interval_ms, 1200);
    }
}

/* invalids (hit all length checks) */
static void test_parse_led_invalid_len_color_lt3(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_LED_OVERRIDE_COLOR, 0xAA};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_led_invalid_len_shift_lt5(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[3] = {CRSF_CMD_LED_OVERRIDE_SHIFT, 0x01, 0x02};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 3, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_led_invalid_len_pulse_lt8(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[64], gl = 0, p[4] = {CRSF_CMD_LED_OVERRIDE_PULSE, 0x00, 0xF0, 0xAA};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 4, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_led_invalid_len_blink_lt8(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[64], gl = 0, p[4] = {CRSF_CMD_LED_OVERRIDE_BLINK, 0x00, 0x10, 0xAA};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 4, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_led_invalid_len_len0(void** state) {
    (void)state;
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    uint8_t frame[48], fl = 0;

    /* Command ID = LED, but zero-byte LED payload */
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_LED, NULL, 0, frame, &fl);

    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);
}
#endif /* RX */

/* default unknown subcmd parses OK */
static void test_parse_led_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_LED, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.LED.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_led_pulse(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[64], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_LED;
    tx.Command.payload.LED.subCommand = CRSF_CMD_LED_OVERRIDE_PULSE;
    tx.Command.payload.LED.overridePulse.duration_ms = 250;
    tx.Command.payload.LED.overridePulse.H_start = 1;
    tx.Command.payload.LED.overridePulse.S_start = 2;
    tx.Command.payload.LED.overridePulse.V_start = 3;
    tx.Command.payload.LED.overridePulse.H_stop = 4;
    tx.Command.payload.LED.overridePulse.S_stop = 5;
    tx.Command.payload.LED.overridePulse.V_stop = 6;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.LED.overridePulse.duration_ms, 250);
}
#endif

/* ------------------------------ 0x0A GENERAL ----------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_general_proposal_resp_default(void** state) {
    (void)state;
    /* proposal port=2, 921600 */
    {
        CRSF_t s;
        uint8_t b[64],
            bl = 0, g[64], gl = 0,
            p[6] = {CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL, 2, (uint8_t)(921600 >> 24), (uint8_t)(921600 >> 16), (uint8_t)(921600 >> 8), (uint8_t)921600};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_GENERAL;
        s.Command.payload.general.subCommand = CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL;
        s.Command.payload.general.protocolSpeedProposal.port_id = 2;
        s.Command.payload.general.protocolSpeedProposal.proposed_baudrate = 921600U;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 6, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* response port=1 accept=1 */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[3] = {CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE, 1, 1};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_GENERAL;
        s.Command.payload.general.subCommand = CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE;
        s.Command.payload.general.protocolSpeedResponse.port_id = 1;
        s.Command.payload.general.protocolSpeedResponse.response = 1;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 3, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_GENERAL;
        s.Command.payload.general.subCommand = (CRSF_CommandGen_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_general_prop_resp(void** state) {
    (void)state;
    /* proposal */
    {
        CRSF_t s;
        uint8_t g[64], gl = 0, p[6];
        CRSF_FrameType_t t = 0;
        uint32_t baud = 115200;
        p[0] = CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL;
        p[1] = 9;
        p[2] = (uint8_t)(baud >> 24);
        p[3] = (uint8_t)(baud >> 16);
        p[4] = (uint8_t)(baud >> 8);
        p[5] = (uint8_t)baud;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 6, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.general.protocolSpeedProposal.port_id, 9);
        assert_int_equal(s.Command.payload.general.protocolSpeedProposal.proposed_baudrate, baud);
    }
    /* response */
    {
        CRSF_t s;
        uint8_t g[64], gl = 0, p[3] = {CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE, 3, 0};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 3, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.general.protocolSpeedResponse.port_id, 3);
        assert_int_equal(s.Command.payload.general.protocolSpeedResponse.response, 0);
    }
}

/* invalids for both checks */
static void test_parse_general_invalid_len_len0(void** state) {
    (void)state;
    uint8_t frame[32], fl = 0;
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_GENERAL, NULL, 0, frame, &fl);
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_general_invalid_len_proposal_lt5(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL, 2};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_general_invalid_len_response_lt2(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

/* default unknown subcmd parses OK */
static void test_parse_general_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_GENERAL, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.general.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_general_prop_and_resp(void** state) {
    (void)state;
    /* proposal */
    {
        CRSF_t tx, rx;
        uint8_t b[64], bl = 0;
        CRSF_FrameType_t t = 0;
        CRSF_init(&tx);
        tx.Command.dest_address = DEST;
        tx.Command.origin_address = ORIG;
        tx.Command.Command_ID = CRSF_CMDID_GENERAL;
        tx.Command.payload.general.subCommand = CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL;
        tx.Command.payload.general.protocolSpeedProposal.port_id = 4;
        tx.Command.payload.general.protocolSpeedProposal.proposed_baudrate = 230400U;
        assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        CRSF_init(&rx);
        assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
        assert_int_equal(rx.Command.payload.general.protocolSpeedProposal.port_id, 4);
        assert_int_equal(rx.Command.payload.general.protocolSpeedProposal.proposed_baudrate, 230400U);
    }
    /* response */
    {
        CRSF_t tx, rx;
        uint8_t b[64], bl = 0;
        CRSF_FrameType_t t = 0;
        CRSF_init(&tx);
        tx.Command.dest_address = DEST;
        tx.Command.origin_address = ORIG;
        tx.Command.Command_ID = CRSF_CMDID_GENERAL;
        tx.Command.payload.general.subCommand = CRSF_CMD_GEN_CRSF_PROTOCOL_SPEED_PROPOSAL_RESPONSE;
        tx.Command.payload.general.protocolSpeedResponse.port_id = 5;
        tx.Command.payload.general.protocolSpeedResponse.response = 1;
        assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        CRSF_init(&rx);
        assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
        assert_int_equal(rx.Command.payload.general.protocolSpeedResponse.port_id, 5);
        assert_int_equal(rx.Command.payload.general.protocolSpeedResponse.response, 1);
    }
}
#endif

/* ---------------------------- 0x10 CROSSFIRE ----------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_cf_bind_model_and_default(void** state) {
    (void)state;
    /* SET_BIND_ID (6b) */
    {
        CRSF_t s;
        uint8_t b[64], bl = 0, g[64], gl = 0, p[7];
        uint8_t id[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};
        p[0] = CRSF_CMD_CF_SET_BIND_ID;
        memcpy(&p[1], id, 6);
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_CROSSFIRE;
        s.Command.payload.crossfire.subCommand = CRSF_CMD_CF_SET_BIND_ID;
        s.Command.payload.crossfire.setBindId.len = 6;
        memcpy(s.Command.payload.crossfire.setBindId.bytes, id, 6);
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 7, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* MODEL_SELECTION */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_CF_MODEL_SELECTION, 3};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_CROSSFIRE;
        s.Command.payload.crossfire.subCommand = CRSF_CMD_CF_MODEL_SELECTION;
        s.Command.payload.crossfire.Model_Number = 3;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_CROSSFIRE;
        s.Command.payload.crossfire.subCommand = (CRSF_CommandCF_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_cf_bind_and_model(void** state) {
    (void)state;
    /* bind len=4 */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[5] = {CRSF_CMD_CF_SET_BIND_ID, 1, 2, 3, 4};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 5, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.crossfire.setBindId.len, 4);
        assert_memory_equal(s.Command.payload.crossfire.setBindId.bytes, p + 1, 4);
    }
    /* model selection */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_CF_MODEL_SELECTION, 9};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 2, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.crossfire.Model_Number, 9);
    }
}

/* invalid: model selection missing model byte */
static void test_parse_cf_invalid_len_len0(void** state) {
    (void)state;
    uint8_t frame[32], fl = 0;
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_CROSSFIRE, NULL, 0, frame, &fl);
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_cf_invalid_len_model_lt1(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_CF_MODEL_SELECTION};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

/* default unknown subcmd parses OK */
static void test_parse_cf_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_CROSSFIRE, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.crossfire.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_cf_current_model_reply(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_CROSSFIRE;
    tx.Command.payload.crossfire.subCommand = CRSF_CMD_CF_CURRENT_MODEL_REPLY;
    tx.Command.payload.crossfire.Model_Number = 11;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.crossfire.Model_Number, 11);
}
#endif

/* --------------------------- 0x20 FLOW CONTROL --------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_flow_sub_unsub_default(void** state) {
    (void)state;
    /* subscribe GPS 250ms */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[4] = {CRSF_CMD_FLOW_SUBSCRIBE, CRSF_FRAMETYPE_GPS, 0x00, 0xFA};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_FLOW_CTRL;
        s.Command.payload.flow.subCommand = CRSF_CMD_FLOW_SUBSCRIBE;
        s.Command.payload.flow.Frame_type = CRSF_FRAMETYPE_GPS;
        s.Command.payload.flow.Max_interval_time_ms = 250;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 4, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* unsubscribe VOLTAGES */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[2] = {CRSF_CMD_FLOW_UNSUBSCRIBE, CRSF_FRAMETYPE_VOLTAGES};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_FLOW_CTRL;
        s.Command.payload.flow.subCommand = CRSF_CMD_FLOW_UNSUBSCRIBE;
        s.Command.payload.flow.Frame_type = CRSF_FRAMETYPE_VOLTAGES;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 2, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_FLOW_CTRL;
        s.Command.payload.flow.subCommand = (CRSF_CommandFlow_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_flow_sub_unsub(void** state) {
    (void)state;
    /* SUBSCRIBE RPM 1024ms */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[4] = {CRSF_CMD_FLOW_SUBSCRIBE, CRSF_FRAMETYPE_RPM, 0x04, 0x00};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 4, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.flow.Frame_type, CRSF_FRAMETYPE_RPM);
        assert_int_equal(s.Command.payload.flow.Max_interval_time_ms, 1024);
    }
    /* UNSUBSCRIBE HEARTBEAT */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_FLOW_UNSUBSCRIBE, CRSF_FRAMETYPE_HEARTBEAT};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 2, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.flow.Frame_type, CRSF_FRAMETYPE_HEARTBEAT);
    }
}

/* invalids */
static void test_parse_flow_invalid_len_len0(void** state) {
    (void)state;
    uint8_t frame[32], fl = 0;
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_FLOW_CTRL, NULL, 0, frame, &fl);
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_flow_invalid_len_sub_lt3(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_FLOW_SUBSCRIBE, CRSF_FRAMETYPE_GPS};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_flow_invalid_len_unsub_lt1(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {CRSF_CMD_FLOW_UNSUBSCRIBE};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

/* default unknown subcmd parses OK */
static void test_parse_flow_default_unknown(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[1] = {0x7E};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_FLOW_CTRL, p, 1, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
    assert_int_equal((uint8_t)s.Command.payload.flow.subCommand, 0x7E);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_flow_subscribe(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_FLOW_CTRL;
    tx.Command.payload.flow.subCommand = CRSF_CMD_FLOW_SUBSCRIBE;
    tx.Command.payload.flow.Frame_type = CRSF_FRAMETYPE_ATTITUDE;
    tx.Command.payload.flow.Max_interval_time_ms = 500;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.flow.Frame_type, CRSF_FRAMETYPE_ATTITUDE);
    assert_int_equal(rx.Command.payload.flow.Max_interval_time_ms, 500);
}
#endif

/* ------------------------------- 0x22 SCREEN ----------------------------- */

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_screen_popup_variants_selection_return_default(void** state) {
    (void)state;
    /* 1) minimal popup (no optionals) */
    {
        CRSF_t s;
        uint8_t b[160], bl = 0, g[160], gl = 0, tmp[64], *p = tmp;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        strcpy(s.Command.payload.screen.popupMessageStart.Header, "H");
        strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "I");
        s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 7;
        s.Command.payload.screen.popupMessageStart.Close_button_option = 1;
        s.Command.payload.screen.popupMessageStart.add_data.present = 0;
        s.Command.payload.screen.popupMessageStart.has_possible_values = 0;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        *p++ = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        pack_cstr(&p, "H");
        pack_cstr(&p, "I");
        *p++ = 7;
        *p++ = 1;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, tmp, (uint8_t)(p - tmp), g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* 2) minimal + add_data only */
    {
        CRSF_t s;
        uint8_t b[160], bl = 0, g[160], gl = 0, tmp[64], *p = tmp;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        strcpy(s.Command.payload.screen.popupMessageStart.Header, "T");
        strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "I");
        s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 10;
        s.Command.payload.screen.popupMessageStart.Close_button_option = 0;
        s.Command.payload.screen.popupMessageStart.add_data.present = 1;
        strcpy(s.Command.payload.screen.popupMessageStart.add_data.selectionText, "S");
        s.Command.payload.screen.popupMessageStart.add_data.value = 3;
        s.Command.payload.screen.popupMessageStart.add_data.minValue = 1;
        s.Command.payload.screen.popupMessageStart.add_data.maxValue = 9;
        s.Command.payload.screen.popupMessageStart.add_data.defaultValue = 5;
        strcpy(s.Command.payload.screen.popupMessageStart.add_data.unit, "u");
        s.Command.payload.screen.popupMessageStart.has_possible_values = 0;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        *p++ = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        pack_cstr(&p, "T");
        pack_cstr(&p, "I");
        *p++ = 10;
        *p++ = 0;
        pack_cstr(&p, "S");
        *p++ = 3;
        *p++ = 1;
        *p++ = 9;
        *p++ = 5;
        pack_cstr(&p, "u");
        assert_true((p - tmp) <= 32);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, tmp, (uint8_t)(p - tmp), g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* 3) possible_values only */
    {
        CRSF_t s;
        uint8_t b[160], bl = 0, g[160], gl = 0, tmp[64], *p = tmp;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        strcpy(s.Command.payload.screen.popupMessageStart.Header, "H2");
        strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "I2");
        s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 2;
        s.Command.payload.screen.popupMessageStart.Close_button_option = 1;
        s.Command.payload.screen.popupMessageStart.add_data.present = 0;
        s.Command.payload.screen.popupMessageStart.has_possible_values = 1;
        strcpy(s.Command.payload.screen.popupMessageStart.possible_values, "A;B");
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        *p++ = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        pack_cstr(&p, "H2");
        pack_cstr(&p, "I2");
        *p++ = 2;
        *p++ = 1;
        pack_cstr(&p, "A;B");
        assert_true((p - tmp) <= 32);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, tmp, (uint8_t)(p - tmp), g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* 4) maximal (both optionals) */
    {
        CRSF_t s;
        uint8_t b[160], bl = 0, g[160], gl = 0, tmp[64], *p = tmp;
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        strcpy(s.Command.payload.screen.popupMessageStart.Header, "H");
        strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "I");
        s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 60;
        s.Command.payload.screen.popupMessageStart.Close_button_option = 1;
        s.Command.payload.screen.popupMessageStart.add_data.present = 1;
        strcpy(s.Command.payload.screen.popupMessageStart.add_data.selectionText, "S");
        s.Command.payload.screen.popupMessageStart.add_data.value = 5;
        s.Command.payload.screen.popupMessageStart.add_data.minValue = 0;
        s.Command.payload.screen.popupMessageStart.add_data.maxValue = 9;
        s.Command.payload.screen.popupMessageStart.add_data.defaultValue = 2;
        strcpy(s.Command.payload.screen.popupMessageStart.add_data.unit, "u");
        s.Command.payload.screen.popupMessageStart.has_possible_values = 1;
        strcpy(s.Command.payload.screen.popupMessageStart.possible_values, "A;B");
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        *p++ = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        pack_cstr(&p, "H");
        pack_cstr(&p, "I");
        *p++ = 60;
        *p++ = 1;
        pack_cstr(&p, "S");
        *p++ = 5;
        *p++ = 0;
        *p++ = 9;
        *p++ = 2;
        pack_cstr(&p, "u");
        pack_cstr(&p, "A;B");
        assert_true((p - tmp) <= 32);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, tmp, (uint8_t)(p - tmp), g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* 5) SELECTION_RETURN (build) — missing earlier */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[3] = {CRSF_CMD_SCREEN_SELECTION_RETURN, 7, 1};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_SELECTION_RETURN;
        s.Command.payload.screen.selectionReturn.value = 7;
        s.Command.payload.screen.selectionReturn.response = 1;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, p, 3, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
    /* 6) default subcmd */
    {
        CRSF_t s;
        uint8_t b[48], bl = 0, g[48], gl = 0, p[1] = {0x7E};
        CRSF_init(&s);
        s.Command.dest_address = DEST;
        s.Command.origin_address = ORIG;
        s.Command.Command_ID = CRSF_CMDID_SCREEN;
        s.Command.payload.screen.subCommand = (CRSF_CommandScreen_subCMD_t)0x7E;
        assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, p, 1, g, &gl);
        assert_int_equal(bl, gl);
        assert_memory_equal(b, g, bl);
    }
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_screen_popup_and_selection_return_and_defaults(void** state) {
    (void)state;
    /* minimal popup */
    {
        CRSF_t s;
        uint8_t g[160], gl = 0, tmp[64], *p = tmp;
        CRSF_FrameType_t t = 0;
        *p++ = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
        pack_cstr(&p, "H");
        pack_cstr(&p, "I");
        *p++ = 7;
        *p++ = 1;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, tmp, (uint8_t)(p - tmp), g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_string_equal(s.Command.payload.screen.popupMessageStart.Header, "H");
        assert_string_equal(s.Command.payload.screen.popupMessageStart.Info_message, "I");
        assert_int_equal(s.Command.payload.screen.popupMessageStart.Max_timeout_interval, 7);
        assert_int_equal(s.Command.payload.screen.popupMessageStart.Close_button_option, 1);
    }
    /* selection return */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p2[3] = {CRSF_CMD_SCREEN_SELECTION_RETURN, 7, 1};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, p2, 3, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal(s.Command.payload.screen.selectionReturn.value, 7);
        assert_int_equal(s.Command.payload.screen.selectionReturn.response, 1);
    }
    /* default unknown subcmd parses OK */
    {
        CRSF_t s;
        uint8_t g[48], gl = 0, p3[1] = {0x7E};
        CRSF_FrameType_t t = 0;
        make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, p3, 1, g, &gl);
        CRSF_init(&s);
        assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_OK);
        assert_int_equal((uint8_t)s.Command.payload.screen.subCommand, 0x7E);
    }
}

/* invalid length cases */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)

static void test_build_screen_overflow_info_exact(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t built[64];
    uint8_t bl = 0;
    char longInfo[20];
    memset(longInfo, 'I', 19);
    longInfo[19] = '\0';

    CRSF_init(&s);
    s.Command.dest_address = 0xE1;
    s.Command.origin_address = 0xE2;
    s.Command.Command_ID = CRSF_CMDID_SCREEN;
    s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;
    strcpy(s.Command.payload.screen.popupMessageStart.Header, "HHHHHHHHHHH");
    strcpy(s.Command.payload.screen.popupMessageStart.Info_message, longInfo);
    s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 7;
    s.Command.payload.screen.popupMessageStart.Close_button_option = 1;
    s.Command.payload.screen.popupMessageStart.add_data.present = 0;
    s.Command.payload.screen.popupMessageStart.has_possible_values = 0;

    assert_int_equal(CRSF_buildFrame(&s, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 0, built, &bl), CRSF_ERROR_TYPE_LENGTH);
}

static void test_build_screen_overflow_add_data_exact(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t built[64];
    uint8_t bl = 0;

    char sel[20];
    memset(sel, 'S', 19);
    sel[20] = '\0'; /* A=20 */
    char unit[5 + 1];
    memset(unit, 'u', 4);
    unit[4] = '\0'; /* B=5 (20+5=25) */

    CRSF_init(&s);
    s.Command.dest_address = 0xE1;
    s.Command.origin_address = 0xE2;
    s.Command.Command_ID = CRSF_CMDID_SCREEN;
    s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;

    strcpy(s.Command.payload.screen.popupMessageStart.Header, "H");
    strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "I");
    s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 10;
    s.Command.payload.screen.popupMessageStart.Close_button_option = 0;

    s.Command.payload.screen.popupMessageStart.add_data.present = 1;
    strcpy(s.Command.payload.screen.popupMessageStart.add_data.selectionText, sel);
    s.Command.payload.screen.popupMessageStart.add_data.value = 3;
    s.Command.payload.screen.popupMessageStart.add_data.minValue = 1;
    s.Command.payload.screen.popupMessageStart.add_data.maxValue = 9;
    s.Command.payload.screen.popupMessageStart.add_data.defaultValue = 5;
    strcpy(s.Command.payload.screen.popupMessageStart.add_data.unit, unit);

    s.Command.payload.screen.popupMessageStart.has_possible_values = 0;

    assert_int_equal(CRSF_buildFrame(&s, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 0, built, &bl), CRSF_ERROR_TYPE_LENGTH);
}

static void test_build_screen_overflow_possible_values_exact(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t built[64];
    uint8_t bl = 0;

    char pv[20];
    memset(pv, 'A', 19);
    pv[19] = '\0';

    CRSF_init(&s);
    s.Command.dest_address = 0xE1;
    s.Command.origin_address = 0xE2;
    s.Command.Command_ID = CRSF_CMDID_SCREEN;
    s.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_POPUP_MESSAGE_START;

    strcpy(s.Command.payload.screen.popupMessageStart.Header, "HHHH");
    strcpy(s.Command.payload.screen.popupMessageStart.Info_message, "III");
    s.Command.payload.screen.popupMessageStart.Max_timeout_interval = 60;
    s.Command.payload.screen.popupMessageStart.Close_button_option = 1;

    s.Command.payload.screen.popupMessageStart.add_data.present = 0;
    s.Command.payload.screen.popupMessageStart.has_possible_values = 1;
    strcpy(s.Command.payload.screen.popupMessageStart.possible_values, pv);

    assert_int_equal(CRSF_buildFrame(&s, CRSF_ADDRESS_FLIGHT_CONTROLLER, CRSF_FRAMETYPE_COMMAND, 0, built, &bl), CRSF_ERROR_TYPE_LENGTH);
}
#endif /* TX */

static void test_parse_screen_invalid_len_missing_subcmd(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0;
    CRSF_FrameType_t t = 0;
    /* requires at least 1 byte subcmd */
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, NULL, 0, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}

static void test_parse_screen_invalid_len_selection_return_lt2(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t g[48], gl = 0, p[2] = {CRSF_CMD_SCREEN_SELECTION_RETURN, 9};
    CRSF_FrameType_t t = 0;
    make_cmd_frame(BUS, DEST, ORIG, CRSF_CMDID_SCREEN, p, 2, g, &gl);
    CRSF_init(&s);
    assert_int_equal(CRSF_processFrame(&s, g, &t), CRSF_ERROR_TYPE_LENGTH);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
static void test_roundtrip_screen_selection_return(void** state) {
    (void)state;
    CRSF_t tx, rx;
    uint8_t b[48], bl = 0;
    CRSF_FrameType_t t = 0;
    CRSF_init(&tx);
    tx.Command.dest_address = DEST;
    tx.Command.origin_address = ORIG;
    tx.Command.Command_ID = CRSF_CMDID_SCREEN;
    tx.Command.payload.screen.subCommand = CRSF_CMD_SCREEN_SELECTION_RETURN;
    tx.Command.payload.screen.selectionReturn.value = 5;
    tx.Command.payload.screen.selectionReturn.response = 1;
    assert_int_equal(CRSF_buildFrame(&tx, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_OK);
    CRSF_init(&rx);
    assert_int_equal(CRSF_processFrame(&rx, b, &t), CRSF_OK);
    assert_int_equal(rx.Command.payload.screen.selectionReturn.value, 5);
    assert_int_equal(rx.Command.payload.screen.selectionReturn.response, 1);
}
#endif

/* Attempt to encode with unsupported Command_ID -> expect CRSF_ERROR_INVALID_FRAME */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
static void test_build_top_level_default_invalid_cmd_id(void** state) {
    (void)state;
    CRSF_t s;
    uint8_t b[48], bl = 0;
    CRSF_init(&s);
    s.Command.dest_address = DEST;
    s.Command.origin_address = ORIG;
    s.Command.Command_ID = CRSF_CMDID_RESERVED_12; /* not implemented by encoder -> default case */
    /* Should fail while building payload; CRSF_buildFrame returns CRSF_ERROR_INVALID_FRAME */
    assert_int_equal(CRSF_buildFrame(&s, BUS, CRSF_FRAMETYPE_COMMAND, 0, b, &bl), CRSF_ERROR_INVALID_FRAME);
}
#endif

#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
static void test_parse_top_level_default_invalid_cmd_id(void** state) {
    (void)state;
    uint8_t frame[32], fl = 0;
    /* Use an unknown Command_ID (e.g., 0x7E) to drive the decoder’s default: path */
    make_cmd_frame(CRSF_ADDRESS_FLIGHT_CONTROLLER, 0xE1, 0xE2, CRSF_CMDID_RESERVED_12, NULL, 0, frame, &fl);
    CRSF_t s;
    CRSF_FrameType_t t = 0;
    CRSF_init(&s);
    /* Decoder default falls through and returns OK */
    assert_int_equal(CRSF_processFrame(&s, frame, &t), CRSF_OK);
    assert_int_equal(t, CRSF_FRAMETYPE_COMMAND);
}
#endif /* RX */

/* ============================================================================
 * MAIN TEST RUNNER
 * ============================================================================ */

int main(void) {
    const struct CMUnitTest tests[] = {
/* ------------------------- 0xFF COMMAND ACK ----------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_ack_min),
        cmocka_unit_test(test_build_ack_with_info),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_ack_min),
        cmocka_unit_test(test_parse_ack_with_info),
        cmocka_unit_test(test_parse_ack_invalid_len),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_ack),
        cmocka_unit_test(test_roundtrip_ack_with_info),
#endif
/* ------------------------------- 0x01 FC -------------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_fc_force_disarm),
        /* default subcommand: unknown value (encode adds only subcmd) */
        cmocka_unit_test(test_build_fc_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_fc_scale_channel),
        cmocka_unit_test(test_parse_fc_invalid_len_lt1),
        /* default: unknown subcmd accepted by decoder (no extra bytes) */
        cmocka_unit_test(test_parse_fc_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_fc_scale_channel),
#endif
/* ---------------------------- 0x03 BLUETOOTH ---------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_bt_all),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_bt_enable_and_reset),
        cmocka_unit_test(test_parse_bt_invalid_len_missing_subcmd),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_bt_enable),
#endif
/* -------------------------------- 0x05 OSD ------------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_osd_buttons_and_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_osd_buttons),
        cmocka_unit_test(test_parse_osd_invalid_len_lt2),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_osd_buttons),
#endif
/* -------------------------------- 0x08 VTX ------------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_vtx_variants_and_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_vtx_freq_and_power),
        /* invalids for all guarded checks */
        /* VTX: CRSF.c:1203 (length < 1) */
        cmocka_unit_test(test_parse_vtx_invalid_len_len0),
        cmocka_unit_test(test_parse_vtx_invalid_len_set_freq_lt2),
        cmocka_unit_test(test_parse_vtx_invalid_len_pitmode_lt1),
        cmocka_unit_test(test_parse_vtx_invalid_len_power_lt1),
        /* default unknown subcmd parses OK */
        cmocka_unit_test(test_parse_vtx_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_vtx_pitmode),
#endif
/* -------------------------------- 0x09 LED ------------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_led_color_shift_pulse_blink_and_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_led_color_shift_blink),
        /* invalids (hit all length checks) */
        cmocka_unit_test(test_parse_led_invalid_len_color_lt3),
        cmocka_unit_test(test_parse_led_invalid_len_shift_lt5),
        cmocka_unit_test(test_parse_led_invalid_len_pulse_lt8),
        cmocka_unit_test(test_parse_led_invalid_len_blink_lt8),
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_led_invalid_len_len0),
#endif /* RX */
        /* default unknown subcmd parses OK */
        cmocka_unit_test(test_parse_led_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_led_pulse),
#endif
/* ------------------------------ 0x0A GENERAL ----------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_general_proposal_resp_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_general_prop_resp),
        /* invalids for both checks */
        cmocka_unit_test(test_parse_general_invalid_len_len0),
        cmocka_unit_test(test_parse_general_invalid_len_proposal_lt5),
        cmocka_unit_test(test_parse_general_invalid_len_response_lt2),
        /* default unknown subcmd parses OK */
        cmocka_unit_test(test_parse_general_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_general_prop_and_resp),
#endif
/* ---------------------------- 0x10 CROSSFIRE ----------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_cf_bind_model_and_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_cf_bind_and_model),
        /* invalid: model selection missing model byte */
        cmocka_unit_test(test_parse_cf_invalid_len_len0),
        cmocka_unit_test(test_parse_cf_invalid_len_model_lt1),
        /* default unknown subcmd parses OK */
        cmocka_unit_test(test_parse_cf_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_cf_current_model_reply),
#endif
/* --------------------------- 0x20 FLOW CONTROL --------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_flow_sub_unsub_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_flow_sub_unsub),
        /* invalids */
        cmocka_unit_test(test_parse_flow_invalid_len_len0),
        cmocka_unit_test(test_parse_flow_invalid_len_sub_lt3),
        cmocka_unit_test(test_parse_flow_invalid_len_unsub_lt1),
        /* default unknown subcmd parses OK */
        cmocka_unit_test(test_parse_flow_default_unknown),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_flow_subscribe),
#endif
/* ------------------------------- 0x22 SCREEN ----------------------------- */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_screen_popup_variants_selection_return_default),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_screen_popup_and_selection_return_and_defaults),
/* invalid length cases */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_screen_overflow_info_exact),
        cmocka_unit_test(test_build_screen_overflow_add_data_exact),
        cmocka_unit_test(test_build_screen_overflow_possible_values_exact),
#endif /* TX */
        cmocka_unit_test(test_parse_screen_invalid_len_missing_subcmd),
        cmocka_unit_test(test_parse_screen_invalid_len_selection_return_lt2),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX) && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_roundtrip_screen_selection_return),
#endif
/* Attempt to encode with unsupported Command_ID -> expect CRSF_ERROR_INVALID_FRAME */
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_TX)
        cmocka_unit_test(test_build_top_level_default_invalid_cmd_id),
#endif
#if CRSF_ENABLE_COMMAND && defined(CRSF_CONFIG_RX)
        cmocka_unit_test(test_parse_top_level_default_invalid_cmd_id),
#endif
    };

    printf("CTEST_FULL_OUTPUT\n");
    return cmocka_run_group_tests(tests, NULL, NULL);
}
