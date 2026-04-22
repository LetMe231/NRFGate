#ifndef GW_MODEL_H
#define GW_MODEL_H

#include <stdint.h>
#include <stdbool.h>

#define GW_IPV6_STR_LEN 40
#define GW_IPV6_BIN_LEN 16

/* ── Transport ─────────────────────────────────────────────── */

typedef enum {
    GW_TR_UNKNOWN = 0,
    GW_TR_BLE_MESH,
    GW_TR_THREAD,
    GW_TR_LORAWAN,
} gw_transport_t;

/* ── Semantic / node states ───────────────────────────────── */

typedef enum {
    GW_STATE_UNKNOWN = 0,
    GW_STATE_IDLE,
    GW_STATE_ACTIVE,
    GW_STATE_ALERT,
    GW_STATE_CRITICAL,
    GW_STATE_LOST,
} gw_state_t;

/* ── Event type ────────────────────────────────────────────── */

typedef enum {
    GW_EVT_SENSOR = 0,
    GW_EVT_BUTTON,
    GW_EVT_ACTUATOR_STATE,
    GW_EVT_STATE_TRANSITION,
    GW_EVT_ACK,
    GW_EVT_TIMEOUT,
    GW_EVT_CMD_PENDING,
} gw_event_type_t;

/* ── Command type ──────────────────────────────────────────── */

typedef enum {
    GW_CMD_NONE = 0,
    GW_CMD_LIGHT_ON,
    GW_CMD_LIGHT_OFF,
    GW_CMD_LIGHT_TOGGLE,
} gw_cmd_type_t;

typedef struct {
    uint32_t cmd_id;
    gw_cmd_type_t cmd_type;
    bool light_on;
} gw_cmd_pending_payload_t;


/* ── Address / identity ───────────────────────────────────── */

typedef struct {
    gw_transport_t transport;
    uint16_t mesh_addr;              /* valid for BLE Mesh */
    uint8_t ipv6[GW_IPV6_BIN_LEN];  /* valid for Thread */
    uint32_t dev_eui_hi;             /* valid for LoRaWAN */
    uint32_t dev_eui_lo;             /* valid for LoRaWAN */
} gw_node_addr_t;

/* ── Connection Metadata ───────────────────────────────────── */

typedef struct {
    bool has_rssi;
    int8_t rssi_dbm;

    bool has_hops;
    uint8_t hops;

    bool has_ttl_rx;
    uint8_t rx_ttl;

    bool has_snr;
    int16_t snr_db_x10;

    // Testing fields
    bool has_tx_ms;
    int32_t tx_ms;
    bool has_boot_id;
    int32_t boot_id;
} gw_rx_meta_t;

/* ── Capability description ────────────────────────────────── */

typedef struct {
    bool has_temp;
    bool has_hum;
    bool has_accel;
    bool has_gyro;
    bool has_tvoc;
    bool has_eco2;
    bool has_heart_rate;
    bool has_spo2;
    bool has_pm25;
    bool has_pm10;
    bool has_switch;
    bool has_light;
} gw_caps_t;

/* ── Presence mask for actual payload contents ─────────────── */

#define GW_HAS_NONE        0u
#define GW_HAS_SEQ         (1u << 0)
#define GW_HAS_TS          (1u << 1)
#define GW_HAS_TEMP        (1u << 2)
#define GW_HAS_HUM         (1u << 3)
#define GW_HAS_ACCEL       (1u << 4)
#define GW_HAS_GYRO        (1u << 5)
#define GW_HAS_TVOC        (1u << 6)
#define GW_HAS_ECO2        (1u << 7)
#define GW_HAS_HEART_RATE  (1u << 8)
#define GW_HAS_SPO2        (1u << 9)
#define GW_HAS_PM25        (1u << 10)
#define GW_HAS_PM10        (1u << 11)
#define GW_HAS_SWITCH      (1u << 12)
#define GW_HAS_LIGHT       (1u << 13)
#define GW_HAS_RAW_RED     (1u << 14)
#define GW_HAS_RAW_IR      (1u << 15)


/* ── Canonical sensor payload ────────────────────────────────
 *
 * Alles als Integer / fixed point:
 * - temp_mc       : milli-degree Celsius
 * - hum_mpermille : milli-percent RH
 * - accel_mg      : milli-g
 * - gyro_mdps     : milli-deg/s
 * - spo2_mpermille: milli-percent
 */
typedef struct {
    uint32_t present;

    int32_t seq;
    int32_t ts;

    int32_t temp_mc;
    int32_t hum_mpermille;

    int32_t ax_mg;
    int32_t ay_mg;
    int32_t az_mg;

    int32_t gx_mdps;
    int32_t gy_mdps;
    int32_t gz_mdps;

    int32_t tvoc_ppb;
    int32_t eco2_ppm;

    int32_t heart_rate_bpm;
    int32_t spo2_mpermille;

    int32_t raw_red;
    int32_t raw_ir;

    int32_t pm25_ugm3;
    int32_t pm10_ugm3;

    bool switch_state;
    bool light_on;
} gw_sensor_payload_t;

/* ── Last known stats ────────────────────────────────── */

typedef struct {
    uint32_t packet_count;
    uint32_t seq_gaps;
    bool has_last_seq;
    int32_t last_seq;

    bool has_last_rssi;
    int8_t last_rssi_dbm;

    bool has_last_hops;
    uint8_t last_hops;
} gw_node_stats_t;

typedef struct {
    bool known;

    gw_node_addr_t addr;
    gw_caps_t caps;
    gw_state_t state;

    gw_node_stats_t stats;
    gw_rx_meta_t last_rx_meta;
    bool has_last_sensor;
    gw_sensor_payload_t last_sensor;

    bool has_last_button;
    bool last_button_pressed;
    int64_t last_button_ms;

    bool has_last_actuator_state;
    bool last_light_on;

    int64_t first_seen_ms;
    int64_t last_seen_ms;

    bool     has_pending_light_cmd;
    bool     pending_light_on;
    uint32_t pending_cmd_id;
    int64_t  pending_since_ms;
} gw_node_record_t;

/* ── Events ────────────────────────────────────────────────── */

typedef struct {
    gw_event_type_t type;

    gw_node_addr_t src;
    gw_rx_meta_t rx_meta;

    int64_t rx_ms;       /* gateway uptime receive timestamp */
    uint32_t event_id;   /* optional internal event id */

    union {
        gw_sensor_payload_t sensor;

        struct {
            int32_t seq;
            bool pressed;
        } button;

        struct {
            int32_t seq;
            bool light_on;
        } actuator_state;

        struct {
            gw_state_t from;
            gw_state_t to;
        } state_transition;

        struct {
            uint32_t cmd_id;
            int32_t seq;
        } ack;

        struct {
            uint32_t cmd_id;
        } timeout;

        struct {
            uint32_t cmd_id;
            gw_cmd_type_t cmd_type;
            bool light_on;
        } cmd_pending;
    } data;
} gw_event_t;

/* ── Commands ──────────────────────────────────────────────── */

typedef struct {
    uint32_t cmd_id;
    gw_cmd_type_t type;
    gw_node_addr_t dst;
    int64_t created_ms;
} gw_command_t;
#endif /* GW_MODEL_H */