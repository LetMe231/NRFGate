#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "ble_nus.h"
#include "nus_handler.h"
#include "scheduler.h"

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

static struct bt_conn *current_conn = NULL;
static bool att_ready = false;

    static char s_cmd_buf[320];
    static size_t s_cmd_len = 0;

static struct k_work_delayable s_snapshot_work;

/* ───────────────────────────────────────────────────────────── */
/* Dashboard latest-state TX coalescing                         */
/* ───────────────────────────────────────────────────────────── */

#define DASH_TX_SLOT_COUNT    6
#define DASH_TX_KEY_MAX       40
#define DASH_TX_JSON_MAX      320
#define DASH_TX_INTERVAL_MS   20   /* 1 packet every 20 ms => 50 pkt/s max */
#define DASH_TX_RETRY_MS      15

struct dash_tx_slot {
    bool in_use;
    bool dirty;
    uint32_t version;
    uint32_t stamp;
    char node_key[DASH_TX_KEY_MAX];
    char json[DASH_TX_JSON_MAX];
};

static struct dash_tx_slot s_dash_slots[DASH_TX_SLOT_COUNT];
static struct k_work_delayable s_dash_flush_work;
static struct k_mutex s_dash_lock;
static uint8_t s_dash_rr_next;
static uint32_t s_dash_stamp;
static const struct bt_le_conn_param s_nus_conn_param = {
    .interval_min = 24,
    .interval_max = 40,
    .latency = 0,
    .timeout = 400,
};

/* ───────────────────────────────────────────────────────────── */

static void snapshot_work_fn(struct k_work *w)
{
    ARG_UNUSED(w);
    nus_handler_send_snapshot();
}

static bool dash_has_dirty_locked(void)
{
    for (int i = 0; i < DASH_TX_SLOT_COUNT; i++) {
        if (s_dash_slots[i].in_use && s_dash_slots[i].dirty) {
            return true;
        }
    }
    return false;
}

static int dash_find_or_alloc_slot_locked(const char *node_key)
{
    int free_idx = -1;
    int oldest_idx = 0;
    uint32_t oldest_stamp = UINT32_MAX;

    for (int i = 0; i < DASH_TX_SLOT_COUNT; i++) {
        if (s_dash_slots[i].in_use) {
            if (strcmp(s_dash_slots[i].node_key, node_key) == 0) {
                return i;
            }
            if (s_dash_slots[i].stamp < oldest_stamp) {
                oldest_stamp = s_dash_slots[i].stamp;
                oldest_idx = i;
            }
        } else if (free_idx < 0) {
            free_idx = i;
        }
    }

    return (free_idx >= 0) ? free_idx : oldest_idx;
}

static int dash_pick_next_locked(uint8_t *out_idx,
                                 uint32_t *out_version,
                                 char *out_json,
                                 size_t out_json_size)
{
    for (int n = 0; n < DASH_TX_SLOT_COUNT; n++) {
        uint8_t i = (s_dash_rr_next + n) % DASH_TX_SLOT_COUNT;

        if (!s_dash_slots[i].in_use || !s_dash_slots[i].dirty) {
            continue;
        }

        *out_idx = i;
        *out_version = s_dash_slots[i].version;

        strncpy(out_json, s_dash_slots[i].json, out_json_size - 1);
        out_json[out_json_size - 1] = '\0';

        s_dash_rr_next = (i + 1) % DASH_TX_SLOT_COUNT;
        return 0;
    }

    return -ENOENT;
}

static int ble_nus_send_now(const char *json)
{
    struct bt_conn *conn;
    int ret;

    if (!json || !current_conn || !att_ready) {
        return -ENOTCONN;
    }

    conn = bt_conn_ref(current_conn);
    if (!conn) {
        return -ENOTCONN;
    }

    ret = bt_nus_send(conn, (const uint8_t *)json, strlen(json));
    bt_conn_unref(conn);

    return ret;
}

static void dash_schedule_flush(k_timeout_t delay)
{
    k_work_schedule(&s_dash_flush_work, delay);
}

static void dash_flush_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);

    char json[DASH_TX_JSON_MAX];
    uint8_t slot_idx;
    uint32_t version;
    bool more_dirty = false;

    k_mutex_lock(&s_dash_lock, K_FOREVER);
    int pick_ret = dash_pick_next_locked(&slot_idx, &version,
                                         json, sizeof(json));
    k_mutex_unlock(&s_dash_lock);

    if (pick_ret) {
        return;
    }

    int ret = ble_nus_send_now(json);

    if (ret == 0) {
        k_mutex_lock(&s_dash_lock, K_FOREVER);

        if (s_dash_slots[slot_idx].in_use &&
            s_dash_slots[slot_idx].version == version) {
            s_dash_slots[slot_idx].dirty = false;
        }

        more_dirty = dash_has_dirty_locked();
        k_mutex_unlock(&s_dash_lock);

        if (more_dirty) {
            dash_schedule_flush(K_MSEC(DASH_TX_INTERVAL_MS));
        }
        return;
    }

    if (ret == -ENOTCONN) {
        /* Verbindung weg – dirty Daten bleiben erhalten und gehen nach Reconnect raus */
        return;
    }

    if (ret == -ENOMEM || ret == -ENOBUFS) {
        dash_schedule_flush(K_MSEC(DASH_TX_RETRY_MS));
        return;
    }

    LOG_WRN("dash flush bt_nus_send failed: %d", ret);
    dash_schedule_flush(K_MSEC(DASH_TX_INTERVAL_MS));
}

static void nus_send_enabled(enum bt_nus_send_status status)
{
    if (status == BT_NUS_SEND_STATUS_ENABLED) {
        att_ready = true;

        if (current_conn) {
            uint16_t mtu = bt_gatt_get_mtu(current_conn);
            LOG_INF("NUS notifications enabled by client (ATT MTU=%u, payload=%u)",
                    mtu, (mtu > 3) ? (mtu - 3) : 0);
        } else {
        LOG_INF("NUS notifications enabled by client");
        }

        dash_schedule_flush(K_NO_WAIT);
        k_work_schedule(&s_snapshot_work, K_MSEC(200));
         
    } else {
        att_ready = false;
        LOG_INF("NUS notifications disabled by client");
    }
}

static void nus_received(struct bt_conn *conn,
                         const uint8_t *data, uint16_t len)
{
    ARG_UNUSED(conn);

    for (uint16_t i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                s_cmd_buf[s_cmd_len] = '\0';

                if (s_cmd_len > 0) {
                    LOG_INF("Full NUS cmd: %s", s_cmd_buf);
                    nus_handler_cmd(s_cmd_buf, s_cmd_len);
                }
                s_cmd_len = 0;
                continue;
            }

        if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) {
            LOG_WRN("Invalid byte in NUS cmd: 0x%02X", (uint8_t)c);
                s_cmd_len = 0;
                continue;
            }

            if (s_cmd_len < sizeof(s_cmd_buf) - 1) {
                s_cmd_buf[s_cmd_len++] = c;
            } else {
                LOG_WRN("NUS cmd buffer overflow, dropping command");
                s_cmd_len = 0;
            }
        }
}

static struct bt_nus_cb nus_cbs = {
    .received = nus_received,
    .send_enabled = nus_send_enabled,
};

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS,
                  BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_NAME_COMPLETE,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int ble_nus_register(void)
{
    int err = bt_nus_init(&nus_cbs);
    if (err) {
        LOG_ERR("bt_nus_init failed: %d", err);
    } else {
        LOG_INF("NUS service registered OK");
    }

    k_work_init_delayable(&s_snapshot_work, snapshot_work_fn);
    k_work_init_delayable(&s_dash_flush_work, dash_flush_work_fn);
    k_mutex_init(&s_dash_lock);

    return err;
}

int ble_nus_advertise(void)
{
    int err;

    LOG_INF("Starting NUS advertising...");

    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
                          ad, ARRAY_SIZE(ad),
                          NULL, 0);
    if (err) {
        LOG_ERR("bt_le_adv_start failed: %d", err);
        return err;
    }

    LOG_INF("NUS advertising started as '%s'", CONFIG_BT_DEVICE_NAME);
    return 0;
}

bool ble_nus_is_ready(void)
{
    return current_conn != NULL && att_ready;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        current_conn = bt_conn_ref(conn);
        att_ready = false;
        s_cmd_len = 0;
        memset(s_cmd_buf, 0, sizeof(s_cmd_buf));
        LOG_INF("NUS client connected");
        // int perr = bt_conn_le_param_update(conn, &s_nus_conn_param);
        // LOG_INF("NUS conn param update requested: %d", perr);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        att_ready = false;
        s_cmd_len = 0;
        memset(s_cmd_buf, 0, sizeof(s_cmd_buf));

        LOG_INF("NUS disc (reason 0x%02x, mesh_active=%d, ms_in_state=%lld)",
        reason,
        scheduler_is_ble_active(),
        scheduler_ms_since_switch());

        nus_handler_on_disconnect();

        int err = ble_nus_advertise();
        if (err) {
            LOG_ERR("Failed to restart NUS adv: %d", err);
        }
    }
}
static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
    ARG_UNUSED(conn);

    LOG_INF("LE param req: min=%u max=%u lat=%u timeout=%u",
            param->interval_min, param->interval_max,
            param->latency, param->timeout);

    /* Nur zum Debuggen: ALLES ablehnen */
    return false;
}

static void le_param_updated(struct bt_conn *conn,
                             uint16_t interval,
                             uint16_t latency,
                             uint16_t timeout)
{
    ARG_UNUSED(conn);
    LOG_INF("LE param updated: interval=%u latency=%u timeout=%u",
            interval, latency, timeout);
}

BT_CONN_CB_DEFINE(conn_cb) = {
    .connected        = connected,
    .disconnected     = disconnected,
    .le_param_req     = le_param_req,
    .le_param_updated = le_param_updated,
};

void ble_nus_send_immediate(const char *json)
{
    static atomic_t s_drop_cnt = ATOMIC_INIT(0);
    static int64_t s_last_log_ms;

    if (!json) {
        return;
    }

    int ret = ble_nus_send_now(json);
    int64_t now = k_uptime_get();

    if (ret == 0 || ret == -ENOTCONN) {
        return;
    }

    if (ret == -ENOMEM || ret == -ENOBUFS) {
        atomic_inc(&s_drop_cnt);
        if ((now - s_last_log_ms) > 5000) {
            s_last_log_ms = now;
            LOG_WRN("NUS immediate TX backpressure, dropped=%ld",
                    (long)atomic_get(&s_drop_cnt));
        }
        return;
    }

    LOG_ERR("bt_nus_send immediate failed: %d", ret);
}

void ble_nus_publish_latest(const char *node_key, const char *json)
{
    if (!node_key || !json) {
        return;
    }

    size_t key_len = strlen(node_key);
    size_t json_len = strlen(json);

    if (key_len == 0 || key_len >= DASH_TX_KEY_MAX) {
        LOG_WRN("ble_nus_publish_latest: invalid node_key len=%u",
                (unsigned)key_len);
        return;
    }

    if (json_len == 0 || json_len >= DASH_TX_JSON_MAX) {
        LOG_WRN("ble_nus_publish_latest: invalid json len=%u",
                (unsigned)json_len);
        return;
    }

    k_mutex_lock(&s_dash_lock, K_FOREVER);

    int idx = dash_find_or_alloc_slot_locked(node_key);
    struct dash_tx_slot *slot = &s_dash_slots[idx];

    slot->in_use = true;
    slot->dirty = true;
    slot->version++;
    slot->stamp = ++s_dash_stamp;

    strncpy(slot->node_key, node_key, sizeof(slot->node_key) - 1);
    slot->node_key[sizeof(slot->node_key) - 1] = '\0';

    strncpy(slot->json, json, sizeof(slot->json) - 1);
    slot->json[sizeof(slot->json) - 1] = '\0';

    k_mutex_unlock(&s_dash_lock);

    if (ble_nus_is_ready()) {
        dash_schedule_flush(K_NO_WAIT);
    }
}

void ble_nus_send(const char *json)
{
    /* Legacy wrapper */
    ble_nus_send_immediate(json);
}