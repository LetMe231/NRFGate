#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

#include "ble_nus.h"
#include "data_handler.h"

LOG_MODULE_REGISTER(ble_nus, LOG_LEVEL_INF);

static struct bt_conn *current_conn = NULL;
static bool att_ready = false;
    static char s_cmd_buf[256];
    static size_t s_cmd_len = 0;

static void nus_send_enabled(enum bt_nus_send_status status)
{
    if (status == BT_NUS_SEND_STATUS_ENABLED) {
        att_ready = true;
        LOG_INF("NUS notifications enabled by client");
    } else {
        att_ready = false;
        LOG_INF("NUS notifications disabled by client");
    }
}

static void nus_received(struct bt_conn *conn,
                         const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
            char c = (char)data[i];

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                s_cmd_buf[s_cmd_len] = '\0';

                if (s_cmd_len > 0) {
                    LOG_INF("Full NUS cmd: %s", s_cmd_buf);
                    data_handler_cmd(s_cmd_buf, s_cmd_len);
                }
                s_cmd_len = 0;
                continue;
            }

            // Reject invalid bytes
            if ((uint8_t) c < 0x20 || (uint8_t) c > 0x7E) {
                LOG_WRN("Invalid byte in NUS cmd: 0x%02X", c);
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
    BT_DATA(0x09,
            CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_le_ext_adv *adv_set;

int ble_nus_register(void)
{
    int err = bt_nus_init(&nus_cbs);
    if (err) {
        LOG_ERR("bt_nus_init failed: %d", err);
    } else {
        LOG_INF("NUS service registered OK");
    }
    return err;
}

int ble_nus_advertise(void)
{
    int err;
    LOG_INF("Starting NUS advertising...");

    if(!adv_set){
        struct bt_le_adv_param param = BT_LE_ADV_PARAM_INIT(
            BT_LE_ADV_OPT_CONN,
            BT_GAP_ADV_FAST_INT_MIN_2,
            BT_GAP_ADV_FAST_INT_MAX_2,
            NULL);

        err = bt_le_ext_adv_create(&param, NULL, &adv_set);
        if (err) { LOG_ERR("ext_adv_create failed: %d", err); return err; }

        err = bt_le_ext_adv_set_data(adv_set, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err) { LOG_ERR("ext_adv_set_data failed: %d", err); return err; }
    }
    err = bt_le_ext_adv_start(adv_set, BT_LE_EXT_ADV_START_DEFAULT);
    if (err) { LOG_ERR("ext_adv_start failed: %d", err); return err; }

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
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
        att_ready = false;
        s_cmd_len = 0;
        memset(s_cmd_buf, 0, sizeof(s_cmd_buf));
        LOG_INF("NUS client disconnected");
        int err = ble_nus_advertise();
        if(err){
            LOG_ERR("Failed to restart NUS adv: %d", err);
        }
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

void ble_nus_send(const char *json)
{
    if (!current_conn || !att_ready) {
        LOG_WRN("NUS not ready (conn=%d att=%d)",
                current_conn != NULL, att_ready);
        return;
    }
    int ret = bt_nus_send(current_conn, (const uint8_t *)json, strlen(json));
    if (ret) LOG_WRN("bt_nus_send failed: %d", ret);
}