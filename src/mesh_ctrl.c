#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>
#include <string.h>

#include "mesh_ctrl.h"

LOG_MODULE_REGISTER(mesh_ctrl, LOG_LEVEL_INF);

// Model pointer set at init, used for sending messages
static const struct bt_mesh_model *s_model = NULL;

// Transaction ID for on/off messages, incremented with each new message
static uint8_t onoff_tid = 0;

// last known on/off state of each node
#define STATE_CACHE_SIZE 128
static uint8_t s_state[STATE_CACHE_SIZE];

// public API
void mesh_ctrl_init(const struct bt_mesh_model *model)
{
    s_model = model;
    memset(s_state, 0xFF, sizeof(s_state)); // 0xFF = unknown
    LOG_INF("mesh_ctrl ready (Generic OnOff Client)");
}

void mesh_ctrl_on_status(uint16_t addr, uint8_t state){
    uint8_t idx = (uint8_t)(addr & (STATE_CACHE_SIZE - 1)); // simple hash to cache index
    s_state[idx] = state;
    LOG_INF("OnOff status cached: 0x%04X = %s", addr, state ? "ON" : "OFF");
}

void mesh_ctrl_set_onoff(uint16_t addr, bool on)
{
    if (!s_model) {
        LOG_ERR("Model not initialized");
        return;
    }

    BT_MESH_MODEL_BUF_DEFINE(msg, BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK, 2);
    bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK);
    net_buf_simple_add_u8(&msg, on ? 1 : 0); // on/off state
    net_buf_simple_add_u8(&msg, onoff_tid++); // transaction ID

    struct bt_mesh_msg_ctx ctx = {
        .net_idx = 0, // using primary network
        .app_idx = 0, // using primary app key
        .addr = addr,
        .send_ttl = BT_MESH_TTL_DEFAULT,
    };

    int err = bt_mesh_model_send(s_model, &ctx, &msg, NULL, NULL);
    if (err) {
        LOG_ERR("Send failed: addr=0x%04X err=%d", addr, err);
        return;
    }
    
    LOG_INF("Sent OnOff Set to 0x%04X: %s", addr, on ? "ON" : "OFF");

    // Optimistically update cache (since unacknowledged)
    uint8_t idx = (int8_t)(addr & (STATE_CACHE_SIZE - 1));
    s_state[idx] = on ? 1 : 0; // update cache with new state
}

void mesh_ctrl_toggle(uint16_t addr)
{
    uint8_t idx = (int8_t)(addr & (STATE_CACHE_SIZE - 1));
    uint8_t current = s_state[idx];
    // unknown state, default to OFF -> toggle to ON
    mesh_ctrl_set_onoff(addr, current == 1 ? false : true);
}