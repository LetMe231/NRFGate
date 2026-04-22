#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gw_store.h"

LOG_MODULE_REGISTER(gw_store, LOG_LEVEL_INF);

/* ── Interner Zustand ──────────────────────────────────────── */
static gw_node_record_t nodes[GW_STORE_MAX_NODES];
static uint8_t node_count = 0;
static K_MUTEX_DEFINE(store_lock);

static int gw_store_find_node_unlocked(const gw_node_addr_t *addr);
static int gw_store_find_or_create_node_unlocked(const gw_node_addr_t *addr,
                                                 int64_t now_ms);
static int gw_store_apply_event_unlocked(const gw_event_t *evt);

/* ── Init ──────────────────────────────────────── */

void gw_store_init(void)
{
    k_mutex_lock(&store_lock, K_FOREVER);
    memset(nodes, 0, sizeof(nodes));
    node_count = 0;
    k_mutex_unlock(&store_lock);
    LOG_INF("Gateway store initialized (max nodes: %d)", GW_STORE_MAX_NODES);
}

/* ── Adress comparison ──────────────────────────────────────── */

bool gw_addr_eq(const gw_node_addr_t *a, const gw_node_addr_t *b)
{
    if (!a || !b) {
        return false;
    }
    if (a->transport != b->transport) return false;

    switch (a->transport) {
        case GW_TR_BLE_MESH: {
            return a->mesh_addr == b->mesh_addr;
        }
        case GW_TR_THREAD: {
            return memcmp(a->ipv6, b->ipv6, GW_IPV6_BIN_LEN) == 0;
        }
        case GW_TR_LORAWAN: {
            return (a->dev_eui_hi == b->dev_eui_hi) &&
                   (a->dev_eui_lo == b->dev_eui_lo);
        }
        default:
            return false;
    }
}

/* ── Node Lookup ──────────────────────────────────────── */

int gw_store_find_node(const gw_node_addr_t *addr)
{
    int idx;

    k_mutex_lock(&store_lock, K_FOREVER);
    idx = gw_store_find_node_unlocked(addr);
    k_mutex_unlock(&store_lock);

    return idx;
}

/* ── Node Lookup Helpers ──────────────────────────────────────── */

static int gw_store_find_node_unlocked(const gw_node_addr_t *addr)
{
    if (!addr) {
        return -EINVAL;
    }

    for (uint8_t i = 0; i < node_count; i++) {
        if (nodes[i].known && gw_addr_eq(&nodes[i].addr, addr)) {
            return i;
        }
    }

    return -ENOENT;
}

/* ── Find or Create Node ──────────────────────────────────────── */
static int gw_store_find_or_create_node_unlocked(const gw_node_addr_t *addr, int64_t now_ms)
{
    // return existing node index if found
    int idx = gw_store_find_node_unlocked(addr);
    if (idx >= 0) {
        return idx;
    }

    // error if store is full
    if (node_count >= GW_STORE_MAX_NODES) {
        LOG_ERR("Node store full, cannot add new node: max_count=%d", GW_STORE_MAX_NODES);
        return -ENOMEM;
    }

    // new node, add to store
    idx = node_count;
    gw_node_record_t *rec = &nodes[idx];

    memset(rec, 0, sizeof(*rec));
    rec->known = true;
    rec->addr = *addr;
    rec->state = GW_STATE_UNKNOWN;
    rec->first_seen_ms = now_ms;
    rec->last_seen_ms = now_ms;

    node_count++;
    LOG_INF("New node added to store: idx=%d transport=%d", idx, addr->transport);
    return idx;
}

/* ── Apply Event (internal) ──────────────────────────────────────── */
static int gw_store_apply_event_unlocked(const gw_event_t *evt)
{
    if (!evt) {
        return -EINVAL;
    }

    int idx = gw_store_find_or_create_node_unlocked(&evt->src, evt->rx_ms);
    if (idx < 0) {
        return idx;
    }

    gw_node_record_t *rec = &nodes[idx];
    rec->last_rx_meta = evt->rx_meta;
    rec->last_seen_ms = evt->rx_ms;

    if (evt->rx_meta.has_rssi) {
        rec->stats.has_last_rssi = true;
        rec->stats.last_rssi_dbm = evt->rx_meta.rssi_dbm;
    }
    if (evt->rx_meta.has_hops) {
        rec->stats.has_last_hops = true;
        rec->stats.last_hops     = evt->rx_meta.hops;
    }

    switch (evt->type) {
    case GW_EVT_SENSOR: {
        const gw_sensor_payload_t *s = &evt->data.sensor;

        if (s->present & GW_HAS_SEQ) {
            if (rec->stats.has_last_seq) {
                int32_t expected = rec->stats.last_seq + 1;
                if (s->seq != expected) {
                    int32_t gap = s->seq - expected;
                    if (gap > 0) {
                        rec->stats.seq_gaps += (uint32_t)gap;
                        LOG_WRN("Node %d seq gap: expected %d, got %d (total gaps: %d)",
                                idx, expected, s->seq, rec->stats.seq_gaps);
                    }
                }
            }
            rec->stats.last_seq = s->seq;
            rec->stats.has_last_seq = true;
        }

        rec->last_sensor     = *s;
        rec->has_last_sensor = true;
        rec->stats.packet_count++;

        if (s->present & GW_HAS_TEMP)       rec->caps.has_temp       = true;
        if (s->present & GW_HAS_HUM)        rec->caps.has_hum        = true;
        if (s->present & GW_HAS_ACCEL)      rec->caps.has_accel      = true;
        if (s->present & GW_HAS_GYRO)       rec->caps.has_gyro       = true;
        if (s->present & GW_HAS_TVOC)       rec->caps.has_tvoc       = true;
        if (s->present & GW_HAS_ECO2)       rec->caps.has_eco2       = true;
        if (s->present & GW_HAS_HEART_RATE) rec->caps.has_heart_rate = true;
        if (s->present & GW_HAS_SPO2)       rec->caps.has_spo2       = true;
        if (s->present & GW_HAS_PM25)       rec->caps.has_pm25       = true;
        if (s->present & GW_HAS_PM10)       rec->caps.has_pm10       = true;
        if (s->present & GW_HAS_SWITCH)     rec->caps.has_switch     = true;
        if (s->present & GW_HAS_LIGHT)      rec->caps.has_light      = true;
        if (s->present & GW_HAS_LIGHT) {
            rec->last_light_on = s->light_on;
            rec->has_last_actuator_state = true;
        }
        break;
    }

    case GW_EVT_BUTTON:
        rec->last_button_pressed = evt->data.button.pressed;
        rec->last_button_ms      = evt->rx_ms;
        rec->has_last_button     = true;
        rec->stats.packet_count++;
        rec->caps.has_switch = true;
        break;

    case GW_EVT_ACTUATOR_STATE: {
        bool light_on = evt->data.actuator_state.light_on;

        rec->last_light_on = light_on;
        rec->has_last_actuator_state = true;
        rec->stats.packet_count++;
        rec->caps.has_light = true;

        rec->has_pending_light_cmd = false;
        rec->pending_cmd_id = 0;
        rec->pending_since_ms = 0;

        LOG_DBG("Node [%d] Actuator state: light is now %s", idx, light_on ? "ON" : "OFF");
        break;
    }
    case GW_EVT_STATE_TRANSITION:
        if (rec->state != evt->data.state_transition.from) {
            LOG_WRN("Node [%d] state transition mismatch: expected from %d, got from %d",
                    idx, rec->state, evt->data.state_transition.from);
        }
        rec->state = evt->data.state_transition.to;
        LOG_INF("Node [%d] State: %d → %d",
                idx,
                evt->data.state_transition.from,
                evt->data.state_transition.to);
        break;
    case GW_EVT_CMD_PENDING: {
        if (evt->data.cmd_pending.cmd_type == GW_CMD_LIGHT_ON ||
            evt->data.cmd_pending.cmd_type == GW_CMD_LIGHT_OFF) {
            rec->has_pending_light_cmd = true;
            rec->pending_light_on      = evt->data.cmd_pending.light_on;
            rec->pending_cmd_id        = evt->data.cmd_pending.cmd_id;
            rec->pending_since_ms      = evt->rx_ms;
        }
        break;
    }
    case GW_EVT_ACK:
        LOG_DBG("Node [%d] ACK received for cmd_id=%d seq=%d",
                idx,
                evt->data.ack.cmd_id,
                evt->data.ack.seq);
    case GW_EVT_TIMEOUT: {
        LOG_DBG("Node [%d] Timeout: cmd_id=%u", idx, evt->data.timeout.cmd_id);

        if (rec->has_pending_light_cmd &&
            rec->pending_cmd_id == evt->data.timeout.cmd_id) {
            rec->has_pending_light_cmd = false;
            rec->pending_cmd_id = 0;
            rec->pending_since_ms = 0;
        }
        break;
    }
    default:
        LOG_WRN("Unknown event type: %d", evt->type);
        break;
    }

    return idx;
}

/* ── Apply Event ──────────────────────────────────────── */

int gw_store_apply_event(const gw_event_t *evt)
{
    int idx;

    if (!evt) {
        return -EINVAL;
    }

    k_mutex_lock(&store_lock, K_FOREVER);
    idx = gw_store_apply_event_unlocked(evt);
    k_mutex_unlock(&store_lock);

    return idx;
}
/* ── Set State ─────────────────────────────────────────────── */

int gw_store_set_state(const gw_node_addr_t *addr,
                       gw_state_t new_state,
                       int64_t now_ms)
{
    if (!addr) {
        return -EINVAL;
    }

    k_mutex_lock(&store_lock, K_FOREVER);

    int idx = gw_store_find_node_unlocked(addr);
    if (idx < 0) {
        k_mutex_unlock(&store_lock);
        return idx;
    }

    gw_event_t evt = {
        .type  = GW_EVT_STATE_TRANSITION,
        .src   = *addr,
        .rx_ms = now_ms,
        .data.state_transition = {
            .from = nodes[idx].state,
            .to   = new_state,
        },
    };

    int ret = gw_store_apply_event_unlocked(&evt);

    k_mutex_unlock(&store_lock);
    return ret;
}
/* ── Get Node Info ──────────────────────────────────────── */

bool gw_store_get_node_copy(uint8_t idx, gw_node_record_t *out)
{
    if (!out) {
        return false;
    }

    k_mutex_lock(&store_lock, K_FOREVER);

    if (idx >= node_count || !nodes[idx].known) {
        k_mutex_unlock(&store_lock);
        return false;
    }

    *out = nodes[idx];

    k_mutex_unlock(&store_lock);
    return true;
}

/* ── Foreach Node ──────────────────────────────────────── */

void gw_store_foreach_node(void (*cb)(const gw_node_record_t *rec, void *ctx),
                           void *ctx)
{
    if (!cb) {
        return;
    }

    gw_node_record_t snapshot[GW_STORE_MAX_NODES];
    uint8_t snapshot_count = 0;

    k_mutex_lock(&store_lock, K_FOREVER);
    for (uint8_t i = 0; i < node_count; i++) {
        if (nodes[i].known) {
            snapshot[snapshot_count++] = nodes[i];
        }
    }
    k_mutex_unlock(&store_lock);

    for (uint8_t i = 0; i < snapshot_count; i++) {
        cb(&snapshot[i], ctx);
    }
}

/* ── Count ──────────────────────────────────────── */

uint8_t gw_store_count(void)
{
    k_mutex_lock(&store_lock, K_FOREVER);
    uint8_t count = node_count;
    k_mutex_unlock(&store_lock);
    return count;
}