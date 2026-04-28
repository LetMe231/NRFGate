#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include "reliability_manager.h"
#include "event_ingest.h"

LOG_MODULE_REGISTER(reliability_manager, LOG_LEVEL_INF);

/* ── Tracked Command Entry ─────────────────────────────────────── */

typedef struct {
    bool         active;
    bool         wants_light_on;
    gw_command_t cmd;
    struct k_work_delayable timeout_work;
} tracked_cmd_t;

/* ── Internal State ───────────────────────────────────────────── */

static tracked_cmd_t s_tracked[RELIABILITY_MAX_TRACKED];
static K_MUTEX_DEFINE(s_lock);

/* ── Forward Declaration ──────────────────────────────────────── */

static void timeout_handler(struct k_work *work);

static int cmd_type_to_desired_light(const gw_command_t *cmd, bool *light_on)
{
    if (!cmd || !light_on) {
        return -EINVAL;
    }

    switch (cmd->type) {
    case GW_CMD_LIGHT_ON:
        *light_on = true;
        return 0;

    case GW_CMD_LIGHT_OFF:
        *light_on = false;
        return 0;

    default:
        return -ENOTSUP;
    }
}

/* ── Init ─────────────────────────────────────────────────────── */

void reliability_manager_init(void)
{
    k_mutex_lock(&s_lock, K_FOREVER);
    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        s_tracked[i].active = false;
        s_tracked[i].wants_light_on = false;
        k_work_init_delayable(&s_tracked[i].timeout_work, timeout_handler);
    }
    k_mutex_unlock(&s_lock);

    LOG_INF("Reliability manager initialized (max tracked: %d)",
            RELIABILITY_MAX_TRACKED);
}

/* ── Track Command ───────────────────────────────────────────── */

int reliability_manager_track(const gw_command_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) {
        return -EINVAL;
    }

    bool wants_light_on = false;
    int map_ret = cmd_type_to_desired_light(cmd, &wants_light_on);

    /* Only track explicit ON/OFF commands */
    if (map_ret == -ENOTSUP) {
        LOG_WRN("Not tracking unsupported cmd_type=%d for cmd_id=%u",
                cmd->type, cmd->cmd_id);
        return 0;
    } else if (map_ret) {
        return map_ret;
    }

    k_mutex_lock(&s_lock, K_FOREVER);

    // Find an empty slot
    int slot = -1;
    int same_dst_slot = -1;

    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        if (s_tracked[i].active) {
            if (gw_addr_equal(&s_tracked[i].cmd.dst, &cmd->dst)) {
                same_dst_slot = i;
                break;
            }
        } else if (slot < 0) {
            slot = i;
        }
    }

    /* replace already pending command to avoid timeouts*/
    if (same_dst_slot >= 0) {
        k_work_cancel_delayable(&s_tracked[same_dst_slot].timeout_work);
        s_tracked[same_dst_slot].active = false;
        slot = same_dst_slot;

        LOG_WRN("Replacing pending cmd_id=%u with cmd_id=%u on same dst",
                s_tracked[same_dst_slot].cmd.cmd_id, cmd->cmd_id);
        }

    if (slot < 0) {
        k_mutex_unlock(&s_lock);
        LOG_ERR("Tracking table full, cannot track cmd_id %u", cmd->cmd_id);
        return -ENOMEM;
    }

    s_tracked[slot].active = true;
    s_tracked[slot].wants_light_on = wants_light_on;
    s_tracked[slot].cmd = *cmd;

    k_mutex_unlock(&s_lock);
    
    k_work_schedule(&s_tracked[slot].timeout_work, K_MSEC(timeout_ms));

    LOG_INF("Tracking cmd_id=%u type=%d desired_light=%d timeout=%u ms",
            cmd->cmd_id, cmd->type, wants_light_on, timeout_ms);

    return 0;
}

/* ── Timeout Handler ─────────────────────────────────────────── */

static void timeout_handler(struct k_work *work)
{
    // Derive slot index from work pointer
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    tracked_cmd_t *entry = CONTAINER_OF(dwork, tracked_cmd_t, timeout_work);

    k_mutex_lock(&s_lock, K_FOREVER);

    if (!entry->active) {
        k_mutex_unlock(&s_lock);
        return; // Already acknowledged
    }

    gw_command_t cmd = entry->cmd;
    entry->active = false; // Mark slot as free

    k_mutex_unlock(&s_lock);

    LOG_WRN("Timeout: cmd_id=%u type=%d transport=%d",
            cmd.cmd_id, cmd.type, cmd.dst.transport);

    gw_event_t evt = {
        .type  = GW_EVT_TIMEOUT,
        .rx_ms = k_uptime_get(),
        .src   = cmd.dst,
        .data.timeout = {
            .cmd_id = cmd.cmd_id,
        },
    };

    event_ingest_submit(&evt);
}

/* ── ACK by explicit cmd_id ─────────────────────────────────── */

int reliability_manager_ack(uint32_t cmd_id, int32_t seq)
{
    k_mutex_lock(&s_lock, K_FOREVER);

    // Find the tracked command
    int slot = -1;
    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        if (s_tracked[i].active && s_tracked[i].cmd.cmd_id == cmd_id) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        k_mutex_unlock(&s_lock);
        LOG_WRN("ACK received for unknown cmd_id %u", cmd_id);
        return -ENOENT;
    }

    gw_command_t cmd = s_tracked[slot].cmd;
    s_tracked[slot].active = false;

    k_mutex_unlock(&s_lock);

    // Stop timeout timer
    k_work_cancel_delayable(&s_tracked[slot].timeout_work);

    LOG_INF("ACK received: cmd_id=%u seq=%d", cmd_id, seq);

    gw_event_t evt = {
        .type  = GW_EVT_ACK,
        .rx_ms = k_uptime_get(),
        .src   = cmd.dst,
        .data.ack = {
            .cmd_id = cmd_id,
            .seq    = seq,
        },
    };
    
    event_ingest_submit(&evt);
    return 0;
}

/* ── ACK by actuator status ─────────────────────────────────── */

int reliability_manager_match_actuator_state(const gw_node_addr_t *src,
                                             bool light_on,
                                             int32_t seq)
{
    if (!src) {
        return -EINVAL;
    }

    k_mutex_lock(&s_lock, K_FOREVER);

    int slot = -1;
    gw_command_t matched_cmd = {0};

    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        if (!s_tracked[i].active) {
            continue;
        }

        if (!gw_addr_equal(&s_tracked[i].cmd.dst, src)) {
            continue;
        }

        if (s_tracked[i].wants_light_on != light_on) {
            continue;
        }

        slot = i;
        matched_cmd = s_tracked[i].cmd;
        s_tracked[i].active = false;
        break;
    }

        k_mutex_unlock(&s_lock);

    if (slot < 0) {
        return -ENOENT;
    }

    k_work_cancel_delayable(&s_tracked[slot].timeout_work);

    LOG_INF("Actuator state matched cmd_id=%u light_on=%d seq=%d",
            matched_cmd.cmd_id, light_on, seq);

        gw_event_t evt = {
            .type  = GW_EVT_ACK,
            .rx_ms = k_uptime_get(),
        .src   = *src,
            .data.ack = {
            .cmd_id = matched_cmd.cmd_id,
            .seq    = seq,
            },
        };

        event_ingest_submit(&evt);
        return 0;
}