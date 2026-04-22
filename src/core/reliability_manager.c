#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "reliability_manager.h"
#include "event_ingest.h"

LOG_MODULE_REGISTER(reliability_manager, LOG_LEVEL_INF);

/* ── Tracked Command Entry ─────────────────────────────────────── */

typedef struct {
    bool         active;
    gw_command_t cmd;
    struct k_work_delayable timeout_work;
} tracked_cmd_t;

/* ── Internal State ─────────────────────────────────────── */

static tracked_cmd_t s_tracked[RELIABILITY_MAX_TRACKED];
static K_MUTEX_DEFINE(s_lock);

/* ── Forward Declaration ─────────────────────────────────────── */

static void timeout_handler(struct k_work *work);

/* ── Init ─────────────────────────────────────── */

void reliability_manager_init(void)
{
    k_mutex_lock(&s_lock, K_FOREVER);
    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        s_tracked[i].active = false;
        k_work_init_delayable(&s_tracked[i].timeout_work, timeout_handler);
    }
    k_mutex_unlock(&s_lock);

    LOG_INF("Reliability manager initialized (max tracked: %d)", RELIABILITY_MAX_TRACKED);
}

/* ── Track Command ─────────────────────────────────────── */

int reliability_manager_track(const gw_command_t *cmd, uint32_t timeout_ms)
{
    if (!cmd) {
        return -EINVAL;
    }

    k_mutex_lock(&s_lock, K_FOREVER);

    // Find an empty slot
    int slot = -1;
    for (int i = 0; i < RELIABILITY_MAX_TRACKED; i++) {
        if (!s_tracked[i].active) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        k_mutex_unlock(&s_lock);
        LOG_ERR("Tracking table full, cannot track cmd_id %u", cmd->cmd_id);
        return -ENOMEM;
    }

    // Store command and start timeout
    s_tracked[slot].active = true;
    s_tracked[slot].cmd = *cmd;
    k_mutex_unlock(&s_lock);
    
    // start timeout work outside of lock to avoid deadlocks if timeout_handler also tries to lock
    k_work_schedule(&s_tracked[slot].timeout_work, K_MSEC(timeout_ms));

    LOG_INF("Tracking command cmd_id=%u (timeout %u ms)", cmd->cmd_id, timeout_ms);
    return 0;
}

/* ── Timeout Handler ─────────────────────────────────────── */

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

    LOG_WRN("Timeout: cmd_id=%u type=%d transport=%d", cmd.cmd_id, cmd.type, cmd.dst.transport);

    gw_event_t evt = {
        .type   = GW_EVT_TIMEOUT,
        .rx_ms  = k_uptime_get(),
        .src    = cmd.dst,
        .data.timeout = {
            .cmd_id = cmd.cmd_id,
        },
    };

    event_ingest_submit(&evt);
}

/* ── Acknowledge Command ─────────────────────────────────────── */

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
    s_tracked[slot].active = false; // Mark slot as free

    k_mutex_unlock(&s_lock);

    // Stop timeout timer
    k_work_cancel_delayable(&s_tracked[slot].timeout_work);

    LOG_INF("ACK received: cmd_id=%u seq=%d", cmd_id, seq);

    gw_event_t evt = {
        .type   = GW_EVT_ACK,
        .rx_ms  = k_uptime_get(),
        .src    = cmd.dst,
        .data.ack = {
            .cmd_id = cmd_id,
            .seq    = seq,
        },
    };
    
    event_ingest_submit(&evt);
    return 0;
}