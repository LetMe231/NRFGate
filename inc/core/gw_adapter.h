#ifndef GW_ADAPTER_H
#define GW_ADAPTER_H

#include <stdint.h>
#include <errno.h>
#include "event_ingest.h"

typedef struct gw_adapter gw_adapter_t;

typedef void (*gw_adapter_emit_fn)(const gw_event_t *evt, void *user);

/**
 * @brief Describes the synchronicity of a transport's send_cmd path.
 *
 * Used by higher-level components (command router, reliability manager,
 * future telemetry) to decide whether a successful send_cmd() return value
 * means "on the air" or "queued for later delivery".
 *
 * Examples:
 *   - GW_SEND_SYNC   : BLE Mesh — bt_mesh_onoff_cli_set() is synchronous,
 *                      success means the message was published.
 *   - GW_SEND_QUEUED : Thread / LoRa — send_cmd() puts the request in an
 *                      internal queue, actual transmission happens later
 *                      in a worker thread.
 */
typedef enum {
    GW_SEND_SYNC = 0,
    GW_SEND_QUEUED,
} gw_send_mode_t;

typedef struct {
    int     (*send_cmd)(gw_adapter_t *self, const gw_command_t *cmd);
    int64_t (*last_rx_ms)(gw_adapter_t *self);

    /**
     * @brief Optional. Returns the synchronicity of send_cmd() for this adapter.
     *
     * If NULL, callers must conservatively assume GW_SEND_QUEUED — i.e. they
     * cannot rely on send_cmd() success implying immediate transmission.
     */
    gw_send_mode_t (*send_mode)(gw_adapter_t *self);
} gw_adapter_api_t;

struct gw_adapter {
    const gw_adapter_api_t *api;
    gw_adapter_emit_fn emit;
    void *emit_user;
};

static inline int gw_adapter_send_cmd(gw_adapter_t *self, const gw_command_t *cmd)
{
    if (self && self->api && self->api->send_cmd) {
        return self->api->send_cmd(self, cmd);
    }
    return -EINVAL;
}

static inline int64_t gw_adapter_last_rx_ms(gw_adapter_t *self)
{
    if (self && self->api && self->api->last_rx_ms) {
        return self->api->last_rx_ms(self);
    }
    return 0;
}

/**
 * @brief Query the send synchronicity of an adapter.
 *
 * Adapters that do not implement send_mode are conservatively reported as
 * GW_SEND_QUEUED: this is the safe default — it prevents callers from
 * incorrectly assuming immediate delivery.
 */
static inline gw_send_mode_t gw_adapter_send_mode(gw_adapter_t *self)
{
    if (self && self->api && self->api->send_mode) {
        return self->api->send_mode(self);
    }
    return GW_SEND_QUEUED;
}

#endif /* GW_ADAPTER_H */
