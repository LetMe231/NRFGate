#ifndef GW_ADAPTER_H
#define GW_ADAPTER_H

#include <stdint.h>
#include <errno.h>
#include "event_ingest.h"

typedef struct gw_adapter gw_adapter_t;

typedef void (*gw_adapter_emit_fn)(const gw_event_t *evt, void *user);

typedef struct {
    int     (*send_cmd)(gw_adapter_t *self, const gw_command_t *cmd);
    int64_t (*last_rx_ms)(gw_adapter_t *self);
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

#endif /* GW_ADAPTER_H */