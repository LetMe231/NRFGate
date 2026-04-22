#ifndef TRANSPORT_PORT_H
#define TRANSPORT_PORT_H

#include <stdbool.h>
#include "gw_model.h"
#include "gw_adapter.h"

typedef struct {
    gw_transport_t transport;
    int  (*init)(void);
    gw_adapter_t *(*get_adapter)(void);
    bool (*is_ready)(void);
} gw_transport_port_t;

const gw_transport_port_t *transport_port_get(gw_transport_t tr);

gw_adapter_t *transport_port_get_adapter(gw_transport_t tr);

#endif /* TRANSPORT_PORT_H */