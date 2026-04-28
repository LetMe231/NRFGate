/**
 * @file transport_port.c
 * @brief Transport-port registry mapping gw_transport_t to concrete adapters.
 *
 * The registry is the single source of truth for "which transports the gateway
 * supports" and is the entry point used by the command router and any other
 * higher-level component that needs to dispatch a command without knowing the
 * concrete transport implementation.
 *
 * Each entry exposes:
 *   - init        : optional one-shot startup hook (runs after main bring-up)
 *   - get_adapter : returns the concrete gw_adapter_t* for that transport
 *   - is_ready    : optional readiness probe
 *
 * Direct calls to the per-transport getters (e.g. ble_mesh_adapter_get())
 * are reserved for the early bring-up path in main.c and for tests; all
 * application-level code should resolve adapters via this registry.
 */

#include <zephyr/sys/util.h>

#include "transport_port.h"
#include "ble_mesh_adapter.h"
#include "ble_mesh_prov.h"
#include "thread_adapter.h"
#include "lorawan_adapter.h"

static const gw_transport_port_t s_transport_ports[] = {
    {
        .transport = GW_TR_BLE_MESH,
        .init = NULL,
        .get_adapter = ble_mesh_adapter_get,
        .is_ready = bt_mesh_is_provisioned,
    },
    {
        .transport = GW_TR_THREAD,
        .init = thread_adapter_start,
        .get_adapter = thread_adapter_get,
        .is_ready = NULL,
    },
    {
        .transport = GW_TR_LORAWAN,
        .init = lorawan_adapter_start,
        .get_adapter = lorawan_adapter_get,
        .is_ready = NULL,
    },
};


const gw_transport_port_t *transport_port_get(gw_transport_t tr)
{
    for (size_t i = 0; i < ARRAY_SIZE(s_transport_ports); i++) {
        if (s_transport_ports[i].transport == tr) {
            return &s_transport_ports[i];
        }
    }
    return NULL;
}

gw_adapter_t *transport_port_get_adapter(gw_transport_t tr)
{
    const gw_transport_port_t *port = transport_port_get(tr);

    if (!port || !port->get_adapter) {
        return NULL;
    }

    return port->get_adapter();
}
