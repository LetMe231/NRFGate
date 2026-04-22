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
        .get_adapter = NULL,
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