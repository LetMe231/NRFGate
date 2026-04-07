#ifndef BLE_MESH_HANDLER_H
#define BLE_MESH_HANDLER_H
 
#include <zephyr/bluetooth/mesh.h>
 
const struct bt_mesh_prov *ble_mesh_handler_prov_init(void);
const struct bt_mesh_comp *ble_mesh_handler_comp_init(void);
void ble_mesh_handler_self_provision(void);
void ble_mesh_handler_start_provisioning(void);
 
/**
 * @brief Unprovision a BLE Mesh node by its unicast address.
 *
 * Sends Config Node Reset to the node (so it returns to unprovisioned
 * state and starts beaconing again), then removes it from the local CDB.
 *
 * Safe to call from any context — the actual mesh operation is
 * dispatched to the config work queue internally.
 *
 * @param mesh_addr  Unicast mesh address of the node to unprovision
 */
void ble_mesh_handler_unprovision_node(uint16_t mesh_addr);
 
void ble_mesh_handler_reconfigure_node(uint16_t mesh_addr);
#endif /* BLE_MESH_HANDLER_H */
 