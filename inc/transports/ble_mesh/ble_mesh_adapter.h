#ifndef BLE_MESH_ADAPTER_H
#define BLE_MESH_ADAPTER_H

#include <stdint.h>
#include <zephyr/bluetooth/mesh.h>

#include "gw_adapter.h"

/* Shared BLE Mesh model ID used by adapter and provisioning code */
#define SENSOR_CLI_MODEL_ID 0x1102

/**
 * @brief BLE Mesh adapter setup for Zephyr's Bluetooth Mesh implementation.
 */

void ble_mesh_adapter_setup(void);

/**
 * @brief Get the BLE Mesh adapter instance.
 * @return Pointer to the BLE Mesh adapter instance
 */
gw_adapter_t *ble_mesh_adapter_get(void);

/**
 * @brief Initialize BLE Mesh adapter internals and return mesh composition.
 *        Must be called before bt_mesh_init().
 *
 * @return Pointer to mesh composition passed to bt_mesh_init()
 */
const struct bt_mesh_comp *ble_mesh_adapter_comp_init(void);

/**
 * @brief Get the mesh provisioning struct.
 *        UUID is derived from hardware device ID.
 *
 * @return Pointer to bt_mesh_prov, passed to bt_mesh_init()
 */
const struct bt_mesh_prov *ble_mesh_adapter_prov_get(void);

/**
 * @brief Register provisioning callbacks from ble_mesh_prov.
 *        Must be called before bt_mesh_init().
 *        The adapter forwards these callbacks unchanged to the bt_mesh_prov struct.
 *
 * @param complete    Called when gateway self-provisioning completes
 * @param node_added  Called when a new node has been provisioned
 * @param link_close  Called when a provisioning link closes unexpectedly
 * @param beacon      Called when an unprovisioned beacon is received
 */
void ble_mesh_adapter_register_prov_cb(
    void (*complete)(uint16_t net_idx, uint16_t addr),
    void (*node_added)(uint16_t net_idx, uint8_t uuid[16],
                       uint16_t addr, uint8_t num_elem),
    void (*link_close)(bt_mesh_prov_bearer_t bearer),
    void (*beacon)(uint8_t uuid[16],
                   bt_mesh_prov_oob_info_t oob_info,
                   uint32_t *uri_hash)
);

#endif /* BLE_MESH_ADAPTER_H */