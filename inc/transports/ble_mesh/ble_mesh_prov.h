/**
 * @file ble_mesh_prov.h
 * @brief BLE Mesh provisioning and node configuration support.
 */

#ifndef BLE_MESH_PROV_H_
#define BLE_MESH_PROV_H_

#include <zephyr/bluetooth/mesh.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Callback used to report or identify lost mesh nodes.
 *
 * The exact meaning depends on where it is used:
 * - during purge it can be used as a predicate
 * - during failure handling it can be used as a notification
 *
 * @param mesh_addr Node unicast address.
 *
 * @return true if the node should be treated as lost, false otherwise
 */
typedef bool (*ble_mesh_prov_node_lost_fn)(uint16_t mesh_addr);

/**
 * @brief Initialize the BLE Mesh provisioning subsystem.
 */
void ble_mesh_prov_init(void);

/**
 * @brief Start gateway self-provisioning or re-run gateway configuration.
 */
void ble_mesh_prov_self_provision(void);

/**
 * @brief Open a provisioning window for new nodes.
 */
void ble_mesh_prov_start_window(void);

/**
 * @brief Schedule unprovisioning of a node.
 *
 * @param mesh_addr Node unicast address.
 */
void ble_mesh_prov_unprovision_node(uint16_t mesh_addr);

/**
 * @brief Schedule reconfiguration of an already known node.
 *
 * @param mesh_addr Node unicast address.
 */
void ble_mesh_prov_reconfigure_node(uint16_t mesh_addr);

/**
 * @brief Purge LOST nodes from the mesh CDB.
 */
void ble_mesh_prov_purge_lost_nodes(void);

/**
 * @brief Perform a full local mesh reset and reboot.
 */
void ble_mesh_prov_full_reset(void);

/**
 * @brief Set the callback used to identify or report lost nodes.
 *
 * @param fn Callback function.
 */
void ble_mesh_prov_set_lost_cb(ble_mesh_prov_node_lost_fn fn);

/**
 * @brief Legacy alias for setting the lost-node callback.
 *
 * @param fn Callback function.
 */
void ble_mesh_set_lost_cb(ble_mesh_prov_node_lost_fn fn);

/**
 * @brief Report whether provisioning/configuration flow is currently busy.
 *
 * @return true if busy, false otherwise
 */
bool ble_mesh_prov_is_busy(void);

/**
 * @brief Count provisioned nodes in the BLE Mesh CDB.
 *
 * Includes the gateway itself if self-provisioned. Subtract 1 if you only
 * want client devices.
 *
 * @return Number of nodes in the CDB.
 */
uint16_t ble_mesh_prov_provisioned_count(void);

#endif /* BLE_MESH_PROV_H_ */