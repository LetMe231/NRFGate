#ifndef MESH_CTRL_H
#define MESH_CTRL_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/mesh.h>

// Model opcode IDs
#define MESH_CTRL_ONOFF_CLI_ID 0x1001

#define BT_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK    BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GEN_ONOFF_SET          BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS       BT_MESH_MODEL_OP_2(0x82, 0x04)

// Public API
/**
 * @brief Initialize the Mesh Control model with a pointer to the registered model.
 * Called from model_handler after bt_mesh_init().
 * @param model Pointer to the Bluetooth Mesh model structure.
 */
void mesh_ctrl_init(const struct bt_mesh_model *model);

/**
 * @brief Called by model_handler's onoff_status_handler
 * updates internal state cache.
 */
void mesh_ctrl_on_status(uint16_t addr, uint8_t state);

/**
 * @brief Send generic on/off set message unacknowledged.
 * @param addr Unicast or group address.
 * @param on New state (true = on, false = off).
 */

void mesh_ctrl_set_onoff(uint16_t addr, bool on);

/**
 * @brief Toggle the on/off state of a node or group.
 * @param addr Unicast or group address.
 */
void mesh_ctrl_toggle(uint16_t addr);

#endif /* MESH_CTRL_H */