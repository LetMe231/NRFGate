#ifndef MODEL_HANDLER_H
#define MODEL_HANDLER_H

#include <zephyr/bluetooth/mesh.h>

/**
 * @brief Initialize provisioning structure and UUID.
 * @return Pointer to the provisioning definition.
 */
const struct bt_mesh_prov *model_handler_prov_init(void);

/**
 * @brief Initialize mesh composition and work queues.
 * @return Pointer to the mesh composition.
 */
const struct bt_mesh_comp *model_handler_comp_init(void);

/**
 * @brief Self-provision the gateway on first boot,
 *        or re-apply configuration if already provisioned.
 */
void model_handler_self_provision(void);

/**
 * @brief Open a 30 s provisioning window for BLE Mesh sensor nodes.
 *
 * Pauses the radio scheduler and enables unprovisioned beacon scanning.
 * The scheduler resumes automatically when the window closes.
 */
void model_handler_start_provisioning(void);

#endif /* MODEL_HANDLER_H */