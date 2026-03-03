/* model_handler.h - BLE Mesh Gateway Model Handler */

/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef MODEL_HANDLER_H__
#define MODEL_HANDLER_H__

#include <zephyr/bluetooth/mesh.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the provisioning parameters and callbacks.
 *
 * returns the bt_mesh_prov structure with:
 *  - UUID
 * - Output OOB size and action
 * - Provisioning complete callback
 *
 * @return Pointer to bt_mesh_prov structure
 */
const struct bt_mesh_prov *model_handler_prov_init(void);

/**
 * @brief initialize the composition data with the models and elements.
 *
 * returns the bt_mesh_comp structure with:
 *   - Config Server
 *   - Config Client
 *   - Health Server
 *   - Sensor Client
 *
 * @return Pointer to bt_mesh_comp structure
 */
const struct bt_mesh_comp *model_handler_comp_init(void);

/**
 * @brief Self-provision the device if not already provisioned.
 *
 * This function checks if the device is already provisioned. If not, it generates a unique UUID
 * based on the hardware ID and provisions the device using that UUID. This allows the gateway to
 * automatically join the mesh network without requiring manual provisioning.
 */
void model_handler_self_provision(void);

#ifdef __cplusplus
}
#endif

#endif /* MODEL_HANDLER_H__ */