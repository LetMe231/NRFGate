#ifndef LORAWAN_ADAPTER_H
#define LORAWAN_ADAPTER_H

#include <stdint.h>
#include "gw_model.h"
#include "gw_adapter.h"

/**
 * @brief Initialize the LoRa adapter.
 *        Configures the LoRa modem and starts the RX thread.
 *        Must be called once at boot after the LoRa device is ready.
 */
void lorawan_adapter_setup(void);

/**
 * @brief Get the LoRa adapter instance.
 *
 * @return Pointer to the LoRa adapter instance
 */
gw_adapter_t *lorawan_adapter_get(void);

/**
 * @brief Start the LoRa adapter.
 *
 * @return 0 on success, negative error code on failure
 */
int lorawan_adapter_start(void);

/**
 * @brief Enable or disable the LoRa adapter.
 *        When disabled, incoming LoRa messages will be ignored and outgoing messages will not be sent
 */
void lorawan_adapter_set_enabled(bool enabled);

#endif /* LORAWAN_ADAPTER_H */