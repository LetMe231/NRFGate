#ifndef COMMAND_ROUTER_H
#define COMMAND_ROUTER_H

#include "gw_model.h"

/**
 * @brief Initialize the command router.
 *        Must be called after all adapters are initialized.
 */
void command_router_init(void);

/**
 * @brief Route and send a command to the correct transport adapter.
 *        Resolves GW_CMD_LIGHT_TOGGLE by looking up current state
 *        in the node registry before dispatching.
 *        Thread-safe — can be called from rule engine or dashboard handler.
 *
 * @param cmd Command to send — must not be NULL
 * @return 0 on success, negative error code on failure
 */
int command_router_send(const gw_command_t *cmd);

/**
 * @brief Convenience function — build and send a command in one call.
 *        Generates a unique cmd_id automatically.
 *
 * @param dst   Destination node address
 * @param type  Command type
 * @return 0 on success, negative error code on failure
 */
int command_router_send_to(const gw_node_addr_t *dst, gw_cmd_type_t type);

#endif /* COMMAND_ROUTER_H */