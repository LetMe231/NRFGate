#ifndef RELIABILITY_MANAGER_H
#define RELIABILITY_MANAGER_H

#include "gw_model.h"

/**
 * @brief Initialize the reliability manager.
 *        Must be called after command_router_init().
 */
void reliability_manager_init(void);

/**
 * @brief Track a sent command for ACK monitoring.
 *        Starts a timeout timer for the command.
 *        If no ACK arrives before timeout, a GW_EVT_TIMEOUT
 *        event is submitted via event_ingest.
 *
 * @param cmd       Command that was sent — must not be NULL
 * @param timeout_ms Timeout in milliseconds before retry/fail
 * @return 0 on success, -ENOMEM if tracking table is full
 */
int reliability_manager_track(const gw_command_t *cmd, uint32_t timeout_ms);

/**
 * @brief Acknowledge a tracked command.
 *        Stops the timeout timer for this cmd_id.
 *        Submits a GW_EVT_ACK event via event_ingest.
 *
 * @param cmd_id    Command ID to acknowledge
 * @param seq       Sequence number from ACK payload
 * @return 0 on success, -ENOENT if cmd_id not tracked
 */
int reliability_manager_ack(uint32_t cmd_id, int32_t seq);

/**
 * @brief Maximum number of simultaneously tracked commands.
 */
#define RELIABILITY_MAX_TRACKED 8

#endif /* RELIABILITY_MANAGER_H */