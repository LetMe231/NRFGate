#ifndef NUS_HANDLER_H
#define NUS_HANDLER_H

#include "gw_model.h"
#include "ble_nus.h"
#include "nus_handler.h"

/**
 * @brief Initialize the NUS handler.
 *        Registers itself as an event_ingest listener.
 *        Must be called after event_ingest_init() and ble_nus_register().
 */
void nus_handler_init(void);

/**
 * @brief Process an incoming NUS command string.
 *        Called by ble_nus.c when a complete line is received.
 *        Parses JSON and routes to command_router or ble_mesh_prov.
 *
 * @param cmd Null-terminated JSON command string
 * @param len Length of command string
 */
void nus_handler_cmd(const char *cmd, size_t len);

void nus_handler_send_snapshot(void);

int nus_handler_send_sched(const char *mode, const char *phase);

void nus_handler_on_disconnect(void);

#endif /* NUS_HANDLER_H */