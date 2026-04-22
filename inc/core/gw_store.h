#ifndef GATEWAY_STORE_H
#define GATEWAY_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include "gw_model.h"

#ifndef GW_STORE_MAX_NODES
#define GW_STORE_MAX_NODES 10
#endif

/**
 * @brief Initialize the node store. Must be called once at boot.
 *        Resets all node records to zero.
 */
void gw_store_init(void);

/**
 * @brief Compare two node addresses (transport-aware).
 *        BLE Mesh: mesh_addr, Thread: ipv6, LoRaWAN: dev_eui_hi/lo.
 *
 * @param a First address
 * @param b Second address
 * @return true if equal, false otherwise
 */
bool gw_addr_eq(const gw_node_addr_t *a, const gw_node_addr_t *b);

/**
 * @brief Look up a node by address.
 *
 * @param addr Node address
 * @return Node index >= 0 if found, -ENOENT otherwise
 */
int gw_store_find_node(const gw_node_addr_t *addr);

/**
 * @brief Apply a canonical event to the store.
 *        Creates the node record if it does not exist yet. Updates
 *        last_seen_ms, rx_meta, stats, and payload-specific fields.
 *        Thread-safe (internal mutex).
 *
 * @param evt Pointer to the event — must not be NULL
 * @return Node index >= 0 on success, negative error code on failure
 */
int gw_store_apply_event(const gw_event_t *evt);

/**
 * @brief Directly set the state of a node without a separate event object.
 *        Internally constructs and applies a GW_EVT_STATE_TRANSITION event.
 *
 * @warning Must not be called while holding store_lock (deadlock risk).
 *
 * @param addr      Address of the target node
 * @param new_state New state to set
 * @param now_ms    Current timestamp in milliseconds
 * @return Node index >= 0 on success, -ENOENT if node not found
 */
int gw_store_set_state(const gw_node_addr_t *addr,
                       gw_state_t new_state,
                       int64_t now_ms);

/**
 * @brief Get a read-only pointer to a node record by index.
 *        No lock is held — caller must treat the pointer as read-only.
 *        For atomic consistency use gw_store_foreach_node instead.
 *
 * @param idx Node index (0 to gw_store_count() - 1)
 * @return Pointer to record, NULL if idx is out of range
 */
bool gw_store_get_node_copy(uint8_t idx, gw_node_record_t *out);

/**
 * @brief Iterate over all known nodes and invoke cb for each one.
 *        The entire iteration runs under the store mutex.
 *
 * @warning Callback must not call gw_store_apply_event or
 *          gw_store_set_state — doing so will deadlock.
 *
 * @param cb  Callback invoked for each known node record
 * @param ctx Optional context pointer passed through to cb unchanged
 */
void gw_store_foreach_node(void (*cb)(const gw_node_record_t *rec, void *ctx),
                           void *ctx);

/**
 * @brief Return the number of currently known nodes.
 *
 * @return Node count (0 to GW_STORE_MAX_NODES)
 */
uint8_t gw_store_count(void);

#endif /* GATEWAY_STORE_H */