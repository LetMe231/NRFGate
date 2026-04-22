#ifndef EVENT_INGEST_H
#define EVENT_INGEST_H

#include "gw_model.h"

/**
 * @brief Initialize the event ingest module.
 *        Must be called once at boot before any adapter submits events.
 */
void event_ingest_init(void);

/**
 * @brief Submit a canonical event from any transport adapter.
 *        Routes the event to node_registry, state_store, and any
 *        registered listeners. Thread-safe — can be called from
 *        BLE, Thread, or LoRa callbacks simultaneously.
 *
 * @param evt Pointer to the event to submit — must not be NULL.
 *            The event is copied internally, caller may free after return.
 */
void event_ingest_submit(const gw_event_t *evt);

/**
 * @brief Listener callback type.
 *        Called synchronously for every submitted event after the
 *        node registry has been updated.
 *
 * @param evt   The submitted event (read-only)
 * @param ctx   Optional context pointer registered with the listener
 */
typedef void (*event_ingest_listener_t)(const gw_event_t *evt, void *ctx);

/**
 * @brief Register a listener to be called on every ingested event.
 *        Listeners are called in registration order.
 *        Must not call event_ingest_submit from within a listener (no reentrance).
 *
 * @param fn    Listener callback — must not be NULL
 * @param ctx   Optional context pointer passed through to fn
 * @return 0 on success, -ENOMEM if listener table is full
 */
int event_ingest_register_listener(event_ingest_listener_t fn, void *ctx);

#endif /* EVENT_INGEST_H */