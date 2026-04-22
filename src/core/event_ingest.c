#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "event_ingest.h"
#include "gw_store.h"

LOG_MODULE_REGISTER(event_ingest, LOG_LEVEL_INF);

/* ── Listener Table ──────────────────────────────────────── */

#define MAX_LISTENERS 8

typedef struct {
    event_ingest_listener_t fn;
    void *ctx;
} listener_entry_t;

static listener_entry_t listeners[MAX_LISTENERS];
static uint8_t listener_count = 0;

static K_MUTEX_DEFINE(ingest_lock);

/* ── Init ──────────────────────────────────────── */

void event_ingest_init(void)
{
    k_mutex_lock(&ingest_lock, K_FOREVER);
    memset(listeners, 0, sizeof(listeners));
    listener_count = 0;
    k_mutex_unlock(&ingest_lock);

    LOG_INF("Event ingest module initialized (max listeners: %d)", MAX_LISTENERS);
}


/* ── Register Listener ──────────────────────────────────────── */

int event_ingest_register_listener(event_ingest_listener_t fn, void *ctx)
{
    if (!fn) {
        return -EINVAL;
    }

    k_mutex_lock(&ingest_lock, K_FOREVER);

    if (listener_count >= MAX_LISTENERS) {
        k_mutex_unlock(&ingest_lock);
        LOG_ERR("Listener table full, (max %d)", MAX_LISTENERS);
        return -ENOMEM;
    }

    listeners[listener_count].fn = fn;
    listeners[listener_count].ctx = ctx;
    listener_count++;

    k_mutex_unlock(&ingest_lock);

    LOG_INF("Registered event listener (%d/%d)", listener_count, MAX_LISTENERS);
    return 0;
}

/* ── Submit Event ──────────────────────────────────────── */

void event_ingest_submit(const gw_event_t *evt)
{
    if (!evt) {
        LOG_ERR("NULL event pointer submitted");
        return;
    }

    // Local copy -- adapters can submit stack variables without worrying about lifetime
    gw_event_t local = *evt;

    LOG_DBG("Event submitted: Local type %d from %s (listeners: %d)",
        local.type,
        (local.src.transport == GW_TR_BLE_MESH) ? "BLE Mesh" :
        (local.src.transport == GW_TR_THREAD) ? "Thread" :
        (local.src.transport == GW_TR_LORAWAN) ? "LoRaWAN" : "Unknown",
        listener_count);
    
    // Snapshot of listeners under lock
    k_mutex_lock(&ingest_lock, K_FOREVER);
    uint8_t count = listener_count;
    listener_entry_t snapshot[MAX_LISTENERS];
    memcpy(snapshot, listeners, sizeof(listener_entry_t) * count);

    k_mutex_unlock(&ingest_lock);
    
    // call listeners outside of lock to avoid deadlocks or long blocking
    for (uint8_t i = 0; i < count; i++) {
        if (snapshot[i].fn) {
            snapshot[i].fn(&local, snapshot[i].ctx);
        }
    }
}