/**
 * @file main.c
 * @brief BLE Mesh + Thread Multiprotocol Gateway (nRF54L15-DK)
 *
 * Runs BLE Mesh and OpenThread on a single radio using application-level
 * scheduling via bt_mesh_suspend()/bt_mesh_resume().
 *
 * Boot sequence:
 *   1. Thread network forms      (60 s wait)
 *   2. BLE Mesh initialises      (bt_ready callback)
 *   3. Settle period             (5 s)
 *   4. BLE NUS advertising starts
 *   5. Radio scheduler starts    (sched_thread_ms / sched_ble_ms cycling)
 *
 * Buttons:
 *   Button 3 — Factory reset (clear CDB + mesh reset + reboot)
 *   Button 2 — Open provisioning window (30 s)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include "main.h"
#include "gw_model.h"
#include "gw_store.h"
#include "event_ingest.h"
#include "semantic_handler.h"
#include "command_router.h"
#include "reliability_manager.h"
#include "ble_mesh_adapter.h"
#include "ble_mesh_prov.h"
#include "thread_adapter.h"
#include "lorawan_adapter.h"
#include "ble_nus.h"
#include "nus_handler.h"
#include "scheduler.h"
#include "rule_engine.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

//* ── Semantic Tick Timer ───────────────────────────────────── */

#define SEMANTIC_TICK_MS 1000

static void semantic_tick_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_semantic_tick_work, semantic_tick_handler);

static void semantic_tick_handler(struct k_work *work)
{
    semantic_handler_tick();
    k_work_schedule(&s_semantic_tick_work, K_MSEC(SEMANTIC_TICK_MS));
}

/* ── Button Handler ────────────────────────────────────────── */

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    if ((has_changed & DK_BTN4_MSK) && (button_state & DK_BTN4_MSK)) {
        LOG_INF("Button 4 — opening provisioning window");
        ble_mesh_prov_start_window();
    }
}

/* ── BLE Mesh LOST Callback ────────────────────────────────── */

static bool node_is_lost(uint16_t mesh_addr)
{
    gw_node_addr_t addr = {
        .transport = GW_TR_BLE_MESH,
        .mesh_addr = mesh_addr,
    };

    int idx = gw_store_find_node(&addr);
    if (idx < 0) {
        return false;
    }

    gw_node_record_t rec;
    if (!gw_store_get_node_copy((uint8_t)idx, &rec)) {
        return false;
    }
    return rec.state == GW_STATE_LOST;
}

static void store_listener(const gw_event_t *evt, void *ctx)
{
    ARG_UNUSED(ctx);
    gw_store_apply_event(evt);
}

int main(void)
{
    LOG_INF("=== Gateway Boot ===");
    int err;

    // 1. Core Layer
    gw_store_init();
    event_ingest_init();

    // 2. Register Listener
    event_ingest_register_listener(store_listener, NULL);
    semantic_handler_init();
    command_router_init();
    reliability_manager_init();

    // 3. Buttons and LEDs
    err = dk_buttons_init(button_handler);
     if (err) {
        LOG_ERR("Buttons init failed: %d", err);
        return err;
    }
    err = dk_leds_init();
    if (err) {
        LOG_ERR("LED init failed: %d", err);
        return err;
    }
    dk_set_led_off(DK_LED1);
    dk_set_led_off(DK_LED2);
    dk_set_led_off(DK_LED3);
    dk_set_led_off(DK_LED4);

    // 4. Bluetooth
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed: %d", err);
        return err;
    }
    LOG_INF("Bluetooth enabled");

    // 5. Register NUS
    err = ble_nus_register();
    if (err) {
        LOG_ERR("ble_nus_register failed: %d", err);
        return err;
    }

    // 6. Load settings
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        err = settings_load();
        if (err) {
            LOG_ERR("Failed to load settings: %d", err);
        } else {
            LOG_INF("Settings loaded");
        }
    }

    // 7. BLE Mesh
    ble_mesh_adapter_setup();
    ble_mesh_prov_init();
    ble_mesh_prov_set_lost_cb(node_is_lost);

    const struct bt_mesh_comp *comp = ble_mesh_adapter_comp_init();
    const struct bt_mesh_prov *prov = ble_mesh_adapter_prov_get();

    err = bt_mesh_init(prov, comp);
    if (err) {
        LOG_ERR("Mesh initialization failed: %d", err);
        return err;
    }
    LOG_INF("Mesh initialized");

    ble_mesh_prov_self_provision();

    // 8. NUS Handler + Advertising
    nus_handler_init();

    // thread_adapter_setup();
    // err = thread_adapter_start();
    // if (err) {
    //     LOG_ERR("Thread adapter init failed: %d", err);
    //     return err;
    // }

    err = ble_nus_advertise();
    if (err) {
        LOG_ERR("ble_nus_advertise failed: %d", err);
        /* non-fatal */
    }
    // 9. Thread/CoAP

    // // 10. LoRaWAN
    // lorawan_adapter_setup();
    // err = lorawan_adapter_start();
    // if (err) {
    //     LOG_ERR("LoRaWAN adapter init failed: %d", err);
    //     return err;
    // }

    // 11. Start semantic tick
    k_work_schedule(&s_semantic_tick_work, K_MSEC(SEMANTIC_TICK_MS));
    rule_engine_init();

    LOG_INF("=== Gateway Initialization Complete ===");

    //scheduler_start();

    while (1) {
        k_sleep(K_SECONDS(10));
        LOG_DBG("Nodes: %u", gw_store_count());
    }
}