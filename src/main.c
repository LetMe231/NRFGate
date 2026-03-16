/**
 * @file main.c
 * @brief BLE Mesh + Thread Multiprotocol Gateway (nRF54L15-DK)
 *
 * Runs BLE Mesh and OpenThread on a single radio using application-level
 * scheduling via bt_mesh_suspend()/bt_mesh_resume().
 *
 * Boot sequence:
 *   1. Thread network forms (60 s wait)
 *   2. BLE Mesh initializes and self-provisions (5 s settle)
 *   3. Radio scheduler starts (THREAD_PHASE_MS / BLE_PHASE_MS cycling)
 *
 * Buttons:
 *   Button 1 — Factory reset (clear CDB + mesh reset + reboot)
 *   Button 2 — Open provisioning window (30 s)
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include "model_handler.h"
#include "thread_handler.h"

/* ── Radio Scheduler Configuration ───────────────────────────── */

#define THREAD_PHASE_MS  700
#define BLE_PHASE_MS     300

static void mesh_scheduler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(mesh_work, mesh_scheduler);
static bool mesh_active = true;
static bool scheduler_enabled = false;

void mesh_scheduler_pause(void)
{
    scheduler_enabled = false;
    if (!mesh_active) {
        bt_mesh_resume();
        mesh_active = true;
    }
}

void mesh_scheduler_start(void)
{
    scheduler_enabled = true;
    k_work_reschedule(&mesh_work, K_MSEC(1000));
}

static void mesh_scheduler(struct k_work *work)
{
    if (!scheduler_enabled) {
        return;
    }

    if (mesh_active) {
        bt_mesh_suspend();
        mesh_active = false;
        k_work_reschedule(&mesh_work, K_MSEC(THREAD_PHASE_MS));
    } else {
        bt_mesh_resume();
        mesh_active = true;
        k_work_reschedule(&mesh_work, K_MSEC(BLE_PHASE_MS));
    }
}

/* ── BLE Ready Callback ─────────────────────────────────────── */

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    err = bt_mesh_init(model_handler_prov_init(),
                       model_handler_comp_init());
    if (err) {
        printk("Mesh init failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    model_handler_self_provision();

    dk_set_led_on(DK_LED1);
    printk("=== BLE Mesh Gateway ready ===\n");
    printk("Press Button 2 to open provisioning window (30 s)\n");
}

/* ── Button Handler ──────────────────────────────────────────── */

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    /* Button 1: Factory reset */
    if (has_changed & DK_BTN1_MSK) {
        printk("Factory reset...\n");
        bt_mesh_cdb_clear();
        bt_mesh_reset();
        sys_reboot(SYS_REBOOT_WARM);
    }

    /* Button 2: Open provisioning window */
    if ((has_changed & DK_BTN2_MSK) && (button_state & DK_BTN2_MSK)) {
        mesh_scheduler_pause();
        model_handler_start_provisioning();
    }
}

/* ── Main ────────────────────────────────────────────────────── */

int main(void)
{
    int err;

    printk("=== BLE Mesh + Thread Gateway starting ===\n");

    dk_buttons_init(button_handler);
    dk_leds_init();

    /* Phase 1: Start Thread network */
    err = thread_handler_init();
    if (err) {
        printk("Thread handler init failed (err %d)\n", err);
    }

    printk("Waiting 60 s for Thread network to form...\n");
    k_sleep(K_SECONDS(60));

    /* Phase 2: Start BLE Mesh */
    err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }

    /* Let mesh config finish before starting scheduler */
    k_sleep(K_SECONDS(5));

    /* Phase 3: Start radio scheduler */
    printk("Starting radio scheduler (%d ms Thread / %d ms BLE)\n",
           THREAD_PHASE_MS, BLE_PHASE_MS);
    mesh_scheduler_start();

    /* Keep main thread alive */
    while (1) {
        k_sleep(K_SECONDS(60));
    }

    return 0;
}