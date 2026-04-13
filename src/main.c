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
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>

#include "main.h"
#include "ble_mesh_handler.h"
#include "thread_handler.h"
#include "lora_handler.h"
#include "ble_nus.h"
#include "data_handler.h"
#include "semantic_handler.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);


/* ── Radio Scheduler Configuration ───────────────────────────── */
uint32_t sched_thread_ms    = 300;
uint32_t sched_ble_ms       = 200;
static int64_t   last_scan_ms      = 0;


// duplicate change later
static const uint8_t net_key[16] = {
    0xF3, 0x43, 0xBB, 0xCD, 0x11, 0x48, 0x9F, 0x37,
    0x21, 0xF3, 0x23, 0xAC, 0xD0, 0x72, 0x9E, 0xBA,
};

// Testing
int64_t  g_ble_active_ms    = 0;
int64_t  g_thread_active_ms = 0;
uint32_t g_ble_switches     = 0;
uint32_t g_thread_switches  = 0;
static int64_t s_last_switch_ms = 0;
static atomic_t s_suppress_sched_report = ATOMIC_INIT(0);

#define PRIORITY_CHECK_MS       50
#define SCAN_WINDOW_BLE_MS      500
#define SCAN_WINDOW_THREAD_MS   5000
#define SCAN_INTERVAL_MS        30000
#define MESH_RX_EXTEND_MS       400

#define WAKE_AHEAD_MS           300
#define WAKE_DURATION_MS        200

static void mesh_scheduler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(mesh_work, mesh_scheduler);

static bool mesh_active         = true;
static K_SEM_DEFINE(s_thread_window_sem, 0, 1); /* given when Thread window opens */

int mesh_scheduler_wait_thread_window(uint32_t timeout_ms)
{
    /* Wenn Thread-Fenster schon offen: sofort zurück */
    if (!mesh_active) return 0;
    /* Sonst warten bis Scheduler das Fenster öffnet */
    int r = k_sem_take(&s_thread_window_sem, K_MSEC(timeout_ms));
    return (r == 0) ? 0 : -EAGAIN;
}
static bool scheduler_enabled   = false;
static bool ble_extended = false;
sched_mode_t sched_mode = SCHED_MODE_NORMAL;

// Priority scheduling state (protected by atomics)
static atomic_t  priority_transport = ATOMIC_INIT(SCHED_PRIORITY_NONE);
static int64_t   priority_until_ms  = 0;

/* Forward declaration — ble_nus.h included below bt headers */
extern bool ble_nus_is_ready(void);
extern void ble_nus_send(const char *str);


// testing
void mesh_scheduler_suppress_reports(bool suppress)
{
    atomic_set(&s_suppress_sched_report, suppress ? 1 : 0);
}

static void sched_report_status(const char *mode_str, const char *phase_str)
{
    if (atomic_get(&s_suppress_sched_report)) return;
    if (!ble_nus_is_ready()) return;
    char buf[64];
    snprintf(buf, sizeof(buf),
             "{\"sched\":\"%s\",\"phase\":\"%s\"}\n",
             mode_str, phase_str);
    ble_nus_send(buf);
}


void mesh_scheduler_pause(void)
{
    scheduler_enabled = false;
    k_work_cancel_delayable(&mesh_work);

    if (!mesh_active) {
        int err = bt_mesh_resume();
        if (err) {
            LOG_ERR("Failed to resume mesh: %d", err);
        }
        mesh_active = true;
    }
    LOG_INF("Scheduler paused");
}

void mesh_scheduler_start(void)
{
    scheduler_enabled = true;
    k_work_reschedule(&mesh_work, K_MSEC(1000));
    LOG_INF("Scheduler started (%d ms Thread / %d ms BLE)", sched_thread_ms, sched_ble_ms);
}

void mesh_scheduler_set_timing(uint32_t thread_ms, uint32_t ble_ms)
{
    if (thread_ms >= 50 && thread_ms <= 2000) sched_thread_ms = thread_ms;
    if (ble_ms    >= 50 && ble_ms    <= 2000) sched_ble_ms    = ble_ms;
    LOG_INF("Scheduler timing: %d ms Thread / %d ms BLE",
            sched_thread_ms, sched_ble_ms);
}

void mesh_scheduler_set_mode(sched_mode_t mode)
{
    if (mode == sched_mode) return;
    sched_mode = mode;
    last_scan_ms = 0;   /* trigger scan window immediately on mode switch */

    const char *s = (mode == SCHED_MODE_BLE_ONLY)    ? "ble_only"
                  : (mode == SCHED_MODE_THREAD_ONLY)  ? "thread_only"
                  :                                      "normal";
    LOG_INF("Scheduler mode → %s", s);
    sched_report_status(s, "switching");

    /* Kick scheduler to apply immediately */
    if (scheduler_enabled) k_work_reschedule(&mesh_work, K_NO_WAIT);
}

void mesh_scheduler_request_priority(sched_priority_t transport, uint32_t duration_ms)
{
    if(!scheduler_enabled) return;

    priority_until_ms = k_uptime_get() + duration_ms;
    atomic_set(&priority_transport, (atomic_val_t)transport);

    k_work_reschedule(&mesh_work, K_NO_WAIT);
    LOG_INF("Priority requested: %s for %d ms", transport == SCHED_PRIORITY_BLE ? "BLE" : "Thread", duration_ms);
}

static void mesh_scheduler(struct k_work *work)
{
    if (!scheduler_enabled) return;

    int64_t now_ms = k_uptime_get();
    sched_priority_t prio = (sched_priority_t)atomic_get(&priority_transport);

    /* ── Priority window ────────────────────────────────────────── */
    if (prio != SCHED_PRIORITY_NONE) {
        if (now_ms < priority_until_ms) {
            if (prio == SCHED_PRIORITY_BLE && !mesh_active) {
                bt_mesh_resume(); mesh_active = true;
            } else if (prio == SCHED_PRIORITY_THREAD && mesh_active) {
                bt_mesh_suspend(); mesh_active = false;
                k_sem_give(&s_thread_window_sem);
            }
        k_work_reschedule(&mesh_work, K_MSEC(PRIORITY_CHECK_MS));
        return;
        }
        atomic_set(&priority_transport, (atomic_val_t)SCHED_PRIORITY_NONE);
        /* Report return to base mode */
        const char *base = (sched_mode == SCHED_MODE_BLE_ONLY)   ? "ble_only"
                         : (sched_mode == SCHED_MODE_THREAD_ONLY) ? "thread_only"
                         :                                           "normal";
        sched_report_status(base, sched_mode == SCHED_MODE_THREAD_ONLY ? "thread" : "ble");
    }

    /* ── BLE-only mode ──────────────────────────────────────────── */
    if (sched_mode == SCHED_MODE_BLE_ONLY) {
        if (!mesh_active) { bt_mesh_resume(); mesh_active = true; }

        /* Periodic short Thread scan window */
        if (last_scan_ms == 0 || (now_ms - last_scan_ms) >= SCAN_INTERVAL_MS) {
            last_scan_ms = now_ms;
            bt_mesh_suspend(); mesh_active = false;
            k_sem_give(&s_thread_window_sem);
            sched_report_status("ble_only", "scan_thread");
            k_work_reschedule(&mesh_work, K_MSEC(SCAN_WINDOW_THREAD_MS));
        } else {
            sched_report_status("ble_only", "ble");
            k_work_reschedule(&mesh_work, K_MSEC(sched_ble_ms));
        }
        return;
    }

    /* ── Thread-only mode ───────────────────────────────────────── */
    if (sched_mode == SCHED_MODE_THREAD_ONLY) {
        if (mesh_active) { bt_mesh_suspend(); mesh_active = false;
            k_sem_give(&s_thread_window_sem); }

        /* Periodic short BLE scan window */
        if (last_scan_ms == 0 || (now_ms - last_scan_ms) >= SCAN_INTERVAL_MS) {
            last_scan_ms = now_ms;
            bt_mesh_resume(); mesh_active = true;
            sched_report_status("thread_only", "scan_ble");
            k_work_reschedule(&mesh_work, K_MSEC(SCAN_WINDOW_BLE_MS));
        } else {
            sched_report_status("thread_only", "thread");
            k_work_reschedule(&mesh_work, K_MSEC(sched_thread_ms));
        }
        return;
    }

    /* ── Normal alternating — MIT adaptivem BLE-Fenster ────────── */
    mesh_node_schedule_t schedules[MAX_NODES];
    int sched_count = data_handler_get_mesh_schedules(
        schedules, MAX_NODES);

    bool wake_needed = false;
    int64_t earliest_wake = INT64_MAX;

    for (int i = 0; i < sched_count; i++) {
        /* Prediction ist nur sinnvoll für seltene Sender.
        * Wenn ein Node öfter sendet als ein voller Zyklus dauert,
        * fängt die normale Alternation ihn sowieso auf. */
        uint32_t cycle_2x = (sched_thread_ms + sched_ble_ms) * 10;
        if (schedules[i].estimated_period_ms < cycle_2x) {
            continue;
        }

        int64_t time_until_next = schedules[i].predicted_next_ms - now_ms;

        if (time_until_next <= WAKE_AHEAD_MS &&
            time_until_next > -WAKE_DURATION_MS) {
            wake_needed = true;
            break;
        }

        int64_t wake_at = schedules[i].predicted_next_ms - WAKE_AHEAD_MS;
        if (wake_at < earliest_wake) {
            earliest_wake = wake_at;
        }
    }

    if (wake_needed) {
        /* BLE-Fenster proaktiv öffnen */
        if (!mesh_active) {
            int64_t n = k_uptime_get();
            g_thread_active_ms += n - s_last_switch_ms;
            s_last_switch_ms = n;
            g_ble_switches++;
            bt_mesh_resume();
            mesh_active = true;
            sched_report_status("normal", "ble_scheduled");
        }
        k_work_reschedule(&mesh_work, K_MSEC(WAKE_DURATION_MS));
        return;
    }

    /* Adaptiver Scheduler (reaktiv, wie bisher) */
    int64_t ms_since_mesh_rx = now_ms - g_last_mesh_rx_ms;
    if (mesh_active) {
        if (!ble_extended && g_last_mesh_rx_ms > 0 && ms_since_mesh_rx < MESH_RX_EXTEND_MS) {
            k_work_reschedule(&mesh_work, K_MSEC(sched_ble_ms));
            ble_extended = true;
            sched_report_status("normal", "ble_extended");
            return;
        }
        /* Zu Thread wechseln */
        int64_t n = k_uptime_get();
        g_ble_active_ms += n - s_last_switch_ms;
        s_last_switch_ms = n;
        g_thread_switches++;
        bt_mesh_suspend();
        mesh_active = false;
        k_sem_give(&s_thread_window_sem);
        ble_extended = false;
        /* Nächsten Wake-up schedulen falls bekannt */
        if (earliest_wake < INT64_MAX) {
            int64_t ms_until_wake = earliest_wake - k_uptime_get();
            if (ms_until_wake > 0 && ms_until_wake < 600000) {
                /* Thread-Fenster bis kurz vor nächstem erwarteten TX */
                int64_t thread_window = MIN(ms_until_wake,
                                            (int64_t)sched_thread_ms);
                sched_report_status("normal", "thread");
                k_work_reschedule(&mesh_work, K_MSEC(thread_window));
                return;
            }
        }

        sched_report_status("normal", "thread");
        k_work_reschedule(&mesh_work, K_MSEC(sched_thread_ms));
    } else {
        /* Thread → BLE */
        int64_t n = k_uptime_get();
        g_thread_active_ms += n - s_last_switch_ms;
        s_last_switch_ms = n;
        g_ble_switches++;
        bt_mesh_resume();
        mesh_active = true;
        ble_extended = false;
        sched_report_status("normal", "ble");
        k_work_reschedule(&mesh_work, K_MSEC(sched_ble_ms));
    }
}

/* ── BLE Ready Callback ─────────────────────────────────────── */

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    err = bt_mesh_init(ble_mesh_handler_prov_init(),
                       ble_mesh_handler_comp_init());
    if (err) {
        LOG_ERR("Mesh init failed (err %d)", err);
        return;
    }


    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    ble_mesh_handler_self_provision();
    
    dk_set_led_on(DK_LED2);
    LOG_INF("=== BLE Mesh Gateway ready ===");
    LOG_INF("Press Button 2 to open provisioning window (30 s)");
}

/* ── Button Handler ──────────────────────────────────────────── */

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    /* Button 1: Factory reset */
    if ((has_changed & DK_BTN4_MSK) && (button_state & DK_BTN4_MSK)) {
        LOG_INF("Factory reset triggered");
        bt_mesh_reset();
        bt_mesh_cdb_clear();
        bt_mesh_cdb_create(net_key);
        if(IS_ENABLED(CONFIG_SETTINGS)) {
            settings_save();
        }
        k_sleep(K_MSEC(500));
        sys_reboot(SYS_REBOOT_WARM);
    }

    /* Button 2: Open provisioning window */
    if ((has_changed & DK_BTN3_MSK) && (button_state & DK_BTN3_MSK)) {
        LOG_INF("Starting provisioning...");
        ble_mesh_handler_start_provisioning();
}
}

/* ── Main ────────────────────────────────────────────────────── */

int main(void)
{
    int err;
    LOG_INF("=== BLE Mesh + Thread Gateway starting ===");

    dk_buttons_init(button_handler);
    dk_leds_init();

    // Phase 1: Start Thread network 
    err = lora_handler_init();
    if (err) {
        LOG_ERR("LoRa init failed (err %d)", err);
    }

    err = data_handler_init();
    if (err) {
        LOG_ERR("Data handler init failed (err %d)", err);
    }

    err = thread_handler_init();
    if (err) {
        LOG_ERR("Thread handler init failed (err %d)", err);
    }
    // Phase 2: BLE stack + Mesh
    ble_nus_register();

    bt_addr_le_t addr;
    bt_addr_le_from_str("DE:AD:BE:EF:CA:FE", "random", &addr);
    bt_id_create(&addr, NULL);

    LOG_INF("Waiting 40 s for Thread network to form...");
    k_sleep(K_SECONDS(40));

    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
    }


    // Phase 3: Settle period before starting scheduler
    LOG_INF("Waiting 5 s before starting scheduler...");
    k_sleep(K_SECONDS(5));

    err = ble_nus_advertise();
    if (err) {
        LOG_ERR("NUS advertise failed (err %d)", err);
    }

    LOG_INF("Starting radio scheduler (%d ms Thread / %d ms BLE)", sched_thread_ms, sched_ble_ms);
    mesh_scheduler_start();

    /* Keep main thread alive */
    while (1) {
        k_sleep(K_SECONDS(10));
        semantic_handler_tick();
    }

    return 0;
}