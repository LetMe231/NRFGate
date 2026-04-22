/**
 * @file scheduler.c
 * @brief Radio time-division scheduler for BLE Mesh + Thread.
 *
 * Implements application-level radio sharing using bt_mesh_suspend() and
 * bt_mesh_resume(). A delayable work item drives the state machine:
 *
 *   [BLE active] ──sched_ble_ms──► [suspend BLE] ──sched_thread_ms──► [resume BLE]
 *
 * Priority requests bypass the normal timing and force the radio into a
 * specific mode for a bounded duration, after which normal scheduling resumes.
 *
 * Thread windows are signalled via a semaphore (s_thread_window_sem) so
 * the CoAP TX thread can block until the radio is available.
 *
 * Predictive BLE wake:
 *   Each time a BLE Mesh packet arrives, the scheduler records the arrival
 *   time and updates an exponentially-smoothed estimate of that node's send
 *   period. Before switching back to Thread, it checks whether any node is
 *   expected to transmit soon. If so, BLE is held open for WAKE_DURATION_MS
 *   to catch the packet. This reduces BLE packet loss without sacrificing
 *   Thread throughput — nodes that send more frequently than one full
 *   BLE+Thread cycle are excluded from prediction since the normal alternation
 *   already catches them reliably.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>

#include "scheduler.h"
#include "nus_handler.h"     /* nus_handler_send_sched() — dashboard status */
#include "transport_port.h"  /* transport_port_get_adapter() */
#include "gw_store.h"        /* gw_store_find_node()  — predictive wake  */
#include "gw_model.h"        /* gw_node_record_t     — predictive wake  */

LOG_MODULE_REGISTER(scheduler, LOG_LEVEL_INF);

/* ── Timing constants ──────────────────────────────────────── */

/** How often to re-check whether a priority window has expired (ms). */
#define PRIORITY_CHECK_MS     50

/** How long to give BLE a scan window in Thread-only mode (ms). */
#define SCAN_WINDOW_BLE_MS    500

/** How long to give Thread a scan window in BLE-only mode (ms). */
#define SCAN_WINDOW_THREAD_MS 5000

/** How often to grant a cross-protocol scan window in single-mode (ms). */
#define SCAN_INTERVAL_MS      30000

/**
 * If a BLE Mesh packet was received within this window, extend the BLE
 * slot to allow the reply / status to arrive before switching to Thread.
 */
#define MESH_RX_EXTEND_MS     400

/* ── Predictive wake constants ─────────────────────────────── */

/**
 * How many milliseconds before an expected BLE Mesh packet to open the
 * BLE window. Accounts for scheduling jitter and node clock drift.
 */
#define WAKE_AHEAD_MS         300

/**
 * How long to keep BLE open when a predictive wake fires.
 * Should be long enough to cover one full node transmit + ACK round-trip.
 */
#define WAKE_DURATION_MS      600

/**
 * Nodes that send more frequently than this threshold are excluded from
 * predictive wake — the normal BLE/Thread alternation catches them anyway.
 * Set to 2× the total cycle length so that every normal cycle covers at
 * least one transmit interval for excluded nodes.
 */
#define MIN_PREDICT_PERIOD_MS 2000

/** Maximum number of BLE Mesh nodes tracked for prediction. */
#define SCHED_MAX_NODES       16

/* ── Predictive wake state ─────────────────────────────────── */

/**
 * Per-node timing state used for predictive BLE wake.
 * Updated on every BLE Mesh RX via scheduler_on_ble_rx().
 */
typedef struct {
    int64_t last_rx_ms;          /**< Uptime at last packet reception       */
    int64_t estimated_period_ms; /**< Smoothed inter-packet period estimate  */
    int64_t predicted_next_ms;   /**< Predicted arrival time of next packet  */
} node_sched_t;

static node_sched_t s_node_sched[SCHED_MAX_NODES];

/* ── Internal scheduler state ──────────────────────────────── */

static bool         s_enabled        = false;
static atomic_t     s_mesh_active    = ATOMIC_INIT(true);  /**< BLE Mesh currently active  */
static bool         s_ble_extended   = false; /**< RX-extend flag             */
static int64_t      s_last_scan_ms   = 0;     /**< Last cross-protocol scan   */
static int64_t      s_last_switch_ms = 0;     /**< Uptime of last radio switch*/
static atomic_t     s_prio_transport = ATOMIC_INIT(SCHED_PRIORITY_NONE);
static int64_t      s_prio_until_ms  = 0;     /**< Priority window end time   */
static sched_mode_t s_mode           = SCHED_MODE_NORMAL;

static uint32_t s_thread_ms = 300; /**< Configured Thread window (ms) */
static uint32_t s_ble_ms    = 200; /**< Configured BLE window (ms)    */

/* ── Dashboard status cache ───────────────────────────── */

static const char *s_last_sched_mode  = NULL;
static const char *s_last_sched_radio = NULL;

/* ── Statistics ────────────────────────────────────────────── */

int64_t  g_ble_active_ms    = 0; /**< Cumulative BLE active time since boot    */
int64_t  g_thread_active_ms = 0; /**< Cumulative Thread active time since boot */
uint32_t g_ble_switches     = 0; /**< Total BLE Mesh radio acquisitions        */
uint32_t g_thread_switches  = 0; /**< Total Thread radio acquisitions          */

/* ── Thread window semaphore ───────────────────────────────── */

/**
 * Signalled once each time the radio switches to Thread mode.
 * scheduler_wait_thread_window() blocks on this semaphore so the CoAP TX
 * thread does not transmit while BLE Mesh holds the radio.
 */
static K_SEM_DEFINE(s_thread_window_sem, 0, 1);

/* ── Work item ─────────────────────────────────────────────── */

static void scheduler_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(s_work, scheduler_work_fn);

/* ── Internal helpers ──────────────────────────────────────── */

/**
 * @brief Send scheduler status to NUS only when it actually changed.
 *
 * Prevents flooding the BLE NUS link with repeated identical scheduler
 * state updates on every scheduler tick.
 *
 * @param mode   Scheduler mode string for dashboard display.
 * @param radio  Current radio owner string for dashboard display.
 */
static void scheduler_report_status(const char *mode, const char *radio)
{
    if (!mode || !radio) {
        return;
    }

    if (s_last_sched_mode != NULL &&
        s_last_sched_radio != NULL &&
        strcmp(s_last_sched_mode, mode) == 0 &&
        strcmp(s_last_sched_radio, radio) == 0) {
        return;
    }

    //nus_handler_send_sched(mode, radio);
    s_last_sched_mode  = mode;
    s_last_sched_radio = radio;
}

/**
 * @brief Switch radio to Thread mode and update statistics.
 *        Signals s_thread_window_sem for waiting CoAP TX thread.
 */
static void switch_to_thread(int64_t now_ms)
{
    g_ble_active_ms  += now_ms - s_last_switch_ms;
    s_last_switch_ms  = now_ms;
    g_thread_switches++;
    bt_mesh_suspend();
    atomic_set(&s_mesh_active, false);
    s_ble_extended = false;
    k_sem_give(&s_thread_window_sem);
    scheduler_report_status("normal", "thread");
}

/**
 * @brief Switch radio to BLE Mesh mode and update statistics.
 */
static void switch_to_ble(int64_t now_ms)
{
    g_thread_active_ms += now_ms - s_last_switch_ms;
    s_last_switch_ms    = now_ms;
    g_ble_switches++;
    bt_mesh_resume();
    atomic_set(&s_mesh_active, true);
    s_ble_extended = false;
    scheduler_report_status("normal", "ble");
}

/* ── Predictive wake ───────────────────────────────────────── */

/**
 * @brief Check whether a predictive BLE wake is warranted right now,
 *        and optionally return the next predicted wake time.
 *
 * Iterates over all tracked nodes. A node is eligible for prediction only
 * if its estimated send period exceeds MIN_PREDICT_PERIOD_MS — frequent
 * senders are caught by the normal alternation without prediction.
 *
 * @param now_ms     Current uptime in milliseconds.
 * @param wake_at_ms Set to the earliest future wake time if no immediate
 *                   wake is needed but one is scheduled. Unchanged if the
 *                   caller should wake now or if no prediction is available.
 * @return true if BLE should be opened immediately to catch an expected
 *         packet; false otherwise.
 */
static bool predictive_wake_needed(int64_t now_ms, int64_t *wake_at_ms)
{
    int64_t earliest_wake = INT64_MAX;
    bool    wake_now      = false;

    for (int i = 0; i < SCHED_MAX_NODES; i++) {
        const node_sched_t *n = &s_node_sched[i];

        /* Skip nodes we have not yet observed. */
        if (n->estimated_period_ms == 0) continue;

        /*
         * Skip frequent senders — their period fits within one normal
         * BLE+Thread cycle so prediction would keep BLE open permanently.
         */
        if (n->estimated_period_ms <
            (int64_t)(s_thread_ms + s_ble_ms) * 2) continue;

        int64_t until_next = n->predicted_next_ms - now_ms;

        if (until_next <= WAKE_AHEAD_MS &&
            until_next > -WAKE_DURATION_MS) {
            /*
             * Packet expected imminently — or arrived slightly late
             * (negative until_next within WAKE_DURATION_MS). Open BLE now.
             */
            wake_now = true;
            break;
        }

        /* Record the earliest future wake point across all eligible nodes. */
        int64_t wa = n->predicted_next_ms - WAKE_AHEAD_MS;
        if (wa > now_ms && wa < earliest_wake) {
            earliest_wake = wa;
        }
    }

    if (!wake_now && earliest_wake != INT64_MAX) {
        *wake_at_ms = earliest_wake;
    }
    return wake_now;
}

/* ── Public API ────────────────────────────────────────────── */

void scheduler_init(void)
{
    memset(s_node_sched, 0, sizeof(s_node_sched));
    s_last_sched_mode = NULL;
    s_last_sched_radio = NULL;
}

void scheduler_pause(void)
{
    s_enabled = false;
    k_work_cancel_delayable(&s_work);

    /* Ensure BLE Mesh is active when paused (e.g. during provisioning). */
    if (!atomic_get(&s_mesh_active)) {
        bt_mesh_resume();
        atomic_set(&s_mesh_active, true);
    }
    LOG_INF("Scheduler paused");
}

void scheduler_start(void)
{
    s_enabled = true;
    /* 1 s initial delay to let BLE Mesh settle after boot. */
    k_work_reschedule(&s_work, K_MSEC(1000));
    LOG_INF("Scheduler started (%u ms Thread / %u ms BLE)",
            s_thread_ms, s_ble_ms);
}

void scheduler_set_timing(uint32_t thread_ms, uint32_t ble_ms)
{
    if (thread_ms >= 50 && thread_ms <= 2000) s_thread_ms = thread_ms;
    if (ble_ms    >= 50 && ble_ms    <= 2000) s_ble_ms    = ble_ms;
}

void scheduler_set_mode(sched_mode_t mode)
{
    if (mode == s_mode) return;
    s_mode         = mode;
    s_last_scan_ms = 0; /* Force scan window immediately on mode change. */
    if (s_enabled) k_work_reschedule(&s_work, K_NO_WAIT);
}

void scheduler_request_priority(sched_priority_t transport,
                                 uint32_t duration_ms)
{
    if (!s_enabled) return;
    s_prio_until_ms = k_uptime_get() + duration_ms;
    atomic_set(&s_prio_transport, (atomic_val_t)transport);
    k_work_reschedule(&s_work, K_NO_WAIT);
}

int scheduler_wait_thread_window(uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        if (!atomic_get(&s_mesh_active)) {
            return 0;
        }

        int64_t now = k_uptime_get();
        if (now >= deadline) {
            return -EAGAIN;
        }

        uint32_t remaining = (uint32_t)(deadline - now);

        if (k_sem_take(&s_thread_window_sem, K_MSEC(remaining)) != 0) {
            return -EAGAIN;
        }

        /* Wake received — loop and re-check actual current state. */
    }
}

/**
 * @brief Notify the scheduler of a received BLE Mesh sensor packet.
 *
 * Uses the node's unicast mesh address (minus 1) as a direct array index.
 * Valid node addresses are 0x0002–0x0015 (SCHED_MAX_NODES = 16).
 * The gateway address 0x0001 is ignored.
 *
 * @param mesh_addr Unicast mesh address of the sending node.
 */
void scheduler_on_ble_rx(uint16_t mesh_addr)
{
    if (mesh_addr <= 1 || mesh_addr > SCHED_MAX_NODES + 1) return;

    uint8_t node_idx = (uint8_t)(mesh_addr - 2); /* 0x0002 → idx 0 */
    if (node_idx >= SCHED_MAX_NODES) return;

    int64_t      now = k_uptime_get();
    node_sched_t *n  = &s_node_sched[node_idx];

    if (n->last_rx_ms > 0) {
        int64_t measured = now - n->last_rx_ms;

        /*
         * Exponential smoothing with 3:1 weight towards the historical
         * estimate. Stable under jitter but adapts to genuine period
         * changes (e.g. firmware update altering the publish interval).
         */
        if (n->estimated_period_ms == 0) {
            n->estimated_period_ms = measured;
        } else {
            n->estimated_period_ms =
                (n->estimated_period_ms * 3 + measured) / 4;
        }
    }

    n->last_rx_ms        = now;
    n->predicted_next_ms = now + n->estimated_period_ms;
}

bool scheduler_is_ble_active(void)
{
    return atomic_get(&s_mesh_active) != 0;
}

int64_t scheduler_ms_since_switch(void)
{
    return k_uptime_get() - s_last_switch_ms;
}

/* ── Scheduler state machine ───────────────────────────────── */

static void scheduler_work_fn(struct k_work *work)
{
    ARG_UNUSED(work);
    if (!s_enabled) return;

    int64_t now_ms = k_uptime_get();
    sched_priority_t prio =
        (sched_priority_t)atomic_get(&s_prio_transport);

    /* ── Priority window ───────────────────────────────────── */
    if (prio != SCHED_PRIORITY_NONE) {
        if (now_ms < s_prio_until_ms) {
            /* Hold the requested transport — switch if not already there. */
            if (prio == SCHED_PRIORITY_BLE && !atomic_get(&s_mesh_active)) {
                bt_mesh_resume();
                atomic_set(&s_mesh_active, true);
                scheduler_report_status("normal", "ble");
            } else if (prio == SCHED_PRIORITY_THREAD && atomic_get(&s_mesh_active)) {
                bt_mesh_suspend();
                atomic_set(&s_mesh_active, false);
                k_sem_give(&s_thread_window_sem);
                scheduler_report_status("normal", "thread");
            }
            k_work_reschedule(&s_work, K_MSEC(PRIORITY_CHECK_MS));
            return;
        }
        /* Priority window expired — fall through to normal scheduling. */
        atomic_set(&s_prio_transport, (atomic_val_t)SCHED_PRIORITY_NONE);
    }

    /* ── BLE-only mode ─────────────────────────────────────── */
    if (s_mode == SCHED_MODE_BLE_ONLY) {
        if (!atomic_get(&s_mesh_active)) {
            bt_mesh_resume();
            atomic_set(&s_mesh_active, true);
        }

        if (s_last_scan_ms == 0 ||
            (now_ms - s_last_scan_ms) >= SCAN_INTERVAL_MS) {
            /* Grant a brief Thread scan window every SCAN_INTERVAL_MS. */
            s_last_scan_ms = now_ms;
            bt_mesh_suspend();
            atomic_set(&s_mesh_active, false);
            k_sem_give(&s_thread_window_sem);
            scheduler_report_status("ble_only", "thread");
            k_work_reschedule(&s_work, K_MSEC(SCAN_WINDOW_THREAD_MS));
        } else {
            scheduler_report_status("ble_only", "ble");
            k_work_reschedule(&s_work, K_MSEC(s_ble_ms));
        }
        return;
    }

    /* ── Thread-only mode ──────────────────────────────────── */
    if (s_mode == SCHED_MODE_THREAD_ONLY) {
        if (atomic_get(&s_mesh_active)) {
            bt_mesh_suspend();
            atomic_set(&s_mesh_active, false);
            k_sem_give(&s_thread_window_sem);
        }

        if (s_last_scan_ms == 0 ||
            (now_ms - s_last_scan_ms) >= SCAN_INTERVAL_MS) {
            /* Grant a brief BLE scan window every SCAN_INTERVAL_MS. */
            s_last_scan_ms = now_ms;
            bt_mesh_resume();
            atomic_set(&s_mesh_active, true);
            scheduler_report_status("thread_only", "ble");
            k_work_reschedule(&s_work, K_MSEC(SCAN_WINDOW_BLE_MS));
        } else {
            scheduler_report_status("thread_only", "thread");
            k_work_reschedule(&s_work, K_MSEC(s_thread_ms));
        }
        return;
    }

    /* ── Normal alternating mode ───────────────────────────── */

    if (atomic_get(&s_mesh_active)) {
        /*
         * RX-extend: if a BLE Mesh packet arrived very recently, stay in
         * BLE a little longer to allow the ACK or OnOff Status to arrive
         * before handing the radio to Thread. Only extends once per cycle
         * to prevent BLE from monopolising the radio when traffic is high.
         */
        gw_adapter_t *adapter = transport_port_get_adapter(GW_TR_BLE_MESH);
        if(!adapter) {
            LOG_ERR("No adapter for BLE Mesh transport");
            return;
        }
        int64_t last_rx_ms = gw_adapter_last_rx_ms(adapter);
        int64_t ms_since_rx = now_ms - last_rx_ms;
        if (!s_ble_extended && last_rx_ms > 0 &&
            ms_since_rx < MESH_RX_EXTEND_MS) {
            s_ble_extended = true;
            k_work_reschedule(&s_work, K_MSEC(s_ble_ms));
            return;
        }

        /* BLE → Thread */
        switch_to_thread(now_ms);
        k_work_reschedule(&s_work, K_MSEC(s_thread_ms));

    } else {
        /*
         * Thread → BLE: run predictive wake logic before switching.
         *
         * If a BLE Mesh node is expected to transmit soon, open BLE now
         * and hold it for WAKE_DURATION_MS so the packet is not missed.
         *
         * If a wake is predicted but not yet imminent, stay in Thread and
         * schedule the work item to fire just before the predicted arrival
         * (capped at s_thread_ms to preserve the normal cadence as a
         * fallback in case the prediction is wrong).
         *
         * If no prediction is available, switch to BLE on the normal cadence.
         */
        int64_t wake_at = 0;

        if (predictive_wake_needed(now_ms, &wake_at)) {
            /* Packet expected imminently — switch to BLE now. */
            switch_to_ble(now_ms);
            k_work_reschedule(&s_work, K_MSEC(WAKE_DURATION_MS));

        } else if (wake_at > 0) {
            /*
             * Next predicted packet is in the future.
             * Stay in Thread and wake just in time.
             */
            int64_t wait_ms = wake_at - now_ms;
            k_work_reschedule(&s_work,
                              K_MSEC(MIN((uint32_t)wait_ms, s_thread_ms)));

        } else {
            /* No prediction available — switch on normal cadence. */
            switch_to_ble(now_ms);
            k_work_reschedule(&s_work, K_MSEC(s_ble_ms));
        }
    }
}

bool scheduler_is_paused(void)
{
    return !s_enabled;
}

int scheduler_wait_ble_window(uint32_t timeout_ms)
{
    int64_t deadline = k_uptime_get() + timeout_ms;

    while (1) {
        if (atomic_get(&s_mesh_active)) {
            return 0;
        }

        int64_t now = k_uptime_get();
        if (now >= deadline) {
            return -EAGAIN;
        }

        k_msleep(10);
    }
}