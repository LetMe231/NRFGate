/**
 * @file ble_mesh_prov.c
 * @brief BLE Mesh provisioning and node configuration support.
 *
 * This module handles:
 * - gateway self-provisioning
 * - provisioning windows for new nodes
 * - automatic node configuration after provisioning
 * - node unprovisioning and reconfiguration
 * - periodic purging of stale CDB entries
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/mesh.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/hwinfo.h>

#include <dk_buttons_and_leds.h>

#include <stdio.h>
#include <string.h>

#include "ble_mesh_prov.h"
#include "ble_mesh_adapter.h"
#include "scheduler.h"
#include "gw_store.h"

LOG_MODULE_REGISTER(ble_mesh_prov, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Constants
 * ───────────────────────────────────────────────────────────── */

/** @brief Unicast address used by the gateway itself. */
#define GATEWAY_ADDR         0x0001

/** @brief Network index used by the mesh. */
#define NET_IDX              0x0000

/** @brief Application key index used by the mesh. */
#define APP_IDX              0x0000

/** @brief Group address used for node status publications. */
#define GROUP_ADDR           0xC000

/** @brief First assignable unicast address for provisioned nodes. */
#define FIRST_NODE_ADDR      0x0002

/** @brief Duration of the provisioning window in seconds. */
#define PROV_WINDOW_SECONDS  60

/** @brief Base retry delay for failed provisioning/configuration. */
#define RETRY_BASE_MS        500

/** @brief Maximum retry backoff shift. */
#define RETRY_MAX_SHIFT      3

/** @brief Maximum retry count before abandoning a node. */
#define NODE_RETRY_CEILING   5

/** @brief Maximum number of nodes allowed in the pending queue. */
#define MAX_PENDING_NODES    5

/** @brief Periodic purge interval for LOST nodes in the CDB. */
#define CDB_PURGE_INTERVAL_S 300

/** @brief UUID prefix byte 0 used to identify supported ESP32 nodes. */
#define ESP32_UUID_PREFIX_0  0x32

/** @brief UUID prefix byte 1 used to identify supported ESP32 nodes. */
#define ESP32_UUID_PREFIX_1  0x10

/* ─────────────────────────────────────────────────────────────
 * Network credentials
 * ───────────────────────────────────────────────────────────── */

/** @brief Static mesh network key. */
static const uint8_t net_key[16] = {
    0xF3, 0x43, 0xBB, 0xCD, 0x11, 0x48, 0x9F, 0x37,
    0x21, 0xF3, 0x23, 0xAC, 0xD0, 0x72, 0x9E, 0xBA,
};

/** @brief Static application key. */
static const uint8_t app_key[16] = {
    0xFC, 0x00, 0x10, 0x2B, 0xC3, 0xBD, 0x0E, 0x62,
    0x19, 0xCC, 0xB1, 0x90, 0xB1, 0x0A, 0x88, 0xA9,
};

/* ─────────────────────────────────────────────────────────────
 * Internal state
 * ───────────────────────────────────────────────────────────── */

/** @brief Whether the provisioning scan window is currently open. */
static bool s_prov_scanning = false;

/** @brief Whether a provisioning procedure is currently in progress. */
static bool s_prov_in_progress = false;

/** @brief Whether an unprovision request is currently pending. */
static bool s_unprov_pending = false;

/** @brief Next candidate unicast address for a new node. */
static uint16_t s_next_node_addr = FIRST_NODE_ADDR;

/** @brief Address of the node currently being configured. */
static uint16_t s_pending_node_addr;

/** @brief Address of the node targeted for unprovisioning. */
static uint16_t s_unprov_target_addr;

/** @brief Queue of pending node addresses awaiting configuration. */
static uint16_t s_pending_addrs[MAX_PENDING_NODES];

/** @brief Current number of entries in @ref s_pending_addrs. */
static int s_pending_count = 0;

/** @brief Retry counter for the currently pending node configuration. */
static int s_node_retry_count = 0;

/** @brief Retry counter for gateway self-configuration. */
static int s_gw_retry_count = 0;

/*
 * Session UUID tracker: remembers UUIDs we've already started provisioning
 * for. This is separate from CDB and pending queue because of a timing race:
 * the ESP32-S3 nodes continue beaconing for a few seconds after provisioning
 * completes, and during that window:
 *   - node_added_cb has set s_prov_in_progress = false
 *   - bt_mesh_cdb_node_get() may not yet return the new node
 *   - the node may not yet be in s_pending_addrs[]
 *
 * Result without this tracker: the same physical node can be provisioned
 * twice at different addresses.
 */
#define SESSION_UUID_MAX MAX_PENDING_NODES

/** @brief UUIDs already accepted during the current provisioning session. */
static uint8_t s_session_uuids[SESSION_UUID_MAX][16];

/** @brief Number of valid UUID entries in @ref s_session_uuids. */
static int s_session_uuid_count = 0;

/** @brief Optional callback used to identify or report lost nodes. */
static ble_mesh_prov_node_lost_fn s_lost_cb = NULL;

/** @brief Protects the pending node queue. */
static K_MUTEX_DEFINE(s_pending_mutex);

/* ─────────────────────────────────────────────────────────────
 * Work queue
 * ───────────────────────────────────────────────────────────── */

/** @brief Dedicated configuration workqueue stack size. */
#define CFG_WQ_STACK_SIZE 4096

/** @brief Stack for the provisioning/configuration workqueue. */
static K_THREAD_STACK_DEFINE(s_cfg_wq_stack, CFG_WQ_STACK_SIZE);

/** @brief Dedicated workqueue for provisioning/configuration operations. */
static struct k_work_q s_cfg_wq;

/** @brief Delayed work item for gateway self-configuration. */
static struct k_work_delayable s_gw_configure_work;

/** @brief Delayed work item for node configuration. */
static struct k_work_delayable s_node_configure_work;

/** @brief Delayed work item for provisioning timeout handling. */
static struct k_work_delayable s_prov_timeout_work;

/** @brief Delayed work item for deferred provisioning start. */
static struct k_work_delayable s_prov_start_work;

/** @brief Delayed work item for node unprovisioning. */
static struct k_work_delayable s_unprovision_work;

/** @brief Delayed work item for periodic CDB purging. */
static struct k_work_delayable s_cdb_purge_work;

/* ─────────────────────────────────────────────────────────────
 * Forward declarations
 * ───────────────────────────────────────────────────────────── */

static bool session_uuid_seen(const uint8_t uuid[16]);
static void session_uuid_remember(const uint8_t uuid[16]);
static void session_uuid_clear(void);

static void schedule_gw_retry(void);
static void schedule_node_retry(void);
static void advance_to_next_pending(void);
static void abandon_failed_node(uint16_t addr);
static void init_next_node_addr(void);

static void gw_configure_handler(struct k_work *work);
static void node_configure_handler(struct k_work *work);
static void prov_timeout_handler(struct k_work *work);
static void prov_start_handler(struct k_work *work);
static void unprovision_handler(struct k_work *work);
static void cdb_purge_handler(struct k_work *work);

static void unprovisioned_beacon_cb(uint8_t uuid[16],
                                    bt_mesh_prov_oob_info_t oob_info,
                                    uint32_t *uri_hash);
static void node_added_cb(uint16_t net_idx,
                          uint8_t uuid[16],
                          uint16_t addr,
                          uint8_t num_elem);
static void prov_link_close_cb(bt_mesh_prov_bearer_t bearer);
static void prov_complete_cb(uint16_t net_idx, uint16_t addr);

static uint8_t purge_lost_cb(struct bt_mesh_cdb_node *node, void *user_data);
static uint8_t cdb_uuid_match_cb(struct bt_mesh_cdb_node *node, void *user_data);
static bool    cdb_has_uuid(const uint8_t uuid[16]);
/* ─────────────────────────────────────────────────────────────
 * CDB UUID lookup helpers
 * ───────────────────────────────────────────────────────────── */

/** @brief Context for cdb_uuid_match_cb. */
struct cdb_uuid_match_ctx {
    const uint8_t *target;
    bool           found;
};

/**
 * @brief Iterator callback matching CDB nodes by UUID.
 */
static uint8_t cdb_uuid_match_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    struct cdb_uuid_match_ctx *ctx = user_data;

    if (memcmp(node->uuid, ctx->target, 16) == 0) {
        ctx->found = true;
        return BT_MESH_CDB_ITER_STOP;
    }

    return BT_MESH_CDB_ITER_CONTINUE;
}

/**
 * @brief Check whether a UUID is already registered in the CDB.
 *
 * Uses bt_mesh_cdb_node_foreach which iterates all actual CDB entries,
 * regardless of their address values. Safer than iterating address ranges.
 */
static bool cdb_has_uuid(const uint8_t uuid[16])
{
    struct cdb_uuid_match_ctx ctx = {
        .target = uuid,
        .found  = false,
    };

    bt_mesh_cdb_node_foreach(cdb_uuid_match_cb, &ctx);
    return ctx.found;
}
/* ─────────────────────────────────────────────────────────────
 * Session UUID helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Check whether a UUID was already accepted in the current session.
 *
 * @param uuid UUID to test.
 *
 * @return true if already seen, false otherwise.
 */
static bool session_uuid_seen(const uint8_t uuid[16])
{
    for (int i = 0; i < s_session_uuid_count; i++) {
        if (memcmp(s_session_uuids[i], uuid, 16) == 0) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Remember a UUID for the current provisioning session.
 *
 * @param uuid UUID to remember.
 */
static void session_uuid_remember(const uint8_t uuid[16])
{
    if (s_session_uuid_count < SESSION_UUID_MAX) {
        memcpy(s_session_uuids[s_session_uuid_count++], uuid, 16);
    }
}

/**
 * @brief Clear all remembered session UUIDs.
 */
static void session_uuid_clear(void)
{
    s_session_uuid_count = 0;
}

/* ─────────────────────────────────────────────────────────────
 * Gateway self-configuration
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Schedule a retry for gateway self-configuration.
 */
static void schedule_gw_retry(void)
{
    int shift = MIN(s_gw_retry_count, RETRY_MAX_SHIFT);
    uint32_t delay_ms = RETRY_BASE_MS << shift;

    LOG_WRN("Gateway config retry #%d scheduled in %u ms",
            s_gw_retry_count + 1, delay_ms);

    s_gw_retry_count++;
    k_work_schedule_for_queue(&s_cfg_wq, &s_gw_configure_work, K_MSEC(delay_ms));
}

/**
 * @brief Work handler that configures the gateway's own mesh models.
 *
 * @param work Work item.
 */
static void gw_configure_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int err;
    uint8_t status;

    LOG_INF("Configuring gateway (addr 0x%04X)", GATEWAY_ADDR);

    err = bt_mesh_app_key_add(APP_IDX, NET_IDX, app_key);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to add app key: %d", err);
        schedule_gw_retry();
        return;
    }

    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX,
                                       GATEWAY_ADDR,
                                       GATEWAY_ADDR,
                                       APP_IDX,
                                       BT_MESH_MODEL_ID_GEN_ONOFF_CLI,
                                       &status);
    if (err && err != -EALREADY) {
        LOG_ERR("OnOff CLI bind failed: %d (status 0x%02X)", err, status);
        schedule_gw_retry();
        return;
    }

    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX,
                                       GATEWAY_ADDR,
                                       GATEWAY_ADDR,
                                       APP_IDX,
                                       SENSOR_CLI_MODEL_ID,
                                       &status);
    if (err) {
        LOG_ERR("Sensor CLI bind failed: %d", err);
        schedule_gw_retry();
        return;
    }

    err = bt_mesh_cfg_cli_mod_sub_add(NET_IDX,
                                      GATEWAY_ADDR,
                                      GATEWAY_ADDR,
                                      GROUP_ADDR,
                                      SENSOR_CLI_MODEL_ID,
                                      &status);
    if (err || status) {
        LOG_ERR("Failed to add subscription: %d (status 0x%02X)", err, status);
        schedule_gw_retry();
        return;
    }

    LOG_INF("Gateway configured - bound to AppKey, subscribed to 0x%04X",
            GROUP_ADDR);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }
}

/* ─────────────────────────────────────────────────────────────
 * Pending node queue helpers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Advance the pending queue to the next node, if any.
 */
static void advance_to_next_pending(void)
{
    k_mutex_lock(&s_pending_mutex, K_FOREVER);

    s_pending_count--;
    if (s_pending_count > 0) {
        memmove(&s_pending_addrs[0],
                &s_pending_addrs[1],
                s_pending_count * sizeof(uint16_t));

        s_pending_node_addr = s_pending_addrs[0];
        s_node_retry_count = 0;
        k_mutex_unlock(&s_pending_mutex);

        /*
         * 4 s settle time to match node_added_cb. Starting configuration too
         * early caused the ESP32-S3 node to still be booting its mesh stack.
         */
        k_work_schedule_for_queue(&s_cfg_wq,
                                  &s_node_configure_work,
                                  K_SECONDS(4));
    } else {
        k_mutex_unlock(&s_pending_mutex);

        if (!s_prov_scanning) {
            scheduler_start();
        }
    }
}

/**
 * @brief Abandon a node that repeatedly failed configuration.
 *
 * @param addr Node unicast address.
 */
static void abandon_failed_node(uint16_t addr)
{
    LOG_ERR("Node 0x%04X failed to configure after %d retries — abandoning",
            addr, NODE_RETRY_CEILING);

    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_del(node, true);
        if (IS_ENABLED(CONFIG_SETTINGS)) {
            settings_save();
        }
    }

    /* Clear stale RPL entry so the address can be reused cleanly later. */
    char rpl_key[24];
    snprintf(rpl_key, sizeof(rpl_key), "bt/mesh/RPL/%x", addr);
    settings_delete(rpl_key);

    if (s_lost_cb) {
        s_lost_cb(addr);
    }

    s_prov_scanning = false;
    dk_set_led_off(DK_LED3);

    scheduler_start();
    advance_to_next_pending();
}

/**
 * @brief Schedule a retry for the current node configuration.
 */
static void schedule_node_retry(void)
{
    if (s_node_retry_count >= NODE_RETRY_CEILING) {
        abandon_failed_node(s_pending_node_addr);
        return;
    }

    int shift = MIN(s_node_retry_count, RETRY_MAX_SHIFT);
    uint32_t delay_ms = RETRY_BASE_MS << shift;

    LOG_WRN("Node config retry #%d in %u ms",
            s_node_retry_count + 1, delay_ms);

    s_node_retry_count++;
    scheduler_start();
    k_work_schedule_for_queue(&s_cfg_wq,
                              &s_node_configure_work,
                              K_MSEC(delay_ms));
}

/* ─────────────────────────────────────────────────────────────
 * Node configuration
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Work handler that configures a newly provisioned node.
 *
 * @param work Work item.
 */
static void node_configure_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int err;
    uint8_t status;
    uint16_t addr = s_pending_node_addr;

    scheduler_pause();
    LOG_INF("Configuring node 0x%04X", addr);

    /*
     * RPL handling for a freshly provisioned node:
     * delete only the stale RPL entry for this address, not the whole RPL.
     */
    char rpl_key[24];
    snprintf(rpl_key, sizeof(rpl_key), "bt/mesh/RPL/%x", addr);
    settings_delete(rpl_key);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }

    k_msleep(200);

    /* Add AppKey. */
    err = bt_mesh_cfg_cli_app_key_add(NET_IDX,
                                      addr,
                                      NET_IDX,
                                      APP_IDX,
                                      app_key,
                                      &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("AppKey add timeout — retrying");
        schedule_node_retry();
        return;
    } else if (err && err != -EALREADY) {
        LOG_ERR("Failed to add AppKey: %d (status 0x%02X)", err, status);
        schedule_node_retry();
        return;
    }

    /* Bind Sensor Server. */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX,
                                       addr,
                                       addr,
                                       APP_IDX,
                                       BT_MESH_MODEL_ID_SENSOR_SRV,
                                       &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("Sensor Srv bind timeout — retrying");
        schedule_node_retry();
        return;
    } else if (err) {
        LOG_ERR("Failed to bind model: %d (status 0x%02X)", err, status);
        schedule_node_retry();
        return;
    }

    /* Bind Generic OnOff Server. */
    err = bt_mesh_cfg_cli_mod_app_bind(NET_IDX,
                                       addr,
                                       addr,
                                       APP_IDX,
                                       BT_MESH_MODEL_ID_GEN_ONOFF_SRV,
                                       &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("OnOff Server bind failed: %d (non-fatal)", err);
    }

    /* Set publication to the shared group address. */
    struct bt_mesh_cfg_cli_mod_pub pub = {
        .addr      = GROUP_ADDR,
        .app_idx   = APP_IDX,
        .ttl       = 5,
        .period    = 0,
        .transmit  = BT_MESH_TRANSMIT(2, 20),
        .cred_flag = false,
    };

    err = bt_mesh_cfg_cli_mod_pub_set(NET_IDX,
                                      addr,
                                      addr,
                                      BT_MESH_MODEL_ID_SENSOR_SRV,
                                      &pub,
                                      &status);
    if (err == -ETIMEDOUT) {
        LOG_WRN("  Pub set timed out — assuming success");
    } else if (err || status) {
        LOG_ERR("  Pub set failed: err=%d status=0x%02X", err, status);
        schedule_node_retry();
        return;
    }

    LOG_INF("=== Node 0x%04X fully configured ===", addr);

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }

    dk_set_led_off(DK_LED3);

    advance_to_next_pending();
}

/* ─────────────────────────────────────────────────────────────
 * Address management
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the next free node address from the CDB.
 */
static void init_next_node_addr(void)
{
    uint16_t addr = bt_mesh_cdb_free_addr_get(1);
    if (addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("No free unicast addresses available in CDB");
        s_next_node_addr = BT_MESH_ADDR_UNASSIGNED;
        return;
    }

    s_next_node_addr = addr;
    LOG_INF("Next node address initialized to 0x%04X", s_next_node_addr);
}

/* ─────────────────────────────────────────────────────────────
 * Provisioning callbacks
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Callback for unprovisioned beacons.
 *
 * Accepts matching ESP32 nodes and starts provisioning for a single node per
 * provisioning window. The ordering of checks is deliberate:
 *   1. Window closed?            → drop immediately
 *   2. Already provisioning?     → drop (single-flight)
 *   3. UUID prefix filter
 *   4. CDB dedup via foreach
 *   5. Session UUID dedup
 *   6. Address availability
 *
 * On accept we close the window before starting provision_adv so that any
 * concurrent beacon bursts can no longer re-enter this path.
 */
static void unprovisioned_beacon_cb(uint8_t uuid[16],
                                    bt_mesh_prov_oob_info_t oob_info,
                                    uint32_t *uri_hash)
{
    ARG_UNUSED(oob_info);
    ARG_UNUSED(uri_hash);

    /* 1. Window closed — drop immediately. */
    if (!s_prov_scanning) {
        return;
    }

    /* 2. Single-flight: one provisioning at a time. */
    if (s_prov_in_progress) {
        return;
    }

    /* 3. UUID prefix filter. */
    if (uuid[0] != ESP32_UUID_PREFIX_0 || uuid[1] != ESP32_UUID_PREFIX_1) {
        LOG_DBG("Ignoring beacon with non-matching UUID prefix");
        return;
    }

    /* 4. CDB dedup via foreach — iterates actual CDB entries, not an
     *    assumed contiguous address range. This is the primary defense
     *    against re-provisioning an already known node. */
    if (cdb_has_uuid(uuid)) {
        LOG_INF("Ignoring beacon — UUID 0x%02X%02X%02X%02X already in CDB",
                uuid[0], uuid[1], uuid[2], uuid[3]);
        return;
    }

    /* 5. Session dedup — guards against the short window where a node was
     *    accepted for provisioning but not yet stored in the CDB. */
    if (session_uuid_seen(uuid)) {
        LOG_INF("Ignoring beacon — UUID 0x%02X%02X%02X%02X already in session",
                uuid[0], uuid[1], uuid[2], uuid[3]);
        return;
    }

    /* 6. Free address. */
    if (s_next_node_addr == BT_MESH_ADDR_UNASSIGNED || s_next_node_addr == 0) {
        init_next_node_addr();
    }
    if (s_next_node_addr == BT_MESH_ADDR_UNASSIGNED) {
        LOG_ERR("Cannot provision new node — no free addresses");
        return;
    }

    LOG_INF("Accepting new ESP32 node: UUID 0x%02X%02X%02X%02X for addr 0x%04X",
            uuid[0], uuid[1], uuid[2], uuid[3], s_next_node_addr);

    /* Atomically commit: set flags, close window, cancel timeout,
     * then actually call bt_mesh_provision_adv. Closing the window
     * BEFORE provision_adv is critical: any beacon that arrives while
     * we are inside bt_mesh_provision_adv will hit step 1 and drop. */
    s_prov_in_progress = true;
    session_uuid_remember(uuid);
    s_prov_scanning = false;
    bt_mesh_prov_disable(BT_MESH_PROV_ADV);
    k_work_cancel_delayable(&s_prov_timeout_work);

    int err = bt_mesh_provision_adv(uuid, NET_IDX, s_next_node_addr, 0);
    if (err) {
        LOG_ERR("Provisioning failed to start: %d", err);

        /* Roll back state so a retry is possible. */
        s_prov_in_progress = false;
        if (s_session_uuid_count > 0 &&
            memcmp(s_session_uuids[s_session_uuid_count - 1], uuid, 16) == 0) {
            s_session_uuid_count--;
        }
        /* Note: window stays closed — user re-opens via button. */
        return;
    }

    LOG_INF("Provisioning started for node 0x%04X (window now closed)",
            s_next_node_addr);
}

/**
 * @brief Callback invoked when a node has been provisioned successfully.
 *
 * @param net_idx   Network index.
 * @param uuid      Node UUID.
 * @param addr      Assigned unicast address.
 * @param num_elem  Number of elements on the node.
 */
static void node_added_cb(uint16_t net_idx,
                          uint8_t uuid[16],
                          uint16_t addr,
                          uint8_t num_elem)
{
    ARG_UNUSED(net_idx);
    ARG_UNUSED(uuid);

    LOG_INF("Node provisioned: addr=0x%04X num_elem=%d", addr, num_elem);

    s_prov_in_progress = false;
    s_next_node_addr = addr + num_elem;

    /* Persist the new CDB entry. */
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);
    if (node) {
        bt_mesh_cdb_node_store(node);
    }

    k_mutex_lock(&s_pending_mutex, K_FOREVER);
    if (s_pending_count < MAX_PENDING_NODES) {
        s_pending_addrs[s_pending_count++] = addr;
    } else {
        LOG_ERR("Pending node queue full, cannot add node 0x%04X", addr);
        k_mutex_unlock(&s_pending_mutex);
        return;
    }
    k_mutex_unlock(&s_pending_mutex);

    if (s_pending_count == 1) {
        s_pending_node_addr = addr;
        s_node_retry_count = 0;

        /*
         * 4 s settle time: ESP32-S3 mesh nodes need time after provisioning
         * before they reliably accept configuration messages.
         */
        k_work_schedule_for_queue(&s_cfg_wq,
                                  &s_node_configure_work,
                                  K_SECONDS(4));
    }
}

/**
 * @brief Callback invoked when a provisioning link closes.
 *
 * @param bearer Provisioning bearer.
 */
static void prov_link_close_cb(bt_mesh_prov_bearer_t bearer)
{
    ARG_UNUSED(bearer);

    if (s_prov_in_progress) {
        LOG_WRN("Provisioning link closed unexpectedly");
        s_prov_in_progress = false;
    }
}

/**
 * @brief Callback invoked when gateway self-provisioning completes.
 *
 * @param net_idx Network index.
 * @param addr    Assigned gateway address.
 */
static void prov_complete_cb(uint16_t net_idx, uint16_t addr)
{
    ARG_UNUSED(net_idx);

    LOG_INF("Gateway self-provisioning complete: addr=0x%04X", addr);
    s_gw_retry_count = 0;
    k_work_schedule_for_queue(&s_cfg_wq, &s_gw_configure_work, K_NO_WAIT);
}

/* ─────────────────────────────────────────────────────────────
 * Provisioning window handlers
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Work handler for provisioning timeout.
 *
 * @param work Work item.
 */
static void prov_timeout_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    LOG_WRN("Provisioning timed out");
    s_prov_scanning = false;
    s_prov_in_progress = false;
    bt_mesh_prov_disable(BT_MESH_PROV_ADV);
    dk_set_led_off(DK_LED3);
    scheduler_start();
}

/**
 * @brief Work handler that enables the provisioning bearer.
 *
 * @param work Work item.
 */
static void prov_start_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    bt_mesh_prov_disable(BT_MESH_PROV_ADV);

    int err = bt_mesh_prov_enable(BT_MESH_PROV_ADV);
    if (err && err != -EALREADY) {
        LOG_ERR("Failed to enable provisioning: %d", err);
        s_prov_scanning = false;
        dk_set_led_off(DK_LED3);
        scheduler_start();
        return;
    }

    LOG_INF("Provisioning ADV enabled");
    k_work_schedule_for_queue(&s_cfg_wq,
                              &s_prov_timeout_work,
                              K_SECONDS(PROV_WINDOW_SECONDS));
}

/* ─────────────────────────────────────────────────────────────
 * Unprovisioning
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Work handler that unprovisions a node.
 *
 * @param work Work item.
 */
static void unprovision_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint16_t addr = s_unprov_target_addr;
    LOG_WRN("Unprovisioning node 0x%04X", addr);

    scheduler_pause();

    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(addr);

    if (!node) {
        LOG_ERR("Node 0x%04X not in CDB — no DevKey, remote reset impossible.",
                addr);
        LOG_ERR("The node likely survived a gateway erase. Factory-reset the "
                "ESP32 node 0x%04X physically to remove it from the network.",
                addr);
    } else {
        int err = bt_mesh_cfg_cli_node_reset(NET_IDX, addr, NULL);
        if (err == 0) {
            LOG_INF("Node 0x%04X unprovisioned via Config Client", addr);
        } else if (err == -ETIMEDOUT) {
            LOG_WRN("Reset timed out — node may be offline. "
                    "Removing from CDB anyway.");
        } else {
            LOG_WRN("Reset failed: %d — removing from CDB anyway.", err);
        }

        bt_mesh_cdb_node_del(node, true);
        LOG_INF("Removed 0x%04X from CDB", addr);
    }

    /* Clean only this address' RPL entry. */
    char rpl_key[24];
    snprintf(rpl_key, sizeof(rpl_key), "bt/mesh/RPL/%x", addr);
    settings_delete(rpl_key);

    k_msleep(200);
    init_next_node_addr();

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_save();
    }

    LOG_INF("=== Node 0x%04X unprovisioned ===", addr);
    s_unprov_pending = false;

    if (s_prov_scanning) {
        k_work_schedule_for_queue(&s_cfg_wq, &s_prov_start_work, K_MSEC(500));
    } else {
        scheduler_start();
    }
}

/* ─────────────────────────────────────────────────────────────
 * CDB purge
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Callback used to inspect each CDB node during purge.
 *
 * @param node       Current CDB node.
 * @param user_data  Pointer to removal counter.
 *
 * @return Iterator continuation code.
 */
static uint8_t purge_lost_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    if (node->addr == GATEWAY_ADDR) {
        return BT_MESH_CDB_ITER_CONTINUE;
    }

    if (s_lost_cb && s_lost_cb(node->addr)) {
        int *count = (int *)user_data;
        (*count)++;
        LOG_WRN("Purging LOST node 0x%04X from CDB", node->addr);
        bt_mesh_cdb_node_del(node, true);
    }

    return BT_MESH_CDB_ITER_CONTINUE;
}

/**
 * @brief Work handler for periodic CDB purge.
 *
 * @param work Work item.
 */
static void cdb_purge_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    ble_mesh_prov_purge_lost_nodes();
    k_work_schedule_for_queue(&s_cfg_wq,
                              &s_cdb_purge_work,
                              K_SECONDS(CDB_PURGE_INTERVAL_S));
}

/* ─────────────────────────────────────────────────────────────
 * Public API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Set the callback used to identify or report lost nodes.
 *
 * @param fn Callback function.
 */
void ble_mesh_set_lost_cb(ble_mesh_prov_node_lost_fn fn)
{
    s_lost_cb = fn;
}

/**
 * @brief Purge LOST nodes from the mesh CDB.
 */
void ble_mesh_prov_purge_lost_nodes(void)
{
    int removed = 0;

    bt_mesh_cdb_node_foreach(purge_lost_cb, &removed);

    if (removed > 0) {
        LOG_INF("Purged %d LOST nodes from CDB", removed);
        if (IS_ENABLED(CONFIG_SETTINGS)) {
            settings_save();
        }
    } else {
        LOG_INF("CDB purge: no stale nodes found");
    }
}

/**
 * @brief Schedule unprovisioning of a node.
 *
 * @param mesh_addr Node unicast address.
 */
void ble_mesh_prov_unprovision_node(uint16_t mesh_addr)
{
    if (s_unprov_pending) {
        LOG_ERR("Unprovision already in progress, ignoring 0x%04X", mesh_addr);
        return;
    }

    s_unprov_target_addr = mesh_addr;
    s_unprov_pending = true;
    k_work_schedule_for_queue(&s_cfg_wq, &s_unprovision_work, K_MSEC(100));
}

/**
 * @brief Schedule reconfiguration of an already known node.
 *
 * @param mesh_addr Node unicast address.
 */
void ble_mesh_prov_reconfigure_node(uint16_t mesh_addr)
{
    struct bt_mesh_cdb_node *node = bt_mesh_cdb_node_get(mesh_addr);
    if (!node) {
        LOG_ERR("Cannot reconfigure unknown node 0x%04X", mesh_addr);
        return;
    }

    atomic_clear_bit(node->flags, BT_MESH_CDB_NODE_CONFIGURED);
    s_pending_node_addr = mesh_addr;
    s_node_retry_count = 0;

    k_work_schedule_for_queue(&s_cfg_wq,
                              &s_node_configure_work,
                              K_MSEC(500));

    LOG_INF("Scheduled reconfiguration for node 0x%04X", mesh_addr);
}

/**
 * @brief Open a provisioning window for new nodes.
 */
void ble_mesh_prov_start_window(void)
{
    if (s_prov_scanning) {
        LOG_INF("Provisioning window already open");
        return;
    }

    if (!bt_mesh_is_provisioned()) {
        LOG_WRN("Gateway not provisioned yet");
        return;
    }

    scheduler_pause();
    s_prov_scanning = true;
    session_uuid_clear();
    dk_set_led_on(DK_LED3);

    LOG_INF("Provisioning window opened for %d seconds", PROV_WINDOW_SECONDS);

    k_work_schedule_for_queue(&s_cfg_wq, &s_prov_start_work, K_NO_WAIT);
}

/**
 * @brief Perform self-provisioning of the gateway if needed.
 */
void ble_mesh_prov_self_provision(void)
{
    uint8_t uuid[16];
    int err = hwinfo_get_device_id(uuid, sizeof(uuid));
    if (err < 0) {
        memset(uuid, 0xFF, sizeof(uuid));
    }

    if (bt_mesh_is_provisioned()) {
        LOG_INF("Already provisioned — re-running gateway config");
        s_gw_retry_count = 0;
        k_work_schedule_for_queue(&s_cfg_wq, &s_gw_configure_work, K_NO_WAIT);
        init_next_node_addr();
        k_work_schedule_for_queue(&s_cfg_wq, &s_cdb_purge_work, K_SECONDS(60));
        return;
    }

    err = bt_mesh_cdb_create(net_key);
    if (err) {
        LOG_ERR("Failed to create CDB: %d", err);
        return;
    }

    err = bt_mesh_provision(net_key, NET_IDX, 0, 0, GATEWAY_ADDR, uuid);
    if (err) {
        LOG_ERR("Self-provisioning failed: %d", err);
        return;
    }

    LOG_INF("Self-provisioning succeeded with UUID %02X%02X%02X%02X",
            uuid[0], uuid[1], uuid[2], uuid[3]);

    init_next_node_addr();
}

/**
 * @brief Fully reset the local mesh state and reboot.
 */
void ble_mesh_prov_full_reset(void)
{
    LOG_WRN("=== FULL MESH RESET ===");

    bt_mesh_cdb_node_foreach((void *)bt_mesh_cdb_node_del, NULL);

    for (uint16_t addr = 1; addr <= 0x20; addr++) {
        char key[32];
        snprintf(key, sizeof(key), "bt/mesh/cdb/Node/%x", addr);
        settings_delete(key);
    }

    bt_mesh_reset();
    settings_delete("bt/mesh/cdb/Net");
    settings_delete("bt/mesh/cdb/Subnet/0");

    LOG_WRN("Reset complete, rebooting...");
    k_msleep(200);
    NVIC_SystemReset();
}

/**
 * @brief Initialize the BLE Mesh provisioning subsystem.
 */
void ble_mesh_prov_init(void)
{
    k_work_queue_init(&s_cfg_wq);
    k_work_queue_start(&s_cfg_wq,
                       s_cfg_wq_stack,
                       K_THREAD_STACK_SIZEOF(s_cfg_wq_stack),
                       K_PRIO_PREEMPT(1),
                       NULL);

    k_work_init_delayable(&s_gw_configure_work,  gw_configure_handler);
    k_work_init_delayable(&s_node_configure_work, node_configure_handler);
    k_work_init_delayable(&s_prov_timeout_work,   prov_timeout_handler);
    k_work_init_delayable(&s_prov_start_work,     prov_start_handler);
    k_work_init_delayable(&s_unprovision_work,    unprovision_handler);
    k_work_init_delayable(&s_cdb_purge_work,      cdb_purge_handler);

    ble_mesh_adapter_register_prov_cb(prov_complete_cb,
                                      node_added_cb,
                                      prov_link_close_cb,
                                      unprovisioned_beacon_cb);

    LOG_INF("BLE Mesh provisioning module initialized");
}

/**
 * @brief Set the callback used to identify or report lost nodes.
 *
 * @param fn Callback function.
 */
void ble_mesh_prov_set_lost_cb(ble_mesh_prov_node_lost_fn fn)
{
    s_lost_cb = fn;
}

/**
 * @brief Report whether provisioning or node configuration is currently busy.
 *
 * Used by the BLE Mesh adapter to suppress normal sensor processing during the
 * critical provisioning/configuration phase.
 *
 * @return true if provisioning flow is busy, false otherwise.
 */
bool ble_mesh_prov_is_busy(void)
{
    return s_prov_scanning || s_prov_in_progress || (s_pending_count > 0);
}

/* ── Status accessors ──────────────────────────────────────── */

static uint8_t cdb_count_cb(struct bt_mesh_cdb_node *node, void *user_data)
{
    ARG_UNUSED(node);
    uint16_t *count = (uint16_t *)user_data;
    (*count)++;
    return BT_MESH_CDB_ITER_CONTINUE;
}

uint16_t ble_mesh_prov_provisioned_count(void)
{
    uint16_t count = 0;
    bt_mesh_cdb_node_foreach(cdb_count_cb, &count);
    return count;
}
