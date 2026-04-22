/**
 * @file command_router.c
 * @brief Generic command router for cross-transport command dispatch.
 *
 * This module accepts generic gateway commands (`gw_command_t`) and forwards
 * them to the correct transport adapter. Commands are handled asynchronously:
 * they are queued first and then sent by a dedicated worker thread.
 *
 * Responsibilities:
 * - resolve toggle commands based on the last known actuator state
 * - look up the correct transport port / adapter
 * - request BLE scheduling priority before BLE Mesh sends
 * - perform transport send with limited retry for BLE Mesh
 * - emit optimistic actuator-state events for light ON/OFF commands
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "scheduler.h"
#include "command_router.h"
#include "gw_store.h"
#include "gw_adapter.h"
#include "transport_port.h"
#include "event_ingest.h"

LOG_MODULE_REGISTER(command_router, LOG_LEVEL_INF);

/* ─────────────────────────────────────────────────────────────
 * Configuration
 * ───────────────────────────────────────────────────────────── */

/** @brief Depth of the command queue. */
#define CMD_ROUTER_Q_LEN       8

/** @brief Worker thread stack size in bytes. */
#define CMD_ROUTER_STACK_SIZE  4096

/** @brief Worker thread priority. */
#define CMD_ROUTER_PRIO        6

/** @brief BLE priority reservation duration in milliseconds. */
#define CMD_BLE_PRIORITY_MS    1000

/* ─────────────────────────────────────────────────────────────
 * Internal state
 * ───────────────────────────────────────────────────────────── */

/** @brief Monotonic command ID counter for commands created locally. */
static atomic_t s_cmd_id = ATOMIC_INIT(1);

/** @brief Command queue for asynchronous dispatch. */
K_MSGQ_DEFINE(s_cmd_q, sizeof(gw_command_t), CMD_ROUTER_Q_LEN, 4);

/** @brief Worker thread stack. */
K_THREAD_STACK_DEFINE(s_cmd_stack, CMD_ROUTER_STACK_SIZE);

/** @brief Worker thread object. */
static struct k_thread s_cmd_thread;

/* ─────────────────────────────────────────────────────────────
 * Forward declarations
 * ───────────────────────────────────────────────────────────── */

static void command_router_thread_fn(void *a, void *b, void *c);
static int command_router_send_now(const gw_command_t *cmd);
static gw_cmd_type_t resolve_toggle(const gw_node_addr_t *dst);

static int command_send_via_adapter(gw_adapter_t *adapter,
                                    const gw_command_t *cmd);
static bool command_is_pending_light_cmd(const gw_command_t *cmd);
static void command_emit_pending(const gw_command_t *cmd);

/* ─────────────────────────────────────────────────────────────
 * Initialization
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the command router and start the worker thread.
 */
void command_router_init(void)
{
    k_thread_create(&s_cmd_thread,
                    s_cmd_stack,
                    CMD_ROUTER_STACK_SIZE,
                    command_router_thread_fn,
                    NULL,
                    NULL,
                    NULL,
                    CMD_ROUTER_PRIO,
                    0,
                    K_NO_WAIT);
    k_thread_name_set(&s_cmd_thread, "cmd_router");
    
    LOG_INF("command router thread=%p", &s_cmd_thread);
    LOG_INF("Command router initialized");
}

/* ─────────────────────────────────────────────────────────────
 * Toggle resolution
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Resolve a toggle command to ON or OFF using the last known actuator state.
 *
 * If the destination node is unknown or no actuator state is available yet,
 * the function falls back to `GW_CMD_LIGHT_ON`.
 *
 * @param dst Destination node address.
 *
 * @return Resolved light command type.
 */
static gw_cmd_type_t resolve_toggle(const gw_node_addr_t *dst)
{
    int idx = gw_store_find_node(dst);
    if (idx < 0) {
        LOG_WRN("TOGGLE: node not found — defaulting to ON");
        return GW_CMD_LIGHT_ON;
    }

    gw_node_record_t rec;
    if (!gw_store_get_node_copy((uint8_t)idx, &rec)) {
        LOG_WRN("TOGGLE: failed to read node state — defaulting to ON");
        return GW_CMD_LIGHT_ON;
    }

    if (!rec.has_last_actuator_state) {
        LOG_WRN("TOGGLE: no actuator state — defaulting to ON");
        return GW_CMD_LIGHT_ON;
    }

    return rec.last_light_on ? GW_CMD_LIGHT_OFF : GW_CMD_LIGHT_ON;
}

/* ─────────────────────────────────────────────────────────────
 * Optimistic local state updates
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Check whether a command may emit an optimistic local actuator update.
 *
 * Optimistic updates are only safe for transports whose send path is
 * synchronous enough that a successful return means the command was
 * actually handed to the transport immediately.
 *
 * Thread is excluded here because thread_send_cmd() currently only queues
 * into the CoAP TX queue; actual send + ACK happens later in the TX thread.
 * Emitting an actuator-state event at queue time would desynchronize the
 * store from the real device state on failed delivery.
 *
 * @param cmd Command to inspect.
 *
 * @return true if an optimistic state update is allowed, false otherwise.
 */
static bool command_should_emit_optimistic_state(const gw_command_t *cmd)
{
    if (!cmd) {
        return false;
    }

    if (cmd->type != GW_CMD_LIGHT_ON && cmd->type != GW_CMD_LIGHT_OFF) {
        return false;
    }

    return cmd->dst.transport == GW_TR_BLE_MESH;
}

/**
 * @brief Emit an optimistic actuator-state event for a successfully sent command.
 *
 * @param cmd Command that completed successfully on a transport that supports
 *            optimistic state updates.
 */
static void command_emit_optimistic_state(const gw_command_t *cmd)
{
    if (!command_should_emit_optimistic_state(cmd)) {
        return;
    }

    gw_event_t evt = {
        .type  = GW_EVT_ACTUATOR_STATE,
        .rx_ms = k_uptime_get(),
        .src   = cmd->dst,
        .data.actuator_state.light_on = (cmd->type == GW_CMD_LIGHT_ON),
    };

    event_ingest_submit(&evt);
}

static bool command_is_pending_light_cmd(const gw_command_t *cmd)
{
    if (!cmd) {
        return false;
    }

    return cmd->type == GW_CMD_LIGHT_ON || cmd->type == GW_CMD_LIGHT_OFF;
}

static void command_emit_pending(const gw_command_t *cmd)
{
    if (!command_is_pending_light_cmd(cmd)) {
        return;
    }

    gw_event_t evt = {
        .type  = GW_EVT_CMD_PENDING,
        .rx_ms = k_uptime_get(),
        .src   = cmd->dst,
        .data.cmd_pending = {
            .cmd_id   = cmd->cmd_id,
            .cmd_type = cmd->type,
            .light_on = (cmd->type == GW_CMD_LIGHT_ON),
        },
    };

    event_ingest_submit(&evt);
}

/* ─────────────────────────────────────────────────────────────
 * Transport send helper
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Send a command through a resolved adapter.
 *
 * For BLE Mesh commands, the function requests BLE scheduler priority and waits
 * briefly before sending. A single retry is performed if the adapter returns
 * `-EAGAIN`.
 *
 * @param adapter Target adapter.
 * @param cmd     Command to send.
 *
 * @retval 0 on success
 * @retval -EINVAL if inputs are invalid
 * @retval <0 adapter-specific error code on failure
 */
static int command_send_via_adapter(gw_adapter_t *adapter,
                                    const gw_command_t *cmd)
{
    if (!adapter || !cmd) {
        return -EINVAL;
    }

    if (cmd->dst.transport == GW_TR_BLE_MESH) {
        //scheduler_request_priority(SCHED_PRIORITY_BLE, CMD_BLE_PRIORITY_MS);

        // int wait_err = scheduler_wait_ble_window(600);
        // if (wait_err) {
        //     LOG_WRN("BLE window not available for cmd_id=%u", cmd->cmd_id);
        //     return wait_err;
        // }
    }

    int err = gw_adapter_send_cmd(adapter, cmd);
    return err;
}

/* ─────────────────────────────────────────────────────────────
 * Internal immediate send path
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Resolve and send a command immediately from the worker context.
 *
 * This function:
 * - resolves toggle commands
 * - resolves the destination transport port and adapter
 * - dispatches the command via the selected adapter
 *
 * @param cmd Command to process.
 *
 * @retval 0 on success
 * @retval -EINVAL if the command is invalid
 * @retval -ENOTSUP if no suitable transport port exists
 * @retval -ENODEV if no adapter is available for the selected transport
 * @retval <0 adapter-specific error code on send failure
 */
static int command_router_send_now(const gw_command_t *cmd)
{
    if (!cmd) {
        return -EINVAL;
    }

    gw_command_t resolved = *cmd;

    if (cmd->type == GW_CMD_LIGHT_TOGGLE) {
        resolved.type = resolve_toggle(&cmd->dst);

        LOG_INF("Resolved TOGGLE to %s for transport %d",
                resolved.type == GW_CMD_LIGHT_ON ? "ON" : "OFF",
                resolved.dst.transport);
    }

    LOG_INF("Routing cmd_id=%u type=%d transport=%d",
            resolved.cmd_id,
            resolved.type,
            resolved.dst.transport);

    const gw_transport_port_t *port = transport_port_get(resolved.dst.transport);
    if (!port || !port->get_adapter) {
        LOG_ERR("No transport port for transport %d", resolved.dst.transport);
        return -ENOTSUP;
    }

    gw_adapter_t *adapter = port->get_adapter();
    if (!adapter) {
        LOG_ERR("No adapter for transport %d", resolved.dst.transport);
        return -ENODEV;
    }

    int err = command_send_via_adapter(adapter, &resolved);
    if (err == 0) {
        command_emit_pending(&resolved);
    }

    return err;
}

/* ─────────────────────────────────────────────────────────────
 * Public send API
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Queue a command for asynchronous dispatch.
 *
 * The function returns when the command was successfully accepted by the
 * router queue. Actual transport dispatch happens later in the worker thread.
 *
 * @param cmd Command to queue.
 *
 * @retval 0 on success
 * @retval -EINVAL if @p cmd is NULL
 * @retval -ENOSPC if the command queue is full
 */
int command_router_send(const gw_command_t *cmd)
{
    if (!cmd) {
        return -EINVAL;
    }

    int err = k_msgq_put(&s_cmd_q, cmd, K_NO_WAIT);
    if (err < 0) {
        LOG_WRN("command_router: queue full, dropping cmd_id=%u", cmd->cmd_id);
        return -ENOSPC;
    }

    return 0;
}

/**
 * @brief Construct and queue a command for a specific destination.
 *
 * @param dst  Destination node address.
 * @param type Command type.
 *
 * @retval 0 on success
 * @retval -EINVAL if @p dst is NULL
 * @retval -ENOSPC if the command queue is full
 */
int command_router_send_to(const gw_node_addr_t *dst, gw_cmd_type_t type)
{
    if (!dst) {
        return -EINVAL;
    }

    gw_command_t cmd = {
        .cmd_id     = (uint32_t)atomic_inc(&s_cmd_id),
        .type       = type,
        .dst        = *dst,
        .created_ms = k_uptime_get(),
    };

    return command_router_send(&cmd);
}

/* ─────────────────────────────────────────────────────────────
 * Worker thread
 * ───────────────────────────────────────────────────────────── */

/**
 * @brief Worker thread entry point for asynchronous command dispatch.
 *
 * @param a Unused.
 * @param b Unused.
 * @param c Unused.
 */
static void command_router_thread_fn(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    while (1) {
        gw_command_t cmd;

        int err = k_msgq_get(&s_cmd_q, &cmd, K_FOREVER);
        if (err != 0) {
            LOG_ERR("command_router: queue get failed: %d", err);
            continue;
        }

        err = command_router_send_now(&cmd);
        if (err < 0) {
            LOG_WRN("command_router: cmd_id=%u failed: %d", cmd.cmd_id, err);
        }
    }
}