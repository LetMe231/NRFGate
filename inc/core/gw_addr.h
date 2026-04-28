#ifndef GW_ADDR_H
#define GW_ADDR_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/net/net_ip.h>
#include "gw_model.h"

/* ── Address scheme ──────────────────────────────────────────────
 *
 * RFC 4193 ULA prefix layout for this gateway:
 *
 *   fd<global_id_5_bytes><subnet_2_bytes>::/64
 *
 * fd 7c da ae 23 1e 10 00 00 00 00 00 00 00 00 02
 *└┬┘ └────┬────────┘└┬┘ └─────────┬─────────┘└┬─┘
 * │       │          │            │           │
 * │       │          │            │           └── mesh_addr (low 16 bits)
 * │       │          │            └── all 0
 * │       │          └── Subnet (T=1 e.g. ble mesh)
 * │       └── 40-bit Global ID
 * └── ULA-Marker (always fd)
 * 
 * The subnet's high nibble encodes the transport (T):
 *
 *   T=1  BLE Mesh    interface-id: lower 16 bits = mesh_addr
 *   T=2  Thread      interface-id: full Thread ML-EID interface part
 *   T=3  LoRaWAN     interface-id: lower 64 bits = dev_eui
 *   T=0  Gateway     reserved for the gateway itself
 *   T=4..F           reserved for future transports
 * ──────────────────────────────────────────────────────────────── */

#define GW_ULA_GID_BYTE0   0x7c   
#define GW_ULA_GID_BYTE1   0xda
#define GW_ULA_GID_BYTE2   0xae
#define GW_ULA_GID_BYTE3   0x2e
#define GW_ULA_GID_BYTE4   0x1e

#define GW_SUBNET_GATEWAY  0x0
#define GW_SUBNET_BLE_MESH 0x1

#define GW_SUBNET_LORAWAN  0x3

/* ── Constructors ────────────────────────────────────────────────
 *
 * Build a gw_node_addr_t from a transport-native identifier.
 * These are the ONLY ways to create a valid address — no direct field
 * assignment is allowed elsewhere in the code base.
 * ──────────────────────────────────────────────────────────────── */

void gw_addr_from_mesh(gw_node_addr_t *out, uint16_t mesh_addr);

void gw_addr_from_thread(gw_node_addr_t *out, const struct in6_addr *thread_ml_eid);

void gw_addr_from_lorawan(gw_node_addr_t *out, uint64_t dev_eui);

/* ── Reverse lookups ─────────────────────────────────────────────
 *
 * Extract transport-native identifier from an address. These return
 * 0 on success, -EINVAL if the address doesn't belong to that transport.
 * ──────────────────────────────────────────────────────────────── */

int gw_addr_to_mesh(const gw_node_addr_t *addr, uint16_t *out_mesh_addr);

int gw_addr_to_lorawan(const gw_node_addr_t *addr, uint64_t *out_dev_eui);

/* For Thread: the IPv6 IS the address (3a). No conversion needed,
 * caller copies addr->ipv6 directly. */

/* ── Routing ─────────────────────────────────────────────────────
 *
 * Determine which transport an address belongs to by inspecting its
 * prefix. Returns GW_TR_UNKNOWN if the address doesn't match any
 * known prefix.
 * ──────────────────────────────────────────────────────────────── */

gw_transport_t gw_addr_route(const gw_node_addr_t *addr);

/* ── Equality and string conversion ──────────────────────────────
 *
 * Two addresses are equal iff their IPv6 bytes are byte-equal.
 * The transport field is derived from the IPv6 prefix, so it doesn't
 * need to be compared separately.
 * ──────────────────────────────────────────────────────────────── */

bool gw_addr_equal(const gw_node_addr_t *a, const gw_node_addr_t *b);

/* Format an address as a string. buf must be at least GW_IPV6_STR_LEN bytes.
 * Returns 0 on success. */
int gw_addr_to_str(const gw_node_addr_t *addr, char *buf, size_t buflen);

/* Parse an IPv6 string into an address. Sets transport via gw_addr_route().
 * Returns 0 on success, -EINVAL on parse failure or unknown prefix. */
int gw_addr_from_str(gw_node_addr_t *out, const char *str);

#endif /* GW_ADDR_H */