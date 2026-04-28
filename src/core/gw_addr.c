#include <string.h>
#include "gw_addr.h"

typedef struct{
    uint8_t prefix_bytes[8];
    uint8_t prefix_mask[8];
    gw_transport_t transport;
    const char *description;
} gw_route_entry_t;

static const gw_route_entry_t s_routes[] = {
    {
        .prefix_bytes = {0xfd, GW_ULA_GID_BYTE0, GW_ULA_GID_BYTE1, GW_ULA_GID_BYTE2,
                         GW_ULA_GID_BYTE3, GW_ULA_GID_BYTE4, (GW_SUBNET_BLE_MESH << 4) | 0x0},
        .prefix_mask  = {0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00},
        .transport    = GW_TR_BLE_MESH,
        .description  = "BLE Mesh subnet",
    },
    {
        .prefix_bytes = {0xfd, GW_ULA_GID_BYTE0, GW_ULA_GID_BYTE1, GW_ULA_GID_BYTE2,
                         GW_ULA_GID_BYTE3, GW_ULA_GID_BYTE4, (GW_SUBNET_LORAWAN << 4) | 0x0},
        .prefix_mask  = {0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00},
        .transport    = GW_TR_LORAWAN,
        .description  = "LoRaWAN subnet",
    },
    {
        .prefix_bytes = {0xfd, 0, 0, 0, 0, 0, 0, 0},
        .prefix_mask  = {0xff, 0, 0, 0, 0, 0, 0, 0},
        .transport    = GW_TR_THREAD,
        .description  = "Thread (any other ULA)",
    },
};


void gw_addr_from_mesh(gw_node_addr_t *out, uint16_t mesh_addr)
{
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    
    out->ipv6[0] = 0xfd; /* ULA prefix */
    out->ipv6[1] = GW_ULA_GID_BYTE0;
    out->ipv6[2] = GW_ULA_GID_BYTE1;
    out->ipv6[3] = GW_ULA_GID_BYTE2;
    out->ipv6[4] = GW_ULA_GID_BYTE3;
    out->ipv6[5] = GW_ULA_GID_BYTE4;
    out->ipv6[6] = (GW_SUBNET_BLE_MESH << 4) | 0x0; /* subnet with T=1 in high nibble */

    out->ipv6[14] = (mesh_addr >> 8) & 0xFF;
    out->ipv6[15] = mesh_addr & 0xFF;

    out->transport = GW_TR_BLE_MESH;
}

void gw_addr_from_thread(gw_node_addr_t *out, const struct in6_addr *thread_ml_eid)
{
    if (!out || !thread_ml_eid) {
        return;
    }
    memset(out, 0, sizeof(*out));
    memcpy(out->ipv6, thread_ml_eid->s6_addr, 16);
    out->transport = GW_TR_THREAD;
}


void gw_addr_from_lorawan(gw_node_addr_t *out, uint64_t dev_eui)
{
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    
    out->ipv6[0] = 0xfd; /* ULA prefix */
    out->ipv6[1] = GW_ULA_GID_BYTE0;
    out->ipv6[2] = GW_ULA_GID_BYTE1;
    out->ipv6[3] = GW_ULA_GID_BYTE2;
    out->ipv6[4] = GW_ULA_GID_BYTE3;
    out->ipv6[5] = GW_ULA_GID_BYTE4;
    out->ipv6[6] = (GW_SUBNET_LORAWAN << 4) | 0x0; /* subnet with T=3 in high nibble */

    /* dev_eui goes in the low 64 bits of the interface ID */
    out->ipv6[8]  = (dev_eui >> 56) & 0xFF;  // MSB of dev_eui
    out->ipv6[9]  = (dev_eui >> 48) & 0xFF;
    out->ipv6[10] = (dev_eui >> 40) & 0xFF;
    out->ipv6[11] = (dev_eui >> 32) & 0xFF;
    out->ipv6[12] = (dev_eui >> 24) & 0xFF;
    out->ipv6[13] = (dev_eui >> 16) & 0xFF;
    out->ipv6[14] = (dev_eui >>  8) & 0xFF;
    out->ipv6[15] = (dev_eui      ) & 0xFF;  // LSB of dev_eui

    out->transport = GW_TR_LORAWAN;
}

int gw_addr_to_mesh(const gw_node_addr_t *addr, uint16_t *out_mesh_addr)
{
    if (!addr || !out_mesh_addr || addr->transport != GW_TR_BLE_MESH) {
        return -EINVAL;
    }

    *out_mesh_addr = ((uint16_t)addr->ipv6[14] << 8) | (uint16_t)addr->ipv6[15];
    return 0;
}

int gw_addr_to_lorawan(const gw_node_addr_t *addr, uint64_t *out_dev_eui)
{
    if (!addr || !out_dev_eui || addr->transport != GW_TR_LORAWAN) {
        return -EINVAL;
    }

    *out_dev_eui = ((uint64_t)addr->ipv6[8]  << 56) |
                   ((uint64_t)addr->ipv6[9]  << 48) |
                   ((uint64_t)addr->ipv6[10] << 40) |
                   ((uint64_t)addr->ipv6[11] << 32) |
                   ((uint64_t)addr->ipv6[12] << 24) |
                   ((uint64_t)addr->ipv6[13] << 16) |
                   ((uint64_t)addr->ipv6[14] <<  8) |
                   (uint64_t)addr->ipv6[15];
    return 0;
}

int gw_addr_to_thread(const gw_node_addr_t *addr, struct in6_addr *out_thread_ml_eid)
{
    if (!addr || !out_thread_ml_eid || addr->transport != GW_TR_THREAD) {
        return -EINVAL;
    }

    memcpy(out_thread_ml_eid->s6_addr, addr->ipv6, 16);
    return 0;
}

gw_transport_t gw_addr_route(const gw_node_addr_t *addr)
{
    if (!addr) {
        return GW_TR_UNKNOWN;
    }

    for (size_t i = 0; i < ARRAY_SIZE(s_routes); i++) {
        const gw_route_entry_t *e = &s_routes[i];
        bool match = true;

        for (size_t b = 0; b < sizeof(e->prefix_bytes); b++) {
            if ((addr->ipv6[b] & e->prefix_mask[b]) != (e->prefix_bytes[b] & e->prefix_mask[b])) {
                match = false;
                break;
            }
        }
        
        if (match) {
            return e->transport;
        }
    }

    return GW_TR_UNKNOWN;
}

bool gw_addr_equal(const gw_node_addr_t *a, const gw_node_addr_t *b)
{
    if (!a || !b) {
        return false;
    }
    return memcmp(a->ipv6, b->ipv6, GW_IPV6_BIN_LEN) == 0;
}

int gw_addr_to_str(const gw_node_addr_t *addr, char *buf, size_t buflen){
    if (!addr || !buf || buflen == 0) {
        return -EINVAL;
    }

    struct in6_addr a;
    memcpy(a.s6_addr, addr->ipv6, GW_IPV6_BIN_LEN);
    int r = net_addr_ntop(AF_INET6, &a, buf, buflen);
    if (r < 0) {
        return -EINVAL;
    }
    return 0;
}

int gw_addr_from_str(gw_node_addr_t *out, const char *str)
{
    if (!out || !str) {
        return -EINVAL;
    }

    struct in6_addr a;
    if (net_addr_pton(AF_INET6, str, &a) < 0) {
        return -EINVAL;
    }

    /* Erst eine korrekte gw_node_addr_t aufbauen */
    gw_node_addr_t tmp = {0};
    memcpy(tmp.ipv6, a.s6_addr, GW_IPV6_BIN_LEN);
    
    /* Dann routen — das gibt uns den Transport */
    gw_transport_t tr = gw_addr_route(&tmp);
    if (tr == GW_TR_UNKNOWN) {
        return -EINVAL;
    }

    /* Output befüllen */
    *out = tmp;
    out->transport = tr;
    return 0;
}