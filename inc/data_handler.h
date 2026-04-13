/**
 * @file data_handler.h
 * @brief Centralized sensor data processing
 *
 * This module owns everything that happens *after* a packet arrives:
 * node registration, data storage, logging, and later forwarding
 * (BLE, LoRa, MQTT, ...).
 *
 *  - Maintains a node table mapping IPv6 addresses to node indices
 *  - Logs received sensor data with node context
 */

#ifndef DATA_HANDLER_H
#define DATA_HANDLER_H

#include <zephyr/kernel.h>

// testing
#include "semantic_handler.h"

#define MAX_NODES 8
#define NODE_ADDR_STR_LEN 46   // INET6_ADDRSTRLEN

//===============================================================
// Transport identity
//===============================================================
typedef enum{
    NODE_TRANSPORT_THREAD,      // source = IPv6 address string
    NODE_TRANSPORT_BLE_MESH,    // source = 16 bit mesh address
    //NODE_TRANSPORT_LORA,      // LoRa future
} node_transport_t;

typedef struct{
    node_transport_t transport;
    union{
        char ipv6[NODE_ADDR_STR_LEN];
        uint16_t mesh_addr;
        // LoRa has no address, maybe use DevEUI or just "lora" string
        // add more transports here
    };
} node_identity_t;

//==============================================================
// sensor presence Bitmask 
//===============================================================

#define SENSOR_HAS_SEQ          BIT(0)
#define SENSOR_HAS_TS           BIT(1)
#define SENSOR_HAS_AX           BIT(2)
#define SENSOR_HAS_AY           BIT(3)
#define SENSOR_HAS_AZ           BIT(4)
#define SENSOR_HAS_GX           BIT(5)
#define SENSOR_HAS_GY           BIT(6)
#define SENSOR_HAS_GZ           BIT(7)

#define SENSOR_HAS_TEMP         BIT(8)
#define SENSOR_HAS_HUM          BIT(9)
#define SENSOR_HAS_TVOC         BIT(10)
#define SENSOR_HAS_ECO2         BIT(11)

#define SENSOR_HAS_HEART_RATE   BIT(12)
#define SENSOR_HAS_SPO2         BIT(13)
#define SENSOR_HAS_RAW_RED      BIT(14)
#define SENSOR_HAS_RAW_IR       BIT(15)
#define SENSOR_HAS_PM25         BIT(16)
#define SENSOR_HAS_PM10         BIT(17)
#define SENSOR_HAS_SWITCH       BIT(18)
#define SENSOR_HAS_LIGHT        BIT(19)
// new nodes here 

// sensor groups for convenience
#define SENSOR_HAS_ACCEL  (SENSOR_HAS_AX | SENSOR_HAS_AY | SENSOR_HAS_AZ)
#define SENSOR_HAS_GYRO   (SENSOR_HAS_GX | SENSOR_HAS_GY | SENSOR_HAS_GZ)
#define SENSOR_HAS_ENV    (SENSOR_HAS_TEMP | SENSOR_HAS_HUM)
#define SENSOR_HAS_AIR    (SENSOR_HAS_TVOC | SENSOR_HAS_ECO2)
#define SENSOR_HAS_BIO    (SENSOR_HAS_HEART_RATE | SENSOR_HAS_SPO2 | SENSOR_HAS_RAW_RED | SENSOR_HAS_RAW_IR)

struct sensor_payload {
    int32_t present;
    int32_t seq;
    int32_t ts;

    // IMU
    int32_t ax, ay, az;
    int32_t gx, gy, gz;

    // environmental data
    int32_t temp, hum;

    // air quality data
    int32_t tvoc, eco2;

    // Biometric data
    int32_t heart_rate, spo2;
    int32_t raw_red, raw_ir;

    uint8_t switch_state;   // 0=off, 1=on, for a generic switch input

    int32_t pm25, pm10;
    uint8_t light_on;       // 0=off, 1=on, for feedback from lamp
    // add more sensor fields here
};

struct node_sensor_data {
    uint8_t node_idx;                   // set by data_handler
    node_identity_t identity;           // filled by caller
    int64_t rx_uptime_ms;               // k_uptime_get() at reception time, set by thread_handler    
    struct sensor_payload payload;
    int8_t  rssi;                       // BLE Mesh: received RSSI (dBm), 0 for Thread
    uint8_t hops;                       // BLE Mesh: relay hop count (0=direct), 0 for Thread
};

//===============================================================
// Actuator command struct
//===============================================================

typedef struct {
    bool known;
    bool light_on;
} node_actuator_state_t;

extern node_actuator_state_t node_actuator_state[MAX_NODES];

//===============================================================
// Internal node table and scheduling state
//===============================================================

typedef struct {
    uint16_t mesh_addr;
    int64_t  last_rx_ms;
    int64_t  estimated_period_ms;
    int64_t  predicted_next_ms;  // last_rx + period
} mesh_node_schedule_t;

//===============================================
// Public API
//===============================================

/**
 * @brief Initialize the data handler (node table, any future queues)
 */
int data_handler_init(void);
 
/**
 * @brief Hand off a received sensor packet for processing.
 *
 * This function:
 *   1. Registers or looks up the source node
 *   2. Logs present fields
 *   3. (Later) forwards data via BLE / LoRa / MQTT
 *
 * @param data  Pointer to packet; the struct is copied internally,
 *              so the caller's stack buffer is safe to reuse.
 */
void data_handler_receive(const struct node_sensor_data *data);

void data_handler_cmd(const char *cmd, uint16_t len);

uint8_t data_handler_get_node_idx_by_mesh_addr(uint16_t mesh_addr);

void identity_str(const node_identity_t *id, char *buf, size_t size);

uint8_t data_handler_get_node_idx_by_ipv6(const char *ipv6);

int data_handler_get_mesh_schedules(mesh_node_schedule_t *out,
                                     uint8_t max_nodes);


// Testing statistics
extern int64_t g_ble_active_ms;
extern int64_t g_thread_active_ms;
extern uint32_t g_ble_switches;
extern uint32_t g_thread_switches;
 
/* Per-Node Diagnose-Info */
typedef struct {
    uint8_t  node_idx;
    uint32_t packet_count;
    int64_t  first_seen_ms;
    int64_t  last_seen_ms;
    int32_t  last_seq;
    uint32_t seq_gaps;
    uint8_t state;
    node_transport_t transport;
    union {
        char     ipv6[NODE_ADDR_STR_LEN];
        uint16_t mesh_addr;
    } addr;
} node_diag_t;
 
/* Gibt Diagnose-Info für alle Nodes zurück */
int data_handler_get_diag(node_diag_t *out, uint8_t max_nodes);

#endif /* DATA_HANDLER_H */