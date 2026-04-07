#ifndef TEST_STUBS_H
#define TEST_STUBS_H

#define MESH_CTRL_H
#define BLE_NUS_H
#define THREAD_HANDLER_H
#define LORA_HANDLER_H
#define MODEL_HANDLER_H
 

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "semantic_handler.h"


/* mesh_ctrl */
void mesh_ctrl_set_onoff(uint16_t addr, bool on);
void mesh_ctrl_toggle(uint16_t addr);
void mesh_ctrl_init(const void *model);
void mesh_ctrl_on_status(uint16_t addr, uint8_t state);

/* ble_nus */
bool ble_nus_is_ready(void);
void ble_nus_send(const char *json);

/* thread_handler */
int thread_handler_coap_put_light(const char *ipv6, bool on);

/* lora_handler */
int  lora_handler_init(void);
void lora_handler_send(const uint8_t *data, uint8_t len);

/* model_handler */
void model_handler_unprovision_node(uint16_t mesh_addr);
void model_handler_start_provisioning(void);
void model_handler_reconfigure_node(uint16_t mesh_addr);
const void *model_handler_prov_init(void);
const void *model_handler_comp_init(void);
void model_handler_self_provision(void);

#endif /* TEST_STUBS_H */