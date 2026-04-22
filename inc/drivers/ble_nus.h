#pragma once
#include <stddef.h>

int ble_nus_register(void);
int ble_nus_advertise(void);
bool ble_nus_is_ready(void);

void ble_nus_send_immediate(const char *json);

void ble_nus_publish_latest(const char *node_key, const char *json);

void ble_nus_send(const char *json);