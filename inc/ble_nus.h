#pragma once
#include <stddef.h>

int  ble_nus_advertise(void);
void ble_nus_send(const char *json);
bool ble_nus_is_ready(void);
int  ble_nus_register(void);
void ble_nus_flush(void);