#ifndef BLE_NUS_H
#define BLE_NUS_H
#include <stdbool.h>
bool ble_nus_is_ready(void);
void ble_nus_send(const char *json);
#endif
