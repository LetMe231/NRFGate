#ifndef MESH_CTRL_H
#define MESH_CTRL_H
#include <stdint.h>
#include <stdbool.h>
void mesh_ctrl_init(const void *model);
void mesh_ctrl_on_status(uint16_t addr, uint8_t state);
void mesh_ctrl_set_onoff(uint16_t addr, bool on);
void mesh_ctrl_toggle(uint16_t addr);
#endif
