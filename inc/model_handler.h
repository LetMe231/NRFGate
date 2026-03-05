#ifndef MODEL_HANDLER_H
#define MODEL_HANDLER_H

#include <zephyr/bluetooth/mesh.h>

const struct bt_mesh_prov *model_handler_prov_init(void);
const struct bt_mesh_comp *model_handler_comp_init(void);
void model_handler_self_provision(void);

#endif /* MODEL_HANDLER_H */