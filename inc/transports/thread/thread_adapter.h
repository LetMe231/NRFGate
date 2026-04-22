#ifndef THREAD_ADAPTER_H
#define THREAD_ADAPTER_H

#include <stdint.h>
#include "gw_adapter.h"

/**
 * @brief Compose the Thread adapter object.
 *        Must be called before thread_adapter_get() or thread_adapter_start().
 */
void thread_adapter_setup(void);

/**
 * @brief Get the Thread adapter instance.
 *
 * @return Pointer to the Thread adapter base interface
 */
gw_adapter_t *thread_adapter_get(void);

/**
 * @brief Start Thread transport runtime.
 *        Applies dataset, opens CoAP socket and starts RX/TX threads.
 *
 * @return 0 on success, negative error code on failure
 */
int thread_adapter_start(void);

#endif /* THREAD_ADAPTER_H */