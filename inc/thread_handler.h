#ifndef THREAD_HANDLER_H
#define THREAD_HANDLER_H

/**
 * @brief Initialize OpenThread CoAP receiver.
 *
 * Opens a UDP socket on port 5683, binds it, and starts
 * a background thread that dispatches incoming CoAP requests.
 *
 * @return 0 on success, negative errno on failure.
 */
int thread_handler_init(void);

#endif /* THREAD_HANDLER_H */