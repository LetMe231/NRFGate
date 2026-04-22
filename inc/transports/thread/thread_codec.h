/**
 * @file thread_codec.h
 * @brief JSON decoder for Thread sensor payloads.
 */

#ifndef THREAD_CODEC_H_
#define THREAD_CODEC_H_

#include <stddef.h>
#include <stdint.h>

#include "gw_model.h"

/**
 * @brief Event callback type used by the Thread codec.
 *
 * @param evt       Parsed gateway event.
 * @param user_data User context pointer.
 */
typedef void (*thread_codec_emit_fn)(const gw_event_t *evt, void *user_data);

/**
 * @brief Parse a Thread JSON sensor payload into gateway events.
 *
 * @param json       JSON payload.
 * @param len        Payload length in bytes.
 * @param ipv6       Source IPv6 address in binary form.
 * @param rx_ms      Receive timestamp in milliseconds.
 * @param emit       Event callback.
 * @param emit_user  User context passed to @p emit.
 *
 * @retval 0 on success
 * @retval -EINVAL if inputs are invalid
 * @retval <0 JSON parsing error
 */
int thread_codec_parse_sensor_json(char *json,
                                   size_t len,
                                   const uint8_t ipv6[GW_IPV6_BIN_LEN],
                                   int64_t rx_ms,
                                   thread_codec_emit_fn emit,
                                   void *emit_user);

#endif /* THREAD_CODEC_H_ */