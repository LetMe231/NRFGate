#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

/**
 * @brief Initialize LoRa modem with predefined settings.
 *
 * Configures the modem for 868 MHz operation with SF7, BW125, CR4/5,
 * and a preamble length of 8. Also sets the TX power to 4 dBm.
 *
 * @return 0 on success, negative errno on failure.
 */
int lora_handler_init(void);

/**
 * @brief Send data via LoRa.
 *
 * @param json The JSON string to send.
 * @param json_len The length of the JSON string.
 */
void lora_handler_send(const char *json, uint8_t json_len);

#endif /* LORA_HANDLER_H */