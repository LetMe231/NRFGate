#include <zephyr/kernel.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#include "lora_handler.h"

LOG_MODULE_REGISTER(lora_handler, LOG_LEVEL_INF);

static const struct spi_dt_spec spi_dev = 
    SPI_DT_SPEC_GET(DT_NODELABEL(lora0), 
                    SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

#define LORA_FREQ      868000000  /* 868 MHz */
#define LORA_BW        BW_125_KHZ
#define LORA_SF        SF_7
#define LORA_CR        CR_4_5
#define LORA_PREAMBLE  8
#define LORA_TX_POWER  4

static const struct device *lora_dev;

static int sx1276_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx_buf[2] = { reg & 0x7F, 0x00 };
    uint8_t rx_buf[2] = { 0 };

    struct spi_buf tx = { .buf = tx_buf, .len = 2 };
    struct spi_buf rx = { .buf = rx_buf, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int err = spi_transceive_dt(&spi_dev, &tx_set, &rx_set);
    if (err) {
        return err;
    }

    *val = rx_buf[1];
    return 0;
}

static void sx1276_dump_state(const char *tag)
{
    uint8_t version = 0, opmode = 0, irq = 0, dio = 0;

    sx1276_read_reg(0x42, &version);  // RegVersion
    sx1276_read_reg(0x01, &opmode);   // RegOpMode
    sx1276_read_reg(0x12, &irq);      // RegIrqFlags
    sx1276_read_reg(0x40, &dio);      // RegDioMapping1

    LOG_INF("%s RegVersion=0x%02X RegOpMode=0x%02X RegIrqFlags=0x%02X RegDioMapping1=0x%02X",
            tag, version, opmode, irq, dio);
}

int lora_handler_init(void)
{
    lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));
    if (!device_is_ready(lora_dev)) {
        LOG_ERR("LoRa device not ready");
        return -ENODEV;
    }

    if (!spi_is_ready_dt(&spi_dev)) {
        LOG_ERR("SPI not ready");
        return -ENODEV;
    }

    struct lora_modem_config cfg = {
        .frequency    = LORA_FREQ,
        .bandwidth    = LORA_BW,
        .datarate     = LORA_SF,
        .coding_rate  = LORA_CR,
        .preamble_len = LORA_PREAMBLE,
        .tx_power     = LORA_TX_POWER,
        .tx           = true,
    };

    int err = lora_config(lora_dev, &cfg);
    if (err) {
        LOG_ERR("LoRa config failed: %d", err);
        return err;
    }

    sx1276_dump_state("after_config");
    return 0;
}


void lora_handler_send(const char *json, uint8_t json_len)
{
    if(!lora_dev || json_len == 0) return;
    int err = lora_send(lora_dev, (uint8_t *)json, json_len);
    if (err) {
        LOG_WRN("LoRa send failed: %d", err);
    } else {
        LOG_INF("LoRa sent");
    }
}