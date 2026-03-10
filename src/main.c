#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/settings/settings.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    err = dk_leds_init();
    if (err) {
        printk("LEDs init failed (err %d)\n", err);
    }

    err = bt_mesh_init(model_handler_prov_init(),
                       model_handler_comp_init());
    if (err) {
        printk("Mesh init failed (err %d)\n", err);
        return;
    }

    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    /* Self-provision with hardcoded keys */
    model_handler_self_provision();

    dk_set_led_on(DK_LED1);
    printk("=== BLE Mesh Gateway ready ===\n");
    printk("Waiting for sensor data...\n");
    printk("Press Button 2 to open provisioning window (30 s)\n");
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
    /* Button 1: Reset mesh */
    if (has_changed & DK_BTN1_MSK) {
        printk("Resetting mesh...\n");
        bt_mesh_cdb_clear();
        bt_mesh_reset();
        sys_reboot(SYS_REBOOT_WARM);
    }

    /* Button 2: Open provisioning window (on press only) */
    if ((has_changed & DK_BTN2_MSK) && (button_state & DK_BTN2_MSK)) {
        model_handler_start_provisioning();
    }
}

int main(void)
{
    int err;

    printk("=== BLE Mesh Gateway starting ===\n");

    err = dk_buttons_init(button_handler);
    if (err) {
        printk("Button init failed (err %d)\n", err);
    }

    err = bt_enable(bt_ready);
    if (err) {
        printk("bt_enable failed (err %d)\n", err);
    }

    return 0;
}