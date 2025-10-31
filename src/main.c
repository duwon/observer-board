/**
 * @file main.c
 * @brief Observer Board (USB CDC + UART 로그 + BLE 스캔)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(observer_main, LOG_LEVEL_INF);

#include "gpio_if.h"
#include "usb_dev.h"
#include "ble_adv.h"

/* UART0 (optional direct write for banner mirroring) */
static const struct device *const uart0_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static inline void _uart_puts(const struct device *dev, const char *s)
{
    if (!dev || !device_is_ready(dev) || !s) return;
    while (*s) uart_poll_out(dev, (unsigned char)(*s++));
}

int main(void)
{
    /* LED init */
    (void)gpio_if_init();

    /* USB CDC init + DTR wait (5s) */
    usbdev_init_and_wait_dtr(5000);

    /* Boot banner (USB CDC + UART0) */
    const char *BOOT_MSG =
        "\r\n======================================\r\n"
        " Observer Board FW Boot Initializing\r\n"
        "======================================\r\n";
    usbdev_puts(BOOT_MSG);               /* USB CDC: 명시 전송 */
    _uart_puts(uart0_dev, BOOT_MSG);     /* UART0  : 직접 전송 */

    /* Start BLE scan */
    (void)ble_rx_start();

    /* Heartbeat via LED (logs are on UART0 due to UART console backend) */
    while (1) {
        gpio_if_led_toggle();
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
