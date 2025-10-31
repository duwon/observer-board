/**
 * @file usb_dev.c
 * @brief USB CDC helper implementation
 */

#include "usb_dev.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

/* CDC instance like CDC_Example */
static const struct device *const s_cdc = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static bool s_have_cdc = false;

static inline void _uart_puts(const struct device *dev, const char *s)
{
    if (!dev || !device_is_ready(dev) || !s) return;
    while (*s) uart_poll_out(dev, (unsigned char)(*s++));
}

void usbdev_init_and_wait_dtr(int32_t timeout_ms)
{
    s_have_cdc = (s_cdc && device_is_ready(s_cdc));
    if (!s_have_cdc) {
        printk("CDC device not ready, skip USB prints.\n");
        return;
    }

    int ret = usb_enable(NULL);
    if (ret) {
        printk("usb_enable failed: %d\n", ret);
        return;
    }

    int64_t start = k_uptime_get();
    uint32_t dtr = 0U;
    printk("Waiting for USB DTR up to %d ms...\n", timeout_ms);
    while ((k_uptime_get() - start) < timeout_ms) {
        (void)uart_line_ctrl_get(s_cdc, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) break;
        k_msleep(100);
    }
    if (dtr) {
        (void)uart_line_ctrl_set(s_cdc, UART_LINE_CTRL_DCD, 1);
        (void)uart_line_ctrl_set(s_cdc, UART_LINE_CTRL_DSR, 1);
        k_msleep(100);
        printk("USB terminal connected.\n");
    } else {
        printk("USB DTR timeout. Continue without terminal.\n");
    }
}

void usbdev_puts(const char *s)
{
    if (!s_have_cdc) return;
    _uart_puts(s_cdc, s);
}

bool usbdev_is_ready(void)
{
    return s_have_cdc;
}
