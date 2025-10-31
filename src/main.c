/**
 * USB CDC는 CDC_Example과 동일한 방식(DEVICE_DT_GET_ONE + DTR 대기)으로 출력,
 * LOG/printk/LOG_INF는 UART0로만 나가도록 prj.conf/overlay에서 UART 콘솔로 설정.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LED_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* USB CDC */
const struct device *const usb_cdc = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

/* UART0 (로그/printk/LOG_INF는 여기로 나감) */
static const struct device *const uart0_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static void uart_puts(const struct device *dev, const char *s)
{
    if (!dev || !device_is_ready(dev) || !s) return;
    while (*s) uart_poll_out(dev, (unsigned char)(*s++));
}

static void Init_Usb(const struct device *cdc_dev, int32_t timeout_ms)
{
    if (!cdc_dev || !device_is_ready(cdc_dev)) {
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
        (void)uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) break;
        k_msleep(100);
    }
    if (dtr) {
        /* 선택: DCD/DSR 설정 및 약간의 지연 */
        (void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DCD, 1);
        (void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DSR, 1);
        k_msleep(100);
        printk("USB terminal connected.\n");
    } else {
        printk("USB DTR timeout. Continue without terminal.\n");
    }
}

int main(void)
{
    /* LED */
    if (!device_is_ready(status_led.port)) {
        printk("LED GPIO port not ready.\n");
        return 0;
    }
    (void)gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);

    /* UART0 준비 확인 (로그는 이미 UART 콘솔로 설정됨) */
    if (!device_is_ready(uart0_dev)) {
        /* 그래도 printk는 콘솔(UART0)로 나가도록 설정되어 있음 */
        printk("UART0 not ready.\n");
    }

    /* USB CDC 활성화 & DTR 대기 */
    Init_Usb(usb_cdc, 5000);

    /* 부트 메시지: USB CDC와 UART0에 각각 출력 */
    const char *BOOT_MSG = "\r\n======================================\r\n Observer Board FW Boot Initializing\r\n======================================\r\n";

    uart_puts(usb_cdc, BOOT_MSG);   /* ← USB CDC */
    uart_puts(uart0_dev, BOOT_MSG); /* ← UART0 */

    LOG_INF("Boot done. LED will toggle every 1s.");

    while (1) {
        gpio_pin_toggle_dt(&status_led);
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
