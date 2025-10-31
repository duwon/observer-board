/**
 * @file gpio_if.c
 * @brief LED GPIO implementation (P0.11 alias: led0)
 */

#include "gpio_if.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define LED_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "No LED alias 'led0' defined in overlay"
#endif

static const struct gpio_dt_spec s_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);
/**
 * @brief LED GPIO 포트를 초기화하고 핀을 출력 비활성(INACTIVE) 상태로 설정합니다.
 * @return 0: 성공, 음수: 실패.
 */

int gpio_if_init(void)
{
    if (!device_is_ready(s_led.port))
    {
        printk("LED port not ready.\n");
        return -ENODEV;
    }
    int rc = gpio_pin_configure_dt(&s_led, GPIO_OUTPUT_INACTIVE);
    if (rc)
        printk("LED configure failed: %d\n", rc);
    return rc;
}
/**
 * @brief LED 켜짐
 */

void gpio_if_led_on(void) { gpio_pin_set_dt(&s_led, 1); }
/**
 * @brief LED 꺼짐
 */
void gpio_if_led_off(void) { gpio_pin_set_dt(&s_led, 0); }
/**
 * @brief LED 상태 토글
 */
void gpio_if_led_toggle(void) { gpio_pin_toggle_dt(&s_led); }