/**
 * @file main.c
 * @brief Observer Board (USB CDC + UART 로그 + BLE 스캔)
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#include "gpio_if.h"
#include "usb_dev.h"
#include "ble_adv.h"

/* UART0 디바이스 */
static const struct device *const uart0_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
/**
 * @brief UART 장치를 사용하여 문자열을 폴링 방식으로 출력합니다.
 * @param dev 문자열을 출력할 UART 장치 포인터.
 * @param s 출력할 Null 종료 문자열.
 */

static inline void _uart_puts(const struct device *dev, const char *s)
{
    if (!dev || !device_is_ready(dev) || !s)
        return;
    while (*s)
        uart_poll_out(dev, (unsigned char)(*s++));
}
/**
 * @brief 메인 실행 루프입
 * @return 0: 정상 종료이나 무한루프로 도달하지 않음.
 */

int main(void)
{
    /*  LED init */
    (void)gpio_if_init();

    /*  USB CDC init + DTR wait (5s) */
    Init_usbdev(5000);

    /*  Print boot message */
    const char *BOOT_MSG =
        "\r\n======================================\r\n"
        " Observer Board FW Boot Initializing\r\n"
        "======================================\r\n";
    usbdev_puts(BOOT_MSG);           /*  USB CDC 전송 */
    _uart_puts(uart0_dev, BOOT_MSG); /*  UART0   전송 */

    /*  ble 시작 */
    (void)ble_rx_start();

    /* 디버깅용 코드 */


    while (1)
    {
        gpio_if_led_toggle();
        k_sleep(K_SECONDS(1));
    }
    return 0;
}