/**
 * @file usb_dev.c
 * @brief USB CDC helper implementation
 */

#include "usb_dev.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

/** USB CDC UART 장치 */
static const struct device *const s_cdc = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static bool s_have_cdc = false;

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
 * @brief USB 장치를 활성화하고 호스트가 DTR 신호를 보낼 때까지 지정된 시간(ms) 동안 대기합니다.
 * @param timeout_ms DTR 신호를 기다릴 최대 시간 (밀리초).
 */

void usbdev_init_and_wait_dtr(int32_t timeout_ms)
{
    s_have_cdc = (s_cdc && device_is_ready(s_cdc));
    if (!s_have_cdc)
    {
        printk("CDC device not ready, skip USB prints.\n");
        return;
    }

    int ret = usb_enable(NULL);
    if (ret)
    {
        printk("usb_enable failed: %d\n", ret);
        return;
    }

    int64_t start = k_uptime_get();
    uint32_t dtr = 0U;
    printk("Waiting for USB DTR up to %d ms...\n", timeout_ms);
    while ((k_uptime_get() - start) < timeout_ms)
    {
        (void)uart_line_ctrl_get(s_cdc, UART_LINE_CTRL_DTR, &dtr);
        if (dtr)
            break;
        k_msleep(100);
    }
    if (dtr)
    {
        (void)uart_line_ctrl_set(s_cdc, UART_LINE_CTRL_DCD, 1);
        (void)uart_line_ctrl_set(s_cdc, UART_LINE_CTRL_DSR, 1);
        k_msleep(100);
        printk("USB terminal connected.\n");
    }
    else
    {
        printk("USB DTR timeout. Continue without terminal.\n");
    }
}
/**
 * @brief 문자열을 내부적으로 캐시된 USB CDC 장치로 출력합니다.
 * @param s USB CDC로 출력할 Null 종료 문자열.
 */

void usbdev_puts(const char *s)
{
    if (!s_have_cdc)
        return;
    _uart_puts(s_cdc, s);
}
/**
 * @brief USB CDC 장치가 존재하고 준비되었는지 확인합니다.
 * @return true: CDC 장치가 존재하고 초기화 준비됨, false: 그렇지 않음.
 */

bool usbdev_is_ready(void)
{
    return s_have_cdc;
}