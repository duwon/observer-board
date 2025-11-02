/**
 * @file usb_dev.c
 * @brief USB CDC helper implementation
 */

#include "usb_dev.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dev, LOG_LEVEL_INF);

/** USB CDC UART 장치 */
static const struct device *const s_cdc = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);
static bool s_have_cdc = false;

/* 수신 라인 버퍼: 개행 단위로 로그 출력 */
#define USB_RX_LINE_MAX 128
static char s_rx_line[USB_RX_LINE_MAX];
static size_t s_rx_len = 0;

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
 * @brief USB CDC 수신 IRQ 콜백.
 *        엔터 또는 버퍼 가득 차면 LOG로 출력한다.
 */
static void _cdc_irq_handler(const struct device *dev, void *user)
{
    ARG_UNUSED(user);

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {

        if (uart_irq_rx_ready(dev)) {
            uint8_t buf[64];
            int rd = uart_fifo_read(dev, buf, sizeof(buf));
            if (rd < 0) rd = 0;

            for (int i = 0; i < rd; i++) {
                char c = (char)buf[i];

                /* 버퍼에 저장 */
                if (s_rx_len < (USB_RX_LINE_MAX - 1)) {
                    s_rx_line[s_rx_len++] = c;
                }

                /* 엔터 입력, 출력 */
                if (c == '\r') {
                    s_rx_line[s_rx_len] = '\0';
                    LOG_INF("USB RX: %s", s_rx_line);
                    s_rx_len = 0;
                }
            }

            /* 버퍼 가득 참, 출력 후 초기화 */
            if (s_rx_len >= (USB_RX_LINE_MAX - 1)) {
                s_rx_line[s_rx_len] = '\0';
                LOG_INF("USB RX: %s", s_rx_line);
                s_rx_len = 0;
            }
        }

        if (uart_irq_tx_ready(dev)) {
            uart_irq_tx_disable(dev);
        }
    }
}

/**
 * @brief USB 장치를 활성화하고 호스트가 DTR 신호를 보낼 때까지 지정된 시간(ms) 동안 대기합니다.
 * @param timeout_ms DTR 신호를 기다릴 최대 시간 (밀리초).
 */

void Init_usbdev(int32_t timeout_ms)
{
    s_have_cdc = (s_cdc && device_is_ready(s_cdc));
    if (!s_have_cdc)
    {
        LOG_INF("CDC device not ready, skip USB prints.\n");
        return;
    }

    int ret = usb_enable(NULL);
    if (ret)
    {
        LOG_INF("usb_enable failed: %d\n", ret);
        return;
    }

    int64_t start = k_uptime_get();
    uint32_t dtr = 0U;
    LOG_INF("Waiting for USB DTR up to %d ms...\n", timeout_ms);
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
        LOG_INF("USB terminal connected.\n");
    }
    else
    {
        LOG_INF("USB DTR timeout. Continue without terminal.\n");
    }


    if (!s_have_cdc) return;

    /* IRQ 콜백 등록 및 RX 인터럽트 활성화 */
    uart_irq_callback_set(s_cdc, _cdc_irq_handler);
    uart_irq_rx_enable(s_cdc);

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
