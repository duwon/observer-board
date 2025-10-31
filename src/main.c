/**
 * @file main.c
 * @brief Observer Board (USB CDC + UART 로그 + BLE 스캔)
 *
 * - UART0: 로그/printk/LOG_INF 출력 (115200, TX=P0.15/RX=P0.17)
 * - USB CDC: DTR 이후에만 텍스트 전송 (BOOT_MSG 등 명시적 송신만)
 * - LED: P0.11 1초 토글
 * - BLE: Active scan, Manufacturer ID==0xFFFF만 필터 출력
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(observer, LOG_LEVEL_INF);

#define LED_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* USB CDC */
const struct device *const usb_cdc_dev = DEVICE_DT_GET_ONE(zephyr_cdc_acm_uart);

/* UART0 (로그/printk/LOG_INF는 여기로 나감) */
static const struct device *const uart0_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));

static inline void uart_puts(const struct device *dev, const char *s)
{
    if (!dev || !device_is_ready(dev) || !s)
        return;
    while (*s)
        uart_poll_out(dev, (unsigned char)(*s++));
}

static void Init_usb(const struct device *cdc_dev, int32_t timeout_ms)
{
    if (!cdc_dev || !device_is_ready(cdc_dev))
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
        (void)uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr)
            break;
        k_msleep(100);
    }
    if (dtr)
    {
        (void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DCD, 1);
        (void)uart_line_ctrl_set(cdc_dev, UART_LINE_CTRL_DSR, 1);
        k_msleep(100);
        printk("USB terminal connected.\n");
    }
    else
    {
        printk("USB DTR timeout. Continue without terminal.\n");
    }
}

/* ===== BLE 스캔 파트 ===== */
#define CID_TARGET 0xFFFF
#define NAME_BUF_LEN 32

struct sensor_adv_data_t
{
    uint16_t company_id; /* LE (0xFFFF) */
    uint8_t structure_version;
    uint8_t model_code;
    uint8_t device_status;
    uint8_t error_info;
    int8_t mcu_temperature;
    uint8_t battery_percent;
    uint8_t value_presence_mask;
    int32_t sensor_value_1;
    int32_t sensor_value_2;
    int32_t sensor_value_3;
    int32_t sensor_value_4;
    int32_t sensor_value_5;
    int32_t sensor_value_6;
} __packed;

#define PHASE2_SR_LEN (2 + 1 + 1 + 1 + 2) /* CID + ver + reg + model + fw */

struct parse_ctx
{
    bool mfg_match;
    bool is_phase1_struct;
    bool is_phase2_meta;
    char name[NAME_BUF_LEN];
    bool has_name;
    struct sensor_adv_data_t s1;
    uint8_t s2_ver, s2_reg_mode, s2_model, s2_fw_major, s2_fw_minor;
};

static inline const char *adv_type_str(uint8_t type)
{
    switch (type)
    {
    case BT_GAP_ADV_TYPE_ADV_IND:
        return "ADV_IND";
    case BT_GAP_ADV_TYPE_ADV_DIRECT_IND:
        return "ADV_DIRECT_IND";
    case BT_GAP_ADV_TYPE_ADV_SCAN_IND:
        return "ADV_SCAN_IND";
    case BT_GAP_ADV_TYPE_ADV_NONCONN_IND:
        return "ADV_NONCONN_IND";
    case BT_GAP_ADV_TYPE_SCAN_RSP:
        return "SCAN_RSP";
    case BT_GAP_ADV_TYPE_EXT_ADV:
        return "EXT_ADV";
    default:
        return "UNKNOWN";
    }
}

static void dump_hex(const uint8_t *p, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        printk("%02X", p[i]);
        if (i + 1 < len)
            printk(" ");
    }
}

static bool parse_cb(struct bt_data *data, void *user_data)
{
    struct parse_ctx *ctx = user_data;

    switch (data->type)
    {
    case BT_DATA_MANUFACTURER_DATA:
        if (data->data_len >= 2)
        {
            uint16_t cid = sys_get_le16(data->data);
            if (cid == CID_TARGET)
            {
                ctx->mfg_match = true;

                if (data->data_len == sizeof(struct sensor_adv_data_t))
                {
                    memcpy(&ctx->s1, data->data, sizeof(ctx->s1));
                    ctx->is_phase1_struct = true;
                }
                else if (data->data_len == PHASE2_SR_LEN)
                {
                    const uint8_t *q = data->data;
                    ctx->s2_ver = q[2];
                    ctx->s2_reg_mode = q[3];
                    ctx->s2_model = q[4];
                    ctx->s2_fw_major = q[5];
                    ctx->s2_fw_minor = q[6];
                    ctx->is_phase2_meta = true;
                }
            }
        }
        break;

    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED:
    {
        size_t n = MIN((size_t)data->data_len, (size_t)(NAME_BUF_LEN - 1));
        memcpy(ctx->name, data->data, n);
        ctx->name[n] = '\0';
        ctx->has_name = true;
        break;
    }

    default:
        break;
    }

    return true; /* 계속 파싱 */
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    struct parse_ctx ctx = {0};

    bt_data_parse(ad, parse_cb, &ctx);
    if (!ctx.mfg_match)
    {
        return; /* 타사 데이터 무시 */
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    /* 공통 헤더는 UART 로그로 */
    LOG_INF("[%s] match CID=0x%04X RSSI=%d from %s len=%u",
            adv_type_str(type), CID_TARGET, rssi, addr_str, ad->len);

    if (ctx.is_phase1_struct)
    {
        struct sensor_adv_data_t *p = &ctx.s1;
        LOG_INF("  Phase1:"
                " ver=%u model=%u status=0x%02X err=0x%02X temp=%d batt=%u%% mask=0x%02X",
                p->structure_version, p->model_code, p->device_status, p->error_info,
                p->mcu_temperature, p->battery_percent, p->value_presence_mask);

        LOG_INF("  s1=%d s2=%d s3=%d s4=%d s5=%d s6=%d",
                (int)sys_le32_to_cpu(p->sensor_value_1),
                (int)sys_le32_to_cpu(p->sensor_value_2),
                (int)sys_le32_to_cpu(p->sensor_value_3),
                (int)sys_le32_to_cpu(p->sensor_value_4),
                (int)sys_le32_to_cpu(p->sensor_value_5),
                (int)sys_le32_to_cpu(p->sensor_value_6));
    }

    if (ctx.is_phase2_meta)
    {
        if (ctx.has_name)
        {
            LOG_INF("  name=\"%s\"", ctx.name);
        }
        LOG_INF("  Phase2: ver=%u reg=%u model=%u fw=%u.%u",
                ctx.s2_ver, ctx.s2_reg_mode, ctx.s2_model,
                ctx.s2_fw_major, ctx.s2_fw_minor);
    }

    if (!ctx.is_phase1_struct && !ctx.is_phase2_meta)
    {
        printk("  MFG raw: ");
        dump_hex(ad->data, ad->len);
        printk("\n");
    }
}

static int start_scan(void)
{
    struct bt_le_scan_param param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE, /* SCAN_RSP 받기 위해 Active */
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0060, /* ~60 ms */
        .window = 0x0030,   /* ~30 ms */
    };
    return bt_le_scan_start(&param, device_found);
}

static void bt_ready(int err)
{
    if (err)
    {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized, starting scan...");
    err = start_scan();
    if (err)
    {
        LOG_ERR("Scan start failed (err %d)", err);
    }
}

/* ===== App entry ===== */
int main(void)
{
    /* LED */
    if (!device_is_ready(status_led.port))
    {
        printk("LED port not ready.\n");
        return 0;
    }
    (void)gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);

    /* UART0 준비 확인 (로그는 이미 UART 콘솔로 설정됨) */
    if (!device_is_ready(uart0_dev)) {
        /* 그래도 printk는 콘솔(UART0)로 나가도록 설정되어 있음 */
        printk("UART0 not ready.\n");
    }

    /* USB CDC: DTR 대기 후에만 출력 사용 */
    Init_usb(usb_cdc_dev, 5000);

    /* 부트 메시지: USB CDC와 UART0에 각각 출력 */
    const char *BOOT_MSG = "\r\n======================================\r\n Observer Board FW Boot Initializing\r\n======================================\r\n";
    uart_puts(usb_cdc_dev, BOOT_MSG);   /* USB CDC */
    uart_puts(uart0_dev, BOOT_MSG); /* UART0 */

    /* BLE 시작 */
    LOG_INF("BLE Receiver starting...");
    int err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("bt_enable failed (err %d)", err);
        return 0;
    }


    while (1)
    {
        gpio_pin_toggle_dt(&status_led);
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
