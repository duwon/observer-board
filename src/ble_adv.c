/**
 * @file ble_adv.c
 * @brief BLE active scan + filter CID=0xFFFF, parse Phase1/2
 */

#include "ble_adv.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_rx_evm, LOG_LEVEL_INF);

/* ----- Common defs ----- */
#define CID_TARGET    0xFFFF
#define NAME_BUF_LEN  32

struct sensor_adv_data_t {
    uint16_t company_id; /* LE (0xFFFF) */
    uint8_t  structure_version;
    uint8_t  model_code;
    uint8_t  device_status;
    uint8_t  error_info;
    int8_t   mcu_temperature;
    uint8_t  battery_percent;
    uint8_t  value_presence_mask;
    int32_t  sensor_value_1;
    int32_t  sensor_value_2;
    int32_t  sensor_value_3;
    int32_t  sensor_value_4;
    int32_t  sensor_value_5;
    int32_t  sensor_value_6;
} __packed;

#define PHASE2_SR_LEN (2 + 1 + 1 + 1 + 2) /* CID + ver + reg + model + fw */

struct parse_ctx {
    bool     mfg_match;
    bool     is_phase1_struct;
    bool     is_phase2_meta;
    char     name[NAME_BUF_LEN];
    bool     has_name;
    struct sensor_adv_data_t s1;
    uint8_t  s2_ver, s2_reg_mode, s2_model, s2_fw_major, s2_fw_minor;
};

static inline const char *adv_type_str(uint8_t type)
{
    switch (type) {
    case BT_GAP_ADV_TYPE_ADV_IND:         return "ADV_IND";
    case BT_GAP_ADV_TYPE_ADV_DIRECT_IND:  return "ADV_DIRECT_IND";
    case BT_GAP_ADV_TYPE_ADV_SCAN_IND:    return "ADV_SCAN_IND";
    case BT_GAP_ADV_TYPE_ADV_NONCONN_IND: return "ADV_NONCONN_IND";
    case BT_GAP_ADV_TYPE_SCAN_RSP:        return "SCAN_RSP";
    case BT_GAP_ADV_TYPE_EXT_ADV:         return "EXT_ADV";
    default:                              return "UNKNOWN";
    }
}

static void dump_hex(const uint8_t *p, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        printk("%02X", p[i]);
        if (i + 1 < len) printk(" ");
    }
}

static bool parse_cb(struct bt_data *data, void *user_data)
{
    struct parse_ctx *ctx = user_data;

    switch (data->type) {
    case BT_DATA_MANUFACTURER_DATA:
        if (data->data_len >= 2) {
            uint16_t cid = sys_get_le16(data->data);
            if (cid == CID_TARGET) {
                ctx->mfg_match = true;

                if (data->data_len == sizeof(struct sensor_adv_data_t)) {
                    memcpy(&ctx->s1, data->data, sizeof(ctx->s1));
                    ctx->is_phase1_struct = true;
                } else if (data->data_len == PHASE2_SR_LEN) {
                    const uint8_t *q = data->data;
                    ctx->s2_ver      = q[2];
                    ctx->s2_reg_mode = q[3];
                    ctx->s2_model    = q[4];
                    ctx->s2_fw_major = q[5];
                    ctx->s2_fw_minor = q[6];
                    ctx->is_phase2_meta = true;
                }
            }
        }
        break;

    case BT_DATA_NAME_COMPLETE:
    case BT_DATA_NAME_SHORTENED: {
        size_t n = MIN((size_t)data->data_len, (size_t)(NAME_BUF_LEN - 1));
        memcpy(ctx->name, data->data, n);
        ctx->name[n] = '\0';
        ctx->has_name = true;
        break;
    }

    default:
        break;
    }

    return true;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    struct parse_ctx ctx = {0};

    bt_data_parse(ad, parse_cb, &ctx);
    if (!ctx.mfg_match) return;

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    LOG_INF("[%s] match CID=0x%04X RSSI=%d from %s len=%u",
            adv_type_str(type), CID_TARGET, rssi, addr_str, ad->len);

    if (ctx.is_phase1_struct) {
        struct sensor_adv_data_t *p = &ctx.s1;
        LOG_INF("  Phase1: ver=%u model=%u status=0x%02X err=0x%02X temp=%d batt=%u%% mask=0x%02X",
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

    if (ctx.is_phase2_meta) {
        if (ctx.has_name) {
            LOG_INF("  name=\"%s\"", ctx.name);
        }
        LOG_INF("  Phase2: ver=%u reg=%u model=%u fw=%u.%u",
                ctx.s2_ver, ctx.s2_reg_mode, ctx.s2_model,
                ctx.s2_fw_major, ctx.s2_fw_minor);
    }

    if (!ctx.is_phase1_struct && !ctx.is_phase2_meta) {
        printk("  MFG raw: ");
        dump_hex(ad->data, ad->len);
        printk("\n");
    }
}

static int start_scan(void)
{
    struct bt_le_scan_param param = {
        .type     = BT_LE_SCAN_TYPE_ACTIVE,
        .options  = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0060, /* ~60 ms */
        .window   = 0x0030, /* ~30 ms */
    };
    return bt_le_scan_start(&param, device_found);
}

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized, starting scan...");
    err = start_scan();
    if (err) {
        LOG_ERR("Scan start failed (err %d)", err);
    }
}

int ble_rx_start(void)
{
    LOG_INF("BLE Receiver starting...");
    int err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return err;
    }
    return 0;
}
