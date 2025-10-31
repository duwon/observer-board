/**
 * @file ble_adv.c
 * @brief BLE active scan + filter CID=0xFFFF, parse adv data
 */

#include "ble_adv.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

LOG_MODULE_REGISTER(ble_adv, LOG_LEVEL_INF);

/** ----- Common defs ----- */
#define CID_TARGET 0xFFFF // 제조사 ID
#define NAME_BUF_LEN 32   // 장치 이름 버퍼의 최대 길이

/** @brief 센서 광고 데이터 구조체 */
struct sensor_adv_data_t
{
    uint16_t company_id;       // 제조사 ID.
    uint8_t structure_version; // 데이터 구조체 버전.
    uint8_t model_code;        // 장치 모델 코드.
    uint8_t device_status;     // 장치 상태 플래그.
    uint8_t error_info;        // 에러 정보 코드.
    int8_t mcu_temperature;    // MCU 온도 (°C).
    uint8_t battery_percent;   // 배터리 잔량 (%).
    uint8_t value_presence_mask;
    int32_t sensor_value_1;
    int32_t sensor_value_2;
    int32_t sensor_value_3;
    int32_t sensor_value_4;
    int32_t sensor_value_5;
    int32_t sensor_value_6;
} __packed;

#define PHASE2_SR_LEN (2 + 1 + 1 + 1 + 2) // 스캔 응답(SR) 데이터의 길이(CID + ver + reg + model + fw)

/** @brief BLE 광고/스캔 응답 파싱 결과를 저장하는 컨텍스트 구조체 */
struct parse_ctx
{
    bool mfg_match;          // CID_TARGET과 제조사 데이터가 일치하는지 여부.
    bool is_phase1_struct;   // 데이터 길이가 확장광고 구조체와 일치하는지 여부.
    bool is_phase2_meta;     // 데이터 길이가 스캔응답 데이터와 일치하는지 여부.
    char name[NAME_BUF_LEN]; // 장치 이름 버퍼.
    bool has_name;           // 장치 이름(AD Type 0x08 또는 0x09)이 발견되었는지 여부.
    struct sensor_adv_data_t s1;
    uint8_t s2_ver, s2_reg_mode, s2_model, s2_fw_major, s2_fw_minor;
};

/**
 * @brief BLE 광고 타입 코드를 문자열로 변환합니다.
 * @param type BLE 광고 타입 코드 (예: BT_GAP_ADV_TYPE_ADV_IND).
 * @return 광고 타입을 나타내는 문자열.
 */
static inline const char *adv_type_str(uint8_t type)
{
    switch (type) // 수신된 광고의 타입 코드를 검사합니다.
    {
    case BT_GAP_ADV_TYPE_ADV_IND: // 연결 가능하고 스캔 가능한 비지향성 광고.
        return "ADV_IND";
    case BT_GAP_ADV_TYPE_ADV_DIRECT_IND: // 연결 가능한 지향성 광고.
        return "ADV_DIRECT_IND";
    case BT_GAP_ADV_TYPE_ADV_SCAN_IND: // 연결 불가능하고 스캔 가능한 비지향성 광고.
        return "ADV_SCAN_IND";
    case BT_GAP_ADV_TYPE_ADV_NONCONN_IND: // 연결 및 스캔 불가능한 비지향성 광고.
        return "ADV_NONCONN_IND";
    case BT_GAP_ADV_TYPE_SCAN_RSP: // 스캔 응답.
        return "SCAN_RSP";
    case BT_GAP_ADV_TYPE_EXT_ADV: // 확장 광고 (LE Coded PHY 등).
        return "EXT_ADV";
    default: // 알려지지 않은 타입.
        return "UNKNOWN";
    }
}
/**
 * @brief 주어진 바이트 배열을 16진수 문자열로 출력합니다.
 * @param p 출력할 바이트 배열의 포인터.
 * @param len 출력할 바이트의 길이.
 */

static void dump_hex(const uint8_t *p, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        printk("%02X", p[i]);
        if (i + 1 < len)
            printk(" ");
    }
    printk("\n");
}
/**
 * @brief BLE 광고 데이터 구조체를 파싱하고, 특정 Company ID(0xFFFF)와 데이터 구조체를 찾습니다.
 * @param data 파싱할 광고 데이터 구조체 (type/data/data_len).
 * @param user_data 파싱 결과를 저장할 parse_ctx 구조체 포인터.
 * @return true: 파싱 계속, false: 파싱 중지.
 */

static bool parse_cb(struct bt_data *data, void *user_data)
{
    struct parse_ctx *ctx = user_data;

    switch (data->type) // 광고 데이터 타입 검사
    {
    case BT_DATA_MANUFACTURER_DATA: // AD Type 0xFF인 경우
        if (data->data_len >= 2)
        {
            uint16_t cid = sys_get_le16(data->data);
            if (cid == CID_TARGET) //  0xFFFF
            {
                ctx->mfg_match = true;

                if (data->data_len == sizeof(struct sensor_adv_data_t)) // 데이터 길이가 광고 구조체와 일치하면
                {
                    memcpy(&ctx->s1, data->data, sizeof(ctx->s1));
                    ctx->is_phase1_struct = true;
                }
                else if (data->data_len == PHASE2_SR_LEN) // 데이터 길이가 스캔응답 데이터 길이와 일치하면
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

    case BT_DATA_NAME_COMPLETE:  // 완전한 로컬 이름 (AD Type 0x09)
    case BT_DATA_NAME_SHORTENED: // 단축된 로컬 이름 (AD Type 0x08)
    {
        size_t n = MIN((size_t)data->data_len, (size_t)(NAME_BUF_LEN - 1));
        memcpy(ctx->name, data->data, n);
        ctx->name[n] = '\0';
        ctx->has_name = true;
        break;
    }

    default: // 그 외의 광고 데이터 타입 무시
        break;
    }

    return true;
}
/**
 * @brief BLE 스캔 결과를 처리하는 콜백 함수입니다. CID=0xFFFF에 매치하는 경우 정보를 로그로 출력합니다.
 * @param addr 광고를 보낸 장치의 주소 정보.
 * @param rssi 수신 신호 강도 지표 (dBm).
 * @param type 광고 타입 (e.g., ADV_IND, SCAN_RSP).
 * @param ad 광고 데이터와 스캔 응답 데이터를 포함하는 버퍼.
 */

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    struct parse_ctx ctx = {0};

    bt_data_parse(ad, parse_cb, &ctx); // 수신된 광고 데이터(ad)를 parse_cb를 사용하여 파싱
    if (!ctx.mfg_match)                // 제조사 ID와 일치하는 데이터가 없으면
        return;

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    LOG_INF("[%s] match CID=0x%04X RSSI=%d from %s len=%u",
            adv_type_str(type), CID_TARGET, rssi, addr_str, ad->len);

    if (ctx.is_phase1_struct) // Phase 1 데이터 구조체와 일치하면
    {
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

    if (ctx.is_phase2_meta) // 스캔응답
    {
        if (ctx.has_name)
        {
            LOG_INF("  name=\"%s\"", ctx.name); // 디바이스 이름이 있으면 출력
        }
        LOG_INF("  Phase2: ver=%u reg=%u model=%u fw=%u.%u",
                ctx.s2_ver, ctx.s2_reg_mode, ctx.s2_model,
                ctx.s2_fw_major, ctx.s2_fw_minor);
    }

    if (!ctx.is_phase1_struct && !ctx.is_phase2_meta)
    {
        printk("  MFG raw: ");
        dump_hex(ad->data, ad->len);
    }
}
/**
 * @brief 정의된 스캔 파라미터(Active Scan)로 BLE 스캔을 시작합니다.
 * @return 0: 성공, 음수: 실패.
 */

static int start_scan(void)
{
    struct bt_le_scan_param param = {
        .type = BT_LE_SCAN_TYPE_ACTIVE, // 스캔 응답 요청
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = 0x0060, // 스캔 간격 120MS
        .window = 0x0030,   // 스캔 윈도우 60MS
    };
    return bt_le_scan_start(&param, device_found); // 콜백 함수 등록
}
/**
 * @brief Bluetooth 스택 초기화가 완료된 후 호출되는 콜백 함수입니다. 성공 시 스캔을 시작합니다.
 * @param err 초기화 결과 코드.
 */

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
/**
 * @brief Bluetooth 스택 활성화를 시도하고, 성공 시 스캔 시작 콜백 함수를 등록합니다.
 * @return 0: 성공, 음수: 실패 (bt_enable 오류).
 */

int ble_rx_start(void)
{
    LOG_INF("BLE Receiver starting...");
    int err = bt_enable(bt_ready);
    if (err)
    {
        LOG_ERR("bt_enable failed (err %d)", err);
        return err;
    }
    return 0;
}