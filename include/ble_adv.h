/**
 * @file ble_adv.h
 * @brief BLE Observer (scan) for Company ID 0xFFFF
 */
#pragma once

/** * @brief Bluetooth를 활성화하고 지정된 파라미터로 BLE Active Scan을 시작합니다.
 * @return 0: 성공, 음수: 실패 (bt_enable 또는 스캔 시작 오류)
 */
int ble_rx_start(void); /** bt_enable + scan start */