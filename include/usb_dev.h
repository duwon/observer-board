/**
 * @file usb_dev.h
 * @brief USB CDC device helper (enable + DTR wait + write)
 */
#pragma once
#include <zephyr/kernel.h>
#include <stdbool.h>

/** * @brief USB 장치를 활성화하고 호스트가 DTR 신호를 보낼 때까지 지정된 시간(ms) 동안 대기합니다.
 * If no CDC present, it safely returns.
 * @param timeout_ms DTR 신호를 기다릴 최대 시간 (밀리초).
 */
void usbdev_init_and_wait_dtr(int32_t timeout_ms);

/** * @brief C-문자열을 USB CDC 포트로 출력합니다 (CDC가 준비되지 않은 경우 아무 작업도 하지 않습니다).
 * @param s USB CDC로 출력할 Null 종료 문자열.
 */
void usbdev_puts(const char *s);

/** * @brief USB CDC 장치가 존재하고 준비되었는지 확인합니다 (DTR 상태는 요구하지 않음).
 * @return true: CDC 장치가 존재하고 초기화 준비됨, false: 그렇지 않음.
 */
bool usbdev_is_ready(void);