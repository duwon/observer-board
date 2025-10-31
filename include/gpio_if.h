/**
 * @file gpio_if.h
 * @brief LED GPIO abstraction
 */
#pragma once
#include <zephyr/kernel.h>

/**
 * @brief LED GPIO 인터페이스를 초기화하고 핀을 출력 비활성(INACTIVE) 상태로 설정합니다.
 * @return 0 이상: 성공, 음수: 실패 (-ENODEV, 설정 오류 등)
 */
int gpio_if_init(void);

/**
 * @brief LED를 웁니다 (ACTIVE 상태로 설정).
 */
void gpio_if_led_on(void);

/**
 * @brief LED를 끕니다 (INACTIVE 상태로 설정).
 */
void gpio_if_led_off(void);

/**
 * @brief LED의 현재 상태를 토글합니다.
 */
void gpio_if_led_toggle(void);