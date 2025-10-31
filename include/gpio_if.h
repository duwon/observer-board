/**
 * @file gpio_if.h
 * @brief LED GPIO abstraction
 */
#pragma once
#include <zephyr/kernel.h>

int  gpio_if_init(void);
void gpio_if_led_on(void);
void gpio_if_led_off(void);
void gpio_if_led_toggle(void);
