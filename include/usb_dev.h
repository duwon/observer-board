/**
 * @file usb_dev.h
 * @brief USB CDC device helper (enable + DTR wait + write)
 */
#pragma once
#include <zephyr/kernel.h>
#include <stdbool.h>

/** @brief Enable USB device and wait for DTR up to timeout_ms (ms).
 * If no CDC present, it safely returns.
 */
void usbdev_init_and_wait_dtr(int32_t timeout_ms);

/** @brief Write a C-string to USB CDC (noop if not ready). */
void usbdev_puts(const char *s);

/** @brief Is USB CDC device present & ready (DTR not required). */
bool usbdev_is_ready(void);
