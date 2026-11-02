#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void usb_serial_init(void);
void usb_serial_poll(void);

bool usb_serial_read_line(char *buffer, size_t buffer_size);
size_t usb_serial_tx_free(void);
void usb_serial_write(const char *text);
void usb_serial_write_n(const char *data, size_t len);
void usb_serial_write_u32(uint32_t value);
void usb_serial_write_i32(int32_t value);
void usb_serial_write_line(const char *text);
