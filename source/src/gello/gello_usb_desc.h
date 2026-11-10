#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
struct gello_usb_endpoints {
    uint8_t control_iface, data_iface, in, out;
    uint16_t in_mps;
};
/* Match exactly one CDC ACM function with its union-linked data interface. */
bool gello_usb_describe(const uint8_t *data, size_t length, struct gello_usb_endpoints *ep);
