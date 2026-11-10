#pragma once
#include "gello_codec.h"
#include <stdbool.h>
int gello_usb_start(void);
bool gello_usb_ready(void);
bool gello_usb_read(uint8_t id, struct gello_sample *sample, uint32_t *bad);
