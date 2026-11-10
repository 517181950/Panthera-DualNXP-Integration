#pragma once
#include <zephyr/drivers/can.h>
void robot_can_trace(const struct can_frame *f,unsigned direction,int status);

void robot_can_trace_enable(bool enabled);
bool robot_can_trace_enabled(void);
