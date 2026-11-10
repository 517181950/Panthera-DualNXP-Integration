#pragma once
#include <stdbool.h>
#include <stdint.h>
struct home_still {float velocity[6],low[6],high[6];uint64_t updated,since;bool initialized;};
bool home_still_update(struct home_still *s,const float q[6],const float dq[6],uint64_t now,bool position_ok);
