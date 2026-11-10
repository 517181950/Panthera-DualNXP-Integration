#pragma once
#include <stdbool.h>
struct home_path {float start[6],goal[6],distance,ramp,speed,duration;};
bool home_path_start_ok(const float start[6]);
bool home_path_reference(const float raw[6],float zero[6]);
void home_path_begin(struct home_path *p,const float start[6],unsigned stage);
void home_path_sample(const struct home_path *p,float seconds,float target[6],float velocity[6]);
