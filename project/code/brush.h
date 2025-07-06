#ifndef BRUSH_H
#define BRUSH_H

#include <zf_common_headfile.h>

using namespace std;

#define PWM_1           "/dev/zf_device_pwm_esc_1"
#define PWM_2           "/dev/zf_device_pwm_esc_2"


void brush_init(int duty);
void brush_off();
#endif