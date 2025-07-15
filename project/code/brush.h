#ifndef BRUSH_H
#define BRUSH_H

#include <zf_common_headfile.h>

using namespace std;
extern int control_circle;
extern float my_error2; // 误差2
#define PWM_1           "/dev/zf_device_pwm_esc_1"
#define PWM_2           "/dev/zf_device_pwm_esc_2"

void brush_init(int duty);
void brush_off();
void brush_control();
#endif