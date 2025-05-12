#include <zf_common_headfile.h>

void brush_init(int duty)
{
    pwm_set_duty(PWM_1, duty);
    pwm_set_duty(PWM_2, duty);
    cerr <<"brush_init" << endl;

}


void brush_off()
{
    pwm_set_duty(PWM_1, 0);
    pwm_set_duty(PWM_2, 0);
}