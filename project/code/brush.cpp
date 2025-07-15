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
int duty_left = 0;
int duty_right = 0;
void brush_control()
{
    if(control_circle ==13)  //straight
    {
        duty_left = 700;
        duty_right = 700;
    }
    else if(control_circle ==11  || control_circle == 6 || control_circle ==7 || control_circle == 8 || control_circle == 9)
    {
        duty_left = 650;
        duty_right = 900;
    }
    else if(control_circle ==12 || control_circle == 2 || control_circle == 3 || control_circle == 4||control_circle==5)
    {
        duty_left = 900;
        duty_right = 650;
    }
    else if(control_circle ==0)
    {
        if(my_error2 > 20)
        {
            duty_left = 800;
            duty_right = 700;
        }
        else if(my_error2 < -20)
        {
            duty_left = 700;
            duty_right = 800;
        }
        else
        {
            duty_left = 800;
            duty_right = 800;
        }
    }

    pwm_set_duty(PWM_1, duty_left);
    pwm_set_duty(PWM_2, duty_right);
}