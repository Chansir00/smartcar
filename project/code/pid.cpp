#include <pid.hpp>
#include <zf_common_headfile.h>

struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
struct pwm_info servo_pwm_info;
float servo_motor_duty = 90.0;
float servo_motor_dir = 1;    
int16 encoder_left;
int16 encoder_right;

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    pwm_set_duty(SERVO_MOTOR1_PWM, 0);   
    pwm_set_duty(MOTOR1_PWM, 0);   
    pwm_set_duty(MOTOR2_PWM, 0);    
}