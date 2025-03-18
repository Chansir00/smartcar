#ifndef __PID_H__
#define __PID_H__
#include <zf_common_headfile.h>
#include <string>
#include <algorithm>
#include <mutex>
#include <iostream>
#include <cmath>

using namespace std;
//motorl
#define MOTOR1_DIR   "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM   "/dev/zf_device_pwm_motor_1"
//motorr
#define MOTOR2_DIR   "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM   "/dev/zf_device_pwm_motor_2"
//encoder
#define ENCODER_1           "/dev/zf_encoder_1"
#define ENCODER_2           "/dev/zf_encoder_2"
//servo
#define SERVO_MOTOR1_PWM            "/dev/zf_device_pwm_servo"
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (motor_1_pwm_info.duty_max)       
#define MOTOR2_PWM_DUTY_MAX    (motor_2_pwm_info.duty_max) 
#define SERVO_MOTOR_FREQ            (servo_pwm_info.freq)
#define PWM_DUTY_MAX                (servo_pwm_info.duty_max)
#define SERVO_MOTOR_L_MAX           (45 )                       
#define SERVO_MOTOR_R_MAX           (125)
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
extern struct pwm_info servo_pwm_info;

// 控制参数
constexpr int MID_W = 160;          // 图像中线
constexpr int PWM_MAX = 10000;      // 电机最大PWM
constexpr int CONTROL_PERIOD_MS = 10;
extern int16 encoder_left;
extern int16 encoder_right;
extern float servo_motor_duty;                                                  // 舵机zhongzhi
extern float servo_motor_dir;    

enum PID_Mode {
    POSITION_PID,
    DELTA_PID
};

struct PID_Controller {
    float Kp, Ki, Kd;
    float integral = 0;
    float error[3] = {0};
    float output = 0;
    float max_output;
    float target;
    float actual;
    PID_Mode mode;
};

class MotionController {
private:
    PID_Controller pidLeft;
    PID_Controller pidRight;
    PID_Controller pidservo;
    int fd_servo;
    std::mutex encoder_mutex;  // 添加互斥锁
    int16 encoder_left = 0;
    int16 encoder_right = 0;
    const int SERVO_MID = 150000; // 舵机中位脉冲(1.5ms)
    const int SERVO_MIN = 100000; // 最小脉冲限制(1.0ms)
    const int SERVO_MAX = 200000; // 最大脉冲限制(2.0ms)

public:
    MotionController()
    {   //设备初始化
        pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
        pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);  //获取PWM设备信息
        pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
        gpio_set_level(MOTOR1_DIR, 0);
        gpio_set_level(MOTOR2_DIR, 1);
        // 初始化PID参数
        init_pid(pidLeft, 0.8f, 0.02f, 0.1f, PWM_MAX,DELTA_PID);
        init_pid(pidRight, 0.8f, 0.02f, 0.1f, PWM_MAX,DELTA_PID);
        init_pid(pidservo, 0.08f, 0.0f, 0.1f, 45.0f, POSITION_PID);
    }

    void pit_callback()
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
        encoder_left = encoder_get_count(ENCODER_1);
        encoder_right = encoder_get_count(ENCODER_2);
}

    void motor_control(int speed, float k, int limit) {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        // 获取编码器值
        pidLeft.actual =  abs(encoder_left);
        pidRight.actual = abs(encoder_right);

        cerr << "left: " << pidLeft.actual << " right: " << pidRight.actual << endl;
        // PID计算
        float outL = calculate_pid(pidLeft, speed);
        float outR = calculate_pid(pidRight, speed);

        outL = std::clamp(abs(outL), 0.0f, static_cast<float>(PWM_MAX));
        outR = std::clamp(abs(outR), 0.0f, static_cast<float>(PWM_MAX));
  
        // 设置电机输出
        pwm_set_duty(MOTOR1_PWM, outL);
        pwm_set_duty(MOTOR2_PWM, outR);
        cerr << "dutyl: " << outL <<"dutyr: " <<  outR << endl;
        cerr << "Tgt:" << speed 
        << " L:" << outL << "/" << pidLeft.actual
        << " R:" << outR << "/" << pidRight.actual << endl;
    }
    void set_servo_angle(int error) {
        // 死区处理 (±4像素不响应)
        if (abs(error) < 4) error = 0;
        
        // 非线性调节（大偏差时增强响应）
        float factor = 1.0f;
        if (abs(error) > 40) factor = 1.5f;
        
        // 设置PID目标为0（期望error归零），实际值为处理后的误差
        pidservo.target = 0;
        pidservo.actual = error * factor;
        
        // 计算PID输出（角度调整量）
        float angle_adjustment = calculate_pid(pidservo, pidservo.target);
        
        // 计算目标角度（中位90度 ± 调整量）并约束范围
        float target_angle = 90.0f + angle_adjustment;
        target_angle = std::clamp(target_angle, 
                                 static_cast<float>(SERVO_MOTOR_L_MAX), 
                                 static_cast<float>(SERVO_MOTOR_R_MAX));
        
        // 转换为舵机占空比并设置PWM
        uint16_t duty = static_cast<uint16_t>(SERVO_MOTOR_DUTY(target_angle));
        pwm_set_duty(SERVO_MOTOR1_PWM, duty);
        cerr << "servoduty: " << duty << endl;

    }
    
private:
    void init_pid(PID_Controller& pid, float Kp, float Ki, float Kd, float max_out, PID_Mode mode = POSITION_PID) {
        pid.Kp = Kp;
        pid.Ki = Ki;
        pid.Kd = Kd;
        pid.max_output = max_out;
        pid.integral = 0;
        memset(pid.error, 0, sizeof(pid.error));
        pid.mode = mode;
        cerr<<"init pid"<<endl;
    }

    float calculate_pid(PID_Controller& pid, float target) {
        float error = target - pid.actual;
        if(abs(error) > 3000)
        {
            error = 0;
        }
        if (pid.mode == POSITION_PID) {
            // 位置式PID
            pid.integral += error;
            pid.integral = std::clamp(pid.integral, -pid.max_output/pid.Ki, pid.max_output/pid.Ki);
            
            float P = pid.Kp * error;
            float I = pid.Ki * pid.integral;
            float D = pid.Kd * (error - pid.error[0]); // 微分项使用当前误差与上一次误差的差
            
            pid.output = P + I + D;
            pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
            
            // 更新误差记录（保存当前误差供下次使用）
            pid.error[0] = error;
        } 
        else if (pid.mode == DELTA_PID) {
            // 增量式PID: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
            
            // 更新误差历史
            pid.error[2] = pid.error[1];
            pid.error[1] = pid.error[0];
            pid.error[0] = error;
            
            // 计算增量
            float delta = pid.Kp * (pid.error[0] - pid.error[1])
                        + pid.Ki * pid.error[0]
                        + pid.Kd * (pid.error[0] - 2*pid.error[1] + pid.error[2]);
            
            // 叠加输出并限幅
            pid.output += delta;
            pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
        }

        return pid.output;
    }
};
void cleanup();
void sigint_handler(int signum);

#endif
