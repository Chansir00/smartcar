#ifndef __PID_H__
#define __PID_H__
#include <fcntl.h>       // 解决O_RDWR等文件控制定义
#include <termios.h>     // 解决termios结构体和串口配置相关定义
#include <unistd.h>      // 解决write/close等POSIX函数
#include <chrono>        // 解决std::chrono相关时间处理
#include <zf_common_headfile.h>
#include <string>
#include <algorithm>
#include <mutex>
#include <iostream>
#include <cmath>
#include <cstdint>
#include "cameratest.h"

constexpr uint8_t weight[39] = {  // 将数组长度调整为38
    1,3,11,13,13,15,15,              //0-7
    15,17,17,19,19,20,20,17,9, //8-16
    9,7,7,5,5,3,3,1,1,   //17-25
    1,1,1,1,1,1,1,1,1,3,4,5,6,1        //26-39     
};

using namespace std;
//+-15测误差变化率范围
#define PB 3  // 正大
#define PM 2  // 正中
#define PS 1  // 正小
#define ZO 0  // 中
#define NS -1 // 负小
#define NM -2 // 负中
#define NB -3 // 负大
#define EC_FACTOR 1
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

// 模糊规则表   kp kd
static const int rule_p[7][7] = { 
    {NB,NB,NM,NM,NS,ZO,ZO},
    {NB,NB,NM,NS,NS,ZO,NS},
    {NM,NM,NM,NS,ZO,NS,NS},
    {NM,NM,NS,ZO,NS,NM,NM},
    {NS,NS,ZO,NS,NS,NM,NM},
    {NS,ZO,NS,NM,NM,NM,NB},
    {ZO,ZO,NM,NM,NM,NB,NB} 
};
static const int rule_d[7][7] = { 
    {PS,NS,NB,NB,NB,NM,PS},
    {PS,NS,NB,NM,NM,NS,ZO},
    {ZO,NS,NM,NM,NS,NS,ZO},
    {ZO,NS,NS,NS,NS,NS,ZO},
    {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
    {PB,NS,PS,PS,PS,PS,PB},
    {PB,PM,PM,PM,PS,PS,PB} 
};

typedef struct {
    float Kp0;           // 小弯kp
    float Kd0;           // 小弯kd
    float threshold;     // 输出变化阈值
    float maximum;       // kp max
    float minimum;       // kp min
    float factor;        // 误差缩放因子（匹配EFF范围）error*factor落在EFF
    float SetValue;      // 目标值 0
    float CurrentValue;  // 当前值 160-wide error
    float err;           // 当前误差
    float errlast;       // 上次误差
    float out;           // 输出值
    float outlast;       // 上次输出
} Fuzzy_PD_t;

typedef struct {
    float EF[2];        // 误差隶属度
    int En[2];          // 误差规则索引
    float DF[2];        // 误差变化率隶属度
    int Dn[2];          // 误差变化率规则索引
} DMF_t;

typedef struct {
    float UFF_P[7];     // 比例系数隶属函数
    float UFF_D[7];     // 微分系数隶属函数
} UFF_t;

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
#define SERVO_MOTOR_MID             (4333 )    
#define SERVO_MOTOR_L_MAX           (4700 )                       
#define SERVO_MOTOR_R_MAX           (3900)
//#define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
extern struct pwm_info servo_pwm_info;

// 控制参数
constexpr int SERIAL_RETRY_INTERVAL = 50;  // 串口重试间隔(ms)
constexpr int MID_W = 160;          // 图像中线
constexpr int PWM_MAX = 10000;      // 电机最大PWM
constexpr int CONTROL_PERIOD_MS = 10;
extern int16 encoder_left;
extern int16 encoder_right;
extern float servo_motor_duty ;                                                  // 舵机zhongzhi
extern float servo_motor_dir;    

enum PID_Mode {
    POSITION_PID,
    DELTA_PID,
    FUZZY_PID
};

struct PID_Controller {
    float Kp, Ki, Kd;
    float integral = 0;
    float error[3] = {0};
    float output = 0;
    float max_output;
    float target;
    float actual;
    float integral_limit_ratio = 0.6f;  // 积分限幅比例
    float filtered_D = 0;               // 滤波后的微分项
    float alpha = 0.6f;                 // 低通滤波系数
    static constexpr int FILTER_WINDOW = 3;  // 滑动窗口大小
    float output_history[FILTER_WINDOW] = {0};
    int history_index = 0;
    PID_Mode mode;

     // 模糊PID扩展参数
     Fuzzy_PD_t fuzzy_pd;
     UFF_t uff;
     DMF_t dmf;
     float EFF[7];       // 误差隶属函数参数      大中小   测误差范围
     float DFF[7];       // 误差变化率隶属函数参数 大中小/2 
     float last_error;   // 用于计算误差变化率
     bool fuzzy_initialized = false;
};

class MotionController {
private:
    int serial_fd = -1;  // 串口文件描述符
    std::chrono::steady_clock::time_point last_serial_retry;  // 最后重试时间
    PID_Controller pidLeft;
    PID_Controller pidRight;
    PID_Controller pidservo;
    int fd_servo;
    std::mutex encoder_mutex;  // 添加互斥锁
    int16 encoder_left = 0;
    int16 encoder_right = 0;
    int car_startline = 115;        // 起始行
    int hope_line = 77;            // 目标行

    void init_serial() {
        serial_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
        if (serial_fd == -1) {
            perror("[UART] Open failed");
            return;
        }
    
        struct termios options;
        tcgetattr(serial_fd, &options);
        cfmakeraw(&options);
        cfsetspeed(&options, B115200);
        
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= (CS8 | CLOCAL | CREAD); // 添加关键标志
        options.c_cc[VTIME] = 1;
        options.c_cc[VMIN]  = 0;
        
        if(tcsetattr(serial_fd, TCSANOW, &options) != 0) {
            perror("[UART] Configure failed");
            close(serial_fd);
            serial_fd = -1;
        }
        
        tcflush(serial_fd, TCIOFLUSH);
        last_serial_retry = std::chrono::steady_clock::now();
        cerr << "Serial initialized. FD: " << serial_fd << endl;
    }

    // 带校验的数据发送
    bool send_packet(const uint8_t* data, size_t len) {
        if(serial_fd == -1) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_serial_retry);
            
            if(elapsed.count() > SERIAL_RETRY_INTERVAL) {
                init_serial();  // 定时重连
                last_serial_retry = now;
            }
            return false;
        }

        uint8_t checksum = 0;
        for(size_t i=0; i<len; ++i) checksum ^= data[i];
        
        // 构造完整数据包：起始码+数据+校验+结束码
        uint8_t packet[len + 4];
        packet[0] = 0x03;       // 帧头
        packet[1] = 0xFC;       
        memcpy(&packet[2], data, len);
        packet[len+2] = checksum;
        packet[len+3] = 0x03;   // 帧尾

        ssize_t sent = write(serial_fd, packet, sizeof(packet));
        if(static_cast<size_t>(sent) != sizeof(packet)){
            perror("[UART] Write failed");
            close(serial_fd);
            serial_fd = -1;
            return false;
        }
        
        tcdrain(serial_fd);  // 等待数据完全发送
        return true;
    }
    //kp
    float Fuzzy_Kp(PID_Controller& pid) {
        float KpgradSums[7] = {0};
        for (int i=0; i<2; ++i) {
            if (pid.dmf.En[i] == -1) continue;
            for (int j=0; j<2; ++j) {
                if (pid.dmf.Dn[j] == -1) continue;
                int rule = rule_p[pid.dmf.En[i]][pid.dmf.Dn[j]];
                KpgradSums[rule+3] += pid.dmf.EF[i] * pid.dmf.DF[j];
            }
        }
        float sum = 0;
        for (int i=0; i<7; ++i)
            sum += pid.uff.UFF_P[i] * KpgradSums[i];
        return sum;
    }
    //kd
    float Fuzzy_Kd(PID_Controller& pid) {
        float KdgradSums[7] = {0};
        for (int i=0; i<2; ++i) {
            if (pid.dmf.En[i] == -1) continue;
            for (int j=0; j<2; ++j) {
                if (pid.dmf.Dn[j] == -1) continue;
                int rule = rule_d[pid.dmf.En[i]][pid.dmf.Dn[j]];
                KdgradSums[rule+3] += pid.dmf.EF[i] * pid.dmf.DF[j];
            }
        }
        float sum = 0;
        for (int i=0; i<7; ++i)
            sum += pid.uff.UFF_D[i] * KdgradSums[i];
        return sum;
    }

    void count_DMF(PID_Controller& pid, float e, float ec) {
        // 误差隶属度计算
        if (e > pid.EFF[0] && e < pid.EFF[6]) {
            for (int i = 0; i < 6; ++i) {
                if (e >= pid.EFF[i] && e <= pid.EFF[i+1]) {
                    pid.dmf.EF[0] = (pid.EFF[i+1] - e) / (pid.EFF[i+1] - pid.EFF[i]);
                    pid.dmf.EF[1] = (e - pid.EFF[i]) / (pid.EFF[i+1] - pid.EFF[i]);
                    pid.dmf.En[0] = i;
                    pid.dmf.En[1] = i+1;
                    break;
                }
            }
        } 
    
        else {
            pid.dmf.En[0] = (e <= pid.EFF[0]) ? 0 : 6;
            pid.dmf.En[1] = -1;
            pid.dmf.EF[0] = 1.0f;
        }

        // 误差变化率隶属度计算
        if (ec > pid.DFF[0] && ec < pid.DFF[6]) {
            for (int i = 0; i < 6; ++i) {
                if (ec >= pid.DFF[i] && ec <= pid.DFF[i+1]) {
                    pid.dmf.DF[0] = (pid.DFF[i+1] - ec) / (pid.DFF[i+1] - pid.DFF[i]);
                    pid.dmf.DF[1] = (ec - pid.DFF[i]) / (pid.DFF[i+1] - pid.DFF[i]);
                    pid.dmf.Dn[0] = i;
                    pid.dmf.Dn[1] = i+1;
                    break;
                }
            }
        } 
        else {
            pid.dmf.Dn[0] = (ec <= pid.DFF[0]) ? 0 : 6;
            pid.dmf.Dn[1] = -1;
            pid.dmf.DF[0] = 1.0f;
        }
    }
public :
    MotionController()
    {   //设备初始化
        pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
        pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);  //获取PWM设备信息
        pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
        gpio_set_level(MOTOR1_DIR, 0);
        gpio_set_level(MOTOR2_DIR, 0);     //yuan 1
        // 初始化PID参数
        init_pid(pidLeft, 1.25f, 0.245f, 0.10f, PWM_MAX,DELTA_PID);       //you
        init_pid(pidRight, 1.25f, 0.205f, 0.18f, PWM_MAX,DELTA_PID);    //zuo
        init_pid(pidservo, 3.0f, 0.0f, 0.3f, 367.0f, POSITION_PID);
        init_serial();
    }

    ~MotionController() {
        if(serial_fd != -1) {
            close(serial_fd);
        }
    }

    void datavision_send() {
        // 准备数据：目标速度、实际速度（转换为uint16_t）
        uint8_t data[4]={
        data[0] = static_cast<uint16_t>(pidLeft.target), 
        data[1] = static_cast<uint16_t>(pidLeft.actual),
        data[2] = static_cast<uint16_t>(pidRight.target),
        data[3] = static_cast<uint16_t>(pidRight.actual)
        };

        uint8_t packet[6] = {0x03, 0xFC};
        memcpy(&packet[2], data, 4);
        packet[6] = 0xFC;
        packet[7] = 0x03;
        
        if(write(serial_fd, packet, 8) != 8) {
            perror("Write failed");
        }

    }

    void send_debug2() {
        if (serial_fd == -1) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_serial_retry);
            if (elapsed.count() > SERIAL_RETRY_INTERVAL) {
                init_serial();
                last_serial_retry = now;
            }
            if (serial_fd == -1) return;
        }
        // 格式化字符串（假设actual/target是float类型）
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "%.2f,%.2f\n",
                          pidRight.actual,
                          300.0f);

                          if (len < 0 || len >= static_cast<int>(sizeof(buffer))) {
                            return; // 格式化错误或缓冲区溢出
                        }
                    
                        const char *ptr = buffer;
                        size_t remaining = len;
                        while (remaining > 0) {
                            ssize_t sent = write(serial_fd, ptr, remaining);
                            if (sent < 0) {
                                perror("[DEBUG2] Write failed");
                                close(serial_fd);
                                serial_fd = -1;
                                return;
                            }
                            remaining -= sent;
                            ptr += sent;
                        }
                    }
    void pit_callback()
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
        encoder_left = encoder_get_count(ENCODER_1);
        encoder_right = encoder_get_count(ENCODER_2);
        //if(abs(encoder_left)>500)
        //{
            //encoder_left = 200 ;
        //}
        //if(abs(encoder_right)>500)
        //{
            //encoder_right = 200 ;
        //}
}
  
    void motor_control(int speed, float k, int limit) {
        std::lock_guard<std::mutex> lock(encoder_mutex);
        // 获取编码器值
        pidLeft.actual =  encoder_left;
        pidRight.actual = -encoder_right;

        //cerr << "left: " << pidLeft.actual << " right: " << pidRight.actual << endl;
        // PID计算
        float raw_outL = calculate_pid(pidLeft, speed);
        float raw_outR = calculate_pid(pidRight, speed);
        gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 0 : 1);  // 假设0为正转
        gpio_set_level(MOTOR2_DIR, (raw_outR >= 0) ? 0 : 1);  // 假设0为转
        float outL = std::clamp(abs(raw_outL), 0.0f, 0.40f*static_cast<float>(PWM_MAX));
        float outR = std::clamp(abs(raw_outR), 0.0f, 0.40f*static_cast<float>(PWM_MAX));
        //float outL = calculate_pid(pidLeft, speed);
        //float outR = calculate_pid(pidRight, speed);

        //outL = std::clamp(abs(outL), 0.0f, static_cast<float>(PWM_MAX));
        //outR = std::clamp(abs(outR), 0.0f, static_cast<float>(PWM_MAX));
  
        // 设置电机输出
        pwm_set_duty(MOTOR1_PWM, outL);
        pwm_set_duty(MOTOR2_PWM, outR);
        //cerr << "dutyl: " << outL <<"dutyr: " <<  outR << endl;
    }
    void set_servo_angle(int error) {
        // 死区处理 (±4像素不响应)
        if(abs(error) < 4) {
            pwm_set_duty(SERVO_MOTOR1_PWM, 4333);
            return;
        }
        // 设置PID目标为0（期望error归零），实际值为处理后的误差
        pidservo.target = 0;
        pidservo.actual = error;
        
        // 计算PID输出（角度调整量）
        float pwm_adjustment = calculate_pid(pidservo, pidservo.target);
        
        // 计算目标角度（中位85度 ± 调整量）并约束范围
        float target_pwm = 4333 + pwm_adjustment;
        target_pwm = std::clamp(target_pwm, static_cast<float>(SERVO_MOTOR_R_MAX), static_cast<float>(SERVO_MOTOR_L_MAX));
        // 转换为舵机占空比并设置PWM
        //uint16_t duty = static_cast<uint16_t>(SERVO_MOTOR_DUTY(target_angle));
        pwm_set_duty(SERVO_MOTOR1_PWM, target_pwm);
        cerr << "pwm" << target_pwm <<endl; 
    }
    float Err_sum(const vector<Point> &centerline)
    {
    int total_steps = 38;
    cerr<<"total_steps: "<<total_steps<<endl;
    int weight_size = sizeof(weight) / sizeof(weight[0]); // 正确计算元素数目
    if (total_steps > weight_size) {
        cerr << "权重数组尺寸不足！" << endl;
        return 0.0f;
    }
        int i = 0;
        float error = 0;
        float weight_count = 0;
        int weight_index = 0;

        for (i = centerline.size()-5;i>=centerline.size()-43;i--)
        {
            cerr<<"centerline[i].x: "<<centerline[i].x<<endl;
            error += (centerline[i].x-80)*weight[weight_index];
            weight_count += weight[weight_index];
            weight_index++;
        }
        error = error/weight_count;
        return error;
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
        
        if (mode == FUZZY_PID) {
            // 初始化模糊参数
            pid.fuzzy_pd = {50.0f, 30.0f, 10, 15, -15, 1.0f};
            float uff_p_max = 21.0f, uff_d_max = 18.0f;
            for(int i=0; i<7; ++i) {
                pid.uff.UFF_P[i] = uff_p_max * (i-3.0f)/3.0f;
                pid.uff.UFF_D[i] = uff_d_max * (i-3.0f)/3.0f;
                pid.EFF[i] = 21.0f * (i-3.0f)/3.0f;//21f误差范围
                pid.DFF[i] = 18.0f * (i-3.0f)/3.0f;//18f变化率范围
            }
            pid.fuzzy_initialized = true;
        }
    }

    float calculate_pid(PID_Controller& pid, float target) {
        float error = target - pid.actual;
        if (error > 3000) {
            error = 6;
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
        if (pid.mode == DELTA_PID) {
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
        else if (pid.mode == FUZZY_PID && pid.fuzzy_initialized) {
            float ec = error - pid.last_error;
            pid.last_error = error;

            // 执行模糊推理
            count_DMF(pid, error*pid.fuzzy_pd.factor, ec*pid.fuzzy_pd.factor*EC_FACTOR);
            float delta_kp = Fuzzy_Kp(pid);
            float delta_kd = Fuzzy_Kd(pid);

            // 计算最终输出
            float output = (pid.fuzzy_pd.Kp0 + delta_kp) * error / 100.0f;
            output += (pid.fuzzy_pd.Kd0 + delta_kd) * ec / 100.0f;
            output *= -0.7f;

            // 输出限幅
            output = clamp(output, pid.fuzzy_pd.minimum, pid.fuzzy_pd.maximum);
            return output;
        } 

        return pid.output;
    }
};
void cleanup();
void sigint_handler(int signum);

#endif