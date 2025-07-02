
#ifndef __PID_H__
#define __PID_H__
#include <camera.hpp>
#include <fcntl.h>   // 解决O_RDWR等文件控制定义
#include <termios.h> // 解决termios结构体和串口配置相关定义
#include <unistd.h>  // 解决write/close等POSIX函数
#include <chrono>    // 解决std::chrono相关时间处理
#include <zf_common_headfile.h>
#include <string>
#include <algorithm>
#include <mutex>
#include <iostream>
#include <cmath>
#include <cstdint>
#include <memory> // 添加智能指针支持
#include <cmath>
class LaneProcessor;
// 车尾向前
//60-90
constexpr float weight[39] = {    //50
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           //60
    1, 1, 1, 2, 3, 3, 3, 15, 16, 16,     //70
    16, 16, 18, 18, 20 ,20, 20, 23, 23, 23,   //80
    25, 25, 17, 17, 6, 5, 4, 3, 2};  //89

constexpr float weight1[39] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 13, 13, 13, 13, 15,
    17, 17, 15, 15, 15, 15, 19, 19, 11, 13, 15,
    17, 19, 20, 20, 1, 1, 1, 1, 1, 1};

// 1,1,1,1,1,1,1,              //0-7
// 1,1,1,1,1,12,12,14,15, //8-16
// 19,17,17,15,15,13,13,1,1,   //17-25
// 1,1,1,1,1,1,1,1,1,1,1,1,1,1        //26-39

// 3,3,5,7,9,11,13,              //0-7
// 15,17,17,19,19,20,20,19,9, //8-16
// 9,7,7,5,5,3,3,10,10,   //17-25
// 1,1,1,1,1,1,1,1,1,3,4,5,6,1        //26-39

// 1,1,1,1,1,1,1,1,1,
// 3,3,3,3,11,13,13,15,
// 17,17,19,19,20,20,19,19,11,13,15,
// 17,19,20,20,19,15,10,5,3,1

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
    //      NB      NM      NS      ZO      PS      PM      PB
    /*NB*/ {PB,     PB,     PM,     PM,     PS,     ZO,     ZO},
    /*NM*/ {PB,     PM,     PM,     PS,     ZO,     ZO,     PS},
    /*NS*/ {PM,     PM,     PS,     ZO,     ZO,     PS,     PM},
    /*ZO*/ {PS,     PS,     ZO,     PS,     PS,     PS,     PS},  // ZO可以考虑设为PM来提升响应
    /*PS*/ {PM,     PS,     ZO,     PS,     PS,     PM,     PM},
    /*PM*/ {ZO,     ZO,     PS,     PM,     PM,     PM,     PB},
    /*PB*/ {PB,     PM,     PM,     PM,     PM,     PB,     PB}

    // // 误差\误差变化率 | NB   NM    NS    ZO    PS    PM    PB
    // /* NB */ {PB, PB, PM, PM, PS, ZO, ZO},
    // /* NM */ {PB, PB, PM, PS, PS, ZO, PS},
    // /* NS */ {PM, PM, PM, PS, ZO, PS, PS},
    // /* ZO */ {PS, PS, PS, ZO, PS, PS, PS}, //{PM, PM, PS, ZO, PS, PM, PM},
    // /* PS */ {PS, PS, ZO, PS, PS, PM, PM},
    // /* PM */ {ZO, ZO, PS, PM, PM, PM, PB},
    // /* PB */ {PB, PB, PB, PB, PM, PB, PB}
    // // 误差\误差变化率 | NB   NM    NS    ZO    PS    PM    PB
    // /* NB (极左) */ {PB, PB, PM, PM, PS, ZO, ZO},  // 需强右转（负输出）
    // /* NM (中左) */ {PB, PB, PM, PS, PS, ZO, NS},
    // /* NS (微左) */ {PM, PM, PS, PS, ZO, NS, NM},
    // /* ZO (中线) */ {PS, PS, ZO, ZO, ZO, NS, NS}, // 对称修正
    // /* PS (微右) */ {NM, NS, ZO, NS, NS, NM, NM},
    // /* PM (中右) */ {NM, NM, NS, NM, NM, NB, NB},
    // /* PB (极右) */ {ZO, ZO, NM, NM, NM, NB, NB}  // 需强左转（正输出）
};
static const int rule_d[7][7] = {
    {PS, NS, NB, NB, NB, NM, PS},
    {PS, NS, NB, NM, NM, NS, ZO},
    {ZO, NS, NM, NM, NS, NS, ZO},
    {ZO, NS, NS, NS, NS, NS, ZO},
    {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    {PB, NS, PS, PS, PS, PS, PB},
    {PB, PM, PM, PM, PS, PS, PB}};

typedef struct
{
    float Kp0;          // 小弯kp
    float Kd0;          // 小弯kd
    float threshold;    // 输出变化阈值
    float maximum;      // kp max
    float minimum;      // kp min
    float factor;       // 误差缩放因子（匹配EFF范围）error*factor落在EFF
    float SetValue;     // 目标值 0
    float CurrentValue; // 当前值 160-wide error
    float err;          // 当前误差
    float errlast;      // 上次误差
    float out;          // 输出值
    float outlast;      // 上次输出
} Fuzzy_PD_t;

typedef struct
{
    float EF[2]; // 误差隶属度
    int En[2];   // 误差规则索引
    float DF[2]; // 误差变化率隶属度
    int Dn[2];   // 误差变化率规则索引
} DMF_t;

typedef struct
{
    float UFF_P[7]; // 比例系数隶属函数
    float UFF_D[7]; // 微分系数隶属函数
} UFF_t;

// motorl
#define MOTOR1_DIR "/dev/zf_driver_gpio_motor_1"
#define MOTOR1_PWM "/dev/zf_device_pwm_motor_1"
// motorr
#define MOTOR2_DIR "/dev/zf_driver_gpio_motor_2"
#define MOTOR2_PWM "/dev/zf_device_pwm_motor_2"
// encoder
#define ENCODER_1 "/dev/zf_encoder_1"
#define ENCODER_2 "/dev/zf_encoder_2"
// servo
#define SERVO_MOTOR1_PWM "/dev/zf_device_pwm_servo"
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX (motor_1_pwm_info.duty_max)
#define MOTOR2_PWM_DUTY_MAX (motor_2_pwm_info.duty_max)
#define SERVO_MOTOR_FREQ (servo_pwm_info.freq)
#define PWM_DUTY_MAX (servo_pwm_info.duty_max)
#define SERVO_MOTOR_MID (4360)
#define SERVO_MOTOR_L_MAX (4800) // 4840
#define SERVO_MOTOR_R_MAX (3940) // 3860
//((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))
// #define SERVO_MOTOR_DUTY(x) ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
extern struct pwm_info servo_pwm_info;

// 控制参数
constexpr int SERIAL_RETRY_INTERVAL = 50; // 串口重试间隔(ms)
constexpr int MID_W = 160;                // 图像中线
constexpr int PWM_MAX = 10000;            // 电机最大PWM
constexpr int CONTROL_PERIOD_MS = 10;
extern int16 encoder_left;
extern int16 encoder_right;
extern float servo_motor_duty; // 舵机zhongzhi
extern float servo_motor_dir;
extern int control_circle; // 环岛状态
enum PID_Mode
{
    POSITION_PID,
    DELTA_PID,
    FUZZY_PID
};

struct PID_Controller
{
    float Kp, Ki, Kd;
    float integral = 0;
    float error[3] = {0};
    float output = 0;
    float max_output;
    float target;
    float actual;
    float integral_limit_ratio = 0.6f; // 积分限幅比例
    float filtered_D = 0;              // 滤波后的微分项

    static constexpr int FILTER_WINDOW = 3; // 滑动窗口大小
    float output_history[FILTER_WINDOW] = {0};
    int history_index = 0;
    PID_Mode mode;

    // 模糊PID扩展参数
    Fuzzy_PD_t fuzzy_pd;
    UFF_t uff;
    DMF_t dmf;
    float EFF[7];     // 误差隶属函数参数      大中小   测误差范围
    float DFF[7];     // 误差变化率隶属函数参数 大中小/2
    float last_error; // 用于计算误差变化率
    bool fuzzy_initialized = false;
};

class MotionController
{
public:
    friend class LaneProcessor;
    LaneProcessor *laneProcessor;                            // 改用原始指针
    int serial_fd = -1;                                      // 串口文件描述符
    std::chrono::steady_clock::time_point last_serial_retry; // 最后重试时间
    PID_Controller pidLeft;
    PID_Controller pidRight;
    PID_Controller pidservo;
    int fd_servo;
    std::mutex encoder_mutex; // 添加互斥锁
    int16 encoder_left = 0;
    int16 encoder_right = 0;
    int car_startline = 100; // 起始行
    int hope_line = 62;      // 目标行

    float lsd_p = 0.35f;
    float lsd_pl = 0.2f; // 滞后比例系数
    float lsd_d = 0.4f;
    int16_t encoder_err_last = 0;

    float Speed_Goal = 0.0f;
    const uint16_t steer_middle = 4360;     // 舵机中位PWM值
    const float Left_Speed = 1.2f;          // 左转差速系数
    const float Right_Speed = 1.1f;         // 右转差速系数
    float current_servo_pwm = steer_middle; // 当前舵机PWM
    const float ackerman_limit = 0.1f;      // 0.3f; // 差速限幅系数

    float lim_cs = 0.9;  // 差速限幅
    float chasu_k = 1.0; // 差速
    float p1 = 0;
    float i1 = 0;
    float d1 = 0;

    float shared_error;
    std::mutex error_mutex; // 新增互斥锁声明

    void init_serial()
    {
        serial_fd = open("/dev/ttyS0", O_RDWR | O_NOCTTY);
        if (serial_fd == -1)
        {
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
        options.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd, TCSANOW, &options) != 0)
        {
            perror("[UART] Configure failed");
            close(serial_fd);
            serial_fd = -1;
        }

        tcflush(serial_fd, TCIOFLUSH);
        last_serial_retry = std::chrono::steady_clock::now();
        cerr << "Serial initialized. FD: " << serial_fd << endl;
    }

    // 带校验的数据发送
    bool send_packet(const uint8_t *data, size_t len)
    {
        if (serial_fd == -1)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_serial_retry);

            if (elapsed.count() > SERIAL_RETRY_INTERVAL)
            {
                init_serial(); // 定时重连
                last_serial_retry = now;
            }
            return false;
        }

        uint8_t checksum = 0;
        for (size_t i = 0; i < len; ++i)
            checksum ^= data[i];

        // 构造完整数据包：起始码+数据+校验+结束码
        uint8_t packet[len + 4];
        packet[0] = 0x03; // 帧头
        packet[1] = 0xFC;
        memcpy(&packet[2], data, len);
        packet[len + 2] = checksum;
        packet[len + 3] = 0x03; // 帧尾

        ssize_t sent = write(serial_fd, packet, sizeof(packet));
        if (static_cast<size_t>(sent) != sizeof(packet))
        {
            perror("[UART] Write failed");
            close(serial_fd);
            serial_fd = -1;
            return false;
        }

        tcdrain(serial_fd); // 等待数据完全发送
        return true;
    }
    // kp
    float Fuzzy_Kp(PID_Controller &pid)
    {
        float KpgradSums[7] = {0};
        for (int i = 0; i < 2; ++i)
        {
            if (pid.dmf.En[i] == -1)
                continue;
            for (int j = 0; j < 2; ++j)
            {
                if (pid.dmf.Dn[j] == -1)
                    continue;
                int rule = rule_p[pid.dmf.En[i]][pid.dmf.Dn[j]];
                KpgradSums[rule + 3] += pid.dmf.EF[i] * pid.dmf.DF[j];
            }
        }
        float sum = 0;
        for (int i = 0; i < 7; ++i)
            sum += pid.uff.UFF_P[i] * KpgradSums[i];
        return sum;
    }
    // kd
    float Fuzzy_Kd(PID_Controller &pid)
    {
        float KdgradSums[7] = {0};
        for (int i = 0; i < 2; ++i)
        {
            if (pid.dmf.En[i] == -1)
                continue;
            for (int j = 0; j < 2; ++j)
            {
                if (pid.dmf.Dn[j] == -1)
                    continue;
                int rule = rule_d[pid.dmf.En[i]][pid.dmf.Dn[j]];
                KdgradSums[rule + 3] += pid.dmf.EF[i] * pid.dmf.DF[j];
            }
        }
        float sum = 0;
        for (int i = 0; i < 7; ++i)
            sum += pid.uff.UFF_D[i] * KdgradSums[i];
        return sum;
    }

    void count_DMF(PID_Controller &pid, float e, float ec)
    {
        // 误差隶属度计算
        if (e > pid.EFF[0] && e < pid.EFF[6])

        {
            for (int i = 0; i < 6; ++i)
            {
                if (e >= pid.EFF[i] && e <= pid.EFF[i + 1])
                {
                    pid.dmf.EF[0] = (pid.EFF[i + 1] - e) / (pid.EFF[i + 1] - pid.EFF[i]);
                    pid.dmf.EF[1] = (e - pid.EFF[i]) / (pid.EFF[i + 1] - pid.EFF[i]);
                    pid.dmf.En[0] = i;
                    pid.dmf.En[1] = i + 1;
                    break;
                }
            }
        }

        else
        {
            pid.dmf.En[0] = (e <= pid.EFF[0]) ? 0 : 6;
            pid.dmf.En[1] = -1;
            pid.dmf.EF[0] = 1.0f;
        }

        // 误差变化率隶属度计算
        if (ec > pid.DFF[0] && ec < pid.DFF[6])
        {
            for (int i = 0; i < 6; ++i)
            {
                if (ec >= pid.DFF[i] && ec <= pid.DFF[i + 1])
                {
                    pid.dmf.DF[0] = (pid.DFF[i + 1] - ec) / (pid.DFF[i + 1] - pid.DFF[i]);
                    pid.dmf.DF[1] = (ec - pid.DFF[i]) / (pid.DFF[i + 1] - pid.DFF[i]);
                    pid.dmf.Dn[0] = i;
                    pid.dmf.Dn[1] = i + 1;
                    break;
                }
            }
        }
        else
        {
            pid.dmf.Dn[0] = (ec <= pid.DFF[0]) ? 0 : 6;
            pid.dmf.Dn[1] = -1;
            pid.dmf.DF[0] = 1.0f;
        }
    }

    // void ackerman_diff_control() {
    //     double y, x;
    //     const float encoder_scale = 0.51619f;
    //     const float poly_coeff[3] = {-0.014344f, 0.0078637f, -0.000014484f};
    //     const float speed_factor =  0.15f;//0.3875f;

    //     if(current_servo_pwm >= steer_middle) { // 左转
    //         x = (current_servo_pwm - steer_middle) * Left_Speed * encoder_scale;
    //         x = std::min(x, 367.0);
    //         y = poly_coeff[0] + poly_coeff[1]*x + poly_coeff[2]*x*x;

    //         pidRight.target = Speed_Goal * (1 + speed_factor * y* ackerman_limit);
    //         pidLeft.target = Speed_Goal * (1 - speed_factor * y );
    //     } else { // 右转 pwm-
    //         x = (steer_middle - current_servo_pwm) * Right_Speed * encoder_scale;
    //         x = std::min(x, 433.0);
    //         y = poly_coeff[0] + poly_coeff[1]*x + poly_coeff[2]*x*x;

    //         pidLeft.target = Speed_Goal * (1 + speed_factor * y* ackerman_limit);
    //         pidRight.target = Speed_Goal * (1 - speed_factor * y);
    //     }
    // }
    void ackerman_diff_control(int Speed_Goal)
    {
        // A 1.05 //差速的大小系数
        // K 1.06 //差速的预知系数  K越大差速越提前
        // AK 1
        const float L = 200.0f, W = 150.0f;
        // float angle1 = (current_servo_pwm / 3000 /0.96 - 0.5)*90;
        // float angle2 = abs(angle1 - 90);g
        // angle2 = max(0.0f, min(angle2, 89.9f));
        float angle1 = abs((4360 - current_servo_pwm) * 0.09278f);
        float angle_rad = angle1 * M_PI / 180.0f;
        float tn = tan(angle_rad);
        //cerr << "tn" << tn << endl;
        // 基础几何模型
        float outer_factor = 1.0f + 0.17 * (W / 2) * tn / L; // 0.10
        float inner_factor = 1.0f - 0.83 * (W / 2) * tn / L; // 0.90

        inner_factor = max(0.8f, min(inner_factor, 1.2f)); // 限制在 0.9~1.1 范围内
        outer_factor = max(0.8f, min(outer_factor, 1.2f)); // 可选：对称限幅
        //cerr << "in:" << inner_factor << endl;
        //cerr << "out:" << outer_factor << endl;
        // float outer_factor = (A*(K+0.5*W*tn/L*AK));
        // float inner_factor = (A*(K-0.5*W*tn/L*AK));
        //  应用差速
        if (current_servo_pwm >= steer_middle)
        {
            pidLeft.target = Speed_Goal * inner_factor;
            pidRight.target = Speed_Goal * outer_factor;
        }
        else
        {
            pidLeft.target = Speed_Goal * outer_factor;
            pidRight.target = Speed_Goal * inner_factor;
        }
    }

public:
    explicit MotionController(LaneProcessor *lp) : laneProcessor(lp)
    {   // 通过构造函数注入实例
        // MotionController();
        //  {   //设备初始化
        pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
        pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info); // 获取PWM设备信息
        pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
        gpio_set_level(MOTOR1_DIR, 0);
        gpio_set_level(MOTOR2_DIR, 0); // yuan 1
        // 初始化PID参数
         init_pid(pidLeft, 28.0f, 2.845f, -0.2f, PWM_MAX,DELTA_PID);       //you  10.0 1.245
         init_pid(pidRight, 30.0f, 2.805f, 0.18f, PWM_MAX,DELTA_PID);    //zuo
        //init_pid(pidLeft, 2.0f, 2.0f, 0.0f, PWM_MAX, DELTA_PID);  // you
        //init_pid(pidRight, 2.0f, 2.0f, 0.0f, PWM_MAX, DELTA_PID); // zuo
        init_pid(pidservo, 6.0f, 0.0f, 14.0f, 441.0f, FUZZY_PID);   //
        init_serial();
    }

    ~MotionController()
    {
        if (serial_fd != -1)
        {
            close(serial_fd);
        }
    }

    void datavision_send()
    {
        // 准备数据：目标速度、实际速度（转换为uint16_t）
        uint8_t data[4] = {
            data[0] = static_cast<uint16_t>(pidLeft.target),
            data[1] = static_cast<uint16_t>(pidLeft.actual),
            data[2] = static_cast<uint16_t>(pidRight.target),
            data[3] = static_cast<uint16_t>(pidRight.actual)};

        uint8_t packet[6] = {0x03, 0xFC};
        memcpy(&packet[2], data, 4);
        packet[6] = 0xFC;
        packet[7] = 0x03;

        if (write(serial_fd, packet, 8) != 8)
        {
            perror("Write failed");
        }
    }

    void send_debug2()
    {
        if (serial_fd == -1)
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_serial_retry);
            if (elapsed.count() > SERIAL_RETRY_INTERVAL)
            {
                init_serial();
                last_serial_retry = now;
            }
            if (serial_fd == -1)
                return;
        }
        // 格式化字符串（假设actual/target是float类型）
        char buffer[64];
        int len = snprintf(buffer, sizeof(buffer), "%.2f,%.2f\n",
                           pidRight.actual,
                           300.0f);

        if (len < 0 || len >= static_cast<int>(sizeof(buffer)))
        {
            return; // 格式化错误或缓冲区溢出
        }

        const char *ptr = buffer;
        size_t remaining = len;
        while (remaining > 0)
        {
            ssize_t sent = write(serial_fd, ptr, remaining);
            if (sent < 0)
            {
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
        // if(abs(encoder_left)>500)
        //{
        // encoder_left = 200 ;
        //}
        // if(abs(encoder_right)>500)
        //{
        // encoder_right = 200 ;
        //}
    }

    void motor_control(int speed, float k, int limit);
    void set_servo_angle(int error)
    {
        // 死区处理 (±4像素不响应)
        static int error_last = 0;
        // if(abs(error_last - error) >30) error = error_last;

        if (abs(error) < 4)
        {
            pwm_set_duty(SERVO_MOTOR1_PWM, 4360);
            error_last = error;
            return;
        }
        // 设置PID目标为0（期望error归零），实际值为处理后的误差
        pidservo.target = 0;
        pidservo.actual = error;

        // 计算PID输出（角度调整量）
        float pwm_adjustment = calculate_pid(pidservo, pidservo.target);

        // 计算目标角度（中位85度 ± 调整量）并约束范围
        float target_pwm = 4360 + pwm_adjustment;
        target_pwm = std::clamp(target_pwm, static_cast<float>(SERVO_MOTOR_R_MAX), static_cast<float>(SERVO_MOTOR_L_MAX));
        // 转换为舵机占空比并设置PWM
        // uint16_t duty = static_cast<uint16_t>(SERVO_MOTOR_DUTY(target_angle));
        pwm_set_duty(SERVO_MOTOR1_PWM, target_pwm);
        cerr << "pwm" << target_pwm << endl;
        current_servo_pwm = target_pwm;
        error_last = error;
    }
    float Err_sum(const vector<Point> &centerline);
    void update_shared_error(float err);
    float get_shared_error();

private:
    void init_pid(PID_Controller &pid, float Kp, float Ki, float Kd, float max_out, PID_Mode mode = POSITION_PID)
    {
        pid.Kp = Kp;
        pid.Ki = Ki;
        pid.Kd = Kd;
        pid.max_output = max_out;
        pid.integral = 0;
        memset(pid.error, 0, sizeof(pid.error));
        pid.mode = mode;

        if (mode == FUZZY_PID)
        {
            // 初始化模糊参数
            pid.fuzzy_pd = {6.0f, 28.0f, 1.0, 35.0, 6.0, 1.0f};
            /* Kp0 */
            /* Kd0 */
            /* deltakp threshold */
            /* maximum */
            /* minimum */
            /* factor */ // 假设误差范围为 ±160 像素，缩放因子 factor=0.5 后为 ±80
            float uff_p_max = 12.0f, uff_d_max = 40.0f;
            for (int i = 0; i < 7; ++i)
            {
                pid.uff.UFF_P[i] = uff_p_max * (i - 3.0f) / 3.0f;
                pid.uff.UFF_D[i] = uff_d_max * (i - 3.0f) / 3.0f;
                pid.EFF[i] = 60.0f * (i - 3.0f) / 3.0f; // 21f误差范围
                if (i == 3)
                    pid.EFF[i] = 20.0f;
                pid.DFF[i] = 20.0f * (i - 3.0f) / 3.0f; // 18f变化率范围
            }
            pid.fuzzy_initialized = true;
        }
    }

    float calculate_pid(PID_Controller &pid, float target)
    {
        float error = target - pid.actual;
        float alpha = 0.95f;                 // 低通滤波系数
        static float filtered_output = 0.0f; // 滤波后的输出
        if (error > 4000)
        {
            error = 6;
        }
        if (pid.mode == POSITION_PID)
        {
            // 位置式PID
            pid.integral += error;
            pid.integral = std::clamp(pid.integral, -pid.max_output / pid.Ki, pid.max_output / pid.Ki);

            float P = pid.Kp * error;
            float I = pid.Ki * pid.integral;
            float D = pid.Kd * (error - pid.error[0]); // 微分项使用当前误差与上一次误差的差

            pid.output = P + I + D;
            pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);

            // 更新误差记录（保存当前误差供下次使用）
            pid.error[0] = error;
        }
        if (pid.mode == DELTA_PID)
        {
            // 增量式PID: Δu = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))

            // 更新误差历史
            pid.error[2] = pid.error[1];
            pid.error[1] = pid.error[0];
            pid.error[0] = error;

            // 计算增量
            float delta = pid.Kp * (pid.error[0] - pid.error[1]) + pid.Ki * pid.error[0] + pid.Kd * (pid.error[0] - 2 * pid.error[1] + pid.error[2]);

            // 叠加输出并限幅
            pid.output += delta;
            pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
        }

        else if (pid.mode == FUZZY_PID && pid.fuzzy_initialized)
        {
            float ec = error - pid.last_error;
            float A = 0.03;
            pid.last_error = error;
            float error_abs = fabs(error);
            // pid.fuzzy_pd.Kp0 = error_abs > 20 ? 10.0f : 4.0f;
            //  执行模糊推理
            count_DMF(pid, error * pid.fuzzy_pd.factor, ec * pid.fuzzy_pd.factor * EC_FACTOR);
            float delta_kp = Fuzzy_Kp(pid);
            //cerr << "ΔKp:" << delta_kp << endl;
            float delta_kd = Fuzzy_Kd(pid);
            //cerr << "ΔKd:" << delta_kd << endl;

            // 计算最终输出
            float P = (pid.fuzzy_pd.Kp0 + delta_kp) * error; // 直接使用模糊调整后的Kp
            // float P = pid.fuzzy_pd.Kp0 + abs(error)*delta_kp * error + ;
            // float D = pid.Kd * ec;
            float D = (pid.fuzzy_pd.Kd0 + delta_kd) * ec;
            //cerr << "kd0:" << pid.fuzzy_pd.Kd0 <<endl;
            float output = 0;
            if (error_abs >= 15)
            {
                output = error * (pid.fuzzy_pd.Kp0 + A * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0; //(pid.fuzzy_pd.Kd0 + delta_kd); // imu963ra_gyro_z * delta_kd;
            }
            else if(error_abs <15 || control_circle ==13 ||control_circle==1||control_circle==6)
            {
                output = error * (pid.fuzzy_pd.Kp0 + 0.1 * delta_kp) + ec * (pid.fuzzy_pd.Kd0 + delta_kd);//pid.fuzzy_pd.Kd0; //(pid.fuzzy_pd.Kd0 + delta_kd); // imu963ra_gyro_z * delta_kd;
                cerr<<"output"<<output<<endl;
            }
            // float output = (pid.fuzzy_pd.Kp0 + delta_kp) * error / 100.0f;
            // output += (pid.fuzzy_pd.Kd0 + delta_kd) * ec / 100.0f;
            // output *= -0.7f;
            //  输出限幅
            filtered_output = alpha * output + (1.0f - alpha) * filtered_output;
            filtered_output = clamp(filtered_output, -pid.max_output, pid.max_output);
            // output = clamp(output, -pid.max_output, pid.max_output);

            return filtered_output;
        }

        return pid.output;
    }

    void LSD_Control()
    {
        int16_t encoder_err = encoder_left - encoder_right;
        int16_t encoder_det = encoder_err - encoder_err_last;

        if (abs(encoder_err) < 10)
            return;

        if (abs(encoder_err) > 50)
        {
            lsd_p = lsd_p * 0.5f; // 过弯时进一步降低比例项
            lsd_d = lsd_d * 0.6f; // 减少微分响应
        }
        else
        {
            lsd_p = lsd_p;
            lsd_d = lsd_d;
        }

        float lsd = encoder_err * lsd_p +
                    encoder_err_last * lsd_pl +
                    encoder_det * lsd_d;

        lsd = std::clamp(lsd, -36.0f, 36.0f); // 12%

        pidLeft.target += static_cast<int16_t>(lsd);
        pidRight.target -= static_cast<int16_t>(lsd);

        encoder_err_last = encoder_err;
    }
    void suibian_control(int speed)
    {
        float angle1 = (current_servo_pwm / 3000 / 0.9688 - 0.5) * 90;
        float angle2 = abs(angle1 - 90);
        float chasu = (chasu_k * angle2) / (2 + chasu_k * angle2) * speed;
        if (chasu > lim_cs * speed) // 差速限幅，不一定是0.9
        {
            chasu = lim_cs * speed;
        }
        if (current_servo_pwm > SERVO_MOTOR_MID) // zuo
        {
            pidLeft.target = speed + chasu;
            pidRight.target = speed;
        }
        else
        {
            pidLeft.target = speed;
            pidRight.target -= speed + chasu;
        }
    }
};
void cleanup();
void sigint_handler(int signum);

#endif