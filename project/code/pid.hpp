#ifndef PID_HPP_
#define PID_HPP_

//==============================================================================
// >> Includes
//==============================================================================
// C++ Standard Library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// C Standard Library
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// External Libraries
#include <zf_common_headfile.h>
#include <camera.hpp>
//==============================================================================
// >> Forward Declarations
//==============================================================================
class LaneProcessor;

//==============================================================================
// >> Constants and Enumerations
//==============================================================================

// --- Device Paths
constexpr char MOTOR1_DIR[] = "/dev/zf_driver_gpio_motor_1";
constexpr char MOTOR1_PWM[] = "/dev/zf_device_pwm_motor_1";
constexpr char MOTOR2_DIR[] = "/dev/zf_driver_gpio_motor_2";
constexpr char MOTOR2_PWM[] = "/dev/zf_device_pwm_motor_2";
constexpr char ENCODER_1[] = "/dev/zf_encoder_1";
constexpr char ENCODER_2[] = "/dev/zf_encoder_2";
constexpr char SERVO_MOTOR1_PWM[] = "/dev/zf_device_pwm_servo";

// --- Control Parameters
constexpr int SERIAL_RETRY_INTERVAL = 50; // 串口重试间隔(ms)
constexpr int MID_W = 160;                // 图像中线
constexpr int PWM_MAX = 10000;            // 电机最大PWM
constexpr int CONTROL_PERIOD_MS = 10;

// --- Servo Parameters
constexpr int SERVO_MOTOR_MID = 4360;
constexpr int SERVO_MOTOR_L_MAX = 4800;
constexpr int SERVO_MOTOR_R_MAX = 3940;

// --- Lane Following Weights
constexpr float weight[39] = { //50
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,           //60
    1, 1, 1, 2, 3, 3, 13, 15, 16, 16,     //70
    16, 16, 18, 18, 20 ,20, 20, 23, 23, 23,   //80
    25, 25, 17, 17, 6, 5, 4, 3, 2};  //89

constexpr float weight1[39] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 13, 13, 13, 13, 15,
    17, 17, 15, 15, 15, 15, 19, 19, 11, 13, 15,
    17, 19, 20, 20, 1, 1, 1, 1, 1, 1};

constexpr float dongtaiquan[21] = {  
7,7,9,9,11,11,13,13,15,15,20,15,15,13,13,11,11,9,9,7,7 // 0-19
};

// --- Fuzzy Logic Constants
enum FuzzyTerm { NB = -3, NM = -2, NS = -1, ZO = 0, PS = 1, PM = 2, PB = 3 };
constexpr int EC_FACTOR = 1;

// Fuzzy rule table for Kp
static const int rule_p[7][7] = {
    //      NB    NM    NS    ZO    PS    PM    PB
    /*NB*/ {PB,   PB,   PM,   PM,   PS,   ZO,   ZO},
    /*NM*/ {PB,   PM,   PM,   PS,   ZO,   ZO,   PS},
    /*NS*/ {PM,   PM,   PS,   ZO,   ZO,   PS,   PM},
    /*ZO*/ {PS,   PS,   ZO,   PS,   PS,   PS,   PS},
    /*PS*/ {PM,   PS,   ZO,   PS,   PS,   PM,   PM},
    /*PM*/ {ZO,   ZO,   PS,   PM,   PM,   PM,   PB},
    /*PB*/ {PB,   PM,   PM,   PM,   PM,   PB,   PB}
};

// Fuzzy rule table for Kd
static const int rule_d[7][7] = {
    {PS, NS, NB, NB, NB, NM, PS},
    {PS, NS, NB, NM, NM, NS, ZO},
    {ZO, NS, NM, NM, NS, NS, ZO},
    {ZO, NS, NS, NS, NS, NS, ZO},
    {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
    {PB, NS, PS, PS, PS, PS, PB},
    {PB, PM, PM, PM, PS, PS, PB}
};

//==============================================================================
// >> Type Definitions and Structs
//==============================================================================

typedef struct
{
    float Kp0;          // 小弯kp
    float Kd0;          // 小弯kd
    float threshold;    // 输出变化阈值
    float maximum;      // kp max
    float minimum;      // kp min
    float factor;       // 误差缩放因子（匹配EFF范围）error*factor落在EFF
    float A = 0.01; // 模糊PID参数A
    // float SetValue;     // 目标值 0
    // float CurrentValue; // 当前值 160-wide error
    // float err;          // 当前误差
    // float errlast;      // 上次误差
    // float out;          // 输出值
    // float outlast;      // 上次输出
} Fuzzy_PD_t;

struct DMF_t {
    float EF[2];
    int En[2];
    float DF[2];
    int Dn[2];
};

struct UFF_t {
    float UFF_P[7];
    float UFF_D[7];
};

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
    float integral_limit_ratio = 0.6f;
    float filtered_D = 0;

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

//==============================================================================
// >> Global Variable Declarations
//==============================================================================
extern struct pwm_info motor_1_pwm_info;
extern struct pwm_info motor_2_pwm_info;
extern struct pwm_info servo_pwm_info;
extern int control_circle; // 环岛状态
extern int16_t encoder_left;
extern int16_t encoder_right;
extern float servo_motor_duty;
extern float servo_motor_dir;

//==============================================================================
// >> Class Definition: MotionController
//==============================================================================
class MotionController {
public:
    friend class LaneProcessor;

    // --- Constructor / Destructor
    explicit MotionController(LaneProcessor* lp);
    ~MotionController();

    MotionController(const MotionController&) = delete;
    MotionController& operator=(const MotionController&) = delete;

    // --- High-Level Control Interface
    float ave_speed();
    int dongtaiqianzhan();
    float Err_sum2(const vector<Point> &centerline);
    void motor_control(int speed, float k, int limit);
    void set_servo_angle(int error);
    float Err_sum(const std::vector<cv::Point>& centerline);


    // --- Thread-Safe Accessors / Updaters
    void pit_callback();
    void update_shared_error(float err);
    float get_shared_error();

    // --- Private Helper Functions
    void init_pid(PID_Controller& pid, float Kp, float Ki, float Kd, float max_out, PID_Mode mode);
    float calculate_pid(PID_Controller& pid, float target);
    void init_serial();
    bool send_packet(const uint8_t* data, size_t len);

    // --- Fuzzy Logic Helpers
    float Fuzzy_Kp(PID_Controller& pid);
    float Fuzzy_Kd(PID_Controller& pid);
    void count_DMF(PID_Controller& pid, float e, float ec);

    // --- Drivetrain Strategy Helpers
    void ackerman_diff_control(int Speed_Goal, int state);
    void LSD_Control();
    void suibian_control(int speed);

    // --- Member Variables
    LaneProcessor* laneProcessor;
    int serial_fd = -1;
    std::chrono::steady_clock::time_point last_serial_retry;
    
    PID_Controller pidLeft;
    PID_Controller pidRight;
    PID_Controller pidservo;

    int car_startline = 100;
    int hope_line = 62;

    float lsd_p = 0.35f;
    float lsd_pl = 0.2f;
    float lsd_d = 0.4f;
    int16_t encoder_err_last = 0;

    float Speed_Goal = 0.0f;
    const uint16_t steer_middle = 4360;
    const float Left_Speed = 1.2f;
    const float Right_Speed = 1.1f;
    float current_servo_pwm = steer_middle;
    const float ackerman_limit = 0.1f;

    float lim_cs = 0.9;
    float chasu_k = 1.0;
    
    float shared_error;
    std::mutex error_mutex;
    std::mutex encoder_mutex;
private:
    //std::unordered_map<int, CircleControlParams> circle_params_map;
    //CircleControlParams default_params;
    float speed_buffer[20] = {0};
    int buffer_index = 0;      // 当前要替换的位置
    int buffer_count = 0;      // 当前缓冲区中的数据个数（用于初始填充）
    float quan_weight[120] = {  
    // 索引0-20（已提供的21个元素）
    0,0,1,
    0,0,1,
    0,0,1,
    0,0,1,
    0,0,1,
    0,0,1,
    0,0,1,
    0,0,1,  // 索引21,22,23
    0,0,1,  // 24,25,26
    0,0,1,  // 27,28,29
    0,0,1,  // 30,31,32
    0,0,1,  // 33,34,35
    0,0,1,  // 36,37,38
    0,0,1,  // 39,40,41
    0,0,1,  // 42,43,44
    0,0,1,  // 45,46,47
    0,0,1,  // 48,49,50
    0,0,1,  // 51,52,53
    0,0,1,  // 54,55,56
    0,0,1,  // 57,58,59
    0,0,1,  // 60,61,62
    0,0,1,  // 63,64,65
    0,0,1,  // 66,67,68
    0,0,1,  // 69,70,71
    0,0,1,  // 72,73,74
    0,0,1,  // 75,76,77
    0,0,1,  // 78,79,80
    0,0,1,  // 81,82,83
    0,0,1,  // 84,85,86
    0,0,1,  // 87,88,89
    0,0,1,  // 90,91,92
    0,0,1,  // 93,94,95
    0,0,1,  // 96,97,98
    0,0,1,  // 99,100,101
    0,0,1,  // 102,103,104
    0,0,1,  // 105,106,107
    0,0,1,  // 108,109,110
    0,0,1,  // 111,112,113
    0,0,1,  // 114,115,116
    0,0,1   // 117,118,119
};
};

//==============================================================================
// >> Free Function Declarations
//==============================================================================
void cleanup();
void sigint_handler(int signum);
int decideSpeed(int state, int speed);

#endif // PID_HPP_