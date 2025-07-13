#include "pid.hpp"
#include <zf_common_headfile.h>
#include <vector>

//==============================================================================
// >> Global Variable Definitions
//==============================================================================
struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
struct pwm_info servo_pwm_info;
float servo_motor_duty = 90.0;
float servo_motor_dir = 1;
int16_t encoder_left;
int16_t encoder_right;
int point_speed_max = 90; // 前瞻最大起始点
int point_speed_min = 65;
; // 最小
int speed_min = speed * 0.9;
int speed_max = speed * 1.2; // 最大s
int white_min = 70;          // 白点最小
int white_max = 118;         // 白点最大
const float BANG_BANG_THRESHOLD = 100.0f;
//==============================================================================
// >> Free Function Implementations
//==============================================================================
void sigint_handler(int signum)
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // brush_off(); // Original function call, ensure it's defined elsewhere
    brush_off();
    pwm_set_duty(SERVO_MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    gpio_set_level(BEEP, 0x0);
}


//==============================================================================
// >> MotionController Class Implementations
//==============================================================================

MotionController::MotionController(LaneProcessor *lp) : laneProcessor(lp)
{
    // Device Initialization
    pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
    pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
    pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
    gpio_set_level(MOTOR1_DIR, 0);
    gpio_set_level(MOTOR2_DIR, 0);

    // PID Parameter Initialization
    init_pid(pidLeft, 28.0f, 2.805f, 0.13f, PWM_MAX, DELTA_PID);
    init_pid(pidRight, 30.0f, 2.845f, 0.18f, PWM_MAX, DELTA_PID);
    init_pid(pidservo, 6.0f, 0.0f, 14.0f, 500.0f, FUZZY_PID);

    init_serial();
}

MotionController::~MotionController()
{
    if (serial_fd != -1)
    {
        close(serial_fd);
    }
}

float MotionController::ave_speed()
{
    std::lock_guard<std::mutex> lock(encoder_mutex);

    float current_speed = (encoder_left - encoder_right) / 2.0;

    if (buffer_count < 5)
    {
        speed_buffer[buffer_index] = current_speed;
        buffer_count++;
    }
    else
    {
        speed_buffer[buffer_index] = current_speed;
    }
    buffer_index = (buffer_index + 1) % 5;
    float average_speed = 0;
    for (int i = 0; i < buffer_count; i++)
    {
        average_speed += speed_buffer[i];
    }
    average_speed /= buffer_count;
    cerr << "average_speed:" << average_speed << endl;
    return average_speed;
}

int MotionController::dongtaiqianzhan()
{
    int average_speed = (encoder_left + encoder_right) / 2;
    cerr << "average_speed:" << average_speed << endl;
    if (speed_max <= speed_min)
    {
        return point_speed_min;
    }
    else if (average_speed > speed_max)
    {
        average_speed = speed_max;
    }
    float speed_range = static_cast<float>(speed_max - speed_min);
    float scale = static_cast<float>(average_speed - speed_min) / speed_range;

    scale = std::clamp(scale, 0.0f, 1.0f);
    int point_speed = static_cast<int>(scale * (point_speed_max - point_speed_min) + point_speed_min);
    return point_speed;
}

float MotionController::Err_sum2(const vector<Point> &centerline)
{
    int steps_used = 0;
    float error = 0.0f;
    float weight_count = 0.0f;
    // int a = dongtaiqianzhan();
    // a = std::clamp(a, 0, 99);
    // cerr << "a:" << a << endl;
    int b = centerline.size();
    if (b < white_min)
        b = white_min;
    else if (b > white_max)
        b = white_max;
    float white_range = static_cast<float>(white_max - white_min);
    if (fabs(white_range) < 1e-5)
        white_range = 1.0f;
    float scale = std::clamp(static_cast<float>(b - white_min) / white_range, 0.0f, 1.0f);
    int point_white = static_cast<int>(scale * (point_speed_max - point_speed_min) + point_speed_min);
    // int point_white = (b - white_min) / (white_max - white_min) * (point_speed_max - point_speed_min) + point_speed_min; // 计算点速度
    point_white = std::clamp(point_white, 0, 89);
    cerr << "point_white:" << point_white << endl;
    if(centerline.size()>point_white+21)
    {
        for (int i = 0; i < 21; i++)
        {
            if(control_circle == 11||control_circle==12||control_circle==2||control_circle==7||control_circle==3||control_circle==8){
                error += (centerline[point_white+i].x - 80) * dongtaiquan2[i];
                weight_count += dongtaiquan2[i];
            }
            else{
                error += (centerline[point_white+i].x - 80) * quan_weight[i];
                weight_count += quan_weight[i];
            }
        }
    }
    else
    {
        for(int i = centerline.size()-21; i < centerline.size(); i++)
        {
                error += (centerline[i].x - 80) * dongtaiquan[i];
                weight_count += dongtaiquan[i];
        }
    }

    float B = error / weight_count;
    return (weight_count != 0.0f) ? B : 0.0f;
}

void MotionController::motor_control(int speed, float k, int limit)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    ackerman_diff_control(speed, control_circle);
    // Get encoder values
    pidLeft.actual = encoder_left;
    pidRight.actual = -encoder_right;

    std::cerr << "left: " << pidLeft.actual << " right: " << pidRight.actual << std::endl;
    std::cerr << "pidLeft.target: " << pidLeft.target << " pidRight.target: " << pidRight.target << std::endl;

    float error_left = pidLeft.target - pidLeft.actual;
    float error_right = pidRight.target - pidRight.actual;

     if (fabs(error_left) > BANG_BANG_THRESHOLD) 
    {
        pidLeft.integral = 0;
        memset(pidLeft.error, 0, sizeof(pidLeft.error));
        
        float raw_outL = (error_left > 0) ? 0.40f * PWM_MAX : -0.40f * PWM_MAX;
        gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 1 : 0);
        pwm_set_duty(MOTOR1_PWM, fabs(raw_outL));
    }
    else 
    {
        float raw_outL = calculate_pid(pidLeft, pidLeft.target);
        gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 1 : 0);
        float outL = std::clamp(fabs(raw_outL), 0.0f, 0.60f * static_cast<float>(PWM_MAX));
        pwm_set_duty(MOTOR1_PWM, outL);
    }
    if (fabs(error_right) > BANG_BANG_THRESHOLD)
    {
        pidRight.integral = 0;
        memset(pidRight.error, 0, sizeof(pidRight.error));
        
        float raw_outR = (error_right > 0) ? 0.40f * PWM_MAX : -0.40f * PWM_MAX;
        gpio_set_level(MOTOR2_DIR, (raw_outR >= 0) ? 1 : 0);
        pwm_set_duty(MOTOR2_PWM, fabs(raw_outR));
    }
    else 
    {
        float raw_outR = calculate_pid(pidRight, pidRight.target);
        gpio_set_level(MOTOR2_DIR, (raw_outR >= 0) ? 1 : 0);
        float outR = std::clamp(fabs(raw_outR), 0.0f, 0.60f * static_cast<float>(PWM_MAX));
        pwm_set_duty(MOTOR2_PWM, outR);
    }
    // PID calculation
    // float raw_outL = calculate_pid(pidLeft, pidLeft.target);
    // float raw_outR = calculate_pid(pidRight, pidRight.target);

    // gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 1 : 0); // right
    // gpio_set_level(MOTOR2_DIR, (raw_outR >= 0) ? 1 : 0); // left

    // float outL = std::clamp(abs(raw_outL), 0.0f, 0.60f * static_cast<float>(PWM_MAX));
    // float outR = std::clamp(abs(raw_outR), 0.0f, 0.60f * static_cast<float>(PWM_MAX));

    // // Set motor output
    // pwm_set_duty(MOTOR1_PWM, outL);
    // pwm_set_duty(MOTOR2_PWM, outR);
    // std::cerr << "dutyl: " << outL << " dutyr: " << outR << std::endl;
}

float MotionController::Err_sum(const std::vector<cv::Point> &centerline)
{
    if (centerline.empty())
        return 0.0f;

    const int total_lines = centerline.size();

    constexpr int weight_len = 39;
    constexpr float weight_middle[39] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
        11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
        20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
        10, 9, 8, 7, 6, 5, 4, 3, 2};

    constexpr float weight_front[39] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 13, 13, 13, 13, 15,
        17, 17, 15, 15, 15, 15, 19, 19, 20, 20, 20,
        17, 19, 20, 20, 1, 1, 1, 1, 1, 1};
    // ↑ 最后几个权重最大（用于最远点）

    float error = 0.0f;
    float weight_count = 0.0f;

    if (total_lines > 90)
    {
        // 正常情况：从第50到第80行（近中段）
        int start = 64;
        for (int i = 0; i < weight_len; ++i)
        {
            int idx = start + i;
            if (idx >= total_lines)
                break;
            error += (centerline[idx].x - 81) * weight_middle[i];
            weight_count += weight_middle[i];
        }
    }
    else if (total_lines > 65)
    {
        // 从第64行开始，向远处最多取30个（反向加权）
        int start = 50;
        int usable_points = std::min(total_lines - start, weight_len);
        for (int i = 0; i < usable_points; ++i)
        {
            int idx = start + i;
            int weight_idx = weight_len - usable_points + i; // 让远处的点权重大
            error += (centerline[idx].x - 81) * weight_front[weight_idx];
            weight_count += weight_front[weight_idx];
        }
    }
    else
    {
        // 点数太少，从最后一个点往前取最多30个（反向加权）
        int usable_points = std::min(total_lines, weight_len);
        for (int i = 0; i < usable_points; ++i)
        {
            int idx = total_lines - usable_points + i;
            int weight_idx = weight_len - usable_points + i; // 越远权重越大
            error += (centerline[idx].x - 81) * weight_front[weight_idx];
            weight_count += weight_front[weight_idx];
        }
    }

    if (fabs(weight_count) < 1e-6f)
        return 0.0f;

    return error / weight_count;
}

void MotionController::update_shared_error(float err)
{
    std::lock_guard<std::mutex> lock(error_mutex);
    shared_error = err;
}

float MotionController::get_shared_error()
{
    std::lock_guard<std::mutex> lock(error_mutex);
    return shared_error;
}

void MotionController::init_serial()
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
    options.c_cflag |= (CS8 | CLOCAL | CREAD);
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
    // std::cerr << "Serial initialized. FD: " << serial_fd << std::endl;
}

void MotionController::pit_callback()
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    encoder_left = encoder_get_count(ENCODER_1);
    encoder_right = encoder_get_count(ENCODER_2);
    // std::cerr << "Encoder Left: " << encoder_left << ", Right: " << encoder_right << std::endl;
}

void MotionController::set_servo_angle(int error)
{
    static int error_last = 0;

    // if (abs(error) < 4)
    // { // Dead zone
    //     pwm_set_duty(SERVO_MOTOR1_PWM, 4360);
    //     error_last = error;
    //     return;
    // }

    pidservo.target = 0;
    pidservo.actual = error;

    float pwm_adjustment = calculate_pid(pidservo, pidservo.target);

    float target_pwm = 4260 + pwm_adjustment;
    // if(target_pwm > 4500){
    //     target_pwm *=1.1;
    // }
    target_pwm = std::clamp(target_pwm, static_cast<float>(SERVO_MOTOR_R_MAX), static_cast<float>(SERVO_MOTOR_L_MAX));

    pwm_set_duty(SERVO_MOTOR1_PWM, target_pwm);
    std::cerr << "pwm" << target_pwm << std::endl;
    current_servo_pwm = target_pwm;
    error_last = error;
    // 死区处理 (±4像素不响应)
    // if(abs(error_last - error) >30) error = error_last;
}

void MotionController::init_pid(PID_Controller &pid, float Kp, float Ki, float Kd, float max_out, PID_Mode mode)
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
        pid.fuzzy_pd = {5.5f, 12.0f, 1.0, 35.0, 6.0, 1.0f}; // 12
        // float uff_p_max = 22.0f, uff_d_max = 60.0f;
        float uff_p_max = 19.0f, uff_d_max = 60.0f;
        for (int i = 0; i < 7; ++i)
        {
            pid.uff.UFF_P[i] = uff_p_max * (i - 3.0f) / 3.0f;
            pid.uff.UFF_D[i] = uff_d_max * (i - 3.0f) / 3.0f;
            pid.EFF[i] = 66.0f * (i - 3.0f) / 3.0f;
            pid.DFF[i] = 30.0f * (i - 3.0f) / 3.0f;
        }
        pid.fuzzy_initialized = true;
    }
}

float MotionController::calculate_pid(PID_Controller &pid, float target)
{
    float error = target - pid.actual;
    float alpha = 0.95f;
    static float filtered_output = 0.0f;

    if (error > 4000)
    {
        error = 6;
    }

    if (pid.mode == POSITION_PID)
    {
        pid.integral += error;
        pid.integral = std::clamp(pid.integral, -pid.max_output / pid.Ki, pid.max_output / pid.Ki);
        float P = pid.Kp * error;
        float I = pid.Ki * pid.integral;
        float D = pid.Kd * (error - pid.error[0]);
        pid.output = P + I + D;
        pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
        pid.error[0] = error;
    }
    else if (pid.mode == DELTA_PID)
    {
        pid.error[2] = pid.error[1];
        pid.error[1] = pid.error[0];
        pid.error[0] = error;

        float delta = pid.Kp * (pid.error[0] - pid.error[1]) + pid.Ki * pid.error[0] + pid.Kd * (pid.error[0] - 2 * pid.error[1] + pid.error[2]);
        float delta_limit = pid.max_output * 0.5f;
        delta = std::clamp(delta, -delta_limit, delta_limit);

        pid.output += delta;
        pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
    }
    else if (pid.mode == FUZZY_PID && pid.fuzzy_initialized)
    {

        float ec = error - pid.last_error;
        float A = 0.09;
        pid.last_error = error;
        float error_abs = fabs(error);
        // pid.fuzzy_pd.Kp0 = error_abs > 20 ? 10.0f : 4.0f;
        //  执行模糊推理
        count_DMF(pid, error * pid.fuzzy_pd.factor, ec * pid.fuzzy_pd.factor * EC_FACTOR);
        float delta_kp = Fuzzy_Kp(pid);
        cerr << "ΔKp:" << delta_kp << endl;
        float delta_kd = abs(Fuzzy_Kd(pid));
        cerr << "ΔKd:" << delta_kd << endl;
        float output = 0.0f;
        float P = 0.0f;
        float D = 0.0f;
        // pid.fuzzy_pd.Kp0 = params.kp_switch;
        // pid.fuzzy_pd.Kd0 = params.kd_switch;
        // pid.fuzzy_pd.A = params.A_switch; // 模糊PID参数A
        if (control_circle == 13 || control_circle == 1 || control_circle == 6)
        {
            pid.fuzzy_pd.Kp0 = 5.0f;
            pid.fuzzy_pd.Kd0 = 12.0f;
            delta_kd = 0.0f;
            delta_kp = 0.0f;
        }
        else if (control_circle == 0)
        {
            pid.fuzzy_pd.Kp0 = 1.f;
            pid.fuzzy_pd.Kd0 = 8.0f;
        }
        P = (pid.fuzzy_pd.Kp0 + delta_kp) * error;
        D = (pid.fuzzy_pd.Kd0 + delta_kd) * ec; //+ delta_kd
        if (control_circle == 11 || control_circle == 12 ||control_circle==2||control_circle ==7||control_circle==3||control_circle==8)
        {
            output = error * (pid.fuzzy_pd.Kp0 + A * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0;
        }
        else if (control_circle == 13 || control_circle == 1 || control_circle == 6 )
        {
            output = P + D;
        }
        else
        {
            output = error * (pid.fuzzy_pd.Kp0 + 0.03 * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0;
        }
    
        //      }else {
        //          output = error * (pid.fuzzy_pd.Kp0 + A * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0; // imu963ra_gyro_z * delta_kd;
        // }
        // float output = error * (pid.fuzzy_pd.Kp0 + A * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0; // imu963ra_gyro_z * delta_kd;
        // float output = (pid.fuzzy_pd.Kp0 + delta_kp) * error / 100.0f;
        // output += (pid.fuzzy_pd.Kd0 + delta_kd) * ec / 100.0f;
        // output *= -0.7f;
        //  输出限幅
        filtered_output = output; // alpha * output + (1.0f - alpha) * filtered_output;
        filtered_output = clamp(filtered_output, -pid.max_output, pid.max_output);
        // output = clamp(output, -pid.max_output, pid.max_output);
        // cerr<<"pwm"<<output<<endl;
        return filtered_output;
    }
    return pid.output;
}

void MotionController::ackerman_diff_control(int Speed_Goal, int state)
{
    const float L = 200.0f, W = 150.0f;
    float angle1 = abs((4260 - current_servo_pwm) * 0.09278f);
    float angle_rad = angle1 * M_PI / 180.0f;
    float tn = tan(angle_rad);

    float outer_factor = 1.0f + 0.2f * (W / 2.0f) * tn / L;
    float inner_factor = 1.0f - 0.8f * (W / 2.0f) * tn / L;

    inner_factor = std::max(0.65f, std::min(inner_factor, 1.2f));
    outer_factor = std::max(1.0f, std::min(outer_factor, 1.2f));

    std::cerr << "in:" << inner_factor << std::endl;
    std::cerr << "out:" << outer_factor << std::endl;
    cerr << "state:" << state << endl;

    switch (state)
    {
    case 13:
        Speed_Goal = Speed_Goal * 1.1;
        break;
    case 1:
        Speed_Goal = Speed_Goal * 0.9;
        break;
    case 2:
        Speed_Goal = Speed_Goal * 0.87;
        break;
    case 3:
        Speed_Goal = Speed_Goal * 0.87;
        break;
    case 4:
        Speed_Goal = Speed_Goal * 0.9;
        break;
    case 6:
        Speed_Goal = Speed_Goal * 0.9;
        break;
    case 7:
        Speed_Goal = Speed_Goal * 0.87;
        break;
    case 8:
        Speed_Goal = Speed_Goal * 0.87;
        break;
    case 9:
        Speed_Goal = Speed_Goal * 0.9;
        break;
    case 11:
        Speed_Goal = Speed_Goal * 0.85;
        break;
    case 12:
        Speed_Goal = Speed_Goal * 0.85;
        break;
    default:
        Speed_Goal = Speed_Goal;
        break;
    }
    cerr << "speed_goal:" << Speed_Goal << endl;
    if (state == 12  || state==3||current_servo_pwm < 3800) //||current_servo_pwm <3800
    {                                                          // right turn
        pidLeft.target = Speed_Goal * outer_factor;
        pidRight.target = Speed_Goal * inner_factor;
    }
    else if (state == 11  ||state==8|| current_servo_pwm > 4700) //||current_servo_pwm > 4700
    {                                                               // left turn
        pidLeft.target = Speed_Goal * inner_factor;
        pidRight.target = Speed_Goal * outer_factor;
    }
    else
    { // straight
        pidLeft.target = Speed_Goal;
        pidRight.target = Speed_Goal;
    }
}

// --- Fuzzy Logic Helper Implementations ---

void MotionController::count_DMF(PID_Controller &pid, float e, float ec)
{
    // Error membership calculation
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

    // Error change rate membership calculation
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

float MotionController::Fuzzy_Kp(PID_Controller &pid)
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
    {
        sum += pid.uff.UFF_P[i] * KpgradSums[i];
    }
    return sum;
}

float MotionController::Fuzzy_Kd(PID_Controller &pid)
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
    {
        sum += pid.uff.UFF_D[i] * KdgradSums[i];
    }
    return sum;
}