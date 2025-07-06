#include <pid.hpp>
#include <zf_common_headfile.h>

struct pwm_info motor_1_pwm_info;
struct pwm_info motor_2_pwm_info;
struct pwm_info servo_pwm_info;
float servo_motor_duty = 90.0;
float servo_motor_dir = 1;
int16 encoder_left;
int16 encoder_right;
int point_speed_max = 60;
int point_speed_min = 30;
int speed_min = 470;
int speed_max = 650;

// enum PID_Mode
// {
//     POSITION_PID,
//     DELTA_PID,
//     FUZZY_PID
// };

void sigint_handler(int signum)
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    brush_off();
    // 关闭电机
    pwm_set_duty(SERVO_MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR1_PWM, 0);
    pwm_set_duty(MOTOR2_PWM, 0);
    gpio_set_level(BEEP, 0x0);
}

// MotionController::MotionController()
//     : laneProcessor(std::make_unique<LaneProcessor>())  // 初始化智能指针
// {
//     // 设备初始化
//     pwm_get_dev_info(MOTOR1_PWM, &motor_1_pwm_info);
//     pwm_get_dev_info(MOTOR2_PWM, &motor_2_pwm_info);
//     pwm_get_dev_info(SERVO_MOTOR1_PWM, &servo_pwm_info);
//     gpio_set_level(MOTOR1_DIR, 0);
//     gpio_set_level(MOTOR2_DIR, 0);

//     // 初始化 PID 参数
//     init_pid(pidLeft, 1.25f, 0.245f, 0.10f, PWM_MAX, DELTA_PID);
//     init_pid(pidRight, 1.25f, 0.205f, 0.18f, PWM_MAX, DELTA_PID);
//     init_pid(pidservo, 5.0f, 0.0f, 10.0f, 367.0f, FUZZY_PID);
//     init_serial();
// }

// 析构函数实现
// MotionController::~MotionController() {
//     if(serial_fd != -1) {
//         close(serial_fd);
//     }
// }
float MotionController::ave_speed() {
    std::lock_guard<std::mutex> lock(encoder_mutex);
    float current_speed = (encoder_left + encoder_right) / 2.0;
    if (buffer_count < 20) {
        speed_buffer[buffer_index] = current_speed;
        buffer_count++;
    } else {
        speed_buffer[buffer_index] = current_speed;
    }
    buffer_index = (buffer_index + 1) % 20;
    float average_speed = 0;
    for (int i = 0; i < buffer_count; i++) {
        average_speed += speed_buffer[i];
    }
    average_speed /= buffer_count;
    return average_speed;
}

int MotionController::dongtaiqianzhan(){                                              
    std::lock_guard<std::mutex> lock(encoder_mutex);
    //int average_speed = (encoder_left + encoder_right) / 2.0;
    int average_speed = ave_speed();
    int point_speed = ((average_speed - speed_min) / (speed_max - speed_min) * (point_speed_max - point_speed_min)) + point_speed_min;// 计算点速度
    return point_speed;
}
                    

void MotionController::motor_control(int speed, float k, int limit)
{
    std::lock_guard<std::mutex> lock(encoder_mutex);
    auto params = laneProcessor->getControlParams();
    float speed_factor = params.speed_factor;
    cerr<< "speed_factor: " << speed_factor << endl;
    //int speed1 = speed * speed_factor;
    float chasu = 0.8;
    float pwm_error = current_servo_pwm - SERVO_MOTOR_MID;
    float k_decision = 8;

    //float speed1 = speed - (speed * (1 - chasu) * abs(pwm_error) * k_decision / SERVO_MOTOR_MID); // 打满时速度为0.825
    float speed1 = speed * speed_factor;
    //float speed1 = speed;
    if (abs(pwm_error) >= 150)
    {
        ackerman_diff_control(speed1);
    }
    else
    {
        pidLeft.target = speed1;
        pidRight.target = speed1;
    }
    // suibian_control(speed1);

    // 获取编码器值
    pidLeft.actual = encoder_left;
    pidRight.actual = -encoder_right;

    cerr << "left: " << pidLeft.actual << " right: " << pidRight.actual << endl;
    //  PID计算

    // pidLeft.target = speed1;
    // pidRight.target = speed1;
    // LSD_Control();

    float raw_outL = calculate_pid(pidLeft, pidLeft.target);
    float raw_outR = calculate_pid(pidRight, pidRight.target);

    // float raw_outL = calculate_pid(pidLeft,pidLeft.target);
    // float raw_outR = calculate_pid(pidRight,pidRight.target);
    gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 1 : 0); // 假设0为正转
    gpio_set_level(MOTOR2_DIR, (raw_outR >= 0) ? 0 : 1); // 假设0为转
    float outL = std::clamp(abs(raw_outL), 0.0f, 0.40f * static_cast<float>(PWM_MAX));
    float outR = std::clamp(abs(raw_outR), 0.0f, 0.40f * static_cast<float>(PWM_MAX));
    // float outL = calculate_pid(pidLeft, speed);
    // float outR = calculate_pid(pidRight, speed);

    // outL = std::clamp(abs(outL), 0.0f, static_cast<float>(PWM_MAX));
    // outR = std::clamp(abs(outR), 0.0f, static_cast<float>(PWM_MAX));

    // 设置电机输出
    pwm_set_duty(MOTOR1_PWM, outL);
    pwm_set_duty(MOTOR2_PWM, outR);
    // cerr << "dutyl: " << outL <<"dutyr: " <<  outR << endl;
}

// float MotionController::Err_sum(const vector<Point> &centerline) {
//     auto params = laneProcessor->getControlParams();

//     int effective_rows = min(params.lookahead_lines, (int)centerline.size());

//     float error = 0.0f;
//     float weight_sum = 0.0f;

//     for(int i=0; i<effective_rows; ++i) {
//         if(params.weight_case == 1.0f){
//         float base_weight = weight1[i % 39];
//         error += (centerline[i].x - 80)*base_weight;
//         weight_sum += base_weight;
//         }
//         else {
//         float base_weight = weight[i % 39];
//         error += (centerline[i].x - 80)*base_weight;
//         weight_sum += base_weight;
//         }
//     }
//     return error / weight_sum;
// }
float MotionController::Err_sum(const vector<Point> &centerline)
{
    auto params = laneProcessor->getControlParams();

    // cerr<<params.weight_case<<endl;
    // cerr<<params.speed_factor<<endl;
    //  1. 检查输入有效性
    if (centerline.empty())
    {
        cerr << "错误：centerline 是空的！" << endl;
        return 0.0f;
    }

    // 2. 检查 centerline 是否至少包含 5 个点
    const int min_required_size = 5;
    if (centerline.size() < min_required_size)
    {
        cerr << "错误：centerline 长度不足（至少需要 " << min_required_size
             << " 个点，实际 " << centerline.size() << "）" << endl;
        return 0.0f;
    }

    // 3. 计算实际可用的步数
    const int total_steps = 38;                      // 默认需要计算的步数
    const int start = 55;                            // 400 50 420 60 430 64
    const int start_idx = centerline.size() - start; // 起始索引
    const int start2 = centerline.size() - 39;
    const int force_start = max(start2, 0);
    int steps_used = 0;
    // cerr<<start_idx<<endl;
    if (start_idx > 0)
    {
        steps_used = min(total_steps, start_idx + 1); // 实际计算的步数
    }
    else
    {
        steps_used = (centerline.size() >= 39) ? 39 : centerline.size();
    }
    // const int steps_used = (centerline.size() >= total_steps) ? total_steps : centerline.size();
    //  4. 检查权重数组是否足够
    const int weight_size = sizeof(weight) / sizeof(weight[0]);
    if (weight_size < steps_used)
    {
        cerr << "错误：权重数组尺寸不足（需要 " << steps_used
             << "，实际 " << weight_size << "）" << endl;
        return 0.0f;
    }

    // 5. 计算加权误差
    float error = 0.0f;
    float weight_count = 0.0f;
    // if (params.weight_case == 1.0f)
    // {
    //     if (start_idx <= 0)
    //     {
    //         for (int i = 0; i < steps_used; ++i)
    //         {
    //             // int idx = start_idx - i; // 自动确保 idx >= 0
    //             error += (centerline[i + force_start].x - 80) * weight1[i];
    //             weight_count += weight1[i];
    //         }
    //     }
    //     else
    //     {
    //         for (int i = 0; i < steps_used; ++i)
    //         {
    //             // int idx = start_idx - i; // 自动确保 idx >= 0
    //             error += (centerline[i + start].x - 80) * weight1[i];
    //             weight_count += weight1[i];
    //         }
    //     }
    // }
    //else
    //{
        for (int i = 0; i < steps_used; ++i)
        {
            // int idx = i - i; // 自动确保 idx >= 0
            error += (centerline[i + start].x - 80) * weight[i];
            weight_count += weight[i];
        }
    //}
    // 6. 检查权重和是否为0（避免除零）
    if (fabs(weight_count) < 1e-6f)
    {
        cerr << "错误：权重和为0！" << endl;
        return 0.0f;
    }

    return error / weight_count;
}
float MotionController::Err_sum2(const vector<Point> &centerline){
    int steps_used = 0;
    float error = 0.0f;
    float weight_count = 0.0f;
    int a = dongtaiqianzhan();
    a = std::clamp(a, 0, 99); 
    for(int i = 0;i < 21;i++){
        quan_weight[i + a] = dongtaiquan[i];
    }
  
    for( steps_used = 0; steps_used < centerline.size(); ++steps_used){
          error += (centerline[steps_used].x - 80) * quan_weight[steps_used];
          weight_count += quan_weight[steps_used];
    }
    return (weight_count != 0.0f) ? error / weight_count : 0.0f;
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
        //auto params = laneProcessor->getControlParams();
        if (mode == FUZZY_PID)
        {
            // 初始化模糊参数
            pid.fuzzy_pd = {6.0f, 28.0f, 1.0, 35.0, 6.0, 1.0f};
            //pid.fuzzy_pd = {params.kp_switch, params.kd_switch, 1.0, 35.0, 6.0, 1.0f,params.A_switch};
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
float MotionController::calculate_pid(PID_Controller &pid, float target)
    {
        float error = target - pid.actual;
        float alpha = 1.0f;                 // 低通滤波系数
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
             float delta_limit = pid.max_output * 0.5f;
            delta = std::clamp(delta, -delta_limit, delta_limit);
            // 叠加输出并限幅
            pid.output += delta;
            pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
            //pid.output += delta;
            //pid.output = std::clamp(pid.output, -pid.max_output, pid.max_output);
        }

        else if (pid.mode == FUZZY_PID && pid.fuzzy_initialized)
        {
            auto params = laneProcessor->getControlParams();
            float ec = error - pid.last_error;
            //pid.fuzzy_pd.A = A;
            pid.last_error = error;
            float error_abs = fabs(error);
            pid.fuzzy_pd.Kp0 = error_abs > 20 ? 10.0f : 4.0f;
            //  执行模糊推理
            count_DMF(pid, error * pid.fuzzy_pd.factor, ec * pid.fuzzy_pd.factor * EC_FACTOR);
            float delta_kp = Fuzzy_Kp(pid);
            cerr << "ΔKp:" << delta_kp << endl;
            float delta_kd = Fuzzy_Kd(pid);
            cerr << "ΔKd:" << delta_kd << endl;
            pid.fuzzy_pd.Kp0 = params.kp_switch;
            pid.fuzzy_pd.Kd0 = params.kd_switch;
            pid.fuzzy_pd.A = params.A_switch; // 模糊PID参数A
            // 计算最终输出
            float P = (pid.fuzzy_pd.Kp0 + delta_kp) * error; // 直接使用模糊调整后的Kp
            // float P = pid.fuzzy_pd.Kp0 + abs(error)*delta_kp * error + ;
            // float D = pid.Kd * ec;
            float D = (pid.fuzzy_pd.Kd0 + delta_kd) * ec;
            cerr << "kd0:" << pid.fuzzy_pd.Kd0 <<endl;
            cerr << "kp0:" << pid.fuzzy_pd.Kp0 << endl;
            cerr << "A:" << pid.fuzzy_pd.A << endl;
            float output = 0;
        
            output = error * (pid.fuzzy_pd.Kp0 + pid.fuzzy_pd.A * (abs(error) * delta_kp)) + ec * pid.fuzzy_pd.Kd0; //(pid.fuzzy_pd.Kd0 + delta_kd); // imu963ra_gyro_z * delta_kd;
            cerr<<"output"<<output<<endl;
            // float output = (pid.fuzzy_pd.Kp0 + delta_kp) * error / 100.0f;
            // output += (pid.fuzzy_pd.Kd0 + delta_kd) * ec / 100.0f;
            // output *= -0.7f;
            //  输出限幅
            filtered_output = output; //+ (1.0f - alpha) * filtered_output;
            filtered_output = clamp(filtered_output, -pid.max_output, pid.max_output);
            // output = clamp(output, -pid.max_output, pid.max_output);

            return filtered_output;
        }

        return pid.output;
    }

// float MotionController::dongtaiqianzhan(){

// }
// float MotionController::Err_sum(const vector<Point> &centerline)
// {
//     auto params = laneProcessor->getControlParams();
//     const int min_required_size = 5;
//     const int total_points = 39;  // 总共需要39个点
//     const int start_offset = 55;  // 正常情况下的起始点偏移量

//     // 检查输入有效性
//     if (centerline.size() < min_required_size) {
//         cerr << "错误：centerline 长度不足（至少需要 " << min_required_size
//              << " 个点，实际 " << centerline.size() << "）" << endl;
//         return 0.0f;
//     }

//     // 计算起始索引和实际步数
//     int begin_index = 0;
//     int steps_used = 0;
    
//     // 情况1: 有足够点取从start_offset开始的39个点
//     if (centerline.size() - start_offset >= total_points) {
//         begin_index = centerline.size() - start_offset;
//         steps_used = total_points;
//     }
//     // 情况2: 点足够但起始位置不足39个点
//     else if (centerline.size() > start_offset) {
//         begin_index = centerline.size() - start_offset;
//         steps_used = min(total_points, static_cast<int>(centerline.size() - begin_index));
//     }
//     // 情况3: 直接使用最上面的39个点
//     else {
//         steps_used = min(total_points, static_cast<int>(centerline.size()));
//         begin_index = centerline.size() - steps_used; // 从尾部开始计算点数
//     }

//     // 检查权重数组尺寸
//     const int weight_size = sizeof(weight) / sizeof(weight[0]);
//     const int weight1_size = sizeof(weight1) / sizeof(weight1[0]);
//     if (steps_used > weight_size || steps_used > weight1_size) {
//         cerr << "错误：权重数组尺寸不足（需要 " << steps_used
//              << "，实际 weight_size=" << weight_size 
//              << ", weight1_size=" << weight1_size << "）" << endl;
//         return 0.0f;
//     }

//     // 计算加权误差
//     float error = 0.0f;
//     float weight_count = 0.0f;
    
//     if (params.weight_case == 1.0f) {
//         for (int i = 0; i < steps_used; ++i) {
//             error += (centerline[begin_index + i].x - 80) * weight1[i];
//             weight_count += weight1[i];
//         }
//     } else {
//         for (int i = 0; i < steps_used; ++i) {
//             error += (centerline[begin_index + i].x - 80) * weight[i];
//             weight_count += weight[i];
//         }
//     }

//     // 检查权重和是否为0
//     if (fabs(weight_count) < 1e-6f) {
//         cerr << "错误：权重和为0！" << endl;
//         return 0.0f;
//     }

//     return error / weight_count;
// }


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