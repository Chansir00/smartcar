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
    brush_off();  
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
void MotionController::motor_control(int speed, float k, int limit) {
    std::lock_guard<std::mutex> lock(encoder_mutex);
    //auto params = laneProcessor->getControlParams();
    //float speed_factor = params.speed_factor;
    //int speed1 = speed * speed_factor;
    float chasu = 0.8;
    float pwm_error = current_servo_pwm - SERVO_MOTOR_MID;
    float k_decision = 8;

    float speed1 = speed - (speed * (1 - chasu) * abs(pwm_error) * k_decision / SERVO_MOTOR_MID); //打满时速度为0.825

    ackerman_diff_control(speed1);
    //suibian_control(speed1);

    // 获取编码器值
    pidLeft.actual =  encoder_left;
    pidRight.actual = -encoder_right;

    //cerr << "left: " << pidLeft.actual << " right: " << pidRight.actual << endl;
    // PID计算

    //pidLeft.target = speed1;
    //pidRight.target = speed1;
    //LSD_Control(); 

    float raw_outL = calculate_pid(pidLeft, pidLeft.target);
    float raw_outR = calculate_pid(pidRight, pidRight.target);

    //float raw_outL = calculate_pid(pidLeft,pidLeft.target);
    //float raw_outR = calculate_pid(pidRight,pidRight.target);
    gpio_set_level(MOTOR1_DIR, (raw_outL >= 0) ? 1 : 0);  // 假设0为正转
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
    //cerr<<params.weight_case<<endl;
    //cerr<<params.speed_factor<<endl;
    // 1. 检查输入有效性
    if (centerline.empty()) {
        cerr << "错误：centerline 是空的！" << endl;
        return 0.0f;
    }
    
    // 2. 检查 centerline 是否至少包含 5 个点
    const int min_required_size = 5;
    if (centerline.size() < min_required_size) {
        cerr << "错误：centerline 长度不足（至少需要 " << min_required_size 
             << " 个点，实际 " << centerline.size() << "）" << endl;
        return 0.0f;
    }
    
    // 3. 计算实际可用的步数
    const int total_steps = 38; // 默认需要计算的步数
    const int start = 56;     //430 52  440 50
    const int start_idx = centerline.size() - start; // 起始索引
    //cerr<<start_idx<<endl;
    const int steps_used = min(total_steps, start_idx); // 实际计算的步数
    
    // 4. 检查权重数组是否足够
    const int weight_size = sizeof(weight) / sizeof(weight[0]);
    if (weight_size < steps_used) {
        cerr << "错误：权重数组尺寸不足（需要 " << steps_used 
             << "，实际 " << weight_size << "）" << endl;
        return 0.0f;
    }
    
    // 5. 计算加权误差
    float error = 0.0f;
    float weight_count = 0.0f;
if(params.weight_case == 1.0f){
    for (int i = 0; i < steps_used; ++i) 
    {
        //int idx = start_idx - i; // 自动确保 idx >= 0
        error += (centerline[i+start].x - 80) * weight1[i];
        weight_count += weight1[i];
    }
}
else{
    for (int i = 0; i < steps_used; ++i) 
    {
        //int idx = i - i; // 自动确保 idx >= 0
        error += (centerline[i+start].x - 80) * weight[i];
        weight_count += weight[i];
    }
}
    // 6. 检查权重和是否为0（避免除零）
    if (fabs(weight_count) < 1e-6f) {
        cerr << "错误：权重和为0！" << endl;
        return 0.0f;
    }
    
    return error / weight_count;
}
void MotionController::update_shared_error(float err) {
    std::lock_guard<std::mutex> lock(error_mutex);
    shared_error = err;
}
float MotionController::get_shared_error() {
    std::lock_guard<std::mutex> lock(error_mutex);
    return shared_error;
}