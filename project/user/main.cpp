#include <zf_common_headfile.h>

// g++ -g -o main main.cpp src/camera.cpp -Iinclude/ `pkg-config --cflags --libs opencv4`;./main

int debugmode = 2;
const int camera = 0;
int flag = 0;
unsigned int send_counter = 0;
const unsigned int SEND_INTERVAL = 20;
static int ready_count = 0;
extern bool slow_down;
int main()
{
    VideoCapture cap = cap_init(camera);
    //  创建车道检测器    }
    LaneProcessor detector;
    detector.initializeVariables(image_w, image_h);
    // int new_socket = 0;
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    MotionController ctrl(&detector);
    // Kalman滤波器初始化
    Kalman kf_roll;
    Kalman_Init(&kf_roll);
    ctrl.motor_control(0, 0, 0);
    brush_init(700); // 初始化
    int new_socket = sendImageOverSocket();
    if (readFlag(start_file))
    {
        pit_ms_init(5, [&ctrl, &detector]()
                    { 
            if (!detector.lost && ready_count > 15)
            {
                ctrl.pit_callback();
                if (!slow_down)
                    ctrl.motor_control(430, 0, 0); // 将控制逻辑移到定时器回调200
                else
                    ctrl.motor_control(200, 0, 0); // 将控制逻辑移
            }
            else if(detector.lost)
            {
                ctrl.motor_control(0, 0, 0);  // 将控制逻辑移到定时器回调200
                brush_off();

            } });
    }
    // pwm_set_duty(SERVO_MOTOR1_PWM, 3870);

    while (true)
    {
        if (ready_count < 20)
            ready_count++;
        clock_t start = clock();
        Mat frame;
        cap >> frame; // 从摄像头读取一帧
        if (frame.empty())
        {
            cerr << "错误：读取帧失败！" << endl;
            break;
        }

        imu_data_get();
        DetectionResult result = detector.detect(frame);

        if (detector.lost)
        {
            break;
        }
        float error = ctrl.Err_sum(detector.centerLine);
        switch (detector.circleState)
        {
        case RIGHT_CIRCLE_DETECTED:
            error += 0;
            break;
        case RIGHT_CIRCLE_INTRY:
            break;
        case LEFT_CIRCLE_INTRY:
            break;
        case RIGHT_CIRCLE_EXITING:
            break;
        case LEFT_CIRCLE_EXITING:
            break;
        default:
            break;
        }
        flag = 1;
        // ctrl.update_shared_error(error);
        cerr << "error: " << error << endl;
        ctrl.set_servo_angle(error);
        // pwm_set_duty(MOTOR1_PWM, 1000); // 小占空比
        // gpio_set_level(MOTOR1_DIR, 1);
        // pwm_set_duty(MOTOR2_PWM, 1000); // 小占空比
        // gpio_set_level(MOTOR2_DIR, 0);
        // cerr << "Forward countl: " << encoder_get_count(ENCODER_1) <<"Forward countr: " << encoder_get_count(ENCODER_2) << endl;
        if (debugmode == 1)
        {
            flag = 1;
        }
        else if (debugmode == 2 && readFlag(show_file))
        {
            vector<uchar> buf;
            vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
            try
            {
                // 将图像编码为JPEG格式
                cv::imencode(".jpg", result.outputImage, buf, params);

                // 发送图像大小
                unsigned long img_size = buf.size();
                send(new_socket, &img_size, sizeof(img_size), 0);

                // 发送图像数据
                send(new_socket, buf.data(), img_size, 0);
            }
            catch (const std::exception &e)
            {
                std::cerr << "错误: " << e.what() << std::endl;
            }
        }
        else if (debugmode == 3)
        {
            if (getchar() == 'k')
            {
                imwrite("test.jpg", result.outputImage);
            }
        }
        if (++send_counter >= SEND_INTERVAL)
        {
            send_counter = 0;
            ctrl.send_debug2(); // 调用数据发送函数
        }
        clock_t end = clock();
        double duration = double(end - start) / CLOCKS_PER_SEC * 1000; // 转为毫秒

        std::cout << "函数耗时: " << duration << " 毫秒" << std::endl;
    }

    // 释放摄像头资源
    cap.release();
    // 释放资源
    close(new_socket);

    return 0;
}
