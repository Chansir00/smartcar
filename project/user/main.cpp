#include <zf_common_headfile.h>

// g++ -g -o main main.cpp src/camera.cpp -Iinclude/ `pkg-config --cflags --libs opencv4`;./main

int debugmode = readFlag("./debugmode");
const int camera = 0;
int flag = 0;
unsigned int send_counter = 0;
int speed = 820; // 车速
const unsigned int SEND_INTERVAL = 20;
int ready_count = 0;
int control_circle = 0;
int speed_circle = 0; 
float my_error = 0.0f; // 误差
float my_error2 = 0.0f; // 误差2
extern bool slow_down;

// 另外一端的IP地址
#define SERVER_IP "192.168.141.172"
// 端口号
#define PORT 8888

uint32 read_len = 0;
uint8 recv_buff[1024];
uint8 temp_str[] = "seekfree this is udp demo.\r\n";
uint8 read_str[] = "read data:\r\n";
char float_str1[32];
char float_str2[32];

int main()
{
    VideoCapture cap = cap_init(camera);
    //  创建车道检测器    }
    LaneProcessor detector;
    detector.initializeVariables(image_w, image_h);
    MotionController ctrl(&detector);
    // int new_socket = 0;
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    // Kalman滤波器初始化
    Kalman kf_roll;
    Kalman_Init(&kf_roll);
    brush_off();
    ctrl.motor_control(0, 0, 0);
    int new_socket = 0;
     if (udp_init(SERVER_IP, PORT) == 0)
     {
         printf("tcp_client ok\r\n");
         udp_send_data(temp_str, sizeof(temp_str));
         udp_send_data(temp_str, sizeof(temp_str));
         udp_send_data(temp_str, sizeof(temp_str));
     }
     else
     {
         printf("tcp_client error\r\n");
         return -1;
     }
    if (!debugmode)
    {
        brush_init(800);
        gpio_set_level(BEEP, 0x1);
        system_delay_ms(1500);
        gpio_set_level(BEEP, 0x0);
        pit_ms_init(5, [&ctrl, &detector]()
                    { 
            if (!detector.lost && ready_count > 50)
            {
                ctrl.pit_callback();
                ctrl.motor_control(speed, 0, 0); // 将控制逻辑移到定时器回调200
            } 
            else if(detector.lost)
            {
                ctrl.motor_control(0, 0, 0);  // 将控制逻辑移到定时器回调200
                brush_off();

            } });
    }
    else
    {
        new_socket = sendImageOverSocket();
    }

    while (true)
    {
        if (ready_count < 2000)
            ready_count++;
        clock_t start = clock();
        Mat frame;
        cap >> frame; // 从摄像头读取一帧
        if (frame.empty())
        {
            cerr << "错误：读取帧失败！" << endl;
            break;
        }
        // 回显UDP数据。
        sprintf(float_str1, "left_encoder: %.2f\r\n", ctrl.pidLeft.actual);
        //sprintf(float_str2, "right_encoder: %.2f\r\n", ctrl.pidRight.actual);
        udp_send_data((uint8_t *)&float_str1, sizeof(float_str1));
        // udp_send_data((uint8_t *)&float_str2, sizeof(float_str2));
        DetectionResult result = detector.detect(frame);
        control_circle = detector.circleState;
        speed_circle = detector.speed_decide();
        cerr << "speed_circle: " << speed_circle << endl;
        if (detector.lost)
        {
            break;
        }
        my_error = ctrl.Err_sum(detector.centerLine);
        my_error2 = ctrl.Err_sum2(detector.centerLine);
        cout << "my_error2: " << my_error2 << endl;
        flag = 1;
        // ctrl.update_shared_error(error);
        cerr << "error: " << my_error << endl;
        ctrl.set_servo_angle(my_error2);
        //  pwm_set_duty(MOTOR1_PWM, 2000); // 小占空比
        //  gpio_set_level(MOTOR1_DIR, 1);
        //  pwm_set_duty(MOTOR2_PWM, 2000); // 小占空比
        //  gpio_set_level(MOTOR2_DIR, 0);
        //  cerr << "Forward countl: " << encoder_get_count(ENCODER_1) <<"Forward countr: " << encoder_get_count(ENCODER_2) << endl;
        if (debugmode == 0)
        {
            brush_control();
            flag = 1;
        }
        else if (debugmode == 1)
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