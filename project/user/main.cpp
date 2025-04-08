#include <zf_common_headfile.h>

// g++ -g -o main main.cpp src/camera.cpp -Iinclude/ `pkg-config --cflags --libs opencv4`;./main

int debugmode = 2;
const int camera = 0;
int flag = 0 ;


int main()
{
    // 打开摄像头
    VideoCapture cap(camera, CAP_V4L2); // 0 表示默认摄像头，如果有多个摄像头可以尝试 1, 2, ...
    if (!cap.isOpened())
    {
        cerr << "错误：无法打开摄像头！" << endl;
        return -1;
    }
    // 设置摄像头分辨率
    cap.set(CAP_PROP_FRAME_WIDTH, 320);
    cap.set(CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(CAP_PROP_FPS, 30);
    cerr<<"摄像头分辨率: " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT) << endl;

    // 创建车道检测器    }
    LaneProcessor detector;
    detector.initializeVariables(image_w, image_h);
    // 主循环：读取帧并处理
    int new_socket = sendImageOverSocket();
    //int new_socket = 0;
    atexit(cleanup);
    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);
    MotionController ctrl;
    pit_ms_init(10, [&ctrl](){ 
        if(flag == 1){
        ctrl.pit_callback();
        ctrl.motor_control(200, 0, 0);  // 将控制逻辑移到定时器回调200
        }
    });


    unsigned int send_counter = 0;
    const unsigned int SEND_INTERVAL = 20;

    while (true)
    {
        Mat frame;
        cap >> frame; // 从摄像头读取一帧
        if (frame.empty())
        {
            cerr << "错误：读取帧失败！" << endl;
            break;
        }
        DetectionResult result = detector.detect(frame);
        float error = ctrl.Err_sum(detector.centerLine);
        flag = 1 ;
        cerr<<"error: "<<error<<endl;
        ctrl.set_servo_angle(error);
        //pwm_set_duty(MOTOR1_PWM, 2000); // 小占空比
        //gpio_set_level(MOTOR1_DIR, 0);
        //pwm_set_duty(MOTOR2_PWM, 2000); // 小占空比
        //gpio_set_level(MOTOR2_DIR, 1);
        //cerr << "Forward countl: " << encoder_get_count(ENCODER_1) <<"Forward countr: " << encoder_get_count(ENCODER_2) << endl;
        if (debugmode==1)
        {
            // imshow("原始帧", frame);
            imshow("二值化图像", result.binaryImage);
            imshow("车道检测结果", result.warpedImage);
            // 按下 ESC 键退出
            if (waitKey(30) == 27)
            {
                cout << "程序已退出。" << endl;
                break;
            }
        }
        else if(debugmode == 2)
        {
            vector<uchar> buf;
            vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
            try {
                // 将图像编码为JPEG格式
                cv::imencode(".jpg", result.outputImage, buf, params);
        
                // 发送图像大小
                unsigned long img_size = buf.size();
                send(new_socket, &img_size, sizeof(img_size), 0);
        
                // 发送图像数据
                send(new_socket, buf.data(), img_size, 0);
        
            } catch (const std::exception& e) {
                std::cerr << "错误: " << e.what() << std::endl;
            }
        }
        else if(debugmode==3)
        {
            if(getchar() == 'k')
            {
                imwrite("test.jpg", result.outputImage);
            }
        }
        if(++send_counter >= SEND_INTERVAL) {
            send_counter = 0;
            ctrl.send_debug2();  // 调用数据发送函数
        }
        
    }

    // 释放摄像头资源
    cap.release();
    destroyAllWindows();
    // 释放资源
    close(new_socket);

    return 0;
}