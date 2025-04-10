#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>

#define PORT 8000
//g++ -g -o socket socket_r.cpp -Iinclude/ `pkg-config --cflags --libs opencv4`;

int main() {
    // 创建Socket
    int sock = 0;
    struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "Socket创建失败！" << std::endl;
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    //192.168.10.92;192.168.66.92
    // 将IP地址从字符串转换为二进制格式
    if (inet_pton(AF_INET, "192.168.108.164", &serv_addr.sin_addr) <= 0) {
        std::cerr << "无效的地址/地址不支持！" << std::endl;
        return -1;
    }

    // 连接到服务器
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "连接失败！" << std::endl;
        return -1;
    }

    std::cout << "已连接到服务端！" << std::endl;

    cv::Mat frame;
    std::vector<uchar> buf;

    try {
        while (true) {
            // 接收图像大小
            unsigned long img_size = 0;
            recv(sock, &img_size, sizeof(img_size), 0);

            if (img_size == 0) break;

            // 调整缓冲区大小
            buf.resize(img_size);

            // 接收图像数据
            size_t total_received = 0;
            while (total_received < img_size) {
                ssize_t received = recv(sock, buf.data() + total_received, img_size - total_received, 0);
                if (received <= 0) {
                    std::cerr << "接收数据失败！" << std::endl;
                    break;
                }
                total_received += received;
            }

            // 解码图像
            frame = cv::imdecode(buf, cv::IMREAD_COLOR);
            if (frame.empty()) {
                std::cerr << "解码图像失败！" << std::endl;
                continue;
            }

            // 显示图像
            cv::imshow("Client", frame);
            if (cv::waitKey(1) == 'q') break; // 按 'q' 键退出
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }

    // 释放资源
    close(sock);
    cv::destroyAllWindows();

    return 0;
}