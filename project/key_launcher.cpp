#include "zf_common_headfile.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#define MAX_KEYS 4

// 按键路径定义
const char* key_paths[MAX_KEYS] = {
    "/dev/zf_driver_gpio_key_0",
    "/dev/zf_driver_gpio_key_1",
    "/dev/zf_driver_gpio_key_2",
    "/dev/zf_driver_gpio_key_3"
};

// 每个按键对应要执行的命令（可自行修改）
const char* key_cmds[MAX_KEYS] = {
    "/home/project",
    "/home/long/program_1",
    "/home/long/program_2",
    "/home/long/program_3"
};

// 上次状态记录（防止重复触发）
int key_last_state[MAX_KEYS] = {1, 1, 1, 1};

// 防抖相关设置
#define DEBOUNCE_CHECKS 3
#define DEBOUNCE_DELAY_MS 10
#define POST_TRIGGER_DELAY_MS 500

// 防抖判断：连续为 0 才判定按下
int is_key_pressed(const char* key_path) {
    for (int i = 0; i < DEBOUNCE_CHECKS; ++i) {
        if (gpio_get_level(key_path) != 0)
            return 0;
        usleep(DEBOUNCE_DELAY_MS * 1000);
    }
    return 1;
}

// 按键轮询函数
void poll_keys() {
    for (int i = 0; i < MAX_KEYS; ++i) {
        int current = gpio_get_level(key_paths[i]);

        if (key_last_state[i] == 1 && current == 0) {
            if (is_key_pressed(key_paths[i])) {
                printf("KEY_%d 被按下，执行命令: %s\n", i, key_cmds[i]);
                system(key_cmds[i]);  // 运行对应程序
                usleep(POST_TRIGGER_DELAY_MS * 1000);
            }
        }

        key_last_state[i] = current;
    }
}

int main(int argc, char** argv)
{
    while (1) {
        poll_keys();  // 轮询所有按键
        usleep(100000);  // 主循环延时100ms
    }
    return 0;
}
