#ifndef _class_thread_wrapper_class_h_
#define _class_thread_wrapper_class_h_


#include <pthread.h>
#include <iostream>
#include <functional>
#include <sys/timerfd.h>
#include <unistd.h>
#include <inttypes.h>
#include <thread>
#include <chrono>

#include "zf_common_headfile.h"

class timer_fd 
{
public:
    timer_fd(int interval, const std::function<void()>& func)
        : interval(interval), func(func), running(false) {}

    ~timer_fd() {
        stop();
    }

    void start() 
    {
        if (running) return;
        running = true;
        timerThread = std::thread(&timer_fd::timer_loop, this);
    }

    void stop() {
        if (!running) return;
        running = false;
        if (timerThread.joinable()) {
            timerThread.join();
        }
    }

private:

    int interval;
    std::function<void()> func;
    std::thread timerThread;
    bool running;

    int init_timer_fd() 
    {
        int fd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (fd == -1) {
            perror("timerfd_create");
            return -1;
        }

        struct itimerspec its;
        its.it_value.tv_sec = 0;
        its.it_value.tv_nsec = interval * 1000000;
        its.it_interval.tv_sec = 0;
        its.it_interval.tv_nsec = interval * 1000000;
 
        if (timerfd_settime(fd, 0, &its, NULL) == -1) {
            perror("timerfd_settime");
            close(fd);
            return -1;
        }

        return fd;
    }

    void timer_loop()
    {
        int fd = init_timer_fd();
        if (fd == -1) return;

   

        // 尝试设置新的高优先级,值越大优先级越高（1-99）
        struct sched_param param;
        param.sched_priority = 10;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

        uint64_t expirations;
        while (running) 
        {
            if (read(fd, &expirations, sizeof(expirations)) != sizeof(expirations)) 
            {
                perror("read");
                break;
            }

            // printf("expirations = %d\r\n", expirations);

            for (uint64_t i = 0; i < expirations; ++i)
            {
                func();
            }
        }
        close(fd);
    }
};

    
#endif