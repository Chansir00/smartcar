#ifndef SOCKET_H
#define SOCKET_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>


int sendImageOverSocket();

#endif