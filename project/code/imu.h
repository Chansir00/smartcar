#ifndef IMU_H
#define IMU_H

#include <zf_common_headfile.h>

using namespace std; 

// 卡尔曼滤波结构体
struct Kalman
{
    float angle;      // 估计的角度 (θ)
    float bias;       // 陀螺仪的零偏 (b)
    float P[2][2];    // 误差协方差矩阵
    float Q_angle;    // 过程噪声（角度）
    float Q_bias;     // 过程噪声（零偏）
    float R_measure;  // 观测噪声（加速度计）
};

void imu_data_get(void);
void Kalman_Init(Kalman *kf);
float Kalman_Update(Kalman *kf, float acc_angle, float gyro_rate, float dt) ;
float rotation(float gyro_z, float dt);
#endif