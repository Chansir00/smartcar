#include<imu.h>

void imu_data_get(void) {
    // 获取加速度计数据
    imu963ra_get_acc();
    // 获取陀螺仪数据
    imu963ra_get_gyro();
    // 获取磁力计数据
    imu963ra_get_mag();
}

// 初始化卡尔曼滤波器
void Kalman_Init(Kalman *kf) {
    kf->angle = 0;
    kf->bias = 0;
    kf->P[0][0] = 0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0;
    kf->Q_angle = 0.001f;   // 调参：角度过程噪声
    kf->Q_bias = 0.003f;    // 调参：零偏过程噪声
    kf->R_measure = 0.03f;  // 调参：加速度计噪声
}

// 卡尔曼滤波更新（返回估计的角度）
float Kalman_Update(Kalman *kf, float acc_angle, float gyro_rate, float dt) {
    // 1. 预测（陀螺仪积分）
    kf->angle += (gyro_rate - kf->bias) * dt;

    // 更新误差协方差矩阵 P = F * P * F^T + Q
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // 2. 更新（加速度计校正）
    float y = acc_angle - kf->angle;  // 测量残差
    float S = kf->P[0][0] + kf->R_measure;  // 创新协方差
    float K[2];  // 卡尔曼增益
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // 更新状态和协方差
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    kf->P[0][0] -= K[0] * kf->P[0][0];
    kf->P[0][1] -= K[0] * kf->P[0][1];
    kf->P[1][0] -= K[1] * kf->P[0][0];
    kf->P[1][1] -= K[1] * kf->P[0][1];

    return kf->angle;
}