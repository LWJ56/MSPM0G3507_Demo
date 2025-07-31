#include "Odometer.h"
#include <math.h>

#define Wheel_track 13.7f //轮距 13.7cm
#define Wheel_radius 1.5f //车轮半径1.5cm
#define Pi 3.14159265358979323846f


// IMU融合参数
#define IMU_WEIGHT 0.98f        // IMU权重 (0.95-0.99)
#define ODOMETER_WEIGHT 0.02f   // 里程计权重 (0.01-0.05)
#define ANGLE_DIFF_THRESHOLD 0.5f // 角度差异阈值(rad)

Odometer Odometer_est;

static float normalize_angle(float angle);
static float angle_difference(float angle1, float angle2);

void Odometer_update(int16_t* V_L,int16_t* V_R)
{
//	Odometer_est.V= (*V_L + *V_R)*Wheel_radius*2*Pi/2/60; //m
	Odometer_est.V= (*V_L + *V_R)*Pi/40.0f; //cm/s
//	Odometer_est.Omega= (*V_R - *V_L)*Wheel_radius*2*Pi/60/Wheel_track; //rad/s
	Odometer_est.Omega= (*V_R - *V_L)*Pi/274.0f;
	
	Odometer_est.dt=0.001f;
	
	Odometer_est.theta_mid = Odometer_est.theta + (Odometer_est.Omega*Odometer_est.dt)/2;
	Odometer_est.x = Odometer_est.x + Odometer_est.V*cosf(Odometer_est.theta_mid)*Odometer_est.dt;
	Odometer_est.y = Odometer_est.y + Odometer_est.V*sinf(Odometer_est.theta_mid)*Odometer_est.dt;
	Odometer_est.theta = Odometer_est.theta + Odometer_est.Omega*Odometer_est.dt;
	// 角度归一化
    Odometer_est.theta = normalize_angle(Odometer_est.theta);


	
	// Odometer_est.x = Odometer_est.x + Odometer_est.V*cosf(Odometer_est.theta)*Odometer_est.dt;
	// Odometer_est.y = Odometer_est.y + Odometer_est.V*sinf(Odometer_est.theta)*Odometer_est.dt;
	// Odometer_est.theta = Odometer_est.theta + (Odometer_est.Omega*Odometer_est.dt);
	// // 角度归一化
    // Odometer_est.theta = normalize_angle(Odometer_est.theta);
}


/**
 * @brief 融合IMU角度数据
 * @param imu_yaw IMU的偏航角 (弧度)
 * @param imu_pitch IMU的俯仰角 (弧度) 
 * @param imu_roll IMU的横滚角 (弧度)
 */
void Odometer_fuse_IMU(float imu_yaw, float imu_pitch, float imu_roll)
{
    // 主要使用偏航角(yaw)进行2D导航
    float imu_yaw_normalized = normalize_angle(imu_yaw);
    
    // 计算角度差异
    float angle_diff = angle_difference(imu_yaw_normalized, Odometer_est.theta);
    
    // 如果角度差异过大，可能是IMU或里程计出现异常
    if (fabsf(angle_diff) < ANGLE_DIFF_THRESHOLD) {
        // 互补滤波器融合
        Odometer_est.theta = IMU_WEIGHT * imu_yaw_normalized + 
                            ODOMETER_WEIGHT * Odometer_est.theta;
    } else {
        // 角度差异过大时，逐渐向IMU角度收敛
        float correction = angle_diff * 0.1f; // 10%的修正
        Odometer_est.theta = Odometer_est.theta + correction;
    }
    
    // 归一化最终角度
    Odometer_est.theta = normalize_angle(Odometer_est.theta);
    
    // 可选：存储俯仰角和横滚角用于倾斜补偿
    // (如果需要3D定位或倾斜补偿)
}

/**
 * @brief 卡尔曼滤波器方式融合IMU (更高级的融合方法)
 * @param imu_yaw IMU偏航角
 * @param imu_gyro_z Z轴角速度 (rad/s)
 */
void Odometer_fuse_IMU_Kalman(float imu_yaw, float imu_gyro_z)
{
    static float P = 1.0f;          // 估计误差协方差
    static float Q = 0.001f;        // 过程噪声
    static float R_angle = 0.01f;   // IMU角度测量噪声
    static float R_gyro = 0.1f;     // 陀螺仪噪声
    
    // 预测步骤
    float theta_predicted = Odometer_est.theta + Odometer_est.Omega * Odometer_est.dt;
    theta_predicted = normalize_angle(theta_predicted);
    P = P + Q;
    
    // 更新步骤 - 使用IMU角度
    float angle_innovation = angle_difference(imu_yaw, theta_predicted);
    float S = P + R_angle;
    float K = P / S;
    
    Odometer_est.theta = theta_predicted + K * angle_innovation;
    P = (1 - K) * P;
    
    // 可选：也可以融合陀螺仪数据
    if (fabsf(imu_gyro_z) > 0.01f) { // 如果有明显的角速度
        float gyro_innovation = imu_gyro_z - Odometer_est.Omega;
        if (fabsf(gyro_innovation) < 1.0f) { // 合理范围内
            Odometer_est.Omega = 0.9f * Odometer_est.Omega + 0.1f * imu_gyro_z;
        }
    }
    
    Odometer_est.theta = normalize_angle(Odometer_est.theta);
}

/**
 * @brief 计算两个角度之间的最小差值
 * @param angle1 角度1
 * @param angle2 角度2  
 * @return 角度差值 [-π, π]
 */
static float angle_difference(float angle1, float angle2)
{
    float diff = angle1 - angle2;
    return normalize_angle(diff);
}


/**
 * @brief 角度归一化到 [-π, π] 范围
 * @param angle 输入角度
 * @return 归一化后的角度
 */
static float normalize_angle(float angle)
{
    while (angle > Pi) {
        angle -= 2.0f * Pi;
    }
    while (angle < -Pi) {
        angle += 2.0f * Pi;
    }
    return angle;
}


/**
 * @brief 获取融合后的姿态信息
 * @param x 输出X坐标
 * @param y 输出Y坐标  
 * @param theta 输出角度
 */
void Odometer_get_pose(float* x, float* y, float* theta)
{
    if (x) *x = Odometer_est.x;
    if (y) *y = Odometer_est.y;
    if (theta) *theta = Odometer_est.theta;
}

/**
 * @brief 重置里程计位姿
 * @param x 新的X坐标
 * @param y 新的Y坐标
 * @param theta 新的角度
 */
void Odometer_reset_pose(float x, float y, float theta)
{
    Odometer_est.x = x;
    Odometer_est.y = y;
    Odometer_est.theta = normalize_angle(theta);
}

