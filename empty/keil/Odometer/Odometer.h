#ifndef __ODOMETER_H__
#define __ODOMETER_H__

#include "ti_msp_dl_config.h"

typedef struct
{
	double theta_mid;
	double theta;
	double Omega;
	double dt;
	double V;
	double x;
	double y;
}Odometer;

void Odometer_update(int16_t* V_L,int16_t* V_R);


// 添加新的函数声明
void Odometer_fuse_IMU(float imu_yaw, float imu_pitch, float imu_roll);
void Odometer_fuse_IMU_Kalman(float imu_yaw, float imu_gyro_z);
void Odometer_get_pose(float* x, float* y, float* theta);
void Odometer_reset_pose(float x, float y, float theta);


#endif 

