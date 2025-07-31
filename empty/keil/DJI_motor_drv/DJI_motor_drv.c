#include "DJI_motor_drv.h"



void DJI_motor_fbdata(DJI_motor_Measure * motor, uint16_t *rx_data)
{
	motor->angle = (uint16_t)rx_data[0]<<8 | rx_data[1];
	motor->speed_rpm = (int16_t)rx_data[2]<<8 | rx_data[3];
	motor->real_current = (uint16_t)rx_data[4]<<8 | rx_data[5];
	motor->temp = (uint8_t)rx_data[6];
}

