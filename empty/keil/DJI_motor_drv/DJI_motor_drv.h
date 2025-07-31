#ifndef __DJI_MOTOR_DRV_H__
#define __DJI_MOTOR_DRV_H__

#include "ti_msp_dl_config.h"

typedef struct
{
	uint16_t 	angle;				//abs angle range:[0,8191]
	int16_t speed_rpm;
	int16_t real_current;
	int8_t temp;
}DJI_motor_Measure;

void DJI_motor_fbdata(DJI_motor_Measure * motor, uint16_t *rx_data);


#endif 

