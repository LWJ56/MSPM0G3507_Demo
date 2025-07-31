#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "ti_msp_dl_config.h"
#include "DJI_motor_drv.h"


void Bsp_can_Init(void);
void Can0_TX(uint32_t id, uint8_t* txdata);

extern DJI_motor_Measure motor_3508_L;
extern DJI_motor_Measure motor_3508_R;

extern DJI_motor_Measure motor_6020_yaw;
extern DJI_motor_Measure motor_6020_pitch;


#endif /* __BSP_CAN_H__ */

