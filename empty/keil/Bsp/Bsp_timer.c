#include "Bsp_timer.h"

#include "Odometer.h"
#include "DJI_motor_drv.h"

extern int16_t V_L;

extern DJI_motor_Measure motor_3508_L,motor_3508_R;


void Bsp_timer_Init(void)
{
	  //清除定时器中断标志
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    //使能定时器中断
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
}



//定时器的中断服务函数 已配置为1秒的周期
void TIMER_0_INST_IRQHandler(void)
{
	//如果产生了定时器中断
	switch( DL_TimerG_getPendingInterrupt(TIMER_0_INST) )
	{
		case DL_TIMER_IIDX_ZERO://如果是0溢出中断
//						//将LED灯的状态翻转
//						DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
//				Odometer_update(&V_L,&motor_3508_R.speed_rpm);
				
				break;

		default://其他的定时器中断
				break;
	}
}


// DL_TimerG_setCaptureCompareValue(PWM_0_INST,i,GPIO_PWM_0_C0_IDX); //PWM占空比修改

