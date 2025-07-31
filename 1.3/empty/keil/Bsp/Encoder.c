#include "Encoder.h"

void Encoder_Init(void)
{
	//清除定时器G6中断标志
	NVIC_ClearPendingIRQ(TIMER_Encoder_INST_INT_IRQN);
	//使能定时器G6中断
	NVIC_EnableIRQ(TIMER_Encoder_INST_INT_IRQN);
}

uint16_t num_gpio=30000;
uint16_t num_gpio_old=0;
int16_t num_gpio_diff=0;
void Encoder_Process(void)
{
	if(DL_GPIO_getEnabledInterruptStatus(QEI_PORT,QEI_PIN_1_PIN))//A相双倍频 
	{

        if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_0_PIN)>0)
        {
            if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_1_PIN)>0)
            {
                num_gpio++;
            }
            else
            {
                num_gpio--;
            }

        }
        else if (DL_GPIO_readPins(QEI_PORT,QEI_PIN_0_PIN)==0)
        {
            if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_1_PIN)==0)
            {
                num_gpio++;
            }
            else
            {
                num_gpio--;
            }
        }
    
		
		DL_GPIO_clearInterruptStatus(QEI_PORT,QEI_PIN_1_PIN);
	}

	if(DL_GPIO_getEnabledInterruptStatus(QEI_PORT,QEI_PIN_0_PIN))//B相双倍频
	{
		if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_1_PIN)>0)
		{
			if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_0_PIN)==0)
			{
				num_gpio++;
			}
			else
			{
				num_gpio--;
			}
		}
		else if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_1_PIN)==0)
		{
			if(DL_GPIO_readPins(QEI_PORT,QEI_PIN_0_PIN)>0)
			{
				num_gpio++;
			}
			else
			{
				num_gpio--;
			}
		}
			DL_GPIO_clearInterruptStatus(QEI_PORT,QEI_PIN_0_PIN);
	}
	
	
}


//定时器G6的中断服务函数 已配置为1ms秒的周期
void TIMER_Encoder_INST_IRQHandler(void)
{
	//如果产生了定时器中断
	switch( DL_TimerG_getPendingInterrupt(TIMER_Encoder_INST) )
	{
		case DL_TIMER_IIDX_ZERO://如果是0溢出中断
//				//将LED灯的状态翻转
//				DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
				if(num_gpio - num_gpio_old<30000 | num_gpio - num_gpio_old>(-30000))
				{
					num_gpio_diff= num_gpio - num_gpio_old;
					num_gpio_old = num_gpio;
				}
				else if(num_gpio - num_gpio_old>30000)//下溢出
				{
					num_gpio_diff= num_gpio - num_gpio_old - 65536;
					num_gpio_old = num_gpio;
				}
				else if(num_gpio - num_gpio_old<(-30000))//上溢出
				{
					num_gpio_diff= num_gpio - num_gpio_old + 65536;
					num_gpio_old = num_gpio;
				}

				break;

		default://其他的定时器中断
				break;
	}
}


