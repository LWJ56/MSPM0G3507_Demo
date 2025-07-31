/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"

#include "Bsp_key.h"
#include "Bsp_can.h"
#include "PID.h"
#include "Bsp_uart.h"
#include "Bsp_timer.h"
#include "Bsp_adc.h"
#include "Odometer.h"
#include "JY901P.h"
#include "NRF24L01.h"

volatile unsigned int delay_times = 0;

//定时器精确延时1ms
void delay_ms(unsigned int ms)
{
        delay_times = ms;
        while( delay_times != 0 );
}


#define RECEIVING_MODE        1 // 1:接收模式 0：发送模式


uint8_t i;


PID PID_3508_L,PID_3508_R,PID_6020_Yaw_Pos,PID_6020_Yaw_Speed,PID_6020_Pitch_Pos,PID_6020_Pitch_Speed;
int16_t temp_current1=0;
int16_t temp_current2=0;
int16_t temp_current3=0;
int16_t temp_current4=0;

//float pos_max_out=20;
float spedd_max_out=1000;

float target_yaw=3400;
float	target_pitch=2875;


uint8_t data_current[8]={0};

//unsigned int adc_value = 0;
//int16_t V_L;
int main(void)
{
	SYSCFG_DL_init();
	
	Bsp_key_Init();
	Bsp_can_Init();
	
	PID_Init(&PID_3508_L);//p=10,i=0.05
	PID_Init(&PID_3508_R);//p=10,i=0.05
	PID_Init(&PID_6020_Yaw_Pos);//p=15,i=0.05,d=0.4
	PID_Init(&PID_6020_Yaw_Speed);//p=15,i=0.4
	PID_Init(&PID_6020_Pitch_Pos);//pos:p=15,i=0.05,d=0.1 
	PID_Init(&PID_6020_Pitch_Speed);//speed p=10,i=0.1/0.4
	
	/////////////////////
	PID_6020_Yaw_Pos.Kp=15;
	PID_6020_Yaw_Pos.Ki=0.05;
	PID_6020_Yaw_Pos.Kd=0.1;
	PID_6020_Yaw_Speed.Kp=8;
	PID_6020_Yaw_Speed.Ki=0.1;
	PID_6020_Yaw_Pos.Target_Val=3400;
	
	PID_6020_Pitch_Pos.Kp=15;
	PID_6020_Pitch_Pos.Ki=0.05;
	PID_6020_Pitch_Pos.Kd=0.1;
	PID_6020_Pitch_Speed.Kp=8;
	PID_6020_Pitch_Speed.Ki=0.1;
	PID_6020_Pitch_Pos.Target_Val=2875;
	/////////////////////
	
	Bsp_uart_Init();
	Bsp_timer_Init();
	Bsp_adc_Init();
	
	delay_ms(2000);//等待外设上电
	
//	uart1_send_string("0x00");
////	uart1_send_char('\0');
//	delay_ms(1000);
//	uart1_send_string("0x00");
////	uart1_send_char('\0');
	
			//SPI初始化
		drv_spi_init( );

	 #if RECEIVING_MODE
		////RX_MODE
		NRF24L01_Gpio_Init_receive( );
		//检测nRF24L01
		NRF24L01_check( );        
		//NRF初始化
		RF24L01_Init( );        
		RF24L01_Set_Mode( MODE_RX );//NRF接收模式
		 #else
		////TX_MODE
		NRF24L01_Gpio_Init_transmit( );
		//检测nRF24L01
		NRF24L01_check( );        
		//NRF初始化
		RF24L01_Init( );        
		RF24L01_Set_Mode( MODE_TX );//NRF发送模式
    #endif
    while(1) 
		{
			
			i++;
//			if(i==100)
//			{
//				DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
//			}
//			
			delay_ms(1);
			
//			temp_current1=(int16_t)AddPID_Realize(&PID_3508_R,motor_3508_R.speed_rpm,10000);
//		
//			data_current[0]=temp_current1>>8;
//			data_current[1]=temp_current1;
//			
//			
//			temp_current2=(int16_t)AddPID_Realize(&PID_3508_L,motor_3508_L.speed_rpm,10000);
//		
//			data_current[2]=temp_current2>>8;
//			data_current[3]=temp_current2;
//		
//			Can0_TX(0x200, data_current);
			
//			adc_value=adc_getValue();
//			
//			V_L=-motor_3508_L.speed_rpm;
//			Odometer_update(&V_L,&motor_3508_R.speed_rpm);
			
			

			if(i==50)
			{
				#if RECEIVING_MODE

				if( 0 != g_RF24L01RxBuffer[0])
				{
//					printf("Data = %s\r\n",g_RF24L01RxBuffer);
					Buff_Clear();
					DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
					i=0;
				}
				#else
				NRF24L01_TxPacket((uint8_t*)"hello LCKFB!\r\n",13);//NRF发送数据
				DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
				#endif
			}



		

			target_yaw -= 0.004f*(int16_t)(host_data[2]<<8|host_data[1]);//0.004
			target_pitch += 0.004f*(int16_t)(host_data[4]<<8|host_data[3]);


			
			
			if(target_yaw>3829) target_yaw=3829;
			if(target_yaw<2843) target_yaw=2843;
			if(target_pitch>3384) target_pitch=3384;
			if(target_pitch<2500) target_pitch=2500;
			if(host_data[0]==-1 && host_data[5]==-2)
			{
				PID_6020_Yaw_Pos.Target_Val=target_yaw;
				PID_6020_Pitch_Pos.Target_Val=target_pitch;
			}
			temp_current1=(int16_t)AddPID_Realize(&PID_6020_Yaw_Pos,motor_6020_yaw.angle,40);
			PID_6020_Yaw_Speed.Target_Val=temp_current1;
			temp_current3=(int16_t)AddPID_Realize(&PID_6020_Yaw_Speed,motor_6020_yaw.speed_rpm,spedd_max_out);
			data_current[0]=temp_current3>>8;
			data_current[1]=temp_current3;

		
			temp_current2=(int16_t)AddPID_Realize(&PID_6020_Pitch_Pos,motor_6020_pitch.angle,40);
			PID_6020_Pitch_Speed.Target_Val=temp_current2;
			temp_current4=(int16_t)AddPID_Realize(&PID_6020_Pitch_Speed,motor_6020_pitch.speed_rpm,spedd_max_out);
			data_current[2]=temp_current4>>8;
			data_current[3]=temp_current4;

			Can0_TX(0x1FF, data_current);
			
			
		}
}




//搭配滴答定时器实现的精确us延时
void delay_us(unsigned long __us) 
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
    ticks = __us * (80000000 / 1000000); //80M主频

    // 获取当前的SysTick值
    told = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的SysTick值
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;

            told = tnow;

            // 如果达到了需要的时钟数，就退出循环
            if (tcnt >= ticks)
                break;
        }
    }
}



void SysTick_Handler(void)
{
        if( delay_times != 0 )
        {
                delay_times--;
        }
}



