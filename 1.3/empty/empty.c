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
#include "Encoder.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"

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

volatile unsigned int delay_times = 0;

//定时器精确延时1ms
void delay_ms(unsigned int ms)
{
        delay_times = ms;
        while( delay_times != 0 );
}


uint8_t i;


PID PID_3508_L,PID_3508_R,PID_6020_Yaw_Pos,PID_6020_Yaw_Speed,PID_6020_Pitch_Pos,PID_6020_Pitch_Speed;
int16_t temp_current1=0;
int16_t temp_current2=0;
int16_t temp_current3=0;
int16_t temp_current4=0;

float pos_max_out=40;
float spedd_max_out=3000;//4000
float k_p=0.004f;//0.008
	
float target_yaw=7476;
float	target_pitch=2749;
#define yaw_max_out 8192
#define yaw_min_out 0
#define pitch_max_out 3800
#define pitch_min_out 2200
#define yaw_init_out 7476
#define pitch_init_out 2749	

uint8_t data_current[8]={0};


uint8_t g_RF24L01RxBuffer[30]; 

float pitch=0,roll=0,yaw=0;   //欧拉角
extern uint8_t mpu_6050_flag;

///////////////////////////
// 在全局变量区域添加状态标志
static uint8_t last_mode = 0;           // 上次模式状态
static uint8_t current_mode = 0;        // 当前模式状态
static uint8_t mode_changed_flag = 0;   // 模式切换标志


// 在全局变量区域添加巡检模式相关变量
#include <math.h>  // 用于fabs函数

// 巡检模式相关变量
static uint32_t patrol_timer = 0;       // 巡检定时器
static uint8_t patrol_step = 0;         // 巡检步骤 (0-2)
static uint8_t patrol_moving = 0;       // 是否正在移动
static float patrol_target = 0;         // 当前巡检目标角度

// 巡检角度定义（直接使用0-8192的值）
#define PATROL_STEP_COUNT 3
static float patrol_angles[PATROL_STEP_COUNT] = {
    0,      // 0
    2730,   // 2730 (约1/3位置)
    5461    // 5461 (约2/3位置)
};

// 角度容差（认为到达目标的误差范围）
#define ANGLE_TOLERANCE 1000

/**
 * @brief 将0-8192范围的值映射到实际yaw角度范围
 * @param value_8192: 0-8192的值
 * @return 映射后的yaw目标值
 */
float Map_8192_To_Yaw(float value_8192)
{
    // 将0-8192映射到yaw_min_out到yaw_max_out
    float range = yaw_max_out - yaw_min_out;  // 8100 - 6200 = 1900
    float normalized = value_8192 / 8192.0f;  // 归一化到0-1
    return yaw_min_out + normalized * range;
}

/**
 * @brief 计算巡检角度数组（映射到实际yaw范围）
 */
void Calculate_Patrol_Angles(void)
{
    for (uint8_t i = 0; i < PATROL_STEP_COUNT; i++) {
        patrol_angles[i] = Map_8192_To_Yaw(patrol_angles[i]);
    }
}

/**
 * @brief 找到下一个合适的巡检位置（保持单一旋转方向）
 * @param current_angle: 当前yaw角度
 * @return 合适的巡检步骤索引 (0-2)
 */
uint8_t Find_Next_Patrol_Step(float current_angle)
{
    // // 巡检角度: 0, 2730, 5461
    // // 为了保持单一旋转方向，选择下一个大于当前角度的位置
    
    // for (uint8_t i = 0; i < PATROL_STEP_COUNT; i++) {
    //     if (patrol_angles[i] > current_angle) {
    //         return i;  // 返回第一个大于当前角度的位置
    //     }
    // }

	// 如果当前角度大于所有巡检角度，选择下一个周期的第一个目标
    // 比如当前8000，下一个应该是2730（下一圈的第二个位置）
    if (current_angle > patrol_angles[0] && current_angle < patrol_angles[1]) {  // 大于2730 小于5461
        return 1;  // 下一个目标是5461
    } else if (current_angle > patrol_angles[1] && current_angle < patrol_angles[2]) {  // 大于5461 小于8192
        return 2;  // 下一个目标是8192
    } else {  // 当前角度大于8192或小于2730
        return 0;  // 下一个目标是2730
    }
    
    // 如果当前角度大于所有巡检角度，回到第一个位置（0）
    return 0;
}

/**
 * @brief 初始化巡检模式（优化版 - 保持旋转方向一致）
 */
void Init_Patrol_Mode(void)
{
    patrol_timer = 0;
    patrol_moving = 1;
    
    // 直接使用原始角度值（不需要映射，因为yaw_min=0, yaw_max=8192）
    patrol_angles[0] = 0;
    patrol_angles[1] = 2730;
    patrol_angles[2] = 5461;
    
    // 根据当前角度选择下一个巡检位置（保持单一旋转方向）
    patrol_step = Find_Next_Patrol_Step(motor_6020_yaw.angle);
    
    // 设置目标为下一个角度位置
    patrol_target = patrol_angles[patrol_step];
    target_yaw = patrol_target;
    target_pitch = pitch_init_out;
    
    // 立即设置PID目标值
    PID_6020_Yaw_Pos.Target_Val = target_yaw;
    PID_6020_Pitch_Pos.Target_Val = target_pitch;
    
    // 清除PID积分项
    PID_6020_Yaw_Pos.Integral = 0;
    PID_6020_Yaw_Speed.Integral = 0;
    PID_6020_Pitch_Pos.Integral = 0;
    PID_6020_Pitch_Speed.Integral = 0;
    
    // 调试输出（可选）
    // printf("Patrol Init: Current=%.1f, Next Step=%d, Target=%.1f\r\n", 
    //        motor_6020_yaw.angle, patrol_step, patrol_target);
}

/**
 * @brief 检查是否到达目标角度
 * @param current_angle: 当前角度
 * @param target_angle: 目标角度
 * @return 1=已到达, 0=未到达
 */
uint8_t Is_Angle_Reached(float current_angle, float target_angle)
{
	float diff = fabs(current_angle - target_angle);
	if(diff > 7500)
	{
		return 1;  // 如果差值大于7500，认为已经到达目标（因为角度是循环的）
	}
    return (fabs(current_angle - target_angle) < ANGLE_TOLERANCE);
}

/**
 * @brief 巡检模式控制逻辑（优化版）
 */
void Patrol_Mode_Control(void)
{
    patrol_timer++;
    
    // 检查是否到达当前目标角度
    if (patrol_moving && Is_Angle_Reached(motor_6020_yaw.angle, patrol_target)) {
        // 到达目标，切换到下一个角度（保持顺序递增）
        patrol_step++;
        
        // 如果完成一个周期，重新开始
        if (patrol_step >= PATROL_STEP_COUNT) {
            patrol_step = 0;
        }
        
        // 设置新的目标角度
        patrol_target = patrol_angles[patrol_step];
        target_yaw = patrol_target;
        
        // 由于yaw是0-8192，可以360°旋转，不需要限幅
        // if(target_yaw > yaw_max_out) target_yaw = yaw_max_out;
        // if(target_yaw < yaw_min_out) target_yaw = yaw_min_out;
        
        // 更新PID目标值
        PID_6020_Yaw_Pos.Target_Val = target_yaw;
        
        // 重置定时器
        patrol_timer = 0;
        
        // 调试输出（可选）
        // printf("Patrol Step %d: Target Yaw = %.1f\r\n", patrol_step, target_yaw);
    }
    
    // 超时保护，如果8秒内没有到达目标，强制切换到下一个
    if (patrol_timer > 8000) {  // 8秒超时
        patrol_step++;
        if (patrol_step >= PATROL_STEP_COUNT) {
            patrol_step = 0;
        }
        
        patrol_target = patrol_angles[patrol_step];
        target_yaw = patrol_target;
        
        PID_6020_Yaw_Pos.Target_Val = target_yaw;
        patrol_timer = 0;
        
        // printf("Patrol Timeout! Force switch to Step %d\r\n", patrol_step);
    }
}


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
	PID_6020_Yaw_Pos.Kp=25;//15 old
	PID_6020_Yaw_Pos.Ki=0.15;//0.05 old
	PID_6020_Yaw_Pos.Kd=0.1;
	PID_6020_Yaw_Speed.Kp=15;//8 old
	PID_6020_Yaw_Speed.Ki=0.15;//0.1 old
	PID_6020_Yaw_Pos.Target_Val=yaw_init_out;

	PID_6020_Pitch_Pos.Kp=15;
	PID_6020_Pitch_Pos.Ki=0.05;
	PID_6020_Pitch_Pos.Kd=0.1;
	PID_6020_Pitch_Speed.Kp=8;
	PID_6020_Pitch_Speed.Ki=0.1;
	PID_6020_Pitch_Pos.Target_Val=pitch_init_out;
	/////////////////////
	
	Bsp_uart_Init();
	Bsp_timer_Init();
	Bsp_adc_Init();
	Encoder_Init();
//	delay_ms(2000);//等待外设上电
	
//	uart1_send_string("0x00");
////	uart1_send_char('\0');
//	delay_ms(1000);
//	uart1_send_string("0x00");
//	uart1_send_char('\0');
	


//		//SPI初始化
//		drv_spi_init( );
//		
//		////RX_MODE
//		NRF24L01_Gpio_Init_receive( );
//		//检测nRF24L01
//		NRF24L01_check( );        
//		//NRF初始化
//		RF24L01_Init( );        
//		RF24L01_Set_Mode( MODE_RX );//NRF接收模式
		
//		////TX_MODE
//		NRF24L01_Gpio_Init_transmit( );
//		//检测nRF24L01
//		NRF24L01_check( );        
//		//NRF初始化
//		RF24L01_Init( );        
//		RF24L01_Set_Mode( MODE_TX );//NRF发送模式

//		 MPU6050_Init();
//		 
//		 //DMP初始化
//      while( mpu_dmp_init() )
//      {
////            printf("dmp error\r\n");
//            delay_ms(200);
//      }
		 
    while(1) 
		{
			
//			 //获取欧拉角
//			if(  i== 20 )
//			{
////				mpu_dmp_get_data(&pitch,&roll,&yaw);
//				i=0;
////						printf("\r\npitch =%d\r\n", (int)pitch);
////						printf("\r\nroll =%d\r\n", (int)roll);
////						printf("\r\nyaw =%d\r\n", (int)yaw);
//			}
			
			// 检测模式状态
			current_mode = (uint8_t)host_data[HostDataBufferSize-3];
			
		// 检测模式切换
    if (current_mode != last_mode) {
        mode_changed_flag = 1;
        last_mode = current_mode;
    }
			
			i++;
			if(i==100)
			{
				DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
			}
			
			delay_ms(1);
			
//			if(i==50)
//			{
//				if( 0 != g_RF24L01RxBuffer[0])
//				{
////					printf("Data = %s\r\n",g_RF24L01RxBuffer);
//					Buff_Clear();
//					DL_GPIO_togglePins(LED1_PORT, LED1_PIN_28_PIN);
//					i=0;
//				}
//				
////				NRF24L01_TxPacket((uint8_t*)"hello LCKFB!\r\n",13);//NRF发送数据

//			}
			
	
			if(current_mode == 0xFF)//瞄靶
			{
				// 如果刚进入瞄靶模式，进行一次初始设置
				if (mode_changed_flag == 1) 
				{
					mode_changed_flag = 0;  // 清除标志

					pos_max_out = 40;  // 设置速度最大输出

					// 进入瞄靶模式时的初始设置
					target_yaw = motor_6020_yaw.angle;      // 重置到当前位置-150
					target_pitch = motor_6020_pitch.angle;  // 重置到当前位置
					
					// 立即设置PID目标值
					PID_6020_Yaw_Pos.Target_Val = target_yaw-150;
					PID_6020_Pitch_Pos.Target_Val = target_pitch;
					spedd_max_out=0;
					delay_ms(5);
					spedd_max_out=2000;
				}
				target_yaw -= 0.010*(int16_t)(host_data[2]<<8|host_data[1]);//0.004 k_p 0.006
//				target_pitch -= k_p*(int16_t)(host_data[4]<<8|host_data[3]);

				
				if(target_yaw>yaw_max_out) target_yaw=yaw_max_out;
				if(target_yaw<yaw_min_out) target_yaw=yaw_min_out;
				if(target_pitch>pitch_max_out) target_pitch=pitch_max_out;
				if(target_pitch<pitch_min_out) target_pitch=pitch_min_out;
				if(host_data[0]==-1 && host_data[HostDataBufferSize-1]==-2)
				{
					PID_6020_Yaw_Pos.Target_Val=target_yaw;
					PID_6020_Pitch_Pos.Target_Val=target_pitch;
				}
			}
			else if(current_mode==0x01)//巡检
			{
				// 如果刚进入巡检模式，进行一次初始设置
				if (mode_changed_flag == 1) 
				{
					mode_changed_flag = 0;  // 清除标志
					// 初始化巡检模式（自动找到最接近的位置开始）
            		Init_Patrol_Mode();
					pos_max_out = 15;  // 设置速度最大输出
					spedd_max_out=2000;
				}
				// 执行巡检控制逻辑
        		Patrol_Mode_Control();
			}
			else  // 其他模式或无效模式
			{
				if (mode_changed_flag == 1) 
				{
					mode_changed_flag = 0;  // 清除标志
					
				}

			}
			temp_current1=(int16_t)AddPID_Realize(&PID_6020_Yaw_Pos,motor_6020_yaw.angle,pos_max_out);
			PID_6020_Yaw_Speed.Target_Val=temp_current1;
			temp_current3=(int16_t)AddPID_Realize(&PID_6020_Yaw_Speed,motor_6020_yaw.speed_rpm,spedd_max_out);
			data_current[0]=temp_current3>>8;
			data_current[1]=temp_current3;

		
			temp_current2=(int16_t)AddPID_Realize(&PID_6020_Pitch_Pos,motor_6020_pitch.angle,pos_max_out);
			PID_6020_Pitch_Speed.Target_Val=temp_current2;
			temp_current4=(int16_t)AddPID_Realize(&PID_6020_Pitch_Speed,motor_6020_pitch.speed_rpm,spedd_max_out);
			data_current[2]=temp_current4>>8;
			data_current[3]=temp_current4;

			Can0_TX(0x1FF, data_current);
			
			
		}
}





void SysTick_Handler(void)
{
        if( delay_times != 0 )
        {
                delay_times--;
        }
}



