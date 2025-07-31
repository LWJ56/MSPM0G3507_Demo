#include "Bsp_uart.h"
#include <stdio.h>
#include <string.h>
#include "JY901P.h"



volatile uint8_t gCheckUART = 0;
volatile uint8_t uart_data = 0;

// DMA接收缓冲区
uint8_t rxdata[JY901P_BUFFER_SIZE] = {0};

uint8_t host_data_buffer[HostDataBufferSize*2] = {0}; // 上位机数据缓冲区
int8_t host_data[HostDataBufferSize] = {0}; // 上位机数据

void uart_clear_overrun_error_correctly(void);



void Bsp_uart_Init(void)
{
	//清除串口0中断标志
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
		//清除串口1中断标志
	NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
	
	//设置DMA0搬运的起始地址
	DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) (&UART_0_INST->RXDATA));
	//设置DMA0搬运的目的地址
	DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) rxdata);
    
  DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, JY901P_BUFFER_SIZE);

	//使能DMA0通道
	DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

    //设置DMA1搬运的起始地址
	DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) (&UART_1_INST->RXDATA));
	//设置DMA1搬运的目的地址
	DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) host_data_buffer);

  DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, HostDataBufferSize*2);

	//使能DMA1通道
	DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

	//使能串口0中断
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
	//使能串口1中断
	NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
}


//串口发送单个字符
void uart1_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_1_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_1_INST, ch);
}
//串口发送字符串
void uart1_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart1_send_char(*str++);
    }
}

volatile uint8_t gCheckUART;



//串口0的中断服务函数
void UART_0_INST_IRQHandler(void)
{

   switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
       case DL_UART_MAIN_IIDX_DMA_DONE_RX:
           gCheckUART = 1;
           DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID); // 关闭 DMA，防止继续搬运数据
           
           // 处理接收到的数据
            JY901P_ProcessBuffer(rxdata, JY901P_BUFFER_SIZE);
            
            // 清空缓冲区
            memset(rxdata, 0, JY901P_BUFFER_SIZE);
            
            // 重新配置DMA
            DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
            DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)rxdata);
            DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, JY901P_BUFFER_SIZE);
            
            // 重新启用DMA
            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

           break;





       default:
           break;
   }
		

}

uint8_t count;
static void Process_Host_buffer(uint8_t* buffer, uint16_t length)
{
     // 处理上位机数据
    for(int i=0;i<length;i++)
    {
        if(buffer[i] == 0xFF && buffer[i+5] == 0xFE)
        {
            // 数据合法，进行处理
            memcpy(host_data, &buffer[i], length);
            count++;
        }
    }
}

  

//串口1的中断服务函数
void UART_1_INST_IRQHandler(void)
{

   switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
       case DL_UART_MAIN_IIDX_DMA_DONE_RX:
       
           DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID); // 关闭 DMA，防止继续搬运数据
           
           // 处理接收到的数据
            Process_Host_buffer(host_data_buffer, HostDataBufferSize);
            
            // 清空缓冲区
            memset(host_data_buffer, 0, HostDataBufferSize*2);
            
            // 重新配置DMA
            DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_1_INST->RXDATA));
            DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)host_data_buffer);
            DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, HostDataBufferSize*2);

            // 重新启用DMA
            DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

           break;

				



       default:
           break;
   }
		

}


// //串口的中断服务函数
// void UART_0_INST_IRQHandler(void)
// {
// //			gCheckUART= DL_UART_getPendingInterrupt(UART_0_INST);

// //     //如果产生了串口中断
// //     switch( DL_UART_getPendingInterrupt(UART_0_INST) )
// //     {
// //         case DL_UART_IIDX_RX://如果是接收中断
// //             //接发送过来的数据保存在变量中
// //             uart_data = DL_UART_Main_receiveData(UART_0_INST);
// // //            //将保存的数据再发送出去
// // //            uart0_send_char(uart_data);
// //             break;
// //         case DL_UART_IIDX_RX_TIMEOUT_ERROR://接收超时错误中断
// //             gCheckUART = 6;
// //             break;

// //         case DL_UART_IIDX_OVERRUN_ERROR://溢出错误中断
// //						
// // 						gCheckUART = 7; // 设置溢出错误标志
// //				 uart_clear_overrun_error_correctly();
// // 						break;

// //         default://其他的串口中断
// //             break;
// //     }
			

//    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
//        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
//            gCheckUART = 1;
//            DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID); // 关闭 DMA，防止继续搬运数据
//             // 检查 FIFO 是否为空，若不为空则清理 FIFO，避免影响后续数据
//            if (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
//                while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
//                    // 可选择读出 FIFO 剩余数据到 err_buff，这里示例注释了实际读出操作
//                    // DL_UART_drainRXFIFO(UART_0_INST, err_buff, 3); 
//                    uart_data = DL_UART_receiveData(UART_0_INST); 
//                }
//                // 若没找到 DMA 复位/目标地址复位寄存器，重置整个 DMA 配置
//                SYSCFG_DL_DMA_init(); 
//                DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
//                DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)rxdata);
//                DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 2*RxPacket_SIZE);
//            } 
//            // 数据合法性校验，根据需求修改判断条件
//            else if (rxdata[0] != 0x55 || rxdata[1] != 0x55 || rxdata[2] != 0x55) {
//                SYSCFG_DL_DMA_init(); 
//                DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
//                DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)rxdata);
//                DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 2*RxPacket_SIZE);
//            } 
//            // 数据合法则回发数据（示例操作，按需调整）
//            else {
//                memcpy(RxPacket,rxdata,sizeof(RxPacket)); 
//            }

//            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID); // 重新使能 DMA 通道
//            break;


//            // 接收超时中断分支，触发条件：FIFO 非空且发生超时
//        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
//            // DL_GPIO_togglePins(RGB_PORTE_PORT, RGB_PORTE_RGB_B_PIN); // 可用于调试的电平翻转，示例注释
//            DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID); // 关闭 DMA

//            // 清理 FIFO 剩余数据
//            while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
//                // DL_UART_drainRXFIFO(UART_0_INST, err_buff, 3); // 读出 FIFO 数据到 err_buff，示例注释
//                uart_data = DL_UART_receiveData(UART_0_INST); 
//            }

//            // 重置 DMA 配置
//            SYSCFG_DL_DMA_init(); 
//            DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_0_INST->RXDATA));
//            DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)rxdata);
//            DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 2*RxPacket_SIZE);

//            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID); // 重新使能 DMA
//            break;




//        default:
//            break;
//    }
		

// }




/**
 * @brief 清除UART Overrun错误
 */
void uart_clear_overrun_error_correctly(void)
{
    // 1. 先清空RX FIFO中的所有数据
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
				{
					volatile uint32_t dummy = DL_UART_receiveData(UART_0_INST);
					(void)dummy; // 避免编译器警告
				}
    
    // 2. 直接操作寄存器清除溢出错误标志位
    UART_0_INST->CPU_INT.ICLR = UART_CPU_INT_ICLR_OVRERR_CLR;
    
    // 3. 验证是否清除成功
    uint32_t status = UART_0_INST->CPU_INT.RIS;
    if (status & UART_CPU_INT_RIS_OVRERR_MASK) 
			{
					// 如果还没清除，再试一次
					UART_0_INST->CPU_INT.ICLR = UART_CPU_INT_ICLR_OVRERR_CLR;
			}
		
//		DL_UART_clearInterruptStatus(UART_0_INST, DL_UART_INTERRUPT_OVERRUN_ERROR);//操作库函数

}

/**
 * @brief 完全重置UART接收部分
 */
void uart_reset_rx_on_overrun(void)
{
    // 1. 禁用UART接收
    UART_0_INST->CTL0 &= ~UART_CTL0_RXE_MASK;
    
    // 2. 等待当前操作完成
    while (DL_UART_isBusy(UART_0_INST)) {
        // 等待
    }
    
    // 3. 清空FIFO
    while (!DL_UART_isRXFIFOEmpty(UART_0_INST)) {
        volatile uint32_t dummy = UART_0_INST->RXDATA;
        (void)dummy;
    }
    
    // 4. 清除所有错误标志
    UART_0_INST->CPU_INT.ICLR = UART_CPU_INT_ICLR_OVRERR_CLR | 
                                UART_CPU_INT_ICLR_BRKERR_CLR |
                                UART_CPU_INT_ICLR_PARERR_CLR |
                                UART_CPU_INT_ICLR_FRMERR_CLR;
    
    // 5. 重新启用UART接收
    UART_0_INST->CTL0 |= UART_CTL0_RXE_ENABLE;
}



int fputc(int ch, FILE *stream)
{
        while( DL_UART_isBusy(UART_0_INST) == true );

        DL_UART_Main_transmitData(UART_0_INST, ch);

        return ch;
}

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
        int handle;
};
#endif
FILE __stdout;
//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
        x = x;
}
#endif

