#include "Bsp_can.h"
#include "DJI_motor_drv.h"


void Bsp_can_Init(void)
{
	NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);//开启CAN中断
	while(DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST))
	{;}

	/**  DL_MCAN_TXBufTransIntrEnable(MCAN_Regs *mcan, uint32_t bufNum, bool enable)
     * @brief 启用或禁用指定发送缓冲区的传输完成中断
     * 
     * 该函数用于配置MCAN发送缓冲区的传输完成中断使能。当指定缓冲区中的消息
     * 发送完成（成功或失败）时，如果启用了中断，将触发相应的中断处理程序。
     * 
     * @param mcan      MCAN控制器实例指针，例如 MCAN0_INST
     * @param bufNum    发送缓冲区编号 (0-31)
     *                  - 范围：0 到 MCANSS_TX_BUFFER_MAX-1 (最大32个缓冲区)
     *                  - 每个缓冲区可以独立配置中断使能
     * @param enable    中断使能控制
     *                  - true:  启用该缓冲区的传输完成中断
     *                  - false: 禁用该缓冲区的传输完成中断
     * 
     * @return int32_t  执行状态
     *         - STW_SOK (0):    操作成功
     *         - STW_EFAIL (-1): 操作失败（bufNum超出范围）
     * 
     * @note 注意事项：
     *       1. 此函数通常在初始化阶段调用一次，不需要每次发送都调用
     *       2. 缓冲区编号必须与发送时使用的缓冲区编号一致
     *       3. 中断使能后，发送完成时会触发 MCAN_IRQHandler 中断处理函数
     *       4. 需要配合 NVIC_EnableIRQ() 使用以启用NVIC中断
    */
		DL_MCAN_TXBufTransIntrEnable(MCAN0_INST, 1U, 1U);
}

void Can0_TX(uint32_t id, uint8_t* txdata)//经典can
{
	DL_MCAN_TxBufElement txMsg;
	txMsg.id = (id << 18U); // 设置ID
	txMsg.dlc = 8;          // 数据长度为8字节

	
    txMsg.rtr = 0U;/* Transmit data frame. */
    
    txMsg.xtd = 0U;/* 11-bit standard identifier. */
    
    txMsg.esi = 0U;/* ESI bit in CAN FD format depends only on gError passive flag. */
    
    txMsg.brs = 0U;/* CAN FD frames transmitted with bit rate switching. */
    
    txMsg.fdf = 0U;/* Frame transmitted in CAN FD format. */
    
    txMsg.efc = 1U;/* Store Tx events. */
    
    txMsg.mm = 0xAAU;/* Message Marker. */

	for (int i = 0; i < 8; i++) {
		txMsg.data[i] = txdata[i];  // 填充数据
	}

	// 将消息写入发送缓冲区
	DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 1U, &txMsg);

	// 启用发送缓冲区的传输完成中断
	DL_MCAN_TXBufAddReq(MCAN0_INST, 1U);
}


DL_MCAN_RxFIFOStatus rxFS;
DL_MCAN_RxBufElement rxMsg;

DJI_motor_Measure motor_3508_L,motor_3508_R;

DJI_motor_Measure motor_6020_yaw,motor_6020_pitch;
 

void ProcessRxCANData(DL_MCAN_RxBufElement *rxData)
{
	 // 确认ID是0x201
	if ( rxData->id == ((uint32_t)0x202 << 18U)+ (0x202*2) ) 
		{
			DJI_motor_fbdata(&motor_3508_L, (rxData->data));
		}
	// 确认ID是0x202
	if(rxData->id == ((uint32_t)0x201 << 18U)+ (0x201*2) )
		{
			DJI_motor_fbdata(&motor_3508_R, (rxData->data));
		}
		
	if(rxData->id == ((uint32_t)0x205 << 18U)+ (0x205*2) ) //id为1，6020 2843~3829
		{
			DJI_motor_fbdata(&motor_6020_yaw, (rxData->data));
		}
	if(rxData->id == ((uint32_t)0x206 << 18U)+ (0x206*2) ) //id为2，6020 2500~3384
		{
			DJI_motor_fbdata(&motor_6020_pitch, (rxData->data));
		}
}

void Can0_rx_callback(uint32_t gInterruptLine1Status)
{
	if ((gInterruptLine1Status & MCAN_IR_RF0N_MASK) == MCAN_IR_RF0N_MASK)
    {
			rxFS.num = DL_MCAN_RX_FIFO_NUM_0;
			
			// 获取当前FIFO状态
			DL_MCAN_getRxFIFOStatus(MCAN0_INST, &rxFS);
			
			// 处理FIFO中的所有消息
			while (rxFS.fillLvl > 0) //一次最多缓存三条信息，第四条之后均舍弃
				{
					// 读取一条消息
					DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U, rxFS.num, &rxMsg);
					DL_MCAN_writeRxFIFOAck(MCAN0_INST, rxFS.num, rxFS.getIdx);
					
					// 处理消息
					ProcessRxCANData(&rxMsg);
					
					// 重新获取FIFO状态，检查是否还有消息
					DL_MCAN_getRxFIFOStatus(MCAN0_INST, &rxFS);
				}
			
			// 清除中断标志
			gInterruptLine1Status &= ~(MCAN_IR_RF0N_MASK);
    }
}


//weak void Can0_rx_callback(uint32_t gInterruptLine1Status)
//{
//	;
//}




/**
 *  @brief MCAN0 中断回调函数
 *
 *  @param  无
 *
 *  @return 无
 */
volatile uint32_t gInterruptLine1Status;


DL_MCAN_ProtocolStatus gProtStatus;
volatile uint8_t CANBusOff = 0;

void MCAN0_INST_IRQHandler(void)
{
	
    switch (DL_MCAN_getPendingInterrupt(MCAN0_INST)) 
    {
        case DL_MCAN_IIDX_LINE1:/*接收FIFO新消息、发送、接收、错误状态等的中断 */
							
							/* Check MCAN interrupts fired during TX/RX of CAN package */
							gInterruptLine1Status |= DL_MCAN_getIntrStatus(MCAN0_INST);
							DL_MCAN_clearIntrStatus(MCAN0_INST, gInterruptLine1Status,DL_MCAN_INTR_SRC_MCAN_LINE_1);

							if (gInterruptLine1Status & DL_MCAN_INTERRUPT_BO)
							 {
								DL_MCAN_getProtocolStatus(MCAN0_INST, &gProtStatus);
								if (gProtStatus.busOffStatus == 1) {
									CANBusOff = 1;  
								} else {
									CANBusOff = 0;  
								}
							}

							Can0_rx_callback(gInterruptLine1Status);
							break;
            
						default:
							break;
    }

}



// MCAN 重启函数
void MCAN0_Restart(void)
{
    DL_MCAN_reset(CANFD0);
    delay_cycles(16);
    DL_MCAN_disablePower(CANFD0);
    delay_cycles(32);
    DL_MCAN_enablePower(CANFD0);
    // 等待 MCAN RAM 初始化完成，根据 CPU 频率延时
	// 1600 CPU CYCIes@CPU32MHZ
	// 4000 CPU cycles@CPU80MHz
    delay_cycles(4000); 
    SYSCFG_DL_MCAN0_init();
}

void Can0_detect(void)
{
	if(CANBusOff==1)
	{
		MCAN0_Restart();
		CANBusOff=0;
	}
}

