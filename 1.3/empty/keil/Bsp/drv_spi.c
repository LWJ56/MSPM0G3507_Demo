/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 立创论坛：https://oshwhub.com/forum
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-04     LCKFB-LP    first version
 */

#include "drv_spi.h"
#include <stdio.h>

extern void delay_us(unsigned long __us); 


/** 硬件SPI */
#define SPI_WAIT_TIMEOUT       ((uint16_t)0xFFFF)

/**
  * @brief :SPI初始化(硬件)
  * @param :无
  * @note  :无
  * @retval:无
  */ 
void drv_spi_init( void )
{
	
}

/**
  * @brief :SPI收发一个字节
  * @param :
  *                        @TxByte: 发送的数据字节
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:接收到的字节
  */
uint8_t drv_spi_read_write_byte( uint8_t TxByte )
{
	uint8_t l_Data = 0;
	uint16_t timeOut = 1000;
        
	//发送数据
	DL_SPI_transmitData8(SPI_0_INST, TxByte);
	
	//等待SPI总线空闲
	while(DL_SPI_isBusy(SPI_0_INST))
	{
		if(timeOut <= 0)
		{
//			printf("TransmitData TIMEOUT!! : %d\r\n",__LINE__);
			break;
		}
		
		timeOut--;
		
		delay_us(2);
	}
    
	
    l_Data = DL_SPI_receiveData8(SPI_0_INST);//读取接收数据
	
	timeOut = 500;
	
	//等待SPI总线空闲
	while(DL_SPI_isBusy(SPI_0_INST))
	{
		if(timeOut <= 0)
		{
//			printf("ReceiveData TIMEOUT!! : %d\r\n",__LINE__);
			break;
		}
		
		timeOut--;
		
		delay_us(2);
	}
        
    return l_Data;  //返回
}

/**
  * @brief :SPI收发字符串
  * @param :
  *                        @ReadBuffer: 接收数据缓冲区地址
  *                        @WriteBuffer:发送字节缓冲区地址
  *                        @Length:字节长度
  * @note  :非堵塞式，一旦等待超时，函数会自动退出
  * @retval:无
  */
void drv_spi_read_write_string( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length )
{
    spi_set_nss_low( );//拉低片选
        while( Length-- )
        {
                *ReadBuffer = drv_spi_read_write_byte( *WriteBuffer );                //收发数据
                ReadBuffer++;
                WriteBuffer++;                                //读写地址加1
        }
    spi_set_nss_high( );//拉高片选
}





