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

#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "ti_msp_dl_config.h"


#define spi_set_nss_high( )     DL_GPIO_setPins(GPIO_CS_PORT, GPIO_CS_CS_PIN)
#define spi_set_nss_low( )      DL_GPIO_clearPins(GPIO_CS_PORT, GPIO_CS_CS_PIN)



void drv_spi_init( void );
uint8_t drv_spi_read_write_byte( uint8_t TxByte );
void drv_spi_read_write_string( uint8_t* ReadBuffer, uint8_t* WriteBuffer, uint16_t Length );

#endif


