#ifndef __BSP_UART_H__
#define __BSP_UART_H__
#include "ti_msp_dl_config.h"

#define HostDataBufferSize 6


void Bsp_uart_Init(void);
void uart1_send_char(char ch);
void uart1_send_string(char* str);

extern int8_t host_data[HostDataBufferSize]; // 上位机数据



#endif /* __BSP_UART_H__ */

