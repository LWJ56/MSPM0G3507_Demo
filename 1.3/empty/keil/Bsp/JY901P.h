#ifndef JY901P_H
#define JY901P_H

#include "ti_msp_dl_config.h"
#include <ti/driverlib/dl_uart_main.h>
#include <stdint.h>
#include <stdbool.h>

// JY901P数据包定义
#define JY901P_PACKET_SIZE 11    // 数据包大小(0x55 + Type + 8字节数据 + SUM)
#define JY901P_BUFFER_SIZE 11    // 缓冲区大小(3个数据包)

// 数据包解析状态枚举
typedef enum {
    STATE_WAIT_HEADER = 0,      // 等待包头0x55
    STATE_WAIT_TYPE,            // 等待数据类型
    STATE_RECEIVING_DATA,       // 接收数据中
    STATE_PACKET_COMPLETE       // 数据包完成
} PacketState;

// JY901P数据类型定义
typedef enum {
    JY901P_TYPE_TIME = 0x50,        // 时间
    JY901P_TYPE_ACC = 0x51,         // 加速度
    JY901P_TYPE_GYRO = 0x52,        // 角速度
    JY901P_TYPE_ANGLE = 0x53,       // 角度
    JY901P_TYPE_MAG = 0x54,         // 磁场
    JY901P_TYPE_DPORT = 0x55,       // 端口状态
    JY901P_TYPE_PRESS = 0x56,       // 气压高度
    JY901P_TYPE_LONLAT = 0x57,      // 经纬度
    JY901P_TYPE_GPSV = 0x58,        // 地速
    JY901P_TYPE_QUAT = 0x59,        // 四元数
    JY901P_TYPE_GPS_ACCURACY = 0x5A, // GPS定位精度
    JY901P_TYPE_READ = 0x5F         // 读取
} JY901P_DataType;

// JY901P数据结构体
typedef struct {
    // 时间数据
    uint8_t year, month, day, hour, minute, second;
    uint16_t millisecond;
    
    // 加速度数据 (单位: g)
    float acc_x, acc_y, acc_z;
    
    // 角速度数据 (单位: °/s)
    float gyro_x, gyro_y, gyro_z;
    
    // 角度数据 (单位: °)
    float roll, pitch, yaw;
    
    // 磁场数据
    int16_t mag_x, mag_y, mag_z;
    
    // 气压高度数据
    int32_t pressure;
    float altitude;
    
    // 温度数据 (单位: °C)
    float temperature;
    
    // 数据有效标志
    bool data_valid[16];
    
    // 数据更新标志
    bool data_updated;
    
} JY901P_Data;

//函数声明
void JY901P_ProcessBuffer(uint8_t* buffer, uint16_t length);

extern JY901P_Data jy901p_data;

#endif /* JY901P_H */


