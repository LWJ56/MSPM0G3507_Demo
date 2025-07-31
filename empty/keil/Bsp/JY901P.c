#include "JY901P.h"



JY901P_Data jy901p_data = {0};    

/**
 * @brief JY901P数据校验和计算
 */
static uint8_t JY901P_CheckSum(uint8_t* data, uint8_t len)
{
    uint8_t sum = 0;
    for(int i = 0; i < len; i++)
    {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief 解析JY901P数据包
 */
static bool JY901P_ParseData(uint8_t* data, JY901P_Data* result)
{
    if(data[0] != 0x55) return false;  // 检查包头
    
    // 验证校验和
    uint8_t calculated_sum = JY901P_CheckSum(data, 10);
    if(calculated_sum != data[10]) return false;
    
    uint8_t type = data[1];
    int16_t temp_data[4];

    // 提取4个16位数据
    for(int i = 0; i < 3; i++)
    {
        temp_data[i] = (int16_t)((data[3 + i*2] << 8) | data[2 + i*2]);
    }
    
    switch(type)
    {
        case JY901P_TYPE_TIME:  // 时间数据
            result->year = data[2];
            result->month = data[3];
            result->day = data[4];
            result->hour = data[5];
            result->minute = data[6];
            result->second = data[7];
            result->millisecond = (data[9] << 8) | data[8];
            result->data_valid[0] = true;
            break;
            
        case JY901P_TYPE_ACC:   // 加速度数据
            result->acc_x = (float)temp_data[0] / 32768.0f * 16.0f;  // g
            result->acc_y = (float)temp_data[1] / 32768.0f * 16.0f;  // g
            result->acc_z = (float)temp_data[2] / 32768.0f * 16.0f;  // g
            result->temperature = (float)temp_data[3] / 100.0f;      // °C
            result->data_valid[1] = true;
            break;
            
        case JY901P_TYPE_GYRO:  // 角速度数据
            result->gyro_x = (float)temp_data[0] / 32768.0f * 2000.0f; // °/s
            result->gyro_y = (float)temp_data[1] / 32768.0f * 2000.0f; // °/s
            result->gyro_z = (float)temp_data[2] / 32768.0f * 2000.0f; // °/s
            result->temperature = (float)temp_data[3] / 100.0f;        // °C
            result->data_valid[2] = true;
            break;
            
        case JY901P_TYPE_ANGLE: // 角度数据
            result->roll = (float)temp_data[0] / 32768.0f * 180.0f;  // °
            result->pitch = (float)temp_data[1] / 32768.0f * 180.0f; // °
            result->yaw = (float)temp_data[2] / 32768.0f * 180.0f;   // °
            result->temperature = (float)temp_data[3] / 100.0f;      // °C
            result->data_valid[3] = true;
            break;
            
        case JY901P_TYPE_MAG:   // 磁场数据
            result->mag_x = temp_data[0];
            result->mag_y = temp_data[1];
            result->mag_z = temp_data[2];
            result->temperature = (float)temp_data[3] / 100.0f;      // °C
            result->data_valid[4] = true;
            break;
            
        case JY901P_TYPE_PRESS: // 气压高度数据
            result->pressure = ((int32_t)temp_data[1] << 16) | (uint16_t)temp_data[0];
            result->altitude = (float)temp_data[2] / 100.0f;         // m
            result->temperature = (float)temp_data[3] / 100.0f;      // °C
            result->data_valid[5] = true;
            break;
            
        default:
            return false;
    }
    
    result->data_updated = true;
    return true;
}


/**
 * @brief 处理接收到的数据
 */
void JY901P_ProcessBuffer(uint8_t* buffer, uint16_t length)
{
    static uint8_t packet_index = 0;    // 当前数据包索引
    static PacketState parse_state = STATE_WAIT_HEADER; // 当前解析状态                     
    static uint8_t expected_sum = 0;    //校验和
    static uint8_t packet_buffer[JY901P_PACKET_SIZE] = {0}; // 用于存储接收到的数据包
    for(uint16_t i = 0; i < length; i++)
    {
       uint8_t byte = buffer[i];
        
        switch(parse_state)
        {
            case STATE_WAIT_HEADER:
                if(byte == 0x55)
                {
                    packet_buffer[0] = byte;
                    packet_index = 1;
                    expected_sum = byte;
                    parse_state = STATE_WAIT_TYPE;
                }
                break;
                
            case STATE_WAIT_TYPE:
                if(byte >= 0x50 && byte <= 0x5F)
                {
                    packet_buffer[packet_index++] = byte;
                    expected_sum += byte;
                    parse_state = STATE_RECEIVING_DATA;
                }
                else
                {
                    parse_state = STATE_WAIT_HEADER;
                }
                break;
                
            case STATE_RECEIVING_DATA:
                packet_buffer[packet_index++] = byte;
                
                if(packet_index < JY901P_PACKET_SIZE)
                {
                    expected_sum += byte;
                }
                else 
                {
                    // 最后一个字节是校验和
                    if(byte == (expected_sum & 0xFF))
                    {
                        // 校验成功，解析数据
                        JY901P_ParseData(packet_buffer, &jy901p_data);
                        parse_state = STATE_PACKET_COMPLETE;
                    }
                    else
                    {
                        // printf("Checksum error: expected 0x%02X, got 0x%02X\n", 
                        //        expected_sum & 0xFF, byte);
                        ;
                    }
                    parse_state = STATE_WAIT_HEADER;
                }
                break;
                
            case STATE_PACKET_COMPLETE:
                parse_state = STATE_WAIT_HEADER;
                break;
        }
    }
}

