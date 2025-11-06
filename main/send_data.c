#include "usart.h"
#include "send_data.h"
#include "esp_log.h"
/**
 * @brief 计算 CRC-16 校验
 * @param data 指向需要校验的数据
 * @param length 数据长度（不包括 CRC 本身）
 * @return uint16_t CRC-16 校验值
 *
 * CRC-16-IBM / ModBus 常用多项式：0x8005
 */
static uint16_t calc_crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // 与当前字节异或
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001; // 多项式反转
            else
                crc = crc >> 1;
        }
    }
    return crc;
}

/**
 * @brief 发送探头数据帧到主机
 *
 * 数据帧结构:
 *  [Header(1)] [Length(1)] [Cmd(1)] [Payload(N)] [CRC-16(2)]
 *
 * Payload:
 *  [float 温度 4字节] [float 光强 4字节] => 总共 8字节
 *
 * 总长度 = 1 + 1 + 1 + 8 + 2 = 13字节
 */
void send_probe_data(float temperature, float light)
{
    uint8_t frame[13]; // 定义完整数据帧
    uint8_t idx = 0;

    // --- 1. 帧头 ---
    frame[idx++] = 0xAA; // 固定帧头，用于主机解析起始位置

    // --- 2. 长度字节 ---
    frame[idx++] = 1 + 8 + 2; // Cmd(1) + Payload(8) + CRC16(2)

    // --- 3. Cmd 指令码 ---
    frame[idx++] = 0x01; // Cmd=0x01 表示普通数据包

    // --- 4. Payload: 温度 & 光强 ---
    memcpy(&frame[idx], &temperature, sizeof(float));
    idx += sizeof(float);
    memcpy(&frame[idx], &light, sizeof(float));
    idx += sizeof(float);

    // --- 5. CRC-16 校验 ---
    uint16_t crc = calc_crc16(&frame[1], idx - 1); // 不包括帧头
    frame[idx++] = crc & 0xFF;                     // CRC低字节
    frame[idx++] = (crc >> 8) & 0xFF;              // CRC高字节

    // --- 6. 发送数据帧到 UART ---

    uart_write_bytes(USART_UX, (const char *)frame, idx);
}