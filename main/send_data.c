#include "usart.h"
#include "send_data.h"
#include "esp_log.h"
#include "nvs_flash.h"
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
    uint8_t frame[13 + 1]; // 定义完整数据帧
    uint8_t idx = 0;

    // --- 1. 帧头 ---
    frame[idx++] = 0xAA; // 固定帧头，用于主机解析起始位置

    // --- 2. 长度字节 ---
    frame[idx++] = 1 + 1 + 1 + 8; //  Payload(8)

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

    // --- 6. 结束字节 ---
    frame[idx++] = 0x00;

    // --- 7. 发送数据帧到 UART ---

    uart_write_bytes(USART_UX, (const char *)frame, idx);
}

void send_heartbeat()
{
    uint8_t frame[1 + 1 + 1 + 2 + 1]; // Header + Length + Cmd + CRC16
    uint8_t idx = 0;
    frame[idx++] = 0xAA;
    frame[idx++] = 3; // 1 + 1 + 1
    frame[idx++] = 0x10;
    uint16_t crc = calc_crc16(&frame[1], idx - 1);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;
    frame[idx++] = 0x00;

    uart_write_bytes(USART_UX, (const char *)frame, idx);
}

bool check_line_connected()
{
    // 1. 发送一个短心跳帧或握手帧
    send_handshake();

    // 2. 等待 ACK 超时
    uint32_t start_tick = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_tick < pdMS_TO_TICKS(50)) // 50ms 超时
    {
        uint8_t rx;
        if (uart_read_bytes(USART_UX, &rx, 1, pdMS_TO_TICKS(10)) > 0)
        {
            if (rx == 0x55) // 假设 ACK 帧头
                return true;
        }
    }
    return false;
}

// 暂时使用硬编码，后续从 NVS 读取
probe_info_t probe_info;

#define probe_info_size sizeof(probe_info)

void send_handshake()
{
    uint8_t frame[1 + 1 + 1 + probe_info_size + 2 + 1]; // Header + Length + Cmd + Payload(4) + CRC16 + End
    uint8_t idx = 0;

    // --- 1. 帧头 ---
    frame[idx++] = 0xAA;

    // --- 2. 长度字节 ---
    frame[idx++] = 1 + 1 + 1 + probe_info_size;

    // --- 3. Cmd 指令码 ---
    frame[idx++] = 0x02; // Cmd=0x02 表示首次通信

    // --- 4. Payload: 时间戳 ---
    const uint8_t *payload = (uint8_t *)&probe_info;
    memcpy(&frame[idx], payload, probe_info_size);
    idx += probe_info_size;

    // --- 5. CRC-16 校验 ---
    uint16_t crc = calc_crc16(&frame[1], idx - 1); // 不包括帧头
    frame[idx++] = crc & 0xFF;                     // CRC低字节
    frame[idx++] = (crc >> 8) & 0xFF;              // CRC高字节

    // --- 6. 结束字节 ---
    frame[idx++] = 0x00;

    // --- 7. 发送数据帧到 UART ---
    uart_write_bytes(USART_UX, (const char *)frame, idx);
}

#define STORAGE_NAMESPACE "storage"

/**
 * @brief 将日期保存到 NVS
 */
static void save_date_to_nvs(void)
{
    nvs_handle_t my_handle;                                 // NVS 句柄
    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle); // 打开 NVS 命名空间

    nvs_set_u16(my_handle, "cal_date", probe_info.cal_date);
    nvs_commit(my_handle); // 提交更改
    nvs_close(my_handle);  // 关闭 NVS 句柄
}



/**
 * @brief 从 NVS 加载日期，如果失败则使用默认值并保存
 */
void load_date_from_storage(esp_err_t ret,nvs_handle_t my_handle)
{


    size_t len = sizeof(probe_info_t);
    ret = nvs_get_blob(my_handle, "probe_info", &probe_info, &len);

    if (ret == ESP_OK) {
        // ESP_LOGI(TAG, "probe_info loaded from NVS");
    } 
    else if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // ESP_LOGW(TAG, "probe_info not found, creating default");

        // ---- 默认值（仅第一次写入）----
        probe_info.band_code = 0x02;
        probe_info.unit_id   = 0x76543210;
        probe_info.hw_version = 0x01;
        probe_info.sw_version = 0x01;
        probe_info.cal_date  = 20251113; // YYYYMMDD
        probe_info.correction_factor = 1.0f;

        // 写入默认值
        nvs_set_blob(my_handle, "probe_info", &probe_info, sizeof(probe_info_t));
        nvs_commit(my_handle);

        // ESP_LOGI(TAG, "Default probe_info saved to NVS");
    } 
    else {
        // ESP_LOGE(TAG, "nvs_get_blob failed: %s", esp_err_to_name(err));
    }

    nvs_close(my_handle);


}