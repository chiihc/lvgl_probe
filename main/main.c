#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "led.h"
#include "iic.h"
#include "usart.h"
#include "tmp102.h"
#include "ads1115.h"

#define ADS1115_ADDR 0x48
#define STORAGE_NAMESPACE "storage"

typedef struct
{
    float temperature;
    float light_intensity;
} probe_data_t;

static uint8_t calc_checksum(const uint8_t *data, uint8_t len) //
{
    uint8_t cs = 0;
    for (int i = 0; i < len; i++)
        cs ^= data[i];
    return cs;
}

void uart_send_probe_data(float temp, float light)
{
    uint8_t frame[1 + 1 + 1 + 8 + 1];
    uint8_t idx = 0;

    frame[idx++] = 0xAA;  // Header
    frame[idx++] = 1 + 8; // Length = Cmd + Payload
    frame[idx++] = 0x01;  // Cmd: DATA

    memcpy(&frame[idx], &temp, 4);
    idx += 4;
    memcpy(&frame[idx], &light, 4);
    idx += 4;

    uint8_t checksum = calc_checksum(frame, idx);
    frame[idx++] = checksum;

    uart_write_bytes(USART_UX, frame, idx);
    ESP_LOGI("TMP102", "Temperature = %.2f °C Volt = %.2f V", temp, light);
    // printf("Temperature = %.2f °C Volt = %.2f V", temp, light);
}

// 从dev_addr设备读取len个字节数据, 存储到data数组中, 从mem_addr地址开始读取
esp_err_t read_bytes(uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, size_t len)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // 创建命令链接

    // Step 1: 写入目标地址
    i2c_master_start(cmd);                                                // 发送起始条件
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true); // 发送设备地址+写位
    i2c_master_write_byte(cmd, mem_addr, true);                           // 发送内存地址

    // Step 2: 重新开始读操作
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static unsigned char day;
static unsigned char month;
static unsigned short year;

void date_init(void)
{
    day = 1;
    month = 1;
    year = 2025;
} // 初始化日期

/**
 * @brief 将日期保存到 NVS
 */
static void save_date_to_nvs(void)
{
    nvs_handle_t my_handle;                                 // NVS 句柄
    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle); // 打开 NVS 命名空间

    nvs_set_u16(my_handle, "year", year); // 保存年份
    nvs_set_u8(my_handle, "month", month);
    nvs_set_u8(my_handle, "day", day);
    nvs_commit(my_handle); // 提交更改
    nvs_close(my_handle);  // 关闭 NVS 句柄
}

/**
 * @brief 从 NVS 加载日期，如果失败则使用默认值并保存
 */
void load_date_from_storage(nvs_handle_t my_handle)
{

    // 尝试读取日期
    esp_err_t err_year = nvs_get_u16(my_handle, "year", &year);
    esp_err_t err_month = nvs_get_u8(my_handle, "month", &month);
    esp_err_t err_day = nvs_get_u8(my_handle, "day", &day);

    if (err_year != ESP_OK || err_month != ESP_OK || err_day != ESP_OK)
    {
        // 读取失败，使用默认日期并保存
        date_init();
        save_date_to_nvs();
    }
}

/**
 * @brief       程序入口
 * @param       无
 * @retval      无
 */
void app_main(void)
{
    esp_err_t ret;

    ret = nvs_flash_init(); /* 初始化NVS */

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    led_init();          /* 初始化LED */
    iic_init(I2C_NUM_0); /* 初始化IIC0 */
    iic_init(I2C_NUM_1); /* 初始化IIC1 */
    usart_init(115200);  /* 初始化串口 */

    // 2. 配置 ADS1115
    ads1115_t ads = ads1115_config(I2C_NUM_1, ADS1115_ADDR);

    // 设置 PGA（量程）：±1.024V  → 适配 0~0.8V 输入
    ads1115_set_pga(&ads, ADS1115_FSR_1_024); // = ±1.024V

    // 选择通道（A0）
    // ads1115_set_mux(&ads, ADS1115_MUX_0_1);

    // 设置为连续采样模式，数据速率 128 SPS（可改）
    ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS);
    ads1115_set_sps(&ads, ADS1115_SPS_128);

    nvs_handle_t my_handle = 0;

    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle); // 打开 NVS 命名空间

    load_date_from_storage(my_handle); // 2. 从 NVS 加载日期，如果不存在则使用默认值
    nvs_close(my_handle);

    for (int addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            printf("Found device at 0x%02X I2C_NUM_0\n", addr);
        }
    }

    for (int addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            printf("Found device at 0x%02X I2C_NUM_1\n", addr);
        }
    }

    while (1)
    {
        float temperature = tmp102_read_temperature();
        float volt = ads1115_get_voltage(&ads);
        // ESP_LOGI("TMP102", "Temperature = %.2f °C Volt = %.2f V", temperature, volt);
        uart_send_probe_data(temperature, volt);
        vTaskDelay(200);
    }
}
