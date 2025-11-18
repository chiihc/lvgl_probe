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
#include "send_data.h"
#include "adc1.h"

#define ADS1115_ADDR 0x48
#define STORAGE_NAMESPACE "storage"

static unsigned char day;
static unsigned char month;
static unsigned short year;

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
    adc_init();          /* 初始化ADC */

    // // 2. 配置 ADS1115
    // ads1115_t ads = ads1115_config(I2C_NUM_1, ADS1115_ADDR);

    // // 设置 PGA（量程）：±1.024V  → 适配 0~0.8V 输入
    // ads1115_set_pga(&ads, ADS1115_FSR_1_024); // = ±1.024V

    // // 设置为连续采样模式，数据速率 128 SPS（可改）
    // ads1115_set_mode(&ads, ADS1115_MODE_CONTINUOUS);
    // ads1115_set_sps(&ads, ADS1115_SPS_128);

    nvs_handle_t my_handle = 0;

    nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle); // 打开 NVS 命名空间

    load_date_from_storage(ret, my_handle); // 2. 从 NVS 加载日期，如果不存在则使用默认值
    nvs_close(my_handle);

    // for (int addr = 1; addr < 127; addr++)
    // {
    //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //     i2c_master_start(cmd);
    //     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    //     i2c_master_stop(cmd);
    //     esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
    //     i2c_cmd_link_delete(cmd);
    //     if (ret == ESP_OK)
    //     {
    //         printf("Found device at 0x%02X I2C_NUM_0\n", addr);
    //     }
    // }

    // for (int addr = 1; addr < 127; addr++)
    // {
    //     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //     i2c_master_start(cmd);
    //     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    //     i2c_master_stop(cmd);
    //     esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 50 / portTICK_PERIOD_MS);
    //     i2c_cmd_link_delete(cmd);
    //     if (ret == ESP_OK)
    //     {
    //         printf("Found device at 0x%02X I2C_NUM_1\n", addr);
    //     }
    // }
    static probe_state_t current_state = PROBE_DISCONNECTED;
    while (1)
    {
        if (current_state == PROBE_CONNECTED)
        {
            float temperature = tmp102_read_temperature();
            float volt = 1;
            uint16_t adcdata;
            adcdata = adc_get_result_average(ADC_ADCX_CHY, 20);
            volt = (float)adcdata * (3.3 / 4095);
            
            // ESP_LOGI("ADC", "ADC Data: %d Voltage: %.2f V", adcdata, volt);

            send_probe_data(temperature, volt);
        }
        else
        {
            if (check_line_connected())
            {
                current_state = PROBE_CONNECTED;
            }
        }

        vTaskDelay(500);
    }
}
