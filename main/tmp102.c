#include "iic.h"
#include "tmp102.h"

float tmp102_read_temperature(void)
{
    uint8_t buf[2];
    uint8_t reg = 0x00;  // 温度寄存器

    // 读两个字节
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TMP102_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TMP102_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buf, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    // 拼接 raw 值 (12-bit)
    int16_t raw = ((buf[0] << 8) | buf[1]) >> 4;

    // 处理负数补码
    if (raw & 0x800) raw -= 4096;

    // 乘温度分辨率
    return raw * 0.0625f;   // °C
}