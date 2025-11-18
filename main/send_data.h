#include "nvs.h"
typedef struct
{
    float temperature;
    float light_intensity;
} probe_data_t;
void send_probe_data(float temperature, float light);

typedef enum
{
    PROBE_DISCONNECTED = 0, // 探头未连接
    PROBE_CONNECTED = 1     // 探头已连接
} probe_state_t;



bool check_line_connected();

typedef struct __attribute__((packed))
{
    uint8_t sw_version;      // 软件版本
    uint8_t hw_version;      // 硬件版本
    uint8_t band_code;       // 波段代码 (0x01/0x02/0x03)
    uint32_t unit_id;        // 探头编号
    uint32_t cal_date;       // 校准日期 YYYYMMDD
    float correction_factor; // 校正系数
} probe_info_t;

void send_handshake();

void load_date_from_storage(esp_err_t ret, nvs_handle_t my_handle);