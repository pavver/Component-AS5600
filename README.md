esp-idf component AS5600

```cpp
#include "driver/gpio.h"
#include "AS5600.h"
#include "esp_log.h"

#define I2C_SDA_GPIO GPIO_NUM_22
#define I2C_SCL_GPIO GPIO_NUM_23

static const char *TAG = "AS5600";

void app_main(void)
{
  AS5600_i2c *AS5600 = new AS5600_i2c(I2C_SDA_GPIO, I2C_SCL_GPIO);

  while (true)
  {    
  AS5600_CONF conf = AS5600->read_CONF();

  ESP_LOGI(TAG, "PowerMode: %s", CONF_PM_String(conf.PowerMode()));
  ESP_LOGI(TAG, "Hysteresis: %s", CONF_HYST_String(conf.Hysteresis()));
  ESP_LOGI(TAG, "OutputStage: %s", CONF_OUTS_String(conf.OutputStage()));
  ESP_LOGI(TAG, "PWMFrequency: %s", CONF_PWMF_String(conf.PWMFrequency()));
  ESP_LOGI(TAG, "SlowFilter: %s", CONF_SF_String(conf.SlowFilter()));
  ESP_LOGI(TAG, "FastFilterThreshold: %s", CONF_FTH_String(conf.FastFilterThreshold()));

  uint16_t RawAngle;
  AS5600->read_RAWANGLE(RawAngle);
  ESP_LOGI(TAG, "RawAngle: %hu", RawAngle);

  AS5600_STATUS status = AS5600_STATUS(0);
  AS5600->read_STATUS(status);
  ESP_LOGI(TAG, "status: MD %s", status.MagnetDetected() ? "1" : "0");
  ESP_LOGI(TAG, "status: ML %s", status.MagnetWeak() ? "1" : "0");
  ESP_LOGI(TAG, "status: MH %s", status.MagnetStrong() ? "1" : "0");

  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
```