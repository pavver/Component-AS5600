#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"

// https://ams.com/documents/20143/36005/AS5600_DS000365_5-00.pdfs
// https://pdf1.alldatasheet.com/pdfjsview/web/viewer.html?file=//pdf1.alldatasheet.com/datasheet-pdf/view/1452014/OSRAM/AS5600/+0_W_2_JXMSSwOd.XCDNY+/datasheet.pdf

#define ACK_CHECK_EN 0x1        /*!< I2C master will check ack from slave */
#define ACK_CHECK_DIS 0x0       /*!< I2C master will not check ack from slave */
                                /* */
#define I2C_SPEED_HZ 100000     /* I2C speed is 100 kHz */
#define I2C_AS5600_ADDRESS 0x36 /* the default AS5600 address */

typedef enum : uint8_t
{

  // ╔════════════════════════════════════════════╗
  // ║          Configuration Registers           ║
  // ╟────────────────────────────────────────────╢
  // ║ Read                                       ║
  // ║ Write                                      ║
  // ║ Burn                                       ║
  // ╚════════════════════════════════════════════╝

  /// @brief
  // AS5600 sensor ZMCO register.
  // ZMCO shows how many times ZPOS
  // and MPOS have been permanently written.
  // This command may only be executed if the presence of the
  // magnet is detected (MD = 1)
  AS5600_REG_ZMCO = 0x00,

  /// @brief
  // AS5600 sensor ZPOS register
  // These registers are used to configure the start position
  AS5600_REG_ZPOS = 0x01,

  /// @brief
  // AS5600 sensor MPOS register
  // stop position
  AS5600_REG_MPOS = 0x03,

  /// @brief
  // AS5600 sensor MANG register
  // narrower angular range. The angular range must be greater
  // than 18 degrees. In case of narrowed angular range, the
  // resolution is not scaled to narrowed range (e.g. 0° to
  // 360°(full-turn) → 4096dec; 0° to180°→2048dec)
  AS5600_REG_MANG = 0x05,

  /// @brief AS5600 sensor CONF register
  AS5600_REG_CONF = 0x07,

  // ╔════════════════════════════════════════════╗
  // ║              Output Registers              ║
  // ╟────────────────────────────────────────────╢
  // ║ Read                                       ║
  // ╚════════════════════════════════════════════╝

  /// @brief
  // AS5600 sensor RAW ANGLE register
  // The RAW ANGLE register contains the unscaled and unmodified
  // angle. The scaled output value is available in the ANGLE register.
  AS5600_REG_RAWANGLE = 0x0C,

  /// @brief
  // AS5600 sensor ANGLE register
  // The ANGLE register has a 10-LSB hysteresis at the limit
  // of the 360 degree range to avoid discontinuity points or
  // toggling of the output within one rotation.
  AS5600_REG_ANGLE = 0x0E,

  // ╔════════════════════════════════════════════╗
  // ║              Status Registers              ║
  // ╟────────────────────────────────────────────╢
  // ║ Read                                       ║
  // ╚════════════════════════════════════════════╝

  /// @brief AS5600 sensor STATUS register
  AS5600_REG_STATUS = 0x0B,

  /// @brief AS5600 sensor AGC register
  AS5600_REG_AGC = 0x1A,

  /// @brief /* AS5600 sensor MAGNITUDE register */
  AS5600_REG_MAGNITUDE = 0x1B,

  // ╔════════════════════════════════════════════╗
  // ║               Burn Register                ║
  // ╟────────────────────────────────────────────╢
  // ║ Write                                      ║
  // ╚════════════════════════════════════════════╝

  /// @brief AS5600 sensor BURN register
  AS5600_REG_BURN = 0xFF,

} AS5600_REG;

// ╔════════════════════════════════════════════╗
// ║                 Structures                 ║
// ╚════════════════════════════════════════════╝

struct AS5600_STATUS
{
  AS5600_STATUS(uint8_t status)
      : _status(status)
  {
  }

  /// @brief AGC minimum gain overflow, magnet too strong
  bool MagnetStrong()
  {
    return _status & 0b1000;
  }

  /// @brief AGC maximum gain overflow, magnet too weak
  bool MagnetWeak()
  {
    return _status & 0b10000;
  }

  /// @brief Magnet was detected
  bool MagnetDetected()
  {
    return _status & 0b100000;
  }

  operator uint8_t() const
  {
    return _status;
  }

private:
  uint8_t _status;
};

/// @brief Power Mode
typedef enum : uint8_t
{
  /// @brief Always on. 6.5 mA
  PM_NOM = 0b00,

  /// @brief Polling time = 5ms. 3.4 mA
  PM_LPM1 = 0b01,

  /// @brief Polling time = 20ms. 1.8 mA
  PM_LPM2 = 0b10,

  /// @brief Polling time = 100ms. 1.5 mA
  PM_LPM3 = 0b11
} AS5600_CONF_PM;

/// @brief To avoid any toggling of the output when the magnet is not moving, a 1 to 3 LSB hysteresis of the 12-bit resolution can be enabled with the HYST bits in the CONF register.
typedef enum : uint8_t
{
  HYST_OFF = 0b00,
  HYST_1LSB = 0b01,
  HYST_2LSB = 0b10,
  HYST_3LSB = 0b11
} AS5600_CONF_HYST;

/// @brief The OUTS bits in the CONF register are used to choose between an analog ratiometric output (default) and a digital PWM output. If PWM is selected, the DAC is powered down.
/// Without regard to which output is enabled, an external unit can read the angle from the ANGLE register through I²C interface at any time.
typedef enum : uint8_t
{
  /// @brief analog (full range from 0% to 100% between GND and VDD
  OUTS_ANALOG_FULL = 0b00,

  /// @brief analog (reduced range from 10% to 90% between GND and VDD
  OUTS_ANALOG = 0b01,

  /// @brief digital PWM
  OUTS_PWM = 0b10,

} AS5600_CONF_OUTS;

typedef enum : uint8_t
{
  /// @brief 115 Hz
  PWMF_115 = 0b00,

  /// @brief 230 Hz
  PWMF_230 = 0b01,

  /// @brief 460 Hz
  PWMF_460 = 0b10,

  /// @brief 920 Hz
  PWMF_920 = 0b11

} AS5600_CONF_PWMF;

/// @brief The AS5600 has a digital post-processing programmable filter which can be set in fast or slow modes. The fast filter mode can be enabled by setting a fast filter threshold in the FTH bits of the CONF register.
typedef enum : uint8_t
{
  /// @brief 16x
  SF_16 = 0b00,

  /// @brief 8x
  SF_8 = 0b01,

  /// @brief 4x
  SF_4 = 0b10,

  /// @brief 2x
  SF_2 = 0b11

} AS5600_CONF_SF;

typedef enum : uint8_t
{
  /// @brief slow filter only
  FTH_0 = 0b000,

  FTH_6 = 0b001,
  FTH_7 = 0b010,
  FTH_9 = 0b011,
  FTH_18 = 0b100,
  FTH_21 = 0b101,
  FTH_24 = 0b110,
  FTH_10 = 0b111,

} AS5600_CONF_FTH;

struct AS5600_CONF
{
  AS5600_CONF(uint16_t conf)
      : _conf(conf)
  {
  }

  AS5600_CONF_PM PowerMode()
  {
    return (AS5600_CONF_PM)(_conf & PM_LPM3);
  }

  AS5600_CONF_HYST Hysteresis()
  {
    return (AS5600_CONF_HYST)((_conf >> 2) & HYST_3LSB);
  }

  AS5600_CONF_OUTS OutputStage()
  {
    return (AS5600_CONF_OUTS)((_conf >> 4) & 0b11);
  }

  AS5600_CONF_PWMF PWMFrequency()
  {
    return (AS5600_CONF_PWMF)((_conf >> 6) & PWMF_920);
  }

  AS5600_CONF_SF SlowFilter()
  {
    return (AS5600_CONF_SF)((_conf >> 8) & SF_2);
  }

  AS5600_CONF_FTH FastFilterThreshold()
  {
    return (AS5600_CONF_FTH)((_conf >> 10) & FTH_10);
  }

private:
  uint16_t _conf;
};

class AS5600_i2c
{
public:
  /// @brief Create i2c driver
  /// @param sda
  /// @param scl
  /// @param i2c_num
  AS5600_i2c(gpio_num_t sda, gpio_num_t scl, i2c_port_t i2c_num = I2C_NUM_0)
      : i2c_num(i2c_num)
  {
    i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    i2c_driver_initialize(sda, scl);
  }

  /// @brief Use already i2c driver port
  /// @param i2c_num
  AS5600_i2c(i2c_port_t i2c_num) : i2c_num(i2c_num)
  {
  }

  esp_err_t read_ZMCO(uint8_t &zmco);

  esp_err_t read_ZPOS(uint16_t &zpos);

  esp_err_t write_ZPOS(uint16_t zpos);

  esp_err_t read_MPOS(uint16_t &mpos);

  esp_err_t write_MPOS(uint16_t mpos);

  esp_err_t read_MANG(uint16_t &mang);

  esp_err_t write_MANG(uint16_t mang);

  esp_err_t read_CONF(AS5600_CONF &conf);

  esp_err_t write_CONF(AS5600_CONF conf);

  esp_err_t read_RAWANGLE(uint16_t &angle);

  esp_err_t read_ANGLE(uint16_t &angle);

  esp_err_t read_STATUS(AS5600_STATUS &status);

  esp_err_t read_AGC(uint8_t &agc);

  esp_err_t read_MAGNITUDE(uint16_t &magnitude);

  esp_err_t Burn_Angle();

  esp_err_t Burn_Setting();

private:
  i2c_port_t i2c_num;

  void i2c_driver_initialize(gpio_num_t sda, gpio_num_t scl);

  esp_err_t read_registr(AS5600_REG reg, uint8_t *value, uint8_t len);
  esp_err_t read_registr(AS5600_REG reg, uint16_t &value);
  esp_err_t read_registr(AS5600_REG reg, uint8_t &value);

  esp_err_t write_registr(AS5600_REG reg, uint8_t *value, uint8_t len);
  esp_err_t write_registr(AS5600_REG reg, uint16_t value);
  esp_err_t write_registr(AS5600_REG reg, uint8_t value);
};
