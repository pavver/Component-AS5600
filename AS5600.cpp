#include "AS5600.h"

void AS5600_i2c::i2c_driver_initialize(gpio_num_t sda, gpio_num_t scl)
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda,
      .scl_io_num = scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {
          .clk_speed = I2C_SPEED_HZ}};

#pragma GCC diagnostic pop

  i2c_param_config(I2C_NUM_0, &i2c_config);
}

esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint8_t *value, uint8_t len)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, I2C_AS5600_ADDRESS << 1 | I2C_MASTER_WRITE, false);
  i2c_master_write_byte(cmd, (uint8_t)reg, false);
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, I2C_AS5600_ADDRESS << 1 | I2C_MASTER_READ, false);
  if (len > 1)
  {
    i2c_master_read(cmd, value, len, I2C_MASTER_LAST_NACK);
  }
  else
  {
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
  }
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint16_t &value)
{
  uint8_t *data = (uint8_t *)calloc(sizeof(uint8_t), 2);
  esp_err_t err = read_registr(reg, data, 2);

  if (err == ESP_OK)
    value = ((uint16_t)data[0] << 8) | data[1];

  free(data);
  return err;
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint8_t *value, uint8_t len)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);

  i2c_master_write_byte(cmd, I2C_AS5600_ADDRESS << 1 | I2C_MASTER_WRITE, false);
  i2c_master_write_byte(cmd, (uint8_t)reg, false);

  i2c_master_write(cmd, value, len, true);

  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint16_t value)
{
  return write_registr(reg, (uint8_t *)&value, 2);
}

esp_err_t AS5600_i2c::write_registr(AS5600_REG reg, uint8_t value)
{
  return write_registr(reg, &value, 1);
}

esp_err_t AS5600_i2c::read_registr(AS5600_REG reg, uint8_t &value)
{
  return read_registr(reg, &value, 1);
}

esp_err_t AS5600_i2c::read_ZMCO(uint8_t &zmco)
{
  return read_registr(AS5600_REG_ZMCO, zmco);
}

esp_err_t AS5600_i2c::read_ZPOS(uint16_t &zpos)
{
  return read_registr(AS5600_REG_ZPOS, zpos);
}

esp_err_t AS5600_i2c::write_ZPOS(uint16_t zpos)
{
  return write_registr(AS5600_REG_ZPOS, zpos);
}

esp_err_t AS5600_i2c::read_MPOS(uint16_t &mpos)
{
  return read_registr(AS5600_REG_MPOS, mpos);
}

esp_err_t AS5600_i2c::write_MPOS(uint16_t zpos)
{
  return write_registr(AS5600_REG_MPOS, zpos);
}

esp_err_t AS5600_i2c::read_MANG(uint16_t &mang)
{
  return read_registr(AS5600_REG_MANG, mang);
}

esp_err_t AS5600_i2c::write_MANG(uint16_t zpos)
{
  return write_registr(AS5600_REG_MANG, zpos);
}

AS5600_CONF AS5600_i2c::read_CONF(esp_err_t &err)
{
  uint16_t ret;
  err = read_registr(AS5600_REG_CONF, ret);
  return (AS5600_CONF)ret;
}

AS5600_CONF AS5600_i2c::read_CONF()
{
  esp_err_t ret;
  return read_CONF(ret);
}

esp_err_t AS5600_i2c::write_CONF(AS5600_CONF conf)
{
  return write_registr(AS5600_REG_CONF, *(uint16_t *)&conf);
}

esp_err_t AS5600_i2c::read_RAWANGLE(uint16_t &rawangle)
{
  return read_registr(AS5600_REG_RAWANGLE, rawangle);
}

esp_err_t AS5600_i2c::read_ANGLE(uint16_t &angle)
{
  return read_registr(AS5600_REG_ANGLE, angle);
}

esp_err_t AS5600_i2c::read_STATUS(AS5600_STATUS &status)
{
  return read_registr(AS5600_REG_STATUS, *(uint8_t *)&status);
}

esp_err_t AS5600_i2c::read_AGC(uint8_t &agc)
{
  return read_registr(AS5600_REG_AGC, agc);
}

esp_err_t AS5600_i2c::read_MAGNITUDE(uint16_t &magnitude)
{
  return read_registr(AS5600_REG_MAGNITUDE, magnitude);
}

esp_err_t AS5600_i2c::Burn_Angle()
{
  return write_registr(AS5600_REG_BURN, (uint8_t)0x80);
}

esp_err_t AS5600_i2c::Burn_Setting()
{
  return write_registr(AS5600_REG_BURN, (uint8_t)0x40);
}

const char *CONF_PM_String(AS5600_CONF_PM pm)
{
  switch (pm)
  {
  case PM_NOM:
    return "NOM";
  case PM_LPM1:
    return "LPM1";
  case PM_LPM2:
    return "LPM2";
  case PM_LPM3:
    return "LPM3";
  default:
    return "Unknown";
  }
  return nullptr;
}

const char *CONF_HYST_String(AS5600_CONF_HYST hyst)
{
  switch (hyst)
  {
  case HYST_OFF:
    return "OFF";
  case HYST_1LSB:
    return "1LSB";
  case HYST_2LSB:
    return "2LSB";
  case HYST_3LSB:
    return "3LSB";
  default:
    return "Unknown";
  }
  return nullptr;
}

const char *CONF_OUTS_String(AS5600_CONF_OUTS outs)
{
  switch (outs)
  {
  case OUTS_ANALOG_FULL:
    return "ANALOG_FULL";
  case OUTS_ANALOG:
    return "ANALOG";
  case OUTS_PWM:
    return "PWM";
  default:
    return "Unknown";
  }
  return nullptr;
}

const char *CONF_PWMF_String(AS5600_CONF_PWMF pwmf)
{
  switch (pwmf)
  {
  case PWMF_115:
    return "115 Hz";
  case PWMF_230:
    return "230 Hz";
  case PWMF_460:
    return "460 Hz";
  case PWMF_920:
    return "920 Hz";
  default:
    return "Unknown";
  }
  return nullptr;
}

const char *CONF_SF_String(AS5600_CONF_SF sf)
{
  switch (sf)
  {
  case SF_16:
    return "16x";
  case SF_8:
    return "8x";
  case SF_4:
    return "4x";
  case SF_2:
    return "2x";
  default:
    return "Unknown";
  }
  return nullptr;
}

const char *CONF_FTH_String(AS5600_CONF_FTH fth)
{
  switch (fth)
  {
  case FTH_0:
    return "Slow filter only";
  case FTH_6:
    return "6";
  case FTH_7:
    return "7";
  case FTH_9:
    return "9";
  case FTH_18:
    return "18";
  case FTH_21:
    return "21";
  case FTH_24:
    return "24";
  case FTH_10:
    return "10";
  default:
    return "Unknown";
  }
  return nullptr;
}
