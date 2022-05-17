/******************************************************************************
  This is a library for the MLX90395 magnetometer.

  Designed specifically to work with the MLX90395 breakout from Adafruit:

  ----> https://www.adafruit.com/products/4022

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code, please
  support Adafruit and open-source hardware by purchasing products from
  Adafruit!

  Written by Kevin Townsend/ktown for Adafruit Industries.

  MIT license, all text above must be included in any redistribution
 *****************************************************************************/

#include "Adafruit_MLX90395_Soft.h"

Adafruit_MLX90395_Soft::Adafruit_MLX90395_Soft(void) {}

bool Adafruit_MLX90395_Soft::begin_I2C(uint8_t sda, uint8_t scl, uint8_t i2c_addr) {
  _i2c_addr = i2c_addr;
  _i2c = new SlowSoftI2CMaster(sda, scl, false);
  _i2c->i2c_init();
  return _init();
}

bool Adafruit_MLX90395_Soft::_init(void) {
  if (!exitMode()) {
    return false;}
  delay(10);
  if (!reset()) {
    return false; }
  delay(10);
  _gain = getGain();
  if (_gain == 8) { // default high field gain
    _uTLSB = 7.14;
  } else {
    _uTLSB = 2.5; // medium field gain
  }
  _resolution = getResolution();
  if (!readRegister(0x26, &uniqueID[0]) || !readRegister(0x27, &uniqueID[1]) ||
      !readRegister(0x28, &uniqueID[2])) {
    return false;
  }
  return true;
}


bool Adafruit_MLX90395_Soft::reset(void) {
  return (command(MLX90395_REG_RT) == MLX90395_STATUS_RESET);
}

bool Adafruit_MLX90395_Soft::exitMode(void) {
  // do once and ignore status
  command(MLX90395_REG_EX);
  return command(MLX90395_REG_EX) == 0;
}


bool Adafruit_MLX90395_Soft::readData(float *x, float *y, float *z) {
  if (!startSingleMeasurement())
    return false;
  while (!readMeasurement(x, y, z))
    delay(1);
  return true;
}

bool Adafruit_MLX90395_Soft::readMeasurement(float *x, float *y, float *z) {

  uint8_t rx[12] = {0};   // status, crc, X16, Y16, Z16, T16, V16

  _i2c->i2c_start((_i2c_addr << 1) | I2C_WRITE);
  _i2c->i2c_write(0x80);
  _i2c->i2c_rep_start((_i2c_addr << 1) | I2C_READ);
  for (int i = 0; i < 11; i++)
  {
    rx[i] = _i2c->i2c_read(0);
  }
  rx[11] = _i2c->i2c_read(1);
  _i2c->i2c_stop();
  
   // check status
  if (rx[0] != MLX90395_STATUS_DRDY) {
    return false;
  }

  int16_t xi, yi, zi;

  // Convert data to uT and float.
  xi = (rx[2] << 8) | rx[3];
  yi = (rx[4] << 8) | rx[5];
  zi = (rx[6] << 8) | rx[7];

  *x = xi;
  *y = yi;
  *z = zi;

  // multiply by gain & LSB
  *x *= gainMultipliers[_gain] * _uTLSB;
  *y *= gainMultipliers[_gain] * _uTLSB;
  *z *= gainMultipliers[_gain] * _uTLSB;

  return true;
}

bool Adafruit_MLX90395_Soft::startSingleMeasurement(void) {
  return (command(MLX90395_REG_SM | 0x0F) == MLX90395_STATUS_SMMODE);
}

mlx90393_osr_t Adafruit_MLX90395_Soft::getOSR(void) {
    uint16_t regValue;
    readRegister(MLX90395_REG_2, &regValue);
    return (mlx90393_osr_t)(regValue & 0x03);
}

bool Adafruit_MLX90395_Soft::setOSR(mlx90393_osr_t osrval) {
  uint16_t regValue;
  readRegister(MLX90395_REG_2, &regValue);
  regValue |= osrval;
  writeRegister(MLX90395_REG_2, regValue);
  return true;
}

uint8_t Adafruit_MLX90395_Soft::getGain(void) {
  uint16_t regValue;
  readRegister(MLX90395_REG_0, &regValue);
  return (regValue >> 4) & 0x0F;
}

bool Adafruit_MLX90395_Soft::setGain(uint8_t gainval) {
  uint16_t regValue;
  readRegister(MLX90395_REG_0, &regValue);
  regValue |= gainval << 4;
  writeRegister(MLX90395_REG_0, regValue);
  return true;
}

mlx90393_res_t Adafruit_MLX90395_Soft::getResolution(void) {
  uint16_t regValue;
  readRegister(MLX90395_REG_2, &regValue);
  return (mlx90393_res_t)((regValue >> 5) & 0x03);
}

bool Adafruit_MLX90395_Soft::setResolution(mlx90393_res_t resval) {
  uint16_t regValue;
  readRegister(MLX90395_REG_2, &regValue);
  regValue |= resval << 5;
  regValue |= resval << 7;
  regValue |= resval << 9;
  writeRegister(MLX90395_REG_2, regValue);
  return true;
}

/****************************************************************/

uint8_t Adafruit_MLX90395_Soft::command(uint8_t cmd) {
  uint8_t status;
  _i2c->i2c_start_wait((_i2c_addr << 1) | I2C_WRITE);
  _i2c->i2c_write(0x80);
  _i2c->i2c_write(cmd);

  _i2c->i2c_rep_start((_i2c_addr << 1) | I2C_READ);
  status = _i2c->i2c_read(1);
  _i2c->i2c_stop();
  return status;
}

bool Adafruit_MLX90395_Soft::readRegister(uint8_t reg, uint16_t *data) {
  _i2c->i2c_start((_i2c_addr << 1) | I2C_WRITE);
  _i2c->i2c_write(reg << 1);
  _i2c->i2c_rep_start((_i2c_addr << 1) | I2C_READ);
  uint8_t upperBits = _i2c->i2c_read(0);
  uint8_t lowerBits = _i2c->i2c_read(1);
  *data = (uint16_t)((upperBits << 8) |  lowerBits);
  _i2c->i2c_stop();
  return true;
}


bool Adafruit_MLX90395_Soft::writeRegister(uint8_t reg, uint16_t data) {
  uint8_t upperBits = data >> 8;
  uint8_t lowerBits = (data) & 0xFF;
  _i2c->i2c_start((_i2c_addr << 1) | I2C_WRITE);
  _i2c->i2c_write(reg << 1);
  _i2c->i2c_write(upperBits);
  _i2c->i2c_write(lowerBits);
  _i2c->i2c_rep_start((_i2c_addr << 1) | I2C_READ);
  _i2c->i2c_read(1);
  _i2c->i2c_stop();
  return true;
}
