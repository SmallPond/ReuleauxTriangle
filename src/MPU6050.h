#ifndef __MT_I2C_H__
#define __MT_I2C_H__

#include <stdint.h>
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
double acc2rotation(double x, double y, double kalAngleZ);

#endif // __MT_I2C_H__
