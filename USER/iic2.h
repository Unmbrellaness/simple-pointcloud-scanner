#ifndef __IIC_H
#define __IIC_H	 
#include "sys.h"


#define TofAddr 0x52

void i2c_init(void);
void i2c_write_reg(u8 deviceAddr, u8 registerAddr, u8 value);
u8 I2C2_ReadRegister(u8 deviceAddr, u8 registerAddr);

#endif