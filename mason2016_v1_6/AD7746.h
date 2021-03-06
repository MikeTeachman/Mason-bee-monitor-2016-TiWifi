#ifndef AD7746_H
#define AD7746_H

//AD7746 definitions
#define AD7746_I2C_ADDRESS  0x48 //0x90 shift one to the right
#define AD7746_REGISTER_STATUS 0x00
#define AD7746_REGISTER_CAP_DATA 0x01
#define AD7746_REGISTER_VT_DATA 0x04
#define AD7746_REGISTER_CAP_SETUP 0x07
#define AD7746_REGISTER_VT_SETUP 0x08
#define AD7746_REGISTER_EXC_SETUP 0x09
#define AD7746_REGISTER_CONFIGURATION 0x0A
#define AD7746_REGISTER_CAP_DAC_A 0x0B
#define AD7746_REGISTER_CAP_DAC_B 0x0B
#define AD7746_REGISTER_CAP_OFFSET 0x0D
#define AD7746_REGISTER_CAP_GAIN 0x0F
#define AD7746_REGISTER_VOLTAGE_GAIN 0x11
#define AD7746_RESET_ADDRESS 0xBF

#endif 
