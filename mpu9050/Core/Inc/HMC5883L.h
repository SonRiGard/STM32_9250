#include "stm32f1xx_hal.h"
//for Digital HMC5883L
#define HMC5883L_ADDRESS 0x3D
#define REGISTER_MODE 0x02
#define REGISTER_CONFIG_B 0x01//in configuration register b, we need to configure the scale
#define REGISTER_CONFIG_A 0x00
#define X_MSB 0x03

