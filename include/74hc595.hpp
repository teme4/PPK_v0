#ifndef _PIN_MASK_FOR_74HC595_
#define _PIN_MASK_FOR_74HC595_
#include <stm32f1xx.h>
#include "hardware_config.hpp"

void HC74_595_SPI(uint32_t data,uint8_t mode);
void HC74_595_SET(uint16_t data1,uint16_t data2,uint8_t mode);

 #endif //PIN_MASK_FOR_74HC595