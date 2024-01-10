#ifndef _PIN_MASK_FOR_74HC165_
#define _PIN_MASK_FOR_74HC165_
#include <stm32f1xx.h>
#include "hardware_config.hpp"
#include "vector"
#include "gpio.hpp"
#include "delay.hpp"


class IC_74HC165
{
private:

public:
uint8_t res[32];
std::vector<uint16_t> vector_pins;
uint16_t flex_cable();
};

#endif //_PIN_MASK_FOR_74HC165_