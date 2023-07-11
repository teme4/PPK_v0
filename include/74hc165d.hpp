#ifndef _PIN_MASK_FOR_74HC165_
#define _PIN_MASK_FOR_74HC165_
#include <stm32f1xx.h>
#include "hardware_config.hpp"
//#include "gpio.hpp"

extern uint8_t res[32];

static const uint8_t pin_flex[32]
{
0,
0xFE,0xFD,0xDF,0xFB,0xEF,0xF7,0xFD,0x7F,
0xBF,0x7F,0xDF,0xFB,0xEF,0xF7,0x7F,0xFE,
0xBF,0xFD,0xDF,0xFB,0xEF,0xF7,0x7F,0xFE,
0xBF,0xFD,0xDF,0xFB,0xEF,0xF7
};

static const uint8_t pin_km1[9]//1 2 3 4 5 6 11 12
{
0,
//0x10,0x20,0x40,0x80,0x10,0x20,0x02,0x01
0x10,0x20,0x40,0x80,0x10,0x20,0x20,0x10
};

static const uint8_t pin_km2[9]//1 2 3 4 5 6 11 12
{
0,
0x10,0x20,0x40,0x80,0x08,0x04,0x40,0x80
};
static const uint8_t dof[21]//1 2 3 4 5 6 11 12
{
0,
0x04,0x08,0x80,0x10,0x20,0x80,0x77,0x20,0x77,0x02,
0x08,0x08,0x10,0x01,0x02,0x40,0x40,0x01,0x02,0x04
};
static const uint8_t dof_[21]//1 2 3 4 5 6 11 12
{
0,  1,  2,  3,  4,  5,  6,  0,  8,  0,
10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20
};
uint16_t flex_cable(uint8_t val);

static const uint8_t km[21]//1 2 3 4 5 6 11 12
{
0,
0x10,   0x01,   0x20,   0x02,   0x77,   0x77,   0x77,   0x77,   0x80,  0x20,
0x10,   0x40,   0x04,   0x77,   0x20,   0x40,   0x40,   0x08,   0x77,  0x80
};

static const uint8_t km_[15]//1 2 3 4 5 6 11 12
{
0,   1,    2,    3,    4,    9,   10,  11,
12,  13,   15,   16,   17,   18,  20
};
 #endif //_PIN_MASK_FOR_74HC165_