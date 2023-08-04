#ifndef _PIN_MASK_FOR_74HC165_
#define _PIN_MASK_FOR_74HC165_
#include <stm32f1xx.h>
#include "hardware_config.hpp"
//#include "gpio.hpp"
#include "vector"



extern uint8_t res[32];
std::vector<uint16_t> flex_cable(void);



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


static const uint8_t km[21]//1 2 3 4 5 6 11 12
{
0,
0x10,   0x01,   0x20,   0x02,   0x77,   0x77,   0x77,   0x77,   0x80,  0x20,
0x10,   0x40,   0x04,   0x80,   0x20,   0x40,   0x40,   0x08,   0x77,  0x80
};

static const uint8_t flex_16[21]//1 2 3 4 5 6 11 12
{
0,
0xFE,   0xFD,   0xDF,   0xFB,   0xEF,   0xF7,   0xFD,   0xFE,
0xBF,   0x7F,   0xDF,   0xFB,   0xEF,   0xF7,   0x7F,   0xFE
};

static const uint8_t flex_16_[21]//1 2 3 4 5 6 11 12
{
0,  1,  2,  3,  4,  5,  6,  7,  8,
9,  10, 11, 12, 13, 14, 15, 16,
};

static const uint8_t flex_16__[17][17]//1 2 3 4 5 6 11 12
{
{0,0}, // 1 1
{0xFE,0},// 2 2
{0xFD,0},
{0xDF,0},
{0xFB,0},
{0xEF,0},
{0xF7,0},
{0xFD,1},
{0xFE,1},
{0xBF,1},
{0x7F,1},
{0xDF,1},
{0xFB,1},
{0xEF,1},
{0xF7,1},
{0x7F,2},
{0xFE,2}
};

/*
****km1****
1pin -  1-1
2pin -  12-1
3pin -  6-1
4pin -  11-1
9pin -  4-1
10pin - 2-1
11pin - 5-1
12pin - 3-1
****km2****
1pin -  1-2
13pin - 6-2
14pin - 12-2
15pin - 2-2
16pin - 11-2
17pin - 3-2
18pin - 5-2
20pin - 4-2
*/
static const uint8_t km_[15]//1 2 3 4 5 6 11 12
{
2,    1,    4,    3,    10,   9,  12,  11,  14, 13,  16,   15,   18,   17,  20
};
 #endif //_PIN_MASK_FOR_74HC165_