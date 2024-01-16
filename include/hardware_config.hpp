#ifndef _PIN_CONFIG_STM32
#define _PIN_CONFIG_STM32

#include <stm32f1xx.h>
#include "gpio.hpp"

//USART1
#define usart_tx_pin						  	9
#define usart_rx_pin						    10

struct gpio_usart
{
    uint8_t TX=9;
    uint8_t RX=10;
};

//Parallel_BUS LCD
struct gpio_lcd_oled
{
    uint8_t D0=0;
    uint8_t D1=1;
    uint8_t D2=2;
    uint8_t D3=3;
    uint8_t D4=4;
    uint8_t D5=5;
    uint8_t D6=6;
    uint8_t D7=7;
    uint8_t RW=15;
    uint8_t EN=12;
    uint8_t RS=11;
};


//LEDS
struct Led
{
    uint8_t green=6;
    uint8_t red=7;
};

//595
#define data_pin						     8
#define clock_pin				             11
#define latcg_pin				             12
#define EN_1      				             2
/*struct _595
{
    uint8_t green=6;
    uint8_t red=7;
};*/

//165
#define data_pin						     8
#define clock_pin				             11
#define CS_595				                 7
#define EN_595     				             12
#define EN_165    				             11
#define pl_165				                 10


#define SCK_595                              13
#define MISO_595   		                     14
#define MOSI_595    		                 15



#define encoder_DT    		                 1
#define encoder_SW    		                 2
#define encoder_CLK    		                 0

void gpio_init(void);
#endif//#endif//_PIN_STM32