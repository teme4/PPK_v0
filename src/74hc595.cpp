#include "74hc595.hpp"
#include "gpio.hpp"
#include "delay.hpp"

extern gpio stm32f103;
extern uint16_t data_state[32];

void spi_transmit(uint16_t data);



void HC74_595_SPI(uint32_t data,uint8_t mode)
{
if(mode==0)
data=~data;

uint16_t data1,data2;
data1=data&0xFF;
data2=data>>8;
stm32f103.set_pin_state(GPIOB,EN_595,1);
stm32f103.set_pin_state(GPIOB,CS_595,0);

spi_transmit(data2);
spi_transmit(data1);
delay_ms(5);
stm32f103.set_pin_state(GPIOB,CS_595,1);
stm32f103.set_pin_state(GPIOB,EN_595,0);
}

void HC74_595_SET(uint16_t data1,uint16_t data2,uint8_t mode)
{
SPI2->CR1 |= static_cast<uint32_t>(0b11);//mode3
HC74_595_SPI(data2,mode);
HC74_595_SPI(data1,mode);
}