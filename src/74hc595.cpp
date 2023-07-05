#include "74hc595.hpp"
#include "gpio.hpp"


extern gpio stm32f103;


void HC74_595(uint16_t data)
{
uint8_t k=7;
stm32f103.set_pin_state(GPIOD,EN_1,1);
stm32f103.set_pin_state(GPIOC,latcg_pin,0);
for(int i=1;i<33;i++)
{
  data_state[i]=data;
  data_state[i]= data_state[i]&mask[k];
  k--;
  if(data_state[i]!=0)
  {
     data_state[i]=1;
  }
}
for(int j=1;j<9;j++)
{
  stm32f103.set_pin_state(GPIOB,data_pin,data_state[j]);
  stm32f103.set_pin_state(GPIOC,clock_pin,1);
  stm32f103.set_pin_state(GPIOC,clock_pin,0);
  stm32f103.set_pin_state(GPIOB,data_pin,0);
}
stm32f103.set_pin_state(GPIOC,latcg_pin,1);
stm32f103.set_pin_state(GPIOD,EN_1,0);
}


uint16_t set_pin_HC74_595(uint8_t val)
{
if((val>=1)&&(val<=8))
{
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
 }
if((val>=9)&&(val<=16))
{
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
}
if((val>=17)&&(val<=24))
{
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
  HC74_595(0xFF);
}
if((val>=25)&&(val<=32))
{
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
}
}