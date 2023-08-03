#include "74hc595.hpp"
#include "gpio.hpp"


extern gpio stm32f103;
extern uint16_t data_state[32];

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

/*
void set_pin_HC74_595(uint8_t _val,uint8_t state)
{
   uint8_t mask,val;
//uint8_t revers=0x00;
//revers=~(pin_HC595[5]);
if(state==1)
{
  mask=0x00;
}
if(state==0)
{
  mask=0xFF;
}

if((_val>=1)&&(_val<=8))
{
  HC74_595(mask);
  HC74_595(mask);
  HC74_595(mask);
  if(state==1)
    HC74_595(~pin_HC595[_val]);
  else
      HC74_595(pin_HC595[_val]);
 }
if((_val>=9)&&(_val<=16))
{
  HC74_595(mask);
  HC74_595(mask);
   if(state==1)
    HC74_595(~pin_HC595[_val]);
  else
   HC74_595(pin_HC595[_val]);
   HC74_595(mask);
}
if((_val>=17)&&(_val<=24))
{
  HC74_595(mask);
   if(state==1)
    HC74_595(~pin_HC595[_val]);
  else
      HC74_595(pin_HC595[_val]);
  HC74_595(mask);
  HC74_595(mask);
}
if((_val>=25)&&(_val<=32))
{
   if(state==1)
    HC74_595(~pin_HC595[_val]);
  else
      HC74_595(pin_HC595[_val]);
  HC74_595(mask);
  HC74_595(mask);
  HC74_595(mask);
}
}
*/