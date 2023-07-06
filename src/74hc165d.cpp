
#include "74hc165d.hpp"
#include "gpio.hpp"
#include "74hc595.hpp"



volatile uint16_t pin_bits[32][5]={{0,0,0,0,0},};
extern gpio stm32f103;


uint16_t flex_cable(uint8_t val)
{
  uint8_t chip=13;
  set_pin_HC74_595(val);
  stm32f103.set_pin_state(GPIOB,pl_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,1);

  stm32f103.set_pin_state(GPIOC,A0,1);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,1);

 //stm32f103.set_pin_state(GPIOB,A1,0);
  for(int i=0;i<chip;i++)
  {
    while(!(SPI1->SR & SPI_SR_TXE)) ;
    SPI1->DR=0x00;
    while(!(SPI1->SR & SPI_SR_RXNE)) ;
    res[i]= SPI1->DR;
  }
 //stm32f103.set_pin_state(GPIOB,A1,1);
                                                                                                                          while(SPI1->SR&SPI_SR_BSY); //Передача завершена
  stm32f103.set_pin_state(GPIOC,A0,0);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,0);

  pin_bits[val][1]=res[0];
  pin_bits[val][2]=res[1];
  pin_bits[val][3]=res[3];
  pin_bits[val][4]=res[3];

  if(val>=1&&val<=6)
    {
      pin_bits[val][0]=res[0];
    }
  if(val>=7&&val<=14)
    {
      pin_bits[val][0]=res[1];
    }
  if(val>=15&&val<=22)
    {
      pin_bits[val][0]=res[2];
    }
     if(val>=23&&val<=30)
    {
      pin_bits[val][0]=res[3];
    }
 } 