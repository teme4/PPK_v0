
#include "74hc165d.hpp"
#include "gpio.hpp"
#include "74hc595.hpp"
#include "vector"
using namespace std;

vector <uint16_t> vector_pins;  // создали вектор



volatile uint16_t pin_bits[32][5]={{0,0,0,0,0},};
extern gpio stm32f103;


uint16_t flex_cable()
{
  uint8_t chip=14;
  vector_pins.reserve(16);  // указали число ячеек
 // set_pin_HC74_595(val,state);
  stm32f103.set_pin_state(GPIOB,EN_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,1);

  for(int i=0;i<chip;i++)
  {
    while(!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR=0x00;
    while(!(SPI2->SR & SPI_SR_RXNE));
    res[i]= SPI2->DR;
  }
  stm32f103.set_pin_state(GPIOB,EN_165,1);
return *res;

  //return vector_pins;
  /*                                                                                                                        while(SPI1->SR&SPI_SR_BSY); //Передача завершена
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
    */
 }
