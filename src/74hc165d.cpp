
#include "74hc165d.hpp"
#include "gpio.hpp"
#include "74hc595.hpp"
#include "delay.hpp"
#include "vector"


std::vector<uint16_t> vector_pins;  // создали вектор
volatile uint16_t pin_bits[32][5]={{0,0,0,0,0},};
extern gpio stm32f103;

//std::vector<uint16_t> flex_cable()
uint16_t flex_cable()
{
   SPI2->CR1 |= static_cast<uint32_t>(0b11);//mode3
  uint8_t chip=15;
  vector_pins.reserve(16);  // указали число ячеек
 // set_pin_HC74_595(val,state);
  stm32f103.set_pin_state(GPIOB,EN_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,1);
  stm32f103.set_pin_state(GPIOB,pl_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,1);
  for(int i=0;i<chip;i++)
  {
    while(!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR=0x00;
    while(!(SPI2->SR & SPI_SR_RXNE));
    vector_pins[i]= SPI2->DR;
    res[i]=SPI2->DR;
  }
  stm32f103.set_pin_state(GPIOB,EN_165,1);
return *res;
 }
