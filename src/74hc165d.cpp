
#include "74hc165d.hpp"


uint16_t IC_74HC165::flex_cable()
{
   SPI2->CR1 |= static_cast<uint32_t>(0b11);//mode3
  uint8_t chip=15;
  vector_pins.reserve(16);
  GPIOB->BSRR = (1<<EN_165+16);//L
    GPIOB->BSRR = (1<<pl_165);//H
      GPIOB->BSRR = (1<<pl_165+16);//L
        GPIOB->BSRR = (1<<pl_165);//H
  for(int i=0;i<chip;i++)
  {
    while(!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR=0x00;
    while(!(SPI2->SR & SPI_SR_RXNE));
    vector_pins[i]= SPI2->DR;
    res[i]=SPI2->DR;
  }
 GPIOB->BSRR = (1<<EN_165);//H
return *res;
}