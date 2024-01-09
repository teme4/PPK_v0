#include "main.hpp"
#include <stm32f1xx.h>
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "rcc.hpp"
#include "lcd.hpp"
#include "lcd_menu.hpp"
//#include "connectors_pins.hpp"

#include "uart.hpp"
#include "gpio.hpp"
//#include "dma.hpp"
//#include "string.h"
#include "hardware_config.hpp"
//#include "math.h"
//#include "delay.hpp"
//#include <string>
//#include <vector>


//dma_usart dma_usart1;

//char str[80];
//uint8_t res[32]={0,};
//uint16_t data_state[32];
//char temp[1];

/*
void breakpoint(const char * data)
{
usart1.uart_tx_bytes(data);
usart1.uart_enter();
}*/

/*
//----------------------------------------------------------
extern "C" void DMA1_Channel4_IRQHandler(void)
{
  if((DMA1->ISR & DMA_ISR_TCIF4) == DMA_ISR_TCIF4)
  {
    DMA1->IFCR|= DMA_IFCR_CTCIF4;
   //dma_usart1::fl_tx = 1;
   dma_usart1._fl_tx=1;
  }
  else if((DMA1->ISR & DMA_ISR_TEIF4)== DMA_ISR_TEIF4)
  {
    //Disable DMA channels
    DMA1_Channel4->CCR&=~ DMA_CCR_EN;
    DMA1_Channel5->CCR&=~ DMA_CCR_EN;
  }
}
//----------------------------------------------------------
extern "C" void DMA1_Channel5_IRQHandler(void)
{
  if(READ_BIT(DMA1->ISR, DMA_ISR_TCIF5) == (DMA_ISR_TCIF5))
  {
    DMA1->IFCR|= DMA_IFCR_CTCIF5;
    dma_usart1._fl_rx = 1;
  }
  else if(READ_BIT(DMA1->ISR, DMA_ISR_TEIF5) == (DMA_ISR_TEIF5))
  {
    //Disable DMA channels
    DMA1_Channel4->CCR&=~ DMA_CCR_EN;
    DMA1_Channel5->CCR&=~ DMA_CCR_EN;
  }
}*/

extern uint32_t SystemCoreClock;

void SettingsSPI (SPI_TypeDef*SPIx ,RegCR1 SPE,
                      RegCR1 MS,
                      double frequency,
                      RegCR1 Type,
                      RegCR1 WordSize,
                      RegCR1 LsbMsbFirst)
                      {
   RCC->APB1ENR |= RCC_APB1ENR_SPI2EN  ; // ??? ???????????? SPI

    SPIx->CR1    = 0;
    SPIx->CR2    = 0;
    SPIx->SR     = 0;
    SPIx->CRCPR  = 0;
    SPIx->RXCRCR = 0;
    SPIx->TXCRCR = 0;
    //BRR = static_cast<uint8_t>(log2(F_SPICLK / (Frequency * 1000000)) - 1);

    SPIx->CR1 |=  SPI_CR1_BR;
    SPIx->CR1 |= static_cast<uint32_t>(Type);
    SPIx->CR1 |= static_cast<uint32_t>(WordSize);
    SPIx->CR1 |= static_cast<uint32_t>(LsbMsbFirst);
    SPIx->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;
    SPIx->CR1 |= static_cast<uint32_t>(MS);
   // SPIx->CR2 |= static_cast<uint32_t>(TxDmacr);
   // SPIx->CR2 |= static_cast<uint32_t>(RxDmacr);

    SPIx->CR1 |= static_cast<uint32_t>(SPE);
                      }

void spi_transmit(uint16_t data)
{
while (!(SPI2->SR & SPI_SR_TXE));
SPI2->DR = data ;
}

int main()
{
//gpio_lcd_oled gpio_lcds;
extern lcd oled;
extern gpio gpio_stm32f103RC;
extern usart usart1;
extern Led mcu_led;

gpio_init();
usart1.usart_init();
SettingsSPI(SPI2,
            RegCR1::ACTIVE,
            RegCR1::MASTER,
            2,
            RegCR1::SPI_MODE1,//1 => 595 3=>165D
            RegCR1::DFF8bit,
            RegCR1::MSBF);

int k=ClockInit();
oled.InitializeLCD();

gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,0);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,1);

oled.ClearLCDScreen();
oled.busy_flag();

adc_init();
start_menu();

while(1)
{


}
}

extern "C"
{
void HardFault_Handler(void)
{
  int k=0;
  while(1)
  {
    k++;
  }
}
}
