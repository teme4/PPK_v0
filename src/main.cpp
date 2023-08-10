#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "math.h"
#include "delay.hpp"

#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "rcc.hpp"

char str[80];

uint8_t res[32]={0,};

usart usart1;
dma_usart dma_usart1;
gpio stm32f103;
extern gpio gpio_stm32f103RC;


uint16_t data_state[32];
//volatile char rx_str[32];
char temp[1];

void breakpoint(const char * data)
{
usart1.uart_tx_bytes(data);
usart1.uart_enter();
}


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
}
//----------------------------------------------------------

void RCC_init()
{
   BKP->CR &=~ BKP_CR_TPE;                                  //The TAMPER pin is free for general purpose I/O
   BKP->CR |= BKP_CR_TPAL;                                  //0: A high level on the TAMPER pin resets all data backup registers (if TPE bit is set).
   BKP->CSR =0;                                             //ничего важного
   PWR->CR |= PWR_CR_DBP;                                   //1: Включен доступ к RTC и резервным регистрам
   RCC->BDCR &=~ RCC_BDCR_BDRST;
   RCC->BDCR &=~RCC_BDCR_RTCEN;                              //0: RTC clock disabled
   RCC->BDCR &=~ RCC_BDCR_LSEON;                            // выключаем LSE
   while (RCC->BDCR & RCC_BDCR_LSERDY){}                    // ждем пока генератор выключится

   RCC->CR &=~ RCC_CR_HSEON;                                // выключаем HSE
   RCC->CR |= RCC_CR_HSION;                                 // включаем HSI генератор
   while (!(RCC->CR & RCC_CR_HSIRDY)){}                     // ждем пока генератор не включится

   RCC->CFGR |= RCC_CFGR_SW_HSI;                            // выбрали HSI в качестве системного тактирования
   RCC->CFGR &=~ RCC_CFGR_PLLSRC;
   RCC->CFGR &=~ RCC_CFGR_PLLMULL_0;                        // выбираем при выключенном PLL !! PLL=x8 (4Мгц *8 =32 Мгц)
   RCC->CFGR|= RCC_CFGR_PLLMULL_1 |RCC_CFGR_PLLMULL_2;

   RCC->CR |= RCC_CR_PLLON;                                 // включаем PLL
   while (!(RCC->CR & RCC_CR_PLLRDY)){}                     // ждем пока умножитель не включится

   RCC->CFGR |= RCC_CFGR_SW_PLL;                            // выбрали PLL в качестве системного тактирования
   RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // ABH Prescaler установлен в деление на 1
   RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 Prescaler установлен в деление на 1
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 Prescaler установлен в деление на 1
}

extern uint32_t SystemCoreClock;
// clang-format off
enum class IRQnSPI
{
    TXEIE                                       = (1<<7),///<прерываниe по переполнени. приемного буфера FIFO
    RXNEIE                                      = (1<<6),///<прерываниe по таймауту приемника (буфер FIFO приемника не пуст и не было попыток его чтения в течение времени таймаута)
    ERRIE                                       = (1<<5),///<прерываниe по заполнению на 50 % и более буфера FIFO приемника
    NONE                                        = (0<<0)
};

enum class RegCR1
{
    SPI_MODE0                                   = (0b00 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 0);
    SPI_MODE1                                   = (0b01 ),///<SPI фирмы Motorola(CPOL = 0, CPHA = 1);
    SPI_MODE2                                   = (0b10 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 0);
    SPI_MODE3                                   = (0b11 ),///<SPI фирмы Motorola(CPOL = 1, CPHA = 1);
    MASTER                                      = (1<<2 ),///<ведущий модуль
    SLAVE                                       = (0<<0 ),///<ведомый модуль
    ACTIVE                                      = (1<<6 ),///<работа разрешена
    INACTIVE                                    = (0<<0 ),///<работа запрещена;
    DFF8bit                                     = (0<<0 ),///<data frame format: 8bit;
    DFF16bit                                    = (1<<11),///<data frame format: 16bit;
    LSBF                                        = (1<<7 ),///<LSB  transmitted first
    MSBF                                        = (0<<0 ),///<MSB transmitted first
};
enum class RegDMACR
{
    RXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO приемника
    RXDMA_EN                                    = 0x01,///<Разрешено формирование запросов DMA буфера FIFO приемника
    TXDMA_DIS                                   = 0x00,///<Запрещено формирование запросов DMA буфера FIFO передатчика
    TXDMA_EN                                    = 0x02,///<Разрешено формирование запросов DMA буфера FIFO передатчика
    NONE                                        = 0x00
};
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
    //SPIx->CR2 |= static_cast<uint32_t>(RxDmacr);

    SPIx->CR1 |= static_cast<uint32_t>(SPE);
                      }

void spi_transmit(uint16_t data)
{
while (!(SPI2->SR & SPI_SR_TXE));
SPI2->DR = data ;
}
uint16_t flex_14_[14]=
{
  1,
  2,
  4,
  8,
  16,
  32,
  64,
  128,
  256,
  512,
  1024,
  2048,
  4096,
  8192,
};

uint16_t SD_CS[14]=
{
0x1,
0x2,
0x4,
0x8,
0x10,
0x20,
0x40,
0x80,
0x1,
0x2,
0x4,
0x8,
0x10,
0x20,
};

void HC74_595_SPI(uint32_t data)
{
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

void HC74_595_SET(uint16_t data1,uint16_t data2)
{
SPI2->CR1 |= static_cast<uint32_t>(0b11);//mode3
HC74_595_SPI(data2);
HC74_595_SPI(data1);
}


uint8_t result[14]={0x77,};
uint8_t result_buff[32]={0,};

void check_SD_SC()
{
for(int i=0;i<14;i++)
{
  HC74_595_SET(flex_14_[i],0x0000);
  flex_cable();
    
    if(i<8) //OK
    {
      if(res[10]!=0 && res[9]==0)//OK
      {
        result[i]=res[10];
      }
      if(res[10]==0 && res[9]==0)//OBR
      {
       result[i]=0x77;
      }
      if(res[10]!=0 && res[9]!=0)
      {
       result[i]=0x55;
      }
    }

    if(i>7) //OK
    {
      if(res[9]!=0 && res[10]==0)//OK
      {
        result[i]=res[9];
      }
      if(res[9]==0 && res[10]==0)//OBR
      {
       result[i]=0x77;
      }
      if(res[9]!=0 && res[10]!=0)
      {
       result[i]=0x55;
      }
    }
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=0x06;//SD_CS
for(int i=0;i<14;i++)
{
  //Условие все верно
  if(result[i]==SD_CS[i])  // OK
  result_buff[i+3]=0x00;
  //Условие обрыв линии
  if(result[i]==0x77)  // OБ
 result_buff[i+3]=0x02;
  //Условие неверная расиновка
  if(result[i]==0x55)  //НР
  result_buff[i+3]=0x03;
}
result_buff[17]=gencrc(result_buff, 17);

for(int k=0;k<18;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}

}





int main()
{
gpio_init();
usart1.usart_init();
SettingsSPI(SPI2,
            RegCR1::ACTIVE,
            RegCR1::MASTER,
            2 /*Mbps*/,
            RegCR1::SPI_MODE1,//1 => 595 3=>165D
            RegCR1::DFF8bit,
            RegCR1::MSBF);

int k=ClockInit();

uint8_t data[32]={0,};
uint8_t test_data[16]={0xAA,0x55,0x02,0x00,0x01,0x02,0x3B,0x00,0x01,0x02,0x53,0xF0};
usart1.uart_tx_bytes("Start");

//AA 55 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 XX;
//0x00 = OK
//0x01 = K3
//0x02 = OB
//0x3x = HP

check_SD_SC();
check_SD_SC();

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
