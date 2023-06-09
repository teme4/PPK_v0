﻿#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "math.h"

#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"

char str[80];

uint8_t res[32]={0,};

usart uart1;
dma_usart dma_usart1;
gpio stm32f103;
extern gpio gpio_stm32f103RC;


uint16_t data_state[32];
//volatile char rx_str[32];
char temp[1];


#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

volatile uint32_t *DWT_CONTROL2 = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT2 = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR2 = (uint32_t *)0xE000EDFC;
uint32_t Mcounter, count;

void delay_us(uint32_t us)
{
   int32_t us_count_tick =  us * (SystemCoreClock/1000000);
   //разрешаем использовать счётчик
   SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   //обнуляем значение счётного регистра
   DWT_CYCCNT  = 0;
   //запускаем счётчик
   DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;
   while(DWT_CYCCNT < us_count_tick);
   //останавливаем счётчик
   DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

void delay_ms(uint32_t ms)
{
   int32_t ms_count_tick =  ms * (SystemCoreClock/1000);
   //разрешаем использовать счётчик
   SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   //обнуляем значение счётного регистра
   DWT_CYCCNT  = 0;
   //запускаем счётчик
   DWT_CONTROL|= DWT_CTRL_CYCCNTENA_Msk;
   while(DWT_CYCCNT < ms_count_tick);
   //останавливаем счётчик
   DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}


void breakpoint(const char * data)
{
uart1.uart_tx_bytes(data);
uart1.uart_enter();
}


uint8_t HC74_165()
{
for(int i=0;i<32;i++)
{
res[i]=0;
}
//flex

stm32f103.set_pin_state(GPIOC,A0,1);
stm32f103.set_pin_state(GPIOB,A1,0);
stm32f103.set_pin_state(GPIOD,A2,1);

// щелкнули защелкой
stm32f103.set_pin_state(GPIOB,clk_165,0);
stm32f103.set_pin_state(GPIOB,pl_165,0);
delay_us(5);
stm32f103.set_pin_state(GPIOB,pl_165,1);
// щелкнули защелкой
for(int i=0;i<32;i++)
{
 if((stm32f103.get_state_pin(GPIOB,1))==1)
    {
    res[i]=i;
    }
    else
    {
    res[i]=0;
    }
//delay_us(5);
stm32f103.set_pin_state(GPIOB,clk_165,1);
delay_us(5);
stm32f103.set_pin_state(GPIOB,clk_165,0);
delay_us(5);
}
delay_us(500);
return *res;
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

void eth_config_out()
{
gpio_stm32f103RC.gpio_conf(GPIOC,eth1_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth2_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth3_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth4_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,eth5_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth6_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth7_out,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,eth8_out,gpio_stm32f103RC.gpio_mode_pp_50);
}


void eth_config_in()
{
gpio_stm32f103RC.gpio_conf(GPIOC,eth1_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth2_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth3_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth4_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOB,eth5_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth6_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth7_out,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOC,eth8_out,gpio_stm32f103RC.input_mode_floating);
}


void eth_test()
{
GPIO_TypeDef *ports_eth[8]={GPIOC,GPIOC,GPIOC,GPIOC,GPIOB,GPIOC,GPIOC,GPIOC};
uint8_t pins_eth[8]={eth1,eth2,eth3,eth4,eth5,eth6,eth7,eth8};
uint8_t pins_eth_out[8]={eth1_out,eth2_out,eth3_out,eth4_out,eth5_out,eth6_out,eth7_out,eth8_out};
uint8_t state[8]={0};
  //visible
  eth_config_out();
  for(int i=0;i<8;i++)
  {
  stm32f103.set_pin_state(GPIOA,pins_eth[i],0);
  delay_ms(1000);
  stm32f103.set_pin_state(ports_eth[i],pins_eth_out[i],1);
  stm32f103.set_pin_state(GPIOA,pins_eth[i],1);
  }
  //otchet
  eth_config_in();
  for(int i=0;i<8;i++)
  {
  stm32f103.set_pin_state(GPIOA,pins_eth[i],1);
  for(int k=0;k<8;k++)
  {
 if(stm32f103.get_state_pin(ports_eth[k],pins_eth_out[k])==1)
  {
  state[k]=k;
  }
  }
  }
 //  eth_config_out();
}

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
   RCC->APB2ENR |= RCC_APB2ENR_SPI1EN  ; // ??? ???????????? SPI

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
while (!(SPI1->SR & SPI_SR_TXE));
SPI1->DR = data ;
delay_ms(1);
}


uint8_t dof_pins[21]={0,};
uint8_t km_pins[21]={0,};
uint8_t km_state[24]={1,};
uint8_t dof_state[24]={'z',};
uint8_t dof_state_[24]={'x',};

uint8_t km_check()
{
for(int k=1;k<21;k++)
{
km_state[k]=1;
km_pins[k]='z';
}
for(int k=0;k<21;k++)
{
  flex_cable(k,1);
  if(res[5]==0x10 && res[8]==0x10)
  {
  km_pins[21]=1;
  }
  if(res[4]==km[k])
  km_pins[k]=k;
  if(res[5]==km[k])
  km_pins[k]=k;
  if(res[7]==km[k])
  km_pins[k]=k;
  if(res[8]==km[k])
  km_pins[k]=k;
  if(res[4]==0 && res[5]==0 && res[7]==0 && res[8]==0)
  km_pins[k]='N';
  delay_ms(1);
}
km_pins[1]=km_pins[21];
if(km_pins[1]==0)
km_pins[1]='N';
delay_ms(1);
//поменять местами четные и нечетные элементы//
uint8_t temp=0;
for(int i=1;i<21;i++)
{
temp=km_pins[i];
km_pins[i]=km_pins[i+1];
km_pins[i+1]=temp;
i++;
}
delay_ms(1);
//занулим не используемые пины//
uint8_t no_pin_km[6]={0,5,6,7,8,20};
for(int i=0;i<6;i++)
{
  km_pins[no_pin_km[i]]=0;
}
delay_ms(1);
int index=0,flag=0,pin=0;
//***********Выкинуть неиспользуемые элементы массива**********************************//
while(1)
{
if(km_pins[index]==0)
{
for(int i=index;i<21;i++)
{
  km_pins[i]=km_pins[i+1];
}
}
index++;
if(index>20)
{
if(flag>2)
break;
index=0;
flag++;
}
}
//*********************************************//
delay_ms(1);
for(int i=0;i<15;i++)
{
if(km_pins[i]==km_[i])
{
km_state[i+3]=0x00;
}
if(km_pins[i]=='N')
{
km_state[i+3]=0x02;
}
delay_ms(1);
if (km_pins[i]!=km_[i] && km_pins[i]!='N')
{
for(int j=1;j<20;j++)
{
  if(km_pins[i]==km_pins[j])
  {
  pin=km_pins[j];
  if(pin==0)
  {
  km_state[i+2]=0x02;
  }
  else
  km_state[i+2]=pin<<2|0x03;
  }
}
}
}

km_state[0]=0xAA;
km_state[1]=0x55;
km_state[2]=0x04;
km_state[18]=gencrc(km_state,18);
for(int k=0;k<19;k++)
{
uart1.uart_tx_byte(km_state[k]);
}
return *km_state;
}



uint8_t dof_check()
{
for(int k=1;k<21;k++)
{
dof_state[k]='z';
}
for(int k=0;k<21;k++)
{
  flex_cable(k,1);
  if(res[9]==dof[k])
  dof_pins[k]=k;
  if(res[10]==dof[k])
  dof_pins[k]=k;
  if(res[11]==dof[k])
  dof_pins[k]=k;
}
  for(int q=1;q<21;q++)
  {
   if(q>8)
    {
      dof_state[q]=dof_state[q+1]; //state
    }
  }
  for(int q=1;q<21;q++)
  {
   if(q>9)
    {
      dof_state[q]=dof_state[q+1]; //state
    }
  }
   for(int q=1;q<19;q++)
  {
   if(dof_pins[q]=='z')
    {
      dof_state[2+q]=0x02; //state
    }
  }
 /*for(int q=3;q<19;q++)
  {
   if(dof_pins[q-3]==dof_[q-3])
    {
      dof_state[q]=0x0;  //state ok
    }
  }*/
  for(int g=0;g<24;g++)
  {
   dof_state_[g+3]=dof_state[g];
  }
 
dof_state_[0]=0xAA;
dof_state_[1]=0x55;
dof_state_[2]=0x11;
 

  



dof_state_[21]=gencrc(dof_state_,20);
for(int k=0;k<24;k++)
{
uart1.uart_tx_byte(dof_state_[k]);
}
return *dof_state_;
}


uint8_t flex_check()
{
for(int k=0;k<21;k++)
{
km_state[k]=2;
km_pins[k]=0;
}
for(int k=1;k<21;k++)
{
  flex_cable(k,0);
  delay_ms(3);
  if(res[0]==flex_16[k])
  km_pins[k]=k;
  if(res[1]==flex_16[k])
  km_pins[k]=k;
  if(res[2]==flex_16[k])
  km_pins[k]=k;
}

delay_ms(1);
//*********************************************//
uint8_t pin=0;
for(int i=0;i<21;i++)
{
if(km_pins[i]==flex_16_[i])
{
km_state[i+3]=0x00;
}
delay_ms(1);
}
km_state[0]=0xAA;
km_state[1]=0x55;
km_state[2]=0x06;
km_state[19]=gencrc(km_state,19);
for(int k=0;k<20;k++)
{
uart1.uart_tx_byte(km_state[k]);
}
return *km_state;
}


int main()
{
gpio_init();
uart1.usart_init();

   SettingsSPI(SPI1,
                RegCR1::ACTIVE,
                  RegCR1::MASTER,
                    1 /*Mbps*/,
                      RegCR1::SPI_MODE3,
                        RegCR1::DFF8bit,
                          RegCR1::MSBF);

uint8_t data[32]={0,};
uint8_t test_data[16]={0xAA,0x55,0x02,0x00,0x01,0x02,0x3B,0x00,0x01,0x02,0x53,0xF0};
//AA 55 02 00 01 02 3B 00 01 02 53 F0
//0x00 = OK
//0x01 = K3
//0x02 = OB
//0x3x = HP
/*
delay_ms(500);
test_data[12]=gencrc(test_data ,12);
for(int k=0;k<12;k++)
{
uart1.uart_tx_byte(test_data[k]);
}*/



while(1)
{
//km_check();
//for(int i=1;i<21;i++)

flex_check();
delay_ms(1000);


//flex_cable(1);
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
