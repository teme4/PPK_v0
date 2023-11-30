#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "math.h"
#include "delay.hpp"

//#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "rcc.hpp"
//#include "tft2.hpp"



char str[80];

uint8_t res[32]={0,};

usart usart1;
dma_usart dma_usart1;
gpio stm32f103;

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

uint32_t pku_nkk_21[20]=
{
1,
0,
2,
3,
0,
4,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
};

uint32_t pku_nkk_22[20]=
{
1,
0,
2,
3,
0,
4,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
};

uint32_t pku_nkk_uart[20]=
{
7,
0,
6,
0,
0,
0,
0,
0,
0,
5,
0,
0,
0,
0,
0,
0,
0,
0,
0,
0,
};
uint32_t dof_pins_2[20]=
{
25,
19,
26,
5,
11,
6,
12,
0,
10,
0,
20,
22,
21,
23,
14,
24,
15,
17,
16,
13,
};

uint32_t km1_2[20]=
{
12,
16,
11,
6,
0,
0,
0,
0,
2,
4,
3,
5,
12,
6,
11,
2,
5,
3,
4,
0,
};

uint32_t flex_14_[14]=
{
  1,//1
  2,//2
  4,//3
  8,//4
  16,//5
  32,//6
  64,//7
  128,//8
  256,//9
  512,//10
  1024,//11
  2048,//12
  4096,//13
  8192,//14
  /*16384,//15
  32768,//16
  65536,//17
  131072,//18
  524288,//19*/
  };


uint32_t resolve(uint32_t val)
{
for(int i=0;i<32;i++)
{
    if((val & 1<<i)==val)
    {
      return i+1;
    }
    else
    {
      i++;
      i--;
    }
}
}

uint32_t find_K3(uint32_t *val)
{
for(int i=0;i<32;i++)
{
  for(int j=0;j<32;j++)
  {
     if(val[i]==val[j] && i!=j)
     {

     }
  }
}
}



uint16_t SD_CS[20]=
{
    0b000000000011111111111110,//1
    0b000000000011111111111101,//2
    0b000000000011111111111011,//3
    0b000000000011111111110111,//4
    0b000000000011111111101111,//5
    0b000000000011111111011111,//6
    0b000000000011111110111111,//7
    0b000000000011111101111111,//8
    0b000000000011111011111111,//9
    0b000000000011110111111111,//10
    0b000000000011101111111111,//11
    0b000000000011011111111111,//12
    0b000000000010111111111111,//13
    0b000000000001111111111111,//14
    0b000000000011111011111111,//15
    0b000000000011110111111111,//16
    0b000000000011101111111111,//17
    0b000000000011011111111111,//18
    0b000000000010111111111111,//19
    0b000000000001111111111111,//20
};

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

uint8_t check_K3(uint8_t value)
{
  uint8_t count=0;
for (int i{0}; i < 8; i ++)
        count += static_cast<bool>(value & (1<<i));
        return count;
}

uint8_t result[32]={0x77,},n=0,m=0;
uint32_t k3[32]={0,};
uint8_t pin_map[32][32]={{0},{0}};
uint32_t ob[32]={0,};
uint32_t kz[32]={0,};
uint32_t obr[32]={0,};
uint8_t error[32]={0x77,};
uint8_t result_buff[32]={0x88,};
uint8_t temp_=0;
uint8_t state_pin[32]={0,};
uint8_t ignore[32]={0,};

uint8_t check_num_0(uint8_t value)
{
uint8_t count=0;
for (uint8_t i=0; i<8; i++)
{
count += static_cast<bool>(value & (1<<i));
}
return count;
}



void check_km(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
}
}
for(int x=0;x<num;x++)
   {
      if(ob[x]!=0)
      kz[x]=resolve(ob[x]);
      if(kz[x]>15)
      kz[x]=kz[x]-16;
    }
count++;
/****************************************************/

for(int x=0;x<num;x++)
   {
          if(kz[x]==km1_2[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1)
              {
              ignore[x]=0x77;
              state_pin[x]=0x00; 
              }
              if(ignore[x]!=0x77)
              {
                state_pin[x]=0x03; //HP
                ignore[x]=0x99;
              }
        }


}
for(int i=0;i<20;i++)
{
if(ob[i]==0)
     state_pin[i]=0x02; //OB
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

//result_buff[4]=0x00;
for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;

}


void check_DOF(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];
}
}
  for(int x=0;x<num;x++)
   {
      kz[x]=resolve(ob[x]);
    if(ob[x]==0)
     {
        state_pin[x]=0x02; //OB
        kz[x]=100;
        ignore[x]=0x88;
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
 

          if(kz[x]==dof_pins_2[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(ignore[x]!=0x77)
              {
                state_pin[x]=0x03; //OB
                ignore[x]=0x99;
              }
        }


}
for(int i=0;i<32;i++)
{
if(ob[i]==0)
     state_pin[i]=0x02; //OB
}

 if(state_pin[7]==state_pin[9])
{
state_pin[7]=0;
state_pin[9]=0;
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
}

void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[1];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=res[1];
}
}
 for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j && kz[i]!=0)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
         if(ob[x]==0)
        {
              state_pin[x]=0x02; //OB
        }
         if(kz[x]==pku_nkk_21[x])
         {
              state_pin[x]=0x00; //OK
         }
         if(kz[x]!=pku_nkk_21[x] && ob[x]!=0)
         {
              state_pin[x]=0x03; //HP
         }
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_PKU_NKK_2_2(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[1];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=res[1];
}
}
 for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j && kz[i]!=0)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
         if(ob[x]==0)
        {
              state_pin[x]=0x02; //OB
        }
          if(kz[x]==pku_nkk_22[x])
         {
              state_pin[x]=0x00; //OK
         }
         if(kz[x]!=pku_nkk_22[x] && ob[x]!=0)
         {
              state_pin[x]=0x03; //HP
         }
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_PKU_NKK_3(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[1];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=res[1];
}
}
 for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j && kz[i]!=0)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
        if(kz[x]==0)
        {
              state_pin[x]=0x02; //OB
        }
          if(kz[x]==pku_nkk_uart[x])
         {
              state_pin[x]=0x00; //OK
         }
         if(kz[x]!=pku_nkk_uart[x] && kz[x]!=0)
         {
              state_pin[x]=0x03; //HP
         }
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_eth(uint8_t num,uint8_t num_cable)
{
uint8_t count=0;
for(int i=0;i<num;i++)
{
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[6];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[6];
}
  for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
    if(ob[x]==0)
     {
        state_pin[x]=0x02; //OB
        kz[x]=100;
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
 

          if(kz[x]==x+1)
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(ignore[x]!=0x77)
              {
                state_pin[x]=0x03; //OB
                ignore[x]=0x99;
              }
        }


}
for(int i=0;i<32;i++)
{
if(ob[i]==0)
  state_pin[i]=0x02; //OB
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_ext_fridge(uint8_t num,uint8_t num_cable)
{
   uint8_t count=0,k;
////////////////////////////////Обнулим буффер
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////Опросим каждый пин

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
}
}
  for(int x=0;x<num;x++)
   {
     count=0;
  for(int z=0;z<num;z++)
   {
        kz[z]=k3[z] & 1<<x;
        if(kz[z]>0)
        count++;
   }
  if(count==0)
  {
    state_pin[x]=0x02; //OBR
   // k3[x]=k3[x] & 1<<x;
  }
  if(count==num-1)
  {
      if(kz[x]==0)
      state_pin[x]=0x00; //OK
      else
      state_pin[x]=0x03; //OK
  }
  if(count<num-1 && count!=0)
   state_pin[x]=0x01; //K3
   }

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

 state_pin[0]=0x00; //OK
 state_pin[1]=0x00; //OK
 state_pin[12]=0x00; //OK
 state_pin[13]=0x00; //OK
 state_pin[14]=0x00; //OK
 state_pin[15]=0x00; //OK

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}

result[21]=0x77;
}

void check_SD_SC2(uint8_t num,uint8_t num_cable)
{
   uint8_t count=0,k;
////////////////////////////////Обнулим буффер
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////Опросим каждый пин

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
}
}
  for(int x=0;x<num;x++)
   {
     count=0;
  for(int z=0;z<num;z++)
   {
        kz[z]=k3[z] & 1<<x;
        if(kz[z]>0)
        count++;
   }
  if(count==0)
  {
    state_pin[x]=0x02; //OBR
   // k3[x]=k3[x] & 1<<x;
  }
  if(count==num-1)
  {
      if(kz[x]==0)
      state_pin[x]=0x00; //OK
      else
      state_pin[x]=0x03; //OK
  }
  if(count<num-1 && count!=0)
   state_pin[x]=0x01; //K3
   }

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}

result[21]=0x77;
}

int main()
{
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





//fillScreen(RED);

  //  writeRegister8(ILI9341_DISPLAYON, 0);
//fillScreen(RED);
//fillScreen(GREEN);
//LCD_DrawPixel(10,15, GREEN);
//check_PKU_NKK(20,0x09);
//check_DOF(20,0x12);


//check_eth(8,0x17);



/*
check_PKU_NKK_3(20,0x08);
check_PKU_NKK_2_1(20,0x09);*/
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
