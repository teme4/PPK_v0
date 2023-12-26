#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include <string>
#include <vector>
#include "hardware_config.hpp"
#include "math.h"
#include "delay.hpp"

#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "rcc.hpp"
#include "lcd.hpp"

#include <string>
#include <vector>

char str[80];

uint8_t res[32]={0,};

usart usart1;
dma_usart dma_usart1;
gpio stm32f103;

uint16_t data_state[32];
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
   BKP->CSR =0;                                             //������ �������
   PWR->CR |= PWR_CR_DBP;                                   //1: ������� ������ � RTC � ��������� ���������
   RCC->BDCR &=~ RCC_BDCR_BDRST;
   RCC->BDCR &=~RCC_BDCR_RTCEN;                              //0: RTC clock disabled
   RCC->BDCR &=~ RCC_BDCR_LSEON;                            // ��������� LSE
   while (RCC->BDCR & RCC_BDCR_LSERDY){}                    // ���� ���� ��������� ����������

   RCC->CR &=~ RCC_CR_HSEON;                                // ��������� HSE
   RCC->CR |= RCC_CR_HSION;                                 // �������� HSI ���������
   while (!(RCC->CR & RCC_CR_HSIRDY)){}                     // ���� ���� ��������� �� ���������

   RCC->CFGR |= RCC_CFGR_SW_HSI;                            // ������� HSI � �������� ���������� ������������
   RCC->CFGR &=~ RCC_CFGR_PLLSRC;
   RCC->CFGR &=~ RCC_CFGR_PLLMULL_0;                        // �������� ��� ����������� PLL !! PLL=x8 (4��� *8 =32 ���)
   RCC->CFGR|= RCC_CFGR_PLLMULL_1 |RCC_CFGR_PLLMULL_2;

   RCC->CR |= RCC_CR_PLLON;                                 // �������� PLL
   while (!(RCC->CR & RCC_CR_PLLRDY)){}                     // ���� ���� ���������� �� ���������

   RCC->CFGR |= RCC_CFGR_SW_PLL;                            // ������� PLL � �������� ���������� ������������
   RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // ABH Prescaler ���������� � ������� �� 1
   RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 Prescaler ���������� � ������� �� 1
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 Prescaler ���������� � ������� �� 1
}

extern uint32_t SystemCoreClock;
// clang-format off
enum class IRQnSPI
{
    TXEIE                                       = (1<<7),///<���������e �� �����������. ��������� ������ FIFO
    RXNEIE                                      = (1<<6),///<���������e �� �������� ��������� (����� FIFO ��������� �� ���� � �� ���� ������� ��� ������ � ������� ������� ��������)
    ERRIE                                       = (1<<5),///<���������e �� ���������� �� 50 % � ����� ������ FIFO ���������
    NONE                                        = (0<<0)
};

enum class RegCR1
{
    SPI_MODE0                                   = (0b00 ),///<SPI ����� Motorola(CPOL = 0, CPHA = 0);
    SPI_MODE1                                   = (0b01 ),///<SPI ����� Motorola(CPOL = 0, CPHA = 1);
    SPI_MODE2                                   = (0b10 ),///<SPI ����� Motorola(CPOL = 1, CPHA = 0);
    SPI_MODE3                                   = (0b11 ),///<SPI ����� Motorola(CPOL = 1, CPHA = 1);
    MASTER                                      = (1<<2 ),///<������� ������
    SLAVE                                       = (0<<0 ),///<������� ������
    ACTIVE                                      = (1<<6 ),///<������ ���������
    INACTIVE                                    = (0<<0 ),///<������ ���������;
    DFF8bit                                     = (0<<0 ),///<data frame format: 8bit;
    DFF16bit                                    = (1<<11),///<data frame format: 16bit;
    LSBF                                        = (1<<7 ),///<LSB  transmitted first
    MSBF                                        = (0<<0 ),///<MSB transmitted first
};
enum class RegDMACR
{
    RXDMA_DIS                                   = 0x00,///<��������� ������������ �������� DMA ������ FIFO ���������
    RXDMA_EN                                    = 0x01,///<��������� ������������ �������� DMA ������ FIFO ���������
    TXDMA_DIS                                   = 0x00,///<��������� ������������ �������� DMA ������ FIFO �����������
    TXDMA_EN                                    = 0x02,///<��������� ������������ �������� DMA ������ FIFO �����������
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

uint32_t km2[20]=
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

uint32_t km1[20]=
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
0,
0,
0,
0,
0,
0,
0,
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
  };


uint32_t resolve(uint32_t val)
{
for(int i=0;i<32;i++)
{
    if((val & 1<<i)==val)
    {
      return i+1;
    }
   /* else
    {
      i++;
      i--;
    }*/
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



void check_km_1(uint8_t num,uint8_t num_cable)
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
          if(kz[x]==km1[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1 || kz[1]==16)
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
{
  state_pin[i]=0x02; //OB
}
if(km1[i]==0)
{
  state_pin[i]=0x04;//NC
}
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
}

void check_km_2(uint8_t num,uint8_t num_cable)
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
          if(kz[x]==km2[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1 || kz[1]==16)
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
{
  state_pin[i]=0x02; //OB
}
if(km2[i]==0)
{
  state_pin[i]=0x04;//NC
}
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
}

void check_km1(uint8_t num,uint8_t num_cable)
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
          if(kz[x]==km1[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1 || kz[1]==16)
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
{
  state_pin[i]=0x02; //OB
}
if(km1[i]==0)
{
  state_pin[i]=0x04;//NC
}
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
////////////////////////////////������� ������
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////������� ������ ���

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
////////////////////////////////������� ������
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////������� ������ ���

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
    if(x<num)
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
lcd oled;
/*
lcd oled;
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
oled.InitializeLCD(); //������������� �������
//oled.ClearLCDScreen();
oled.PrintStr("Start");

*/
extern gpio gpio_stm32f103RC;
Led mcu_led;

std::string menu[10];
menu[0]="����� ������";
menu[1]="����������������";
menu[2]="�����������������";
oled.InitializeLCD();


gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,0);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,1);
//fillScreen(RED);

  //  writeRegister8(ILI9341_DISPLAYON, 0);
//fillScreen(RED);
//fillScreen(GREEN);
//LCD_DrawPixel(10,15, GREEN);
//check_PKU_NKK(20,0x09);
//check_DOF(20,0x12);


//check_eth(8,0x17);


oled.ClearLCDScreen();
oled.busy_flag();
/*
oled.Cursor(1,0);
oled.LCD_String_Cirilic(menu[1]);
oled.busy_flag();
oled.Cursor(2,0);
oled.busy_flag();
oled.LCD_String_Cirilic(menu[2]);
oled.busy_flag();
oled.Cursor(3,0);
oled.busy_flag();
v
oled.busy_flag();
oled.Cursor(4,0);
oled.busy_flag();
oled.LCD_String_Cirilic(menu[2]);
oled.busy_flag();*/

oled.Cursor(0,0);
oled.LCD_String_Cirilic("AAAAAAA");
oled.Cursor(1,2);
oled.LCD_String_Cirilic("BBBBBB");
while(1)
{
//oled.LCD_String_Cirilic("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");



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
lcd oled;
extern gpio gpio_stm32f103RC;
Led mcu_led;







oled.InitializeLCD();


gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,0);
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.red,1);


oled.ClearLCDScreen();
oled.busy_flag();


  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN; //�������� ������������ ���
  ADC1->CR2 |= ADC_CR2_CAL; //������ ���������� ���
  while (!(ADC1->CR2 & ADC_CR2_CAL))
    ; //������� ��������� ����������
  ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0); //������
                                                                                            // ������������ �������
  ADC1->CR2 |= ADC_CR2_JEXTSEL; //�������������� ��������������� ������
                                                       //���������� ���������� ���� JSWSTART
  ADC1->CR2 |= ADC_CR2_JEXTTRIG; //��������� ������� ������ ��������������� ������
  ADC1->CR2 |= ADC_CR2_CONT; //�������������� ����������� ���� �� ������
  ADC1->CR1 |= ADC_CR1_JAUTO; //��������� �������������� ��������������� ������
                                     //����� ����������. �� ������� �����, �� ��� ����� �� ��������
  ADC1->JSQR |= (10<<15); //������ ����� ������ (������ ADC1)
  ADC1->CR2 |= ADC_CR2_ADON;//������ �������� ���
  ADC1->CR2 |= ADC_CR2_JSWSTART; //������ ��������������
  while (!(ADC1->SR & ADC_SR_JEOC)); //���� ���� ������ �������������� ����������
  //������ ����� ������ ��������� �� JDR1
  uint32_t adc_res; //����������� ���������� ��� �������. ����� � ��� ��




//std::vector<string> cables;  //declaring a vector of string


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
