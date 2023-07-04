#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"

#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"

char str[80];



usart uart1;
dma_usart dma_usart1;
gpio stm32f103;
char rx_str[255];
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

extern gpio gpio_stm32f103RC;



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

void breakpoint(const char * data)
{
uart1.uart_tx_bytes(data);
uart1.uart_enter();
}


volatile uint8_t res[32]={0,};

uint8_t HC74_165_()
{
stm32f103.set_pin_state(GPIOC,A0,1);
stm32f103.set_pin_state(GPIOB,A1,0);
stm32f103.set_pin_state(GPIOD,A2,1);

stm32f103.set_pin_state(GPIOB,pl_165,0);
//stm32f103.set_pin_state(GPIOB,clk_165,0); 
//stm32f103.set_pin_state(GPIOB,clk_165,1); 
stm32f103.set_pin_state(GPIOB,pl_165,1);
//delay_us(2);
for(int i=0;i<8;i++)
{   
 if((stm32f103.get_state_pin(GPIOB,1))==1)
    {
    res[i]=77;
    }
    else
    {
    res[i]=99;  
    } 
stm32f103.set_pin_state(GPIOB,clk_165,0); 
delay_us(1);
stm32f103.set_pin_state(GPIOB,clk_165,1);
delay_us(1);
   // delay_us(5);

}
return *res;
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
const uint8_t mask[8]={1,2,4,8,16,32,64,128};
uint16_t data_state[32];
void HC74_595(uint16_t data)
{
//data--;
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
    delay_us(1);
  stm32f103.set_pin_state(GPIOC,clock_pin,1);
  delay_us(1);
  stm32f103.set_pin_state(GPIOC,clock_pin,0);
    delay_us(1);
  stm32f103.set_pin_state(GPIOB,data_pin,0);
  delay_us(1);
}
stm32f103.set_pin_state(GPIOC,latcg_pin,1);
stm32f103.set_pin_state(GPIOD,EN_1,0);
}


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



                          // RegDMACR::RXDMA_DIS,
                         // RegDMACR::TXDMA_DIS);

void init_SPI1(SPI_TypeDef*SPIx )
{
   //SPI1->CR1 |= SPI_CR1_BIDIMODE;          // 1 line
   //SPI1->CR1 |= SPI_CR1_BIDIOE;            // MOSI
   SPI1->CR1 |= SPI_CR1_BR;                //Baud rate = Fpclk/4 = 30/4 = 7.5 ???
   SPI1->CR1 |= SPI_CR1_DFF;               //16 ??? ???????
   SPI1->CR1 &= ~SPI_CR1_CPOL;             //Polarity signal CPOL = 0;
   SPI1->CR1 &= ~SPI_CR1_CPHA;             //Phase signal    CPHA = 0;

/*
   SPI1->CR1 |=SPI_CR1_CPOL;             //Polarity signal CPOL = 0;
   SPI1->CR1 |=SPI_CR1_CPHA;             //Phase signal    CPHA = 0;
*/

   SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // ???????? ??? ??? ????? ??????. Nss ????????? ?????? ??? ??????
   SPI1->CR1 |= SPI_CR1_MSTR;              //Mode Master
   SPI1->CR2 |= SPI_CR2_SSOE;
   SPI1->CR1 |= SPI_CR1_SPE;                //Enable SPI2
}

void spi_transmit(uint16_t data)
{
while (!(SPI1->SR & SPI_SR_TXE));
SPI1->DR = data ;
delay_ms(1);
}

uint16_t set_pin_HC74_595(uint8_t val)
{
if((val>=1)&&(val<=8))
{
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
 }
if((val>=9)&&(val<=16))
{
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
}
if((val>=17)&&(val<=24))
{
  HC74_595(0xFF);
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
  HC74_595(0xFF);
}
if((val>=25)&&(val<=32))
{
  HC74_595(pin_HC595[val]);
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
}
}


uint16_t flex_cable(uint8_t val)
{
  set_pin_HC74_595(val);
  stm32f103.set_pin_state(GPIOB,pl_165,0);
  stm32f103.set_pin_state(GPIOB,pl_165,1);

  stm32f103.set_pin_state(GPIOC,A0,1);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,1);

  for(int i=0;i<4;i++)
  {
    while(!(SPI1->SR & SPI_SR_TXE)) ;
    SPI1->DR=0x00;
    while(!(SPI1->SR & SPI_SR_RXNE)) ;
    res[i]= SPI1->DR;
  }
                                                                                                                           while(SPI1->SR&SPI_SR_BSY); //Передача завершена
  stm32f103.set_pin_state(GPIOC,A0,0);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,0);
  return *res;
}

uint16_t km_cable(void)
{
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
  HC74_595(0xFF);
/*
  stm32f103.set_pin_state(GPIOC,A0,0);
  stm32f103.set_pin_state(GPIOB,A1,1);
  stm32f103.set_pin_state(GPIOD,A2,0);
*/

  stm32f103.set_pin_state(GPIOB,pl_165,0);
  delay_us(5);
  stm32f103.set_pin_state(GPIOB,pl_165,1);

  for(int i=0;i<5;i++)
  {
    while(!(SPI1->SR & SPI_SR_TXE)) ;  
    SPI1->DR=0x00;  
    while(!(SPI1->SR & SPI_SR_RXNE)) ;  
    res[4+i]= SPI1->DR;
  }

  stm32f103.set_pin_state(GPIOC,A0,0);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,0);
  return *res;
}

uint16_t check_flex_cable()
{
uint16_t state[32]={0,};
uint16_t cable_map[32][3]={0,};

for(int i=1;i<20+1;i++)
{
cable_map[i][2]=i;
flex_cable(i);

if(i>0&&i<7)
{
cable_map[i][0]=res[0];
cable_map[i][1]=i;
}
if(i>6&&i<15)
{
cable_map[i][0]=res[1];
cable_map[i][1]=i;
}
if(i>14&&i<23)
{
cable_map[i][0]=res[2];
cable_map[i][1]=i;
}
if(i>22&&i<31)
{
cable_map[i][0]=res[3];
cable_map[i][1]=i;
}
if(i==8)
{
cable_map[i][0]=res[2];
cable_map[i][1]=i;
}

if(cable_map[i][0]==pin_flex[i])
{
  state[i]=77;
}
else
 state[i]=cable_map[i][1];
}

return *state;
}



volatile uint8_t buf[33][2]={{0,0},};
void test()
{
for(int k=1;k<33;k++)
{
flex_cable(5);
buf[k][1]=k;
for(int n=1;n<33;n++)
{
if(res[0]==pin_flex[n])
{
buf[n][0]=n;
}
if(res[1]==pin_flex[n])
{
buf[n][0]=n;
}
if(res[2]==pin_flex[n])
{
buf[n][0]=n;
}
if(res[3]==pin_flex[n])
{
buf[n][0]=n;
}
}
}
}



int main()
{
//RCC_init();
gpio_init();
dma_usart1.DMA1_Init();
uart1.usart_init();

//AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
//gpio_stm32f103RC.gpio_conf(GPIOA,WR,gpio_stm32f103RC.gpio_mode_pp_10);



breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");

   SettingsSPI(SPI1, 
                      RegCR1::ACTIVE,
                          RegCR1::MASTER,
                          1 /*Mbps*/,
                          RegCR1::SPI_MODE3,//3
                          RegCR1::DFF8bit,
                          RegCR1::MSBF);
//uint32_t colors[8]={0x0000,0x1111, 0x2222,0x3333,0x4444,0x5555,0x6666,0x7777};

/*
breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");
*/

/*
test();
delay_ms(5);
*/

for(int k=0;k<33;k++)
{
  flex_cable(k);
  buf[k][1]=k;
   if(k>=1&&k<=6)
   {
    //1 register
    if(pin_flex[k]==res[0])
    {
      buf[k][0]=1;
    }
    }

  if(k>=7&&k<=14)
   {
    //2 register
    if(pin_flex[k]==res[1])
    {
      buf[k][0]=1;
    }
    }

    if(k>=15&&k<=22)
   {
    //3 register
    if(pin_flex[k]==res[2])
    {
      buf[k][0]=1;
    }
    }

     if(k>=23&&k<=30)
   {
    //4 register
    if(pin_flex[k]==res[3])
    {
      buf[k][0]=1;
    }
    }

}

  delay_ms(50);
/*
buf[i][0]=i;

if(pin_flex[i]==res[0])
{
buf[i][1]=1;
}
else
{
buf[i][1]=0;
}
}*/


delay_ms(5);



while(1)
{

//check_flex_cable();
//flex_cable(8);

//flex_cable(15);
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