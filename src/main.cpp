#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "registers.hpp"



#define TFTWIDTH   240
#define TFTHEIGHT  320
// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


usart uart1;
dma_usart dma_usart1;
gpio stm32f103;
char rx_str[255];
char temp[1];

#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
 
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
/*
void WR_IDLE()
{
GPIOA->BRR= (1<<15);
}

void RESET_IDLE()
{
//stm32f103.set_pin_state(GPIOA,RST,1);
GPIOA->BSRR= (1<<10);
}

void CS_IDLE()
{
//stm32f103.set_pin_state(GPIOA,CS,1);
GPIOA->BRR= (1<<11);
//delay_us(10);
}

void RD_IDLE()
{
stm32f103.set_pin_state(GPIOC,RD,1);
//GPIOC->BRR= (1<<10);
}



void RESET_ACTIVE()
{
//stm32f103.set_pin_state(GPIOA,RST,0);
GPIOA->BSRR= (1<<10);
}

void CS_ACTIVE()
{
//stm32f103.set_pin_state(GPIOA,CS,0);
GPIOA->BRR= (1<<11);
//delay_us(5000);
}
void CD_DATA()
{
//stm32f103.set_pin_state(GPIOA,RS,1);
GPIOA->BSRR= (1<<12);
}
void CD_CMD()
{
//stm32f103.set_pin_state(GPIOA,RS,0);
GPIOA->BRR= (1<<12);
}

void RD_ACTIVE()
{
//stm32f103.set_pin_state(GPIOC,RD,0);
GPIOC->BSRR= (1<<10);
}

void WR_ACTIVE()
{
//stm32f103.set_pin_state(GPIOA,WR,0);
GPIOA->BSRR= (1<<15);
}*/

// write strobe
/*void WR_STROBE()
  {
   GPIOA->BSRR= (1<<15);
   WR_IDLE();
  }*/

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





uint8_t cmd_temp=0,mask[8]={1,2,4,8,16,32,64,128};  
GPIO_TypeDef *ports[8]={GPIOB,GPIOB,GPIOB,GPIOB,GPIOC,GPIOC,GPIOA,GPIOA};
uint8_t pins[8]={12,13,14,15,8,9,8,9};

void port_data(uint8_t cmd)
{
//GPIOA->BSRR= (1<<3);
  
*((int*)(GPIOB_BASE+0x10)) = 0x10000000;//D0=0   
asm ("nop");
*((int*)(GPIOB_BASE+0x10)) = 0x20000000;//D1=0  
asm ("nop");
*((int*)(GPIOB_BASE+0x10)) = 0x40000000;//D2=0  
asm ("nop");
*((int*)(GPIOB_BASE+0x10)) = 0x80000000;//D3=0  
asm ("nop");
*((int*)(GPIOC_BASE+0x10)) = 0x1000000;//D4=0  
asm ("nop");
*((int*)(GPIOC_BASE+0x10)) = 0x2000000;//D5=0 
asm ("nop");
*((int*)(GPIOA_BASE+0x10)) = 0x1000000;//D6=0  
asm ("nop");
*((int*)(GPIOA_BASE+0x10)) = 0x2000000;//D7=0 
asm ("nop");
for(int i=0;i<8;i++)
{
  cmd_temp=cmd;
  cmd_temp=cmd_temp&mask[i];

switch ( i )
 {
case 0:
if(cmd_temp)
 *((int*)(GPIOB_BASE+0x10)) = 0x1000;//WR=0   
 asm ("nop");
  break;
case 1:
if(cmd_temp)
   *((int*)(GPIOB_BASE+0x10)) = 0x2000;//WR=0 
   asm ("nop");
  break;
case 2:
if(cmd_temp)
      *((int*)(GPIOB_BASE+0x10)) = 0x4000;//WR=0 
   asm ("nop");
  break;
  case 3:
  if(cmd_temp)
   *((int*)(GPIOB_BASE+0x10)) = 0x8000;//WR=0 
   asm ("nop");
  break;
  case 4:
  if(cmd_temp)
    *((int*)(GPIOC_BASE+0x10)) = 0x100;//WR=0 
   asm ("nop");
  break;
  case 5:
  if(cmd_temp)
*((int*)(GPIOC_BASE+0x10)) = 0x200;//D7=0 
  break;
  case 6:
  if(cmd_temp)
*((int*)(GPIOA_BASE+0x10)) = 0x100;//D7=0 
  break;
  case 7:
  if(cmd_temp)
*((int*)(GPIOA_BASE+0x10)) = 0x200;//D7=0 
asm ("nop");
  break;

}
//GPIOA->BSRR= (1<<19);
}
}








void write8(uint8_t data)
{                          
    port_data(data);
    *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
      asm ("NOP");  asm ("NOP");  asm ("NOP");  asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
}

void writeRegister8(uint8_t a,uint8_t d) 
{ 
  GPIOA->BSRR= (1<<28);
   write8(a); 
  GPIOA->BSRR= (1<<12); 
   write8(d); 
}

void writeRegister16(uint16_t a, uint16_t d)
 { 
  uint8_t hi, lo; 
  hi = (a) >> 8; 
  lo = (a); 
GPIOA->BRR= (1<<12);
  write8(hi); 
  write8(lo); 
  hi = (d) >> 8;
  lo = (d); 
GPIOA->BSRR= (1<<12);
  write8(hi);
  write8(lo); 
  }


void writeRegister32(uint8_t r, uint32_t d)
 {
  uint8_t temp;
GPIOA->BSRR= (1<<27);
GPIOA->BSRR= (1<<28);
  write8(r);
GPIOA->BSRR= (1<<12);
 // delay_us(10);
  temp=d >> 24;
  write8(temp);
  //delay_us(10);
  temp=d >> 16;
  write8(temp);
 // delay_us(10);
  temp=d >> 8;
  write8(temp);
 // delay_us(10);
  write8(d);
GPIOA->BSRR= (1<<27);
}

void reset()
 {
GPIOA->BRR= (1<<11);
stm32f103.set_pin_state(GPIOC,RD,1);
GPIOA->BSRR= (1<<10);
   // delay_ms(2);
GPIOA->BSRR= (1<<10);
GPIOA->BRR= (1<<11);
GPIOA->BRR= (1<<12);
  write8(0x00);
  for(uint8_t i=0; i<3; i++)
  {
     GPIOA->BSRR= (1<<15);
    GPIOA->BRR= (1<<15);
 //WR_STROBE(); // Three extra 0x00s
  } 
GPIOA->BRR= (1<<11);
}


void setAddrWindow(int x1, int y1, int x2, int y2)
 {
 GPIOA->BRR= (1<<11);
  uint32_t t;
    t = x1;
    t <<= 16;
    t |= x2;
    writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
    t = y1;
    t <<= 16;
    t |= y2;
    writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!
GPIOA->BRR= (1<<11);
}

void begin() 
{
 // uint8_t i = 0;
 reset();
 port_data(0x00);
 //RESET_IDLE();
  //delay_ms(500);
  GPIOA->BRR= (1<<11);
    writeRegister8(ILI9341_SOFTRESET, 0);
    writeRegister8(ILI9341_DISPLAYOFF, 0);
    writeRegister8(ILI9341_POWERCONTROL1, 0x23);
    writeRegister8(ILI9341_POWERCONTROL2, 0x10);
    writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
    writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
    writeRegister8(ILI9341_MEMCONTROL,0x88 );//ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR
    writeRegister8(ILI9341_PIXELFORMAT, 0x55);
    writeRegister16(ILI9341_FRAMECONTROL, 0x0310);//0x001B
    writeRegister8(ILI9341_ENTRYMODE, 0x07);
    writeRegister8(ILI9341_SLEEPOUT, 0);
    delay_ms(250);
    writeRegister8(ILI9341_DISPLAYON, 0);
    delay_ms(500);
    setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);    
 }


void flood(uint16_t color, uint32_t len) 
{
  uint16_t blocks;
  uint8_t  i, hi = color >> 8,
              lo = color;
 GPIOA->BRR= (1<<11);
 GPIOA->BRR= (1<<12);
  write8(0x2C);
  
    // Write first pixel normally, decrement counter by 1
GPIOA->BSRR= (1<<12);
  write8(hi);
  write8(lo);
  len--;

  blocks = (uint16_t)(len / 64); // 64 pixels/block
  if(hi == lo) {
    // High and low bytes are identical.  Leave prior data
    // on the port(s) and just toggle the write strobe.
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
    *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
      *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
   // asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
      *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
   // asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
  *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     /*   WR_STROBE();  WR_STROBE();  WR_STROBE();  WR_STROBE(); // 2 bytes/pixel
         WR_STROBE();  WR_STROBE();  WR_STROBE();  WR_STROBE(); // x 4 pixels*/
      } while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
   *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
    //asm ("NOP");
      *((int*)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
    //asm ("NOP");
    *((int*)(GPIOA_BASE+0x10)) =0x8000; //WR=1 
     /*  WR_STROBE();
       WR_STROBE();*/
    }
  } else {
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do {
        write8(hi); write8(lo); write8(hi); write8(lo);
        write8(hi); write8(lo); write8(hi); write8(lo);
      } while(--i);
    }
    for(i = (uint8_t)len & 63; i--; ) {
      write8(hi);
      write8(lo);
    }
  }
GPIOA->BRR= (1<<11);
}

void fillScreen(uint16_t color) 
{
  setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}














int main()
{

RCC->CFGR&=~RCC_CFGR_PLLMULL_0;//12
RCC->CFGR|=RCC_CFGR_PLLMULL_1;
RCC->CFGR&=~RCC_CFGR_PLLMULL_2;
RCC->CFGR|=RCC_CFGR_PLLMULL_3;

RCC->CFGR|=RCC_CFGR_MCOSEL_PLL_DIV2;
RCC->CFGR&=~RCC_CFGR_SW_0;
RCC->CFGR|=RCC_CFGR_SW_1;
RCC->CR|=RCC_CR_PLLON;
/*
while(1)
{
uint32_t temp=(RCC->CR & RCC_CR_PLLRDY);
break;
}
*/
gpio_init();
dma_usart1.DMA1_Init();
uart1.usart_init();
SET_BIT(AFIO->MAPR,AFIO_MAPR_SWJ_CFG_JTAGDISABLE);
gpio_stm32f103RC.gpio_conf(GPIOA,WR,gpio_stm32f103RC.gpio_mode_pp_50);
/*
breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");
*/

//uint32_t colors[8]={0x0000,0x001F, 0xF800,0x07E0,  0x07FF,0xF81F,0xFFE0, 0xFFFF};
  
  
  
  
  reset();  
  begin();
 // fillScreen(MAGENTA);



breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");


while(1)
{

GPIOA->BSRR= (1<<27);                
      fillScreen(YELLOW);        
    
GPIOA->BSRR= (1<<11);
 


//*((int *)(GPIOA_BASE+0x10)) = 0x3000000;//PA8 PA9 =0
//*((int *)(GPIOA_BASE+0x10)) =0x300; //PA8 PA9 =1

//GPIOA->BSRR= (1<<15);
//GPIOA->BRR= (1<<15);
//*((int *)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
//*((int *)(GPIOA_BASE+0x10)) =0x8000; //WR=1

}
}
