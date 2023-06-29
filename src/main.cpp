#include <stm32f1xx.h>
#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"


char str[80];
#define TFTWIDTH   320
#define TFTHEIGHT  240
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





uint8_t cmd_temp;
const uint8_t mask[8]={1,2,4,8,16,32,64,128};  
GPIO_TypeDef *ports[8]={GPIOB,GPIOB,GPIOB,GPIOB,GPIOC,GPIOC,GPIOA,GPIOA};
uint8_t pins[8]={12,13,14,15,8,9,8,9};
uint8_t pins2[8]={12,13,14,15,8,9,8,9};
/*
void port_data(uint8_t cmd)
{ 
GPIOB->BSRR = ((1<<(28))|(1<<(29))|(1<<(30))|(1<<(31)));
GPIOC->BSRR = ((1<<(24))|(1<<(25)));
GPIOA->BSRR = ((1<<(24))|(1<<(25)));

for(int i=0;i<8;i++)
{
  cmd_temp=cmd;
  cmd_temp=cmd_temp&mask[i];
  if(cmd_temp==0)
  {
    pins2[i]=pins2[i]+16;   
  }
GPIOB->BSRR = (1<<pins2[0])|(1<<pins2[1]|(1<<pins2[2])|(1<<pins2[3]));  
GPIOC->BSRR = (1<<pins2[4])|(1<<pins2[5]);  
GPIOA->BSRR = (1<<pins2[6])|(1<<pins2[7]);  
}
}*/


void port_data(uint8_t cmd)
{
  
for(int i=0;i<8;i++)
{
  cmd_temp=cmd;
  cmd_temp=cmd_temp&mask[i];
  if(cmd_temp==0)
  {
     ports[i]->BSRR = (1<<(pins[i]+16));
  }
  else
  { 
  ports[i]->BSRR = (1<<pins[i]);
  }
}
}



void write8(uint8_t data)
{                          
    port_data(data);
    GPIOA->BSRR= (1<<31);
    GPIOA->BSRR= (1<<15); 
     //delay_us(10);
     //asm ("NOP"); asm ("NOP");asm ("NOP"); //asm ("NOP");  //asm ("NOP");// asm ("NOP");     
}

void writeRegister8(uint8_t a,uint8_t d) 
{ 
  GPIOA->BSRR= (1<<28);
  write8(a); 
  delay_us(5);
  GPIOA->BSRR= (1<<12); 
  write8(d); 
  delay_us(5);
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
  delay_us(5);
  GPIOA->BSRR= (1<<12);
  write8(hi);
  write8(lo);   
  delay_us(5);
  }


void writeRegister32(uint8_t r, uint32_t d)
 {
  uint8_t temp;
  GPIOA->BSRR= (1<<27);
  GPIOA->BSRR= (1<<28);
  write8(r);  
  delay_us(1);
  GPIOA->BSRR= (1<<12);
  temp=d >> 24;
  write8(temp);
  delay_us(1);
  temp=d >> 16;
  write8(temp); 
  delay_us(1);
  temp=d >> 8;
  write8(temp); 
  delay_us(1);
  write8(d);
  GPIOA->BSRR= (1<<27);
  delay_us(1);
}

void reset()
{
//GPIOA->BRR= (1<<11);
stm32f103.set_pin_state(GPIOC,RD,1);
GPIOA->BSRR= (1<<10);
delay_ms(50);
GPIOA->BSRR= (1<<10);
delay_ms(50);
//GPIOA->BRR= (1<<11);
GPIOA->BRR= (1<<12);
  write8(ILI9341_SOFTRESET);
  GPIOA->BSRR= (1<<12);
  write8(0x80);
  write8(0x80);
  for(uint8_t i=0; i<3; i++)
  {
    GPIOA->BSRR= (1<<15);
    GPIOA->BRR= (1<<15);
  } 

//GPIOA->BRR= (1<<11);
}

  uint32_t t;
void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
 {
 //GPIOA->BRR= (1<<11);
    t = x1;
    t <<= 16;
    t |= x2;
    writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
    delay_us(5);
     t = y1;
    t <<= 16;
    t |= y2;
     writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!
delay_us(5);
//GPIOA->BRR= (1<<11);
}

void begin() 
{
 reset();
 delay_ms(128);
 //port_data(0x00);
  //GPIOA->BRR= (1<<11);
    writeRegister8(ILI9341_SOFTRESET, 0);
    delay_us(100);
    writeRegister8(ILI9341_DISPLAYOFF, 0);
   delay_us(100);
    writeRegister8(ILI9341_POWERCONTROL1, 0x23);
    delay_us(100);
    writeRegister8(ILI9341_POWERCONTROL2, 0x10);
    delay_us(100);
    writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
    delay_us(100);
    writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
    delay_us(100);
    writeRegister8(ILI9341_MEMCONTROL,0x38);//ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR
    delay_us(100);
    //case ROTATE_0:      send_data(0x58);
    //case ROTATE_90:      send_data(0x28);   
    //case ROTATE_180:   send_data(0x88);   
    //case ROTATE_270:   send_data(0xE8);   
    writeRegister8(0x37, 0x00);//0x001B
 writeRegister8(0x53,0x00);
      writeRegister8(ILI9341_GAMMASET,0x01);
 writeRegister8(0x51,0x0);
    writeRegister8(ILI9341_PIXELFORMAT, 0x55);
    delay_us(100);
    writeRegister16(ILI9341_FRAMECONTROL, 0x001B);//0x001B
    delay_us(100);
    writeRegister8(ILI9341_ENTRYMODE, 0x07);
   delay_us(100);
    writeRegister8(ILI9341_SLEEPOUT, 0);
     delay_ms(500);
    writeRegister8(ILI9341_DISPLAYON, 0);
    delay_ms(750);
    setAddrWindow(0, 0, 319, 239);    
 }


void flood(uint16_t color, uint32_t len) 
{
  uint16_t blocks;
  uint8_t  i, hi = color >> 8,
              lo = color;
// GPIOA->BRR= (1<<11);
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
    while(blocks--)
    {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do
       {
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15); 
      GPIOA->BSRR= (1<<31);
      GPIOA->BSRR= (1<<15);
     } 
     while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
    GPIOA->BSRR= (1<<31);
    GPIOA->BSRR= (1<<15); 
    GPIOA->BSRR= (1<<31);
    GPIOA->BSRR= (1<<15);     
    }
  } 
  else
  {
    while(blocks--) {
      i = 16; // 64 pixels/block / 4 pixels/pass
      do
      {
        write8(hi); write8(lo); write8(hi); write8(lo);
        write8(hi); write8(lo); write8(hi); write8(lo);
      } 
      while(--i);
    }
    for(i = (uint8_t)len & 63; i--; ) {
      write8(hi);
      write8(lo);
    }
  }
//GPIOA->BRR= (1<<11);
}

void fillScreen(uint16_t color) 
{
  setAddrWindow(0, 0, TFTWIDTH+61, TFTHEIGHT+16);
  delay_us(1);
   //setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
  //delay_us(1);
  flood(color, TFTWIDTH * TFTHEIGHT);
}







void DrawPixel(uint16_t x, uint16_t y,uint16_t color) 
{
 if((x<0)||(y<0)||(x>=239)||(y>=319))
return;
 setAddrWindow(x, y, x, y);
 writeRegister16(0x2C, color);
}



void LCD_DrawPixel(uint32_t x, uint32_t y, uint32_t color) {
    setAddrWindow(x, y, x, y);
    GPIOA->BSRR= (1<<28);
    write8(ILI9341_MEMORYWRITE); 
    GPIOA->BSRR= (1<<12); 
    write8(color >> 8);
    write8(color & 0xFF);
}





     




void lcdFillRGB(uint16_t color)
{
  setAddrWindow(0, 0, 240 - 1, 320 - 1);
  int dimensions = 320 * 240;
  GPIOA->BSRR= (1<<28);
  write8(0x2C); 
  GPIOA->BSRR= (1<<12); 
  while(dimensions--)
  {
    write8(color >> 8); 
    write8(color & 0xFF); 
  }
}
//void func_hard_fault(void);

extern const font_type TimesNewRoman;

uint32_t LCD_Putchar(uint32_t x, uint32_t y, char c) {
        uint32_t i, j;
        unsigned short Data;        
        uint32_t offset = (c-32)*TimesNewRoman.height;
        uint16_t width = TimesNewRoman.width;        
    	for (i = 0; i < TimesNewRoman.height; i++) {
            Data = TimesNewRoman.data_table[offset+i];     
               for (j = 0; j < width; j++) {
                if ((Data << j) & 0x8000) {
                    LCD_DrawPixel(x + j, (y + i), 0xFFFF);  //white
                    delay_us(10);
                } else {
                    LCD_DrawPixel(x + j, (y + i), 0x0000);  //black
                    delay_us(10);
                }
            }
        }        
        return x+width;
}

void LCD_DrawString(uint32_t x, uint32_t y, char *str)
{
    while(*str) {
       x = LCD_Putchar(x,y,*str++);
      delay_ms(1000);
    }
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

//en_165
//km
/*
stm32f103.set_pin_state(GPIOC,A0,0);
stm32f103.set_pin_state(GPIOB,A1,1);
stm32f103.set_pin_state(GPIOD,A2,0);
*/
//en_165
//db_26
/*
stm32f103.set_pin_state(GPIOC,A0,1);
stm32f103.set_pin_state(GPIOB,A1,1);
stm32f103.set_pin_state(GPIOD,A2,0);
*/
//en_165

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
//stm32f103.set_pin_state(GPIOC,clk_165,0); 

/*

// щелкнули защелкой
stm32f103.set_pin_state(GPIOC,clk_165,1); 
stm32f103.set_pin_state(GPIOC,cs_165,0);
//delay_ms(1); 
stm32f103.set_pin_state(GPIOC,cs_165,1);
// щелкнули защелкой
for(int i=8;i<17;i++)
{
    if((stm32f103.get_state_pin(GPIOB,pin))==1)
    {
  res[i]=i;
    }
    else
    {
    res[i]=0;  
    } 
    stm32f103.set_pin_state(GPIOC,clk_165,0); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,clk_165,1);
}


// щелкнули защелкой
stm32f103.set_pin_state(GPIOC,clk_165,1); 
stm32f103.set_pin_state(GPIOC,cs_165,0);
//delay_ms(1); 
stm32f103.set_pin_state(GPIOC,cs_165,1);
// щелкнули защелкой
for(int i=16;i<25;i++)
{
    if((stm32f103.get_state_pin(GPIOB,pin))==1)
    {
    res[i]=i;
    }
    else
    {
    res[i]=0;  
    } 
    stm32f103.set_pin_state(GPIOC,clk_165,0); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,clk_165,1);
}

// щелкнули защелкой
stm32f103.set_pin_state(GPIOC,clk_165,1); 
stm32f103.set_pin_state(GPIOC,cs_165,0);
//delay_ms(1); 
stm32f103.set_pin_state(GPIOC,cs_165,1);
// щелкнули защелкой
for(int i=24;i<33;i++)
{
    if((stm32f103.get_state_pin(GPIOB,pin))==1)
    {
    res[i]=i;
    }
    else
    {
    res[i]=0;  
    } 
    stm32f103.set_pin_state(GPIOC,clk_165,0); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,clk_165,1);
}*/
return *res;
}
/*
enum class registor_in 
 {
  //1 74HC595
    pin1 =   0x40,                
    pin2=    0x80,                    
    pin3 =   0x10,                  
    pin4 =   0x20,
    pin5 =   0x04,                
    pin6 =   0x08,                    
    pin7 =   0x01,                  
    pin8 =   0x02,   
  //2 74HC595
    pin9 =    0x80,                
    pin10=    0x40,                    
    pin11 =   0x20,                  
    pin12 =   0x10,
    pin13 =   0x08,                
    pin14 =   0x04,                    
    pin15 =   0x02,                  
    pin16 =   0x01,
    //3 74HC595
    pin17 =   0x40,                
    pin18=    0x80,                    
    pin19 =   0x10,                  
    pin20 =   0x20,
    pin21 =   0x04,                
    pin22 =   0x08,                    
    pin23 =   0x02,                  
    pin24 =   0x01,
    //4 74HC595
    pin25 =   0x40,                
    pin26 =   0x80,                    
    pin27 =   0x10,                  
    pin28 =   0x20,
    pin29 =   0x04,                
    pin30 =   0x08,                    
    pin31 =   0x02,                  
    pin32 =   0x01
};*/

 //1 74HC595
  uint8_t  pin1 =   0x40;               
  uint8_t  pin2 =   0x80;                  
  uint8_t  pin3 =   0x10;                  
  uint8_t  pin4 =   0x20;
  uint8_t  pin5 =   0x04;                
  uint8_t  pin6 =   0x08;                   
  uint8_t  pin7 =   0x01;                
  uint8_t  pin8 =   0x02;   
  //2 74HC595
  uint8_t  pin9 =    0x80;               
  uint8_t  pin10=    0x40;                    
  uint8_t  pin11 =   0x20;                 
  uint8_t  pin12 =   0x10;
  uint8_t  pin13 =   0x08;               
  uint8_t   pin14 =  0x04;                    
  uint8_t  pin15 =   0x02;                 
  uint8_t  pin16 =   0x01;
    //3 74HC595
  uint8_t  pin17 =   0x40;               
  uint8_t  pin18=    0x80;                   
  uint8_t  pin19 =   0x10;                  
  uint8_t  pin20 =   0x20;
  uint8_t  pin21 =   0x04;               
  uint8_t  pin22 =   0x08;                    
  uint8_t pin23 =    0x02;                 
  uint8_t  pin24 =   0x01;
    //4 74HC595
  uint8_t  pin25 =   0x40;                
  uint8_t   pin26 =  0x80;                    
  uint8_t  pin27 =   0x10;                  
  uint8_t  pin28 =   0x20;
  uint8_t  pin29 =   0x04;                
  uint8_t  pin30 =   0x08;                    
  uint8_t  pin31 =   0x02;               
  uint8_t  pin32 =   0x01;


uint16_t data_state[32];
void HC74_595(uint16_t data)
{
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
   SPI1->CR2  |= SPI_CR2_SSOE;     
   SPI1->CR1 |= SPI_CR1_SPE;                //Enable SPI2
}

void spi_transmit(uint16_t data)
{
while (!(SPI1->SR & SPI_SR_TXE));  
SPI1->DR = data ;
delay_ms(1);  
}

uint16_t flex_cable(uint8_t val)
{  
if(val==1)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin1);
}
if(val==2)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin2);
}
if(val==3)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin3);
}
if(val==4)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin4);
}
if(val==5)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin5);
}
if(val==6)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin6);
}
if(val==7)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin7);
}
if(val==8)
{
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(pin8);
}
  stm32f103.set_pin_state(GPIOC,A0,1);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,1);

  stm32f103.set_pin_state(GPIOB,pl_165,0);
  delay_us(5);
  stm32f103.set_pin_state(GPIOB,pl_165,1);

  for(int i=0;i<4;i++)
  {
    while(!(SPI1->SR & SPI_SR_TXE)) ;  
    SPI1->DR=0x00;  
    while(!(SPI1->SR & SPI_SR_RXNE)) ;  
    res[i]= SPI1->DR;
  }
  /*                                                                                                                           while(SPI1->SR&SPI_SR_BSY); //Передача завершена
  stm32f103.set_pin_state(GPIOC,A0,0);
  stm32f103.set_pin_state(GPIOB,A1,0);
  stm32f103.set_pin_state(GPIOD,A2,0);*/
  
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


int main()
{
//-------------------------------------------------------
  
/*
RCC->CFGR|=RCC_CFGR_PLLMULL_0;//12
RCC->CFGR|=RCC_CFGR_PLLMULL_1;
RCC->CFGR|=RCC_CFGR_PLLMULL_2;
RCC->CFGR|=RCC_CFGR_PLLMULL_3;*/
/*
RCC->CFGR|=(0x9)<<RCC_CFGR_PLLMULL_Pos;//0xF
//RCC->CFGR|=(0x8)<<RCC_CFGR_HPRE_Pos;
RCC->CFGR|=RCC_CFGR_MCOSEL_PLL_DIV2;
RCC->CFGR&=~RCC_CFGR_SW_0;
RCC->CFGR|=RCC_CFGR_SW_1;
RCC->CR|=RCC_CR_PLLON;*/


//RCC_init();
gpio_init();
dma_usart1.DMA1_Init();
uart1.usart_init();

//AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
//gpio_stm32f103RC.gpio_conf(GPIOA,WR,gpio_stm32f103RC.gpio_mode_pp_10);



breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");
//init_SPI1(SPI1); 
   SettingsSPI(SPI1, 
                      RegCR1::ACTIVE,
                          RegCR1::MASTER,
                          1 /*Mbps*/,
                          RegCR1::SPI_MODE1,//3
                          RegCR1::DFF8bit,
                          RegCR1::MSBF);
uint32_t colors[8]={0x0000,0x1111, 0x2222,0x3333,0x4444,0x5555,0x6666,0x7777};


/*
  
  reset();  
  begin();
  fillScreen(0x0000);
  asm ("nop");*/
//LCD_Putchar(50,50,0x0000);
//LCD_DrawString(100,50,"Hello_world!");

//delay_us(10);
  //LCD_DrawString(0,0,"1234512345123451234512345");

  /*
  for(int i=0;i<320;i++)
  {
    DrawPixel(i,10,WHITE);
      DrawPixel(i,11,WHITE);
        DrawPixel(i,12,WHITE);
          DrawPixel(i,13,WHITE);
          sprintf(str,"%f",i);
          breakpoint(str);
    delay_ms(500);
  }*/


/*
for(int i=0;i<239;i++)
{
  DrawPixel(i+2,i+2,BLACK);
}

 for(int x = 0; x < 320; x++) 
 {
 for(int y = 0; y < 240; y++)
{
// заполнение дисплея чёрным попиксельно
 DrawPixel(y, x, 0x0000);
 }
 }*/
 
 
//test (0, 0, 240, 320,0x0000);


/*
breakpoint("gpio_init!");
breakpoint("usart_init!");
breakpoint("DMA_init!");
*/
/*
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);
  HC74_595(0x00);*/


 uint64_t pinValues;

/*
  HC74_595(0xFF); //byte3
  HC74_595(0xFF); //byte2
  HC74_595(0xFF); //byte1
  HC74_595(0xFF); //byte0*/


while(1)
{




flex_cable(5);
km_cable();





delay_ms(5);


//HC74_165_();

//HC74_165_();
//pinValues = read();
//delay_ms(500);
 // breakpoint("************************************");
/*
 breakpoint("************************************");
 for(int i = 0; i < 8; i++)
 {

        if((pinValues >> i) & 1)
        {
          breakpoint("on");
        }else{
           breakpoint("off");
        }         
 }
  breakpoint("************************************");*/
//
//HC165ReadData();
/*

stm32f103.set_pin_state(GPIOC,PL,0); 
delay_us(3);
stm32f103.set_pin_state(GPIOC,PL,1);
delay_us(3);

stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,1); 
delay_us(1);
stm32f103.set_pin_state(GPIOC,CP,0);  
delay_us(1);

*/

//eth_test();
  /*
lcdFillRGB(0x0000);
//delay_ms(1000);
lcdFillRGB(0xFFFF);
*/

/*
port_data(0xFF);
delay_ms(10);*/




/*
for(int i=0;i<9;i++)
{
GPIOA->BSRR= (1<<27);                
fillScreen(colors[i]);      
GPIOA->BSRR= (1<<11);
delay_ms(300);
}*/
  /*
 SCB_DEMCR |= 0x01000000;
 DWT_CONTROL|= 1; // enable the counter
 DWT_CYCCNT  = 0;

 


//Mcounter=*DWT_CYCCNT2;

  GPIOA->BSRR= (1<<27);                
   writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);   
  GPIOA->BSRR= (1<<11);  
   //asm ("nop");
 


 count=DWT_CYCCNT;
breakpoint("gpio_init!");
*/



//*((int *)(GPIOA_BASE+0x10)) = 0x3000000;//PA8 PA9 =0
//*((int *)(GPIOA_BASE+0x10)) =0x300; //PA8 PA9 =1

//GPIOA->BSRR= (1<<15);
//GPIOA->BRR= (1<<15);
//*((int *)(GPIOA_BASE+0x10)) = 0x80000000;//WR=0
//*((int *)(GPIOA_BASE+0x10)) =0x8000; //WR=1

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
