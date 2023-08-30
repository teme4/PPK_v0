#include <stm32f1xx.h>
#include <delay.hpp>
#include <tft.hpp>
#include "registers.hpp"
#include "gpio.hpp"
#include "hardware_config.hpp"

  uint32_t t;
extern gpio stm32f103;


void writeRegister32(uint8_t r, uint32_t d)
 {
  uint8_t temp;
  GPIOA->BSRR= (1<<CS+16);
  GPIOA->BSRR= (1<<RS+16);
  write8(r);
  delay_us(1);
  GPIOA->BSRR= (1<<RS);
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
  GPIOA->BSRR= (1<<CS+16);
  delay_us(1);
}

void write8(uint8_t data)
{
    port_data(data);
    GPIOA->BSRR= (1<<WR+16);
    GPIOA->BSRR= (1<<WR);
}

void writeRegister8(uint8_t a,uint8_t d)
{
  GPIOA->BSRR= (1<<RS+16);
  write8(a);
  delay_us(5);
  GPIOA->BSRR= (1<<RS);
  write8(d);
  delay_us(5);
}

void writeRegister16(uint16_t a, uint16_t d)
 {
  uint8_t hi, lo;
  hi = (a) >> 8;
  lo = (a);
  GPIOA->BRR= (1<<RS);
  write8(hi);
  write8(lo);
  hi = (d) >> 8;
  lo = (d);
  delay_us(5);
  GPIOA->BSRR= (1<<RS);
  write8(hi);
  write8(lo);
  delay_us(5);
  }


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
    GPIOA->BSRR= (1<<RS+16);
    write8(ILI9341_MEMORYWRITE);
    GPIOA->BSRR= (1<<RS);
    write8(color >> 8);
    write8(color & 0xFF);
}

void lcdFillRGB(uint16_t color)
{
  setAddrWindow(0, 0, 240 - 1, 320 - 1);
  int dimensions = 320 * 240;
  GPIOA->BSRR= (1<<RS+16);
  write8(0x2C);
  GPIOA->BSRR= (1<<RS);
  while(dimensions--)
  {
    write8(color >> 8);
    write8(color & 0xFF);
  }
}
/*
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
}*/

void LCD_DrawString(uint32_t x, uint32_t y, char *str)
{
    while(*str) {
       x = LCD_Putchar(x,y,*str++);
      delay_ms(1000);
    }
}

void begin()
{
    reset();
    delay_ms(128);
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
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
     }
     while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
      GPIOA->BSRR= (1<<WR+16);
      GPIOA->BSRR= (1<<WR);
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

void reset()
{
GPIOA->BSRR= (1<<CS);
GPIOA->BSRR= (1<<WR);
GPIOA->BSRR= (1<<RD);
delay_ms(2);
GPIOA->BSRR= (1<<RD);
delay_ms(50);
GPIOA->BSRR= (1<<CS+16);
GPIOA->BSRR= (1<<RS+16);
  write8(ILI9341_SOFTRESET);
  GPIOA->BSRR= (1<<RS);
  write8(0x80);
  write8(0x80);
  for(uint8_t i=0; i<3; i++)
  {
    GPIOA->BSRR= (1<<WR);
    GPIOA->BRR= (1<<WR);
  }
}



/*
uint8_t cmd_temp;
const uint8_t mask[8]={1,2,4,8,16,32,64,128};
GPIO_TypeDef *ports[8]={GPIOB,GPIOB,GPIOB,GPIOB,GPIOC,GPIOC,GPIOA,GPIOA};
uint8_t pins[8]={12,13,14,15,8,9,8,9};
uint8_t pins2[8]={12,13,14,15,8,9,8,9};
*/

void port_data(uint8_t cmd)
{
/*
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
}*/
GPIOA->BSRR =cmd;
}



