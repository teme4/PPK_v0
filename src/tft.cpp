
#include <stm32f1xx.h>
#include "gpio.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "delay.hpp"

#include "registers.hpp"
#include "tft.hpp"

extern gpio stm32f103;

void WR_IDLE()
{
stm32f103.set_pin_state(GPIOA,WR,1);
}

void RESET_IDLE()
{
stm32f103.set_pin_state(GPIOA,RD,1);
}

void CS_IDLE()
{
stm32f103.set_pin_state(GPIOA,CS,1);
}





void RESET_ACTIVE()
{
stm32f103.set_pin_state(GPIOA,RD,0);
}

void CS_ACTIVE()
{
stm32f103.set_pin_state(GPIOA,CS,0);
}
void CD_DATA()
{
stm32f103.set_pin_state(GPIOA,RS,1);
}
void CD_CMD()
{
stm32f103.set_pin_state(GPIOA,RS,0);
}



void WR_ACTIVE()
{
stm32f103.set_pin_state(GPIOA,WR,0);
}

// write strobe
void WR_STROBE()
  {
  WR_ACTIVE();
  delay_ms(500);
  WR_IDLE();
  }

void port_data(uint8_t cmd)
{
uint8_t cmd_temp=0,mask[8]={1,2,4,8,16,32,64,128},state=0;
uint8_t pins[8]={0,1,2,3,4,5,6,7};

for(int i=0;i<8;i++)
{
  cmd_temp=cmd;
  cmd_temp=cmd_temp&mask[i];
  if(cmd_temp==0)
  {
  state=0;
  }
  else
  {
  state=1;
  }
stm32f103.set_pin_state(GPIOA,pins[i],state);
}
}





void write8(uint8_t data)
{                          
    port_data(data);
    WR_STROBE();
}

void writeRegister8(uint8_t a,uint8_t d) 
{ 
   CD_CMD(); 
   write8(a); 
   CD_DATA() ; 
   write8(d); 
}

void writeRegister16(uint16_t a, uint16_t d)
 { 
  uint8_t hi, lo; 
  hi = (a) >> 8; 
  lo = (a); 
  CD_CMD(); 
  write8(hi); 
  write8(lo); 
  hi = (d) >> 8;
  lo = (d); 
  CD_DATA() ; 
  write8(hi);
  write8(lo); 
  }


void writeRegister32(uint8_t r, uint32_t d)
 {
  uint8_t temp;
  CS_ACTIVE();
  CD_CMD();
  write8(r);
  CD_DATA();
  delay_us(10);
  temp=d >> 24;
  write8(temp);
  delay_us(10);
  temp=d >> 16;
  write8(temp);
  delay_us(10);
  temp=d >> 8;
  write8(temp);
  delay_us(10);
  write8(d);
  CS_IDLE();
}

void reset()
 {
  CS_IDLE();
  WR_IDLE();
  //RD_IDLE();
  RESET_ACTIVE();
    delay_ms(50);
    RESET_IDLE();
  CS_ACTIVE();
  CD_CMD();
  write8(0x80);
    write8(0x80);
  for(uint8_t i=0; i<3; i++)
  {
 WR_STROBE(); // Three extra 0x00s
  }
  CS_IDLE();
}


void setAddrWindow(int x1, int y1, int x2, int y2)
 {
  CS_ACTIVE();
  uint32_t t;
    t = x1;
    t <<= 16;
    t |= x2;
    writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
    t = y1;
    t <<= 16;
    t |= y2;
    writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!
    CS_IDLE();
}

void begin() 
{
 // uint8_t i = 0;
 reset();
 port_data(0x00);
 RESET_IDLE();
  delay_ms(1000);
    CS_ACTIVE();
    writeRegister8(ILI9341_SOFTRESET, 0);
    delay_ms(1);
    writeRegister8(ILI9341_DISPLAYOFF, 0);
    writeRegister8(ILI9341_POWERCONTROL1, 0x23);
    writeRegister8(ILI9341_POWERCONTROL2, 0x10);
    writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
    writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
    writeRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
    writeRegister8(ILI9341_PIXELFORMAT, 0x55);
    writeRegister16(ILI9341_FRAMECONTROL, 0x001B);
    writeRegister8(ILI9341_ENTRYMODE, 0x07);
    writeRegister8(ILI9341_SLEEPOUT, 0);
    delay_ms(150);
    writeRegister8(ILI9341_DISPLAYON, 0);
    delay_ms(500);
    CS_IDLE();
	  setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);    
 }


void flood(uint16_t color, uint32_t len) 
{
  uint16_t blocks;
  uint8_t  i, hi = color >> 8,
              lo = color;
  CS_ACTIVE();
  CD_CMD();  
  write8(0x2C);
  
    // Write first pixel normally, decrement counter by 1
  CD_DATA();
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
        WR_STROBE();  WR_STROBE();  WR_STROBE();  WR_STROBE(); // 2 bytes/pixel
         WR_STROBE();  WR_STROBE();  WR_STROBE();  WR_STROBE(); // x 4 pixels
      } while(--i);
    }
    // Fill any remaining pixels (1 to 64)
    for(i = (uint8_t)len & 63; i--; ) {
       WR_STROBE();
       WR_STROBE();
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
  CS_IDLE();
}


void fillScreen(uint16_t color) 
{
  setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
  flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}
