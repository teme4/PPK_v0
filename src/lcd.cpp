#include <stm32f1xx.h>
#include "hardware_config.hpp"
#include "lcd.hpp"
#include "gpio.hpp"
#include "delay.hpp"

#define LCD_Goto(x,y) LCD_WriteCom((((((y)& 1)*0x40)+((x)& 7))|128),0)

extern gpio_lcd_oled gpio_lcds;
extern gpio gpio_stm32f103RC;
const std::map<char,uint8_t> lcd::_cyrillic
{
{'',65},
{'',160},
{'',66},
{'',161},
{'',224},
{'',69},
{'',162},
{'',163},
{'',164},
{'',165},
{'',166},
{'',75},
{'',167},
{'',77},
{'',72},
{'',79},
{'',168},
{'',80},
{'',67},
{'',84},
{'',169},
{'',170},
{'',192},
{'',225},
{'',171},
{'',172},
{'',226},
{'',173},
{'',174},
{'',196},
{'',175},
{'',176},
{'',177},

{'',97},
{'',178},
{'',179},
{'',180},
{'',227},
{'',101},
{'',181},
{'',182},
{'',183},
{'',184},
{'',185},
{'',186},
{'',187},
{'',188},
{'',189},
{'',111},
{'',190},
{'',112},
{'',101},
{'',191},
{'',121},
{'',228},
{'',192},
{'',229},
{'',192},
{'',193},
{'',230},
{'',194},
{'',195},
{'',196},
{'',197},
{'',198},
{'',199}
};

//---     ,   " " EN---//
void lcd:: PulseLCD()
{
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);
    delay_ms(1);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,1);
    delay_ms(1);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);
    delay_ms(1);
}

//uint32_t temp;
void lcd::SendByte(char ByteToSend, int IsData)
{
    if (IsData == 1)
    {
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,1);
    }
    if (IsData == 0)
    {
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,0);
    }
    GPIOA->ODR &=~0xFF;
    GPIOA->ODR  |= ByteToSend;
     PulseLCD();
     delay_us(40);
}

void lcd:: ClearLCDScreen()
{
    SendByte(0x01, 0);
    SendByte(0x02, 0);
}
void lcd:: busy_flag()
{
    while(1)
    {
    _temp=1;
    gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D7,gpio_stm32f103RC.input_mode_pull_down);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,0);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RW,1);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,1);
    delay_ms(100);
    _temp=gpio_stm32f103RC.get_state_pin(GPIOA,gpio_lcds.D7);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,0);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RW,0);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);
    gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D7,gpio_stm32f103RC.gpio_mode_pp_50);
   if(_temp==0)
   break;
    }
  }

void lcd:: InitializeLCD(void)
{
        GPIOA->ODR &=~0xFF;
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RW,0);
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,0);
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);

        delay_ms(500); //   500   00 5   
        SendByte(0x3A, 0);// 
        busy_flag();
        SendByte(0x08, 0);// 
        busy_flag();
        SendByte(0x01, 0);// 
        busy_flag();
        SendByte(0x06, 0);//9
        busy_flag();
        SendByte(0x0C, 0);//9
        busy_flag();
}
  //---  ---//
void lcd:: Cursor(char Row, char Col)
{ 
 char address;
   if (Row == 0)
   address = 0;
   else
   address =0x40 | Col;
   SendByte(0x80 | address, 0);
}

//--- ---//
void lcd:: PrintStr(char *Text)
{
    char *c;
    c = Text;
    while ((c != 0) && (*c != 0))
    {
        SendByte(*c, 1);
        c++;
    }
}

void lcd:: LCD_String(char* st)
{
  uint8_t i=0;
  while(st[i]!=0)
  {
    SendByte(st[i],1);
    i++;
  }
}

void lcd:: LCD_String_Cirilic(std::string st)
{
  uint8_t i=0;
  for(char &symbol : st)
  {
    if(!_cyrillic.count(symbol))
   {
    SendByte(symbol,1);
   }
   else{
   SendByte(_cyrillic.at(symbol),1);
   }
  
  }
}






/**/
void lcd:: fake_ClearLCD()
{
Cursor(0,0);
PrintStr("                ");
Cursor(1,0);
PrintStr("                ");
}