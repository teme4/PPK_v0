#include <stm32f1xx.h>
#include "hardware_config.hpp"
#include "lcd.hpp"
#include "gpio.hpp"
#include "delay.hpp"


extern gpio_lcd_oled gpio_lcds;
extern gpio gpio_stm32f103RC;
const std::map<char,uint8_t> lcd::_cyrillic
{
{'А',65},
{'Б',160},
{'В',66},
{'Г',161},
{'Д',224},
{'Е',69},
{'Ё',162},
{'Ж',163},
{'З',164},
{'И',165},
{'Й',166},
{'К',75},
{'Л',167},
{'М',77},
{'Н',72},
{'О',79},
{'П',168},
{'Р',80},
{'С',67},
{'Т',84},
{'У',169},
{'Ф',170},
{'Х',192},
{'Ц',225},
{'Ч',171},
{'Ш',172},
{'Щ',226},
{'Ъ',173},
{'Ы',174},
{'Ь',196},
{'Э',175},
{'Ю',176},
{'Я',177},

{'а',97},
{'б',178},
{'в',179},
{'г',180},
{'д',227},
{'е',101},
{'ё',181},
{'ж',182},
{'з',183},
{'и',184},
{'й',185},
{'к',186},
{'л',187},
{'м',188},
{'н',189},
{'о',111},
{'п',190},
{'р',112},
{'с',101},
{'т',191},
{'у',121},
{'ф',228},
{'х',192},
{'ц',229},
{'ч',192},
{'ш',193},
{'щ',230},
{'ъ',194},
{'ы',195},
{'ь',196},
{'э',197},
{'ю',198},
{'я',199}
};

//---Нужная функция для работы с дисплеем, по сути "дергаем ножкой" EN---//
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

        delay_ms(500); // не менее 500 мс и 0х0 5 раз в подряд
        SendByte(0x3E, 0);//функциональные установки
        busy_flag();
        SendByte(0x08, 0);//включить дисплей
        busy_flag();
        SendByte(0x01, 0);//отчистка экрана
        busy_flag();
        SendByte(0x06, 0);//9
        busy_flag();
        SendByte(0x0C, 0);//9
        busy_flag();
}
  //---Установка позиции курсора---//
void lcd:: Cursor(char Row, char Col)
{
    switch (Row)
    {
    case 1:
        SendByte(0x80 | (0x00 + Col), 0);      //0-16
        break;
    case 2:
        SendByte(0x80 | (0x40 + Col), 0); //17-32
        break;
    
      case 3:
        SendByte(0x80 | (0x14 + Col), 0); //20-39
        break;
    case 4:
        SendByte(0x80 | (0x54 + Col), 0);
        break;
}
}

//---Печать строки---//
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