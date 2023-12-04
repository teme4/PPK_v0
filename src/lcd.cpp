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
{'Я',177}
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
   char address;
   if (Row == 0)
   address = 0;
   else
   address = 0x40;
   address |= Col;
   SendByte(0x80 | address, 0);
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
  for(char &simbol : st)
  {
    SendByte(_cyrillic.at(simbol),1);
  }
}
/**/
void lcd:: fake_ClearLCD()
{
Cursor(0,5);
PrintStr("                ");
Cursor(1,5);
PrintStr("                ");
}