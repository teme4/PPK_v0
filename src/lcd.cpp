#include "lcd.hpp"

#define LCD_Goto(x,y) LCD_WriteCom((((((y)& 1)*0x40)+((x)& 7))|128),0)

extern gpio_lcd_oled gpio_lcds;
extern lcd oled;
//extern gpio gpio_stm32f103RC;


const std::map<char,uint8_t> lcd::_cyrillic
{
{'À',65},
{'Á',160},
{'Â',66},
{'Ã',161},
{'Ä',224},
{'Å',69},
{'¨',162},
{'Æ',163},
{'Ç',164},
{'È',165},
{'É',166},
{'Ê',75},
{'Ë',167},
{'Ì',77},
{'Í',72},
{'Î',79},
{'Ï',168},
{'Ð',80},
{'Ñ',67},
{'Ò',84},
{'Ó',169},
{'Ô',170},
{'Õ',192},
{'Ö',225},
{'×',171},
{'Ø',172},
{'Ù',226},
{'Ú',173},
{'Û',174},
{'Ü',196},
{'Û',175},
{'Þ',176},
{'ß',177},

{'à',97},
{'á',178},
{'â',179},
{'ã',180},
{'ä',227},
{'å',101},
{'¸',181},
{'æ',182},
{'ç',183},
{'è',184},
{'é',185},
{'ê',186},
{'ë',187},
{'ì',188},
{'í',189},
{'î',111},
{'ï',190},
{'ð',112},
{'ñ',99},//101
{'ò',191},
{'ó',121},
{'ô',228},
{'õ',192},
{'ö',229},
{'÷',192},
{'ø',193},
{'ù',230},
{'ú',194},
{'û',195},
{'ü',196},
{'ý',197},
{'þ',198},
{'ÿ',199}
};

//---˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜˜, ˜˜ ˜˜˜˜ "˜˜˜˜˜˜˜ ˜˜˜˜˜˜" EN---//
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

        delay_ms(500); // ˜˜ ˜˜˜˜˜ 500 ˜˜ ˜ 0˜0 5 ˜˜˜ ˜ ˜˜˜˜˜˜
        SendByte(0x3A, 0);//˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜
        busy_flag();
        SendByte(0x08, 0);//˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜
        busy_flag();
        SendByte(0x01, 0);//˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
        busy_flag();
        SendByte(0x06, 0);//9
        busy_flag();
        SendByte(0x0C, 0);//9
        busy_flag();
}
  //---˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜---//
void lcd:: Cursor(char Row, char Col)
{
 char address;
   if (Row == 0)
   address = 0;
   else
   address =0x40 | Col;
   SendByte(0x80 | address, 0);
}

//---˜˜˜˜˜˜ ˜˜˜˜˜˜---//
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