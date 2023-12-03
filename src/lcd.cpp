#include <stm32f1xx.h>
#include "hardware_config.hpp"
#include "lcd.hpp"
#include "gpio.hpp"
#include "delay.hpp"


extern gpio_lcd_oled gpio_lcds;
extern gpio gpio_stm32f103RC;



/*
//---Функция задержки---//
void lcd:: delay(int a)
{
    int i = 0;
    int f = 0;
    while(f < a)
    {
        while(i<60)
            {i++;}
        f++;
    }
}*/

//---Нужная функция для работы с дисплеем, по сути "дергаем ножкой" EN---//
void lcd:: PulseLCD()
{  
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);
    delay_us(1);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,1);
    delay_us(1);
    gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.EN,0);
    delay_us(1);
} 

//uint32_t temp;
void lcd::SendByte(char ByteToSend, int IsData)
{
    GPIOA->ODR &=~0xFF;
    GPIOA->ODR  |= ByteToSend;
     gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RW,0);
    if (IsData == 1)
    {
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,1);
    }      
    else
    {
        gpio_stm32f103RC.set_pin_state(GPIOA,gpio_lcds.RS,0);
    }
    
     PulseLCD();
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

        delay_ms(50); // не менее 500 мс и 0х0 5 раз в подряд
       /* SendByte(0, 0);//1
        delay_ms(100);
        SendByte(0, 0);//2
        delay_ms(10);
        SendByte(0x0, 0);//3
        delay_ms(10);
        SendByte(0x0, 0);//4
        delay_ms(10);
        SendByte(0x0, 0);//5
        delay_ms(10);*/

        SendByte(0x3F, 0);//функциональные установки28  
        delay_ms(39);
        SendByte(0x3F, 0);//функциональные установки28  
        delay_ms(39);
        SendByte(0x08, 0);//включить дисплей
        delay_ms(39);
        SendByte(0x01, 0);//отчистка экрана 
        delay_ms(39);  
        SendByte(0x06, 0);//mode set   
        delay_ms(39);
        //SendByte(0x0C, 0);//9        
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

void lcd:: fake_ClearLCD()
{
Cursor(0,5);
PrintStr("                ");
Cursor(1,5);
PrintStr("                ");
}