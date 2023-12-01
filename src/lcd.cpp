#include <stm32f1xx.h>
#include "hardware_config.hpp"
#include "lcd.hpp"
#include "gpio.hpp"
#include "delay.hpp"


/*
#define LAST    ((uint8_t)0x01)
#define    DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define    DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define    SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC

void delay_ns(uint32_t ns)
{
   int32_t ns_count_tick =  ns * (SystemCoreClock/16000000);
   //разрешаем использовать счётчик
   SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   //обнуляем значение счётного регистра
   DWT_CYCCNT  = 0;
   //запускаем счётчик
   DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;
   while(DWT_CYCCNT < ns_count_tick);
   //останавливаем счётчик
   DWT_CONTROL &= ~DWT_CTRL_CYCCNTENA_Msk;
}


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
}*/



//extern gpio gpio_stm32f103RC;
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
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.EN,0);
    delay_us(1);
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.EN,1);
    delay_us(1);
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.EN,0);
    delay_us(1);
} 

//uint32_t temp;
void lcd::SendByte(char ByteToSend, int IsData)
{
    GPIOD->ODR &=~0x3c0;
    GPIOD->ODR  |= ((ByteToSend & 0xF0)<<2);
 
    if (IsData == 1)
        gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,1);
    else
     gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,0);
     PulseLCD();
     GPIOD->ODR &=~0x3c0;
     GPIOD->ODR  |= (((ByteToSend & 0x0F) << 4)<<2);
 
    if (IsData == 1)
          gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,1);
    else
          gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,0); 
    PulseLCD();
}

void lcd:: ClearLCDScreen()
{
    SendByte(0x01, 0);
    SendByte(0x02, 0);
}
/*
void lcd:: busy_flag()
{
    //gpio_stm32f103RC.gpio_conf(GPIOD,gpio_lcds.data_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_input,gpio_speed::gpio_speed_very_high);  
    while(1)
    {
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,0);
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RW,1); 
  //  _temp=gpio_stm32f103RC.get_state_pin(GPIOD,gpio_lcds.data_7); 
    if(_temp==0)
    {
        break;
    }
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,0);
    gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RW,0);
    }   
    gpio_stm32f103RC.gpio_init(GPIOD,gpio_lcds.data_7,gpio_type::gpio_type_pp,gpio_mode::gpio_mode_general,gpio_speed::gpio_speed_very_high);  
}*/

void lcd:: InitializeLCD(void)
{
       GPIOD->ODR &=~0x3C0;
        gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.RS,0);
        gpio_stm32f103RC.set_pin_state(GPIOD,gpio_lcds.EN,0);
        delay_ms(100); // не менее 500 мс и 0х0 5 раз в подряд
        SendByte(0, 0);//1
        delay_ms(20);
        SendByte(0, 0);//2
        delay_ms(5);
        SendByte(0x20, 0);//9
        delay_ms(2);
        SendByte(0x2A, 0);//функциональные установки28
        delay_ms(2);
        SendByte(0x08, 0);//включить дисплей
        delay_ms(2);
        SendByte(0x01, 0);//отчистка экрана
        delay_ms(2);
        SendByte(0x06, 0);//9
        delay_ms(2);
        SendByte(0x0C, 0);//9
        //SendByte(0b00001100, 0); //Курсор выключен
        //SendByte(0b00001110, 0); //Курсор не мигает
        //SendByte(0b00001111, 0); //Курсор мигает
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