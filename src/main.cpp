#include <stm32f1xx.h>
#include "gpio.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "delay.hpp"

#include "registers.hpp"
#include "arial.hpp"
#include "stdio.h"
#include "rcc.hpp"
#include "tft.hpp"
#include "tft2.hpp"


extern gpio gpio_stm32f103RC;

gpio stm32f103;
uint16_t data_state[32];
//volatile char rx_str[32];
char temp[1];

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


void setReadDir()
{
gpio_stm32f103RC.gpio_conf(GPIOA,D0,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D1,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D2,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D3,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D4,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D5,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D6,gpio_stm32f103RC.input_mode_pull_down);
gpio_stm32f103RC.gpio_conf(GPIOA,D7,gpio_stm32f103RC.input_mode_pull_down);
}

void setWriteDir()
{
gpio_stm32f103RC.gpio_conf(GPIOA,D0,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D1,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D2,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D3,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D4,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D5,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D6,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D7,gpio_stm32f103RC.gpio_mode_pp_50);
}

uint16_t read8inline(uint16_t buff)
{
    GPIOA->BSRR= (1<<RD+16);//1
    delay_ms(1);
    buff = GPIOA->IDR & 0xFF;
    GPIOA->BSRR= (1<<RD);
    return buff;
}

uint16_t readID(void)
{
  uint8_t hi, lo;
  GPIOA->BSRR= (1<<CS+16);//0
  GPIOA->BSRR= (1<<RS+16);//cmd
  write8(0x00);
  GPIOA->BSRR= (1<<WR+16);//0
  GPIOA->BSRR= (1<<WR);//0
  setReadDir();  // Set up LCD data port(s) for READ operations
  GPIOA->BSRR= (1<<RS);//data
  read8inline(hi);
  read8inline(lo);
  setWriteDir();  // Restore LCD data port(s) to WRITE configuration
  GPIOA->BSRR= (1<<CS);//0;
  return (hi << 8) | lo;
}

void TFT1520_Reset2(void)
 {
 //CS_IDLE; 
 GPIOA->BSRR= (1<<CS);//0
 //RD_IDLE;
   GPIOA->BSRR= (1<<RD);//0
 //WR_IDLE;
   GPIOA->BSRR= (1<<WR);//0
 //RESET_IDLE;
 delay_ms(5); // Задержка на 5 мкс
 //RESET_ACTIVE; 
 //delay_us(15);
 //RESET_IDLE; 
// delay_us(15);
  } 


int main()
{
gpio_init();
//int k=ClockInit();
TFT1520_Reset2();
TFT1520_init();
//begin();
uint16_t id;


while(1)
{


  id=readID();
  id=0;
  /*port_data(0x00);
  delay_ms(2000);
  port_data(0x55);
  delay_ms(2000);*/

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
