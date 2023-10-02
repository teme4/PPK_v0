#include "hardware_config.hpp"
#include "gpio.hpp"

gpio gpio_stm32f103RC;
extern gpio stm32f103;

void gpio_init()
{
//parallel BUS
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
 AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // только SWD без JTAG
 //***************//
gpio_stm32f103RC.gpio_conf(GPIOA,D0,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D1,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D2,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D3,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D4,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D5,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D6,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D7,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,RD,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,CS,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,RS,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,WR,gpio_stm32f103RC.gpio_mode_pp_50);

}