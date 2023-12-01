#include "hardware_config.hpp"
#include "gpio.hpp"



gpio gpio_stm32f103RC;
extern gpio stm32f103;
gpio_lcd_oled gpio_lcds;

void gpio_init()
{
//usart
gpio_stm32f103RC.gpio_conf(GPIOA,usart_tx_pin,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,usart_rx_pin,gpio_stm32f103RC.input_mode_floating);

//parallel BUS
 RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
 AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // только SWD без JTAG
 //***************//
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D0,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D1,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D2,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D3,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D4,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D5,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D6,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.D7,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.EN,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.RS,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,gpio_lcds.RW,gpio_stm32f103RC.gpio_mode_pp_50);

gpio_stm32f103RC.gpio_conf(GPIOB,SCK_595,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,MISO_595,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOB,MOSI_595,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,EN_595,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,EN_165,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,CS_595,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,pl_165,gpio_stm32f103RC.gpio_mode_pp_50);

stm32f103.set_pin_state(GPIOB,EN_595,1);
stm32f103.set_pin_state(GPIOB,CS_595,1);
}