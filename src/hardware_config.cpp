#include "hardware_config.hpp"
#include "gpio.hpp"

gpio gpio_stm32f103RC;
extern gpio stm32f103;

void gpio_init()
{
//usart
gpio_stm32f103RC.gpio_conf(GPIOA,usart_tx_pin,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,usart_rx_pin,gpio_stm32f103RC.input_mode_floating);
//parallel BUS
/*
RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
gpio_stm32f103RC.gpio_conf(GPIOB,D0,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,D1,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,D2,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,D3,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,D4,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,D5,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D6,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,D7,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,RD,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,RST,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,CS,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,RS,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,WR,gpio_stm32f103RC.gpio_mode_pp_50);
*/

/*
gpio_stm32f103RC.gpio_conf(GPIOB,data_pin,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,clock_pin,gpio_stm32f103RC.gpio_mode_pp_50);

gpio_stm32f103RC.gpio_conf(GPIOC,latcg_pin,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOD,EN_1,gpio_stm32f103RC.gpio_mode_pp_50);
*/




/*
gpio_stm32f103RC.gpio_conf(GPIOB,data_db,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOB,data_km,gpio_stm32f103RC.input_mode_floating);
AFIO->MAPR|= AFIO_MAPR_PD01_REMAP; //A2 remap
*/
/*RCC->APB1ENR |= RCC_APB1ENR_BKPEN|RCC_APB1ENR_PWREN;
gpio_stm32f103RC.gpio_conf(GPIOB,clk_165,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,cs_165,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,data_flex,gpio_stm32f103RC.input_mode_floating);

gpio_stm32f103RC.set_pin_state(GPIOB,cs_165,1);
gpio_stm32f103RC.set_pin_state(GPIOB,clk_165,0);*/
/*
gpio_stm32f103RC.gpio_conf(GPIOA,SCK_165,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOA,MISO_165,gpio_stm32f103RC.input_mode_floating);
gpio_stm32f103RC.gpio_conf(GPIOA,MOSI_165,gpio_stm32f103RC.alternate_mode_pp_50);
*/

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