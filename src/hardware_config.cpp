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