#include "hardware_config.hpp"

gpio gpio_stm32f103RC;
gpio_lcd_oled gpio_lcds;
Led mcu_led;

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

//
gpio_stm32f103RC.gpio_conf(GPIOC,0,gpio_stm32f103RC.input_mode_analog);

gpio_stm32f103RC.gpio_conf(GPIOB,encoder_CLK,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,encoder_DT,gpio_stm32f103RC.alternate_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOB,encoder_SW,gpio_stm32f103RC.input_mode_floating);

gpio_stm32f103RC.set_pin_state(GPIOB,EN_595,1);
gpio_stm32f103RC.set_pin_state(GPIOB,CS_595,1);

//leds
gpio_stm32f103RC.gpio_conf(GPIOC,mcu_led.green,gpio_stm32f103RC.gpio_mode_pp_50);
gpio_stm32f103RC.gpio_conf(GPIOC,mcu_led.red,gpio_stm32f103RC.gpio_mode_pp_50);
}