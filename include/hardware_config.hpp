#include <stm32f1xx.h>



//USART1
#define usart_port							       	 GPIOB
#define usart_tx_pin							      	6
#define usart_rx_pin							        7
#define usart_tx_pin_mode					1 //   gpio.alternate_mode_pp_50;
#define usart_rx_pin_mode						1///gpio.input_mode_floating;
//Parallel_BUS
#define D0							       	 12
#define D1							         13
#define D2							         14
#define D3					                 15
#define D4							       	 8
#define D5							         9
#define D6						             8
#define D7				                     9		
#define RD							       	 10
#define WR							         15
#define RS						             12
#define CS				                     11	
#define RST				                     10	


void gpio_init(void);
