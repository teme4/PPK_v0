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
//595
#define data_pin						     8
#define clock_pin				             11
#define latcg_pin				             12
#define EN_1      				             2
//eth_in
#define eth1						     0
#define eth2				             1
#define eth3				             2
#define eth4                             7
#define eth5						     6
#define eth6				             5
#define eth7				             4
#define eth8                             3
//eth_out
#define eth1_out						     0
#define eth2_out				             1
#define eth3_out				             2
#define eth4_out                             3
#define eth5_out						     10
#define eth6_out				             5
#define eth7_out				             7
#define eth8_out                             6
//165
#define data_pin						     8
#define clock_pin				             11
#define latcg_pin				             12
#define EN_1      				             2
//138
#define A0						             15
#define A1				                     9
#define A2				                     0
#define clk_165						         13 //Clock inut                 D0 12
#define cs_165				                 12  //parallel load             D1 13
#define data_km   		                     0   //data
#define data_flex                            14
#define data_db     		                 2 //data

#define SCK_165                              5
#define MISO_165     		                 6 //data

void gpio_init(void);
