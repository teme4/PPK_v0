#include <stm32f1xx.h>

//USART1
#define usart_tx_pin						  	9
#define usart_rx_pin						    10

//Parallel_BUS
/*
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
*/
//595
#define data_pin						     8
#define clock_pin				             11
#define latcg_pin				             12
#define EN_1      				             2

//165
#define data_pin						     8
#define clock_pin				             11
#define CS_595				                 7
#define EN_595     				             12
#define EN_165    				             11
#define pl_165				                 10

//138
#define clk_165						         13 //Clock inut                 D0 12
#define data_km   		                     0   //data
#define data_flex                            14
#define data_db     		                 2 //data

#define SCK_595                              13
#define MISO_595   		                     14
#define MOSI_595    		                 15

void gpio_init(void);
