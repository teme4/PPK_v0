#include <stm32f1xx.h>
#include "uart.hpp"
#include <string.h>



uint8_t len=0;

/*Trancmited 1 byte*/
void usart::uart_tx_byte( uint8_t data)
{
while ((USART1->SR & USART_SR_TXE) == 0)  {}
USART1->DR = data;
} 
/*Trancmited array bytes*/

void usart::uart_tx_bytes(const char * data)
{
len = strlen(data); 
while(len--)
{
uart_tx_byte(*data++);
}
}  




/*
void uart_tx_data(unsigned char * data)
{
len = strlen(data); 
while(len--)
{
uart_tx_byte(*data++);
}
}
*/

/*Enter*/
void usart::uart_enter()
{
while ((USART1->SR & USART_SR_TXE) == 0)  {}
USART1->DR = 0x0D;
while ((USART1->SR & USART_SR_TXE) == 0)  {}
USART1->DR = 0x0A;
} 
/*Init*/
void usart::usart_init()
{
RCC->APB2ENR |= RCC_APB2ENR_USART1EN|RCC_APB2ENR_AFIOEN; 
AFIO->MAPR|=AFIO_MAPR_USART1_REMAP;
USART1->BRR = 70;//625     256000 //31
USART1->CR1 |= USART_CR1_UE| USART_CR1_RE |USART_CR1_TE; 
USART1->CR2&=~ (USART_CR2_LINEN | USART_CR2_CLKEN);
USART1->CR3|=USART_CR3_DMAT|USART_CR3_DMAR;

//NVIC_EnableIRQ(USART1_IRQn);
//__enable_irq();
}

/*
extern "C" void USART1_IRQHandler()
{
RX_data[counter]=USART1->DR;
counter++;
uint8_t data=USART1->DR;
USART1->DR=data;
*data_RX=data;
}*/
