#include <stm32f1xx.h>
#include "uart.hpp"
#include <string.h>

uint8_t len=0;
uint8_t RX_data[32];
void check_SD_SC2(uint8_t num,uint8_t num_cable);


/*Trancmited 1 byte*/
void usart::uart_tx_byte(uint8_t data)
{
while (!(USART1->SR & USART_SR_TXE))  {}
USART1->DR = data;
while ((USART1->SR & USART_SR_TC)){}
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
RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
USART1->CR3  = 0;
USART1->CR2 =  0;
USART1->CR1  = 0;
USART1->BRR = 625;//625     256000 //31
USART1->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_RE |USART_CR1_IDLEIE|USART_CR1_TE;
NVIC_EnableIRQ(USART1_IRQn);
__enable_irq();
}

typedef unsigned char uint8_t;
uint8_t gencrc(uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    size_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x07);
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint8_t flag_pream1=0,flag_pream2=0,counter=0,data=0,crc=0;

extern "C" void USART1_IRQHandler()
{
    USART1->SR |= USART_SR_RXNE;
    data = (uint8_t)(USART1->DR);
    RX_data[counter]=data;
    if(RX_data[1]==0x55 && (flag_pream1==1))
    {
        flag_pream2=1;
    }
    else
    {
        if(flag_pream1==1)
        {
        flag_pream1=0;
        flag_pream2=0;
        counter=0;
        RX_data[1]=0;
        }
    }

    if(RX_data[0]==0xAA)
    {
        flag_pream1=1;
    }
   else
    {
        flag_pream1=0;
        counter=0;
        RX_data[0]=0;
    }

    if((flag_pream1==1)&&(flag_pream2==1)&&(counter>=3))
    {
      crc = gencrc(RX_data,3);
     counter++;
if(RX_data[3]==crc)
{
switch (RX_data[2])
{
case 0x01:
    check_SD_SC2(16,0x01);
    counter=0;
    break;
case 0x02:
    check_SD_SC2(20,0x02);
    counter=0;
    break;
case 0x03:

    counter=0;
    break;
case 0x04:

    counter=0;
    break;
case 0x05:

    counter=0;
    break;
case 0x06:
   check_SD_SC2(14,0x06);
    counter=0;
    break;
case 0x07:
    check_SD_SC2(10,0x07);
    counter=0;
    break;
case 0x08:

    counter=0;
    break;
case 0x09:

    counter=0;
    break;
case 0x10:

    counter=0;
    break;
case 0x11:

    counter=0;
    break;
case 0x12:

    counter=0;
    break;
case 0x13:

    counter=0;
    break;
case 0x14:

    counter=0;
    break;
case 0x15:

    counter=0;
    break;
case 0x16:

    counter=0;
    break;

default:
break;
}
}

}
 counter++;
}

