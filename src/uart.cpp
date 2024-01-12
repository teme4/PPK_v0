#include "uart.hpp"

uint8_t len=0;
uint8_t RX_data[32];

lcd oled;
usart usart1;

std::vector<uint16_t> SD_CS
{
    0b000000000011111111111110,//1
    0b000000000011111111111101,//2
    0b000000000011111111111011,//3
    0b000000000011111111110111,//4
    0b000000000011111111101111,//5
    0b000000000011111111011111,//6
    0b000000000011111110111111,//7
    0b000000000011111101111111,//8
    0b000000000011111011111111,//9
    0b000000000011110111111111,//10
    0b000000000011101111111111,//11
    0b000000000011011111111111,//12
    0b000000000010111111111111,//13
    0b000000000001111111111111,//14
    0b000000000011111011111111,//15
    0b000000000011110111111111,//16
    0b000000000011101111111111,//17
    0b000000000011011111111111,//18
    0b000000000010111111111111,//19
    0b000000000001111111111111,//20
};


std::vector<uint16_t>dof
{
25,
19,
26,
5,
11,
6,
12,
0,
10,
0,
20,
22,
21,
23,
14,
24,
15,
17,
16,
13,
};

std::vector<uint16_t> km_2
{
12,
16,
11,
6,
0,
0,
0,
0,
2,
4,
3,
5,
12,
6,
11,
2,
5,
3,
4,
0,
};

std::vector<uint16_t> km_1
{
12,
16,
11,
6,
0,
0,
0,
0,
2,
4,
3,
5,
0,
0,
0,
0,
0,
0,
0,
0,
};


/*Trancmited 1 byte*/
void usart::uart_tx_byte(uint8_t data)
{
while (!(USART1->SR & USART_SR_TXE)){}
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
USART1->BRR = 24;//281//200000     256000 //281
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
    check_flex_cables(SD_CS,16,0x01,0);
    counter=0;
    break;
case 0x02:
    check_flex_cables(SD_CS,20,0x02,0);
    counter=0;
    break;
case 0x03:
    check_flex_cables(SD_CS,8,0x03,0);
    counter=0;
    break;
case 0x04:
    check_univers(km_1,0x04,0);
    counter=0;
    break;
case 0x05:
    check_univers(km_2,0x05,0);
    counter=0;
    break;
case 0x06:
    check_flex_cables(SD_CS,14,0x06,0);
    counter=0;
    break;
case 0x07:
    check_flex_cables(SD_CS,10,0x07,0);
    counter=0;
    break;
case 0x08:
    check_PKU_NKK_3(20,0x08,0);
    counter=0;
    break;
case 0x09:
    check_PKU_NKK_2_1(20,0x09,0);
    counter=0;
    break;
case 0x10:
    check_univers(dof,0x10,0);
    counter=0;
    break;
case 0x11:
    check_ext_fridge(16,0x11,0);
    counter=0;
    break;
case 0x12:
    check_eth(8,0x12,0);
    counter=0;
    break;
case 0x13:
     check_flex_cables(SD_CS,20,0x013,0);
    counter=0;
    break;
case 0x95:
    check_UART();
    counter=0;
    break;

default:
break;
}
}

}
 counter++;
}