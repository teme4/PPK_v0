#ifndef DMA
#define DMA

#include "stm32f1xx.h"


class dma_usart
{
private:
char _rx_str[255],_tx_str[255];

public:
uint8_t _fl_rx=0, _fl_tx=0;
void DMA1_Init( void);
void usart_tx(uint8_t* dt);
void usart_rx (uint8_t* dt);
//void DMA1_Channel5_IRQHandler(void);
//void DMA1_Channel4_IRQHandler(void);
//void DMA1_Channel4_IRQHandler(void);
//void DMA1_Channel5_IRQHandler(void);
};
#endif

