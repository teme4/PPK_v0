#include <stm32f1xx.h>
#include <dma.hpp>

void dma_usart::DMA1_Init()
{
  //DMA controller clock enable
  RCC->AHBENR|= RCC_AHBENR_DMA1EN;
  //DMA1_Channel4_IRQn interrupt init
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  //DMA1_Channel5_IRQn interrupt init
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);

  //USART1 DMA Init
  //USART1_RX Init
  //Set transfer direction (Peripheral to Memory)
  DMA1_Channel5->CCR&= ~(DMA_CCR_DIR | DMA_CCR_MEM2MEM);
  //DMA1->
  //Set priority level
  DMA1_Channel5->CCR&= ~DMA_CCR_PL;
  //Transfer mode NORMAL
  DMA1_Channel5->CCR&= ~DMA_CCR_CIRC;
  //Set peripheral no increment mode
  DMA1_Channel5->CCR&= ~DMA_CCR_PINC;
  //Set memory increment mode
  DMA1_Channel5->CCR&= ~DMA_CCR_MINC;
  //Set peripheral data width
  DMA1_Channel5->CCR&= ~(DMA_CCR_PSIZE_1 | DMA_CCR_PSIZE_0);
  //Set memory data width
  DMA1_Channel5->CCR&= ~(DMA_CCR_MSIZE_1 | DMA_CCR_MSIZE_0);
  //SPI1_TX Init
  //Set transfer direction (Memory to Peripheral)
  DMA1_Channel4->CCR&=~ DMA_CCR_MEM2MEM;
  DMA1_Channel4->CCR|=DMA_CCR_DIR;
  //Set priority level
  DMA1_Channel4->CCR&= ~DMA_CCR_PL;
  //Transfer mode NORMAL
  //DMA1_Channel4->CCR&= ~DMA_CCR_CIRC;
    DMA1_Channel4->CCR&=~DMA_CCR_CIRC;
 //Set peripheral no increment mode
  DMA1_Channel4->CCR&=~DMA_CCR_PINC;
  //Set memory increment mode
  DMA1_Channel4->CCR|=DMA_CCR_MINC;
  //Set peripheral data width
  DMA1_Channel4->CCR&= ~DMA_CCR_PSIZE_1 | DMA_CCR_PSIZE_0;
  //Set memory data width
  DMA1_Channel4->CCR&= ~DMA_CCR_MSIZE_1 | DMA_CCR_MSIZE_0;
  DMA1_Channel4->CCR&=~ DMA_CCR_EN;
DMA1_Channel5->CCR&=~ DMA_CCR_EN;


  //Clear Channel 4 global interrupt flag
  DMA1->IFCR|= DMA_IFCR_CGIF4;
  //Clear Channel 5 global interrupt flag
  DMA1->IFCR|=DMA_IFCR_CGIF5;
  //Clear Channel 4  transfer complete flag
  DMA1->IFCR|=DMA_IFCR_CTCIF4;
  //Clear Channel 4 transfer error flag
  DMA1->IFCR|=DMA_IFCR_CTCIF5;
  //Clear Channel 5 transfer error flag

  DMA1_Channel4->CCR|= DMA_CCR_TCIE;
  //Enable Channel 4 Transfer error interrupt
  DMA1_Channel4->CCR|= DMA_CCR_TEIE;
  //Enable Channel 5 Transfer complete interrupt
  DMA1_Channel5->CCR|= DMA_CCR_TCIE;
  //Enable Channel 5 Transfer error interrupt
  DMA1_Channel5->CCR|= DMA_CCR_TEIE;

  DMA1_Channel5->CPAR|=   (uint32_t)&(USART1->DR);
  DMA1_Channel5->CMAR|=   (uint32_t)&_rx_str;
  DMA1_Channel4->CPAR|=   (uint32_t)&(USART1->DR);
  DMA1_Channel4->CMAR|= (uint32_t)&_tx_str;
}
 
void dma_usart::usart_tx (uint8_t* dt)
{
  //Disable DMA channel 4
  DMA1_Channel4->CCR&=~ DMA_CCR_EN;
  //Set Number of data to transfer
  uint16_t sz=sizeof(dt);
  DMA1_Channel4->CNDTR|=sz<<DMA_CNDTR_NDT_Pos;
  //Enable DMA channel 4
  DMA1_Channel4->CCR|= DMA_CCR_EN;  
  while (!_fl_tx)
   {

   }
  _fl_tx=0;
}

void dma_usart::usart_rx (uint8_t* dt)
{
  //Disable DMA channel 5
  DMA1_Channel5->CCR&=~ DMA_CCR_EN;
  //Set Number of data to transfer
  uint16_t sz=sizeof(dt);
  DMA1_Channel5->CNDTR|= sz<<DMA_CNDTR_NDT_Pos;//DMA_CNDTR_NDT_Pos
 // MODIFY_REG(DMA1_Channel5->CNDTR, DMA_CNDTR_NDT, sz);
  //Enable DMA channel 5
  DMA1_Channel5->CCR|= DMA_CCR_EN;
  while (!_fl_rx) 
  {

  }
  _fl_rx=0;
}

