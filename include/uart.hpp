#include "stm32f1xx.h"


class usart
{
private:


public:
void uart_tx_byte( uint8_t data);
void uart_tx_bytes(const char * data);
void uart_enter(void);
void usart_init(void);
void USART1_IRQHandler();
};


