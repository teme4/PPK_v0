#include "stm32f1xx.h"
#include "stdio.h"

class usart
{
private:


public:
void uart_tx_byte( uint8_t data);
void uart_tx_bytes(const char * data);
void uart_enter(void);
void usart_init(void);
};

extern "C" void USART1_IRQHandler();
uint16_t USART_RX_TX_Str (uint8_t* rx_dt);
uint8_t gencrc(uint8_t *data, size_t len);
