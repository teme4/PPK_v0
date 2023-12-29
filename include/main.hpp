#ifndef LIST_H_
#define LIST_H_
#include <stm32f1xx.h>

enum class IRQnSPI
{
    TXEIE                                       = (1<<7),///<˜˜˜˜˜˜˜˜˜e ˜˜ ˜˜˜˜˜˜˜˜˜˜˜. ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜ FIFO
    RXNEIE                                      = (1<<6),///<˜˜˜˜˜˜˜˜˜e ˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ (˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜ ˜˜ ˜˜˜˜ ˜ ˜˜ ˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜˜ ˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜)
    ERRIE                                       = (1<<5),///<˜˜˜˜˜˜˜˜˜e ˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜ 50 % ˜ ˜˜˜˜˜ ˜˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜
    NONE                                        = (0<<0)
};

enum class RegCR1
{
    SPI_MODE0                                   = (0b00 ),///<SPI ˜˜˜˜˜ Motorola(CPOL = 0, CPHA = 0);
    SPI_MODE1                                   = (0b01 ),///<SPI ˜˜˜˜˜ Motorola(CPOL = 0, CPHA = 1);
    SPI_MODE2                                   = (0b10 ),///<SPI ˜˜˜˜˜ Motorola(CPOL = 1, CPHA = 0);
    SPI_MODE3                                   = (0b11 ),///<SPI ˜˜˜˜˜ Motorola(CPOL = 1, CPHA = 1);
    MASTER                                      = (1<<2 ),///<˜˜˜˜˜˜˜ ˜˜˜˜˜˜
    SLAVE                                       = (0<<0 ),///<˜˜˜˜˜˜˜ ˜˜˜˜˜˜
    ACTIVE                                      = (1<<6 ),///<˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜
    INACTIVE                                    = (0<<0 ),///<˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜;
    DFF8bit                                     = (0<<0 ),///<data frame format: 8bit;
    DFF16bit                                    = (1<<11),///<data frame format: 16bit;
    LSBF                                        = (1<<7 ),///<LSB  transmitted first
    MSBF                                        = (0<<0 ),///<MSB transmitted first
};
enum class RegDMACR
{
    RXDMA_DIS                                   = 0x00,///<˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ DMA ˜˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜
    RXDMA_EN                                    = 0x01,///<˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ DMA ˜˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜
    TXDMA_DIS                                   = 0x00,///<˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ DMA ˜˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜˜˜
    TXDMA_EN                                    = 0x02,///<˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ DMA ˜˜˜˜˜˜ FIFO ˜˜˜˜˜˜˜˜˜˜˜
    NONE                                        = 0x00
};

void breakpoint(const char * data);
void SettingsSPI(SPI_TypeDef*SPIx ,RegCR1 SPE,
                      RegCR1 MS,
                      double frequency,
                      RegCR1 Type,
                      RegCR1 WordSize,
                      RegCR1 LsbMsbFirst);



#endif