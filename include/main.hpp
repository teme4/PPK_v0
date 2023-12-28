#ifndef LIST_H_
#define LIST_H_

#include <stm32f1xx.h>
#include "stdio.h"
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "rcc.hpp"
#include "lcd.hpp"
#include "lcd_menu.hpp"
#include "connectors_pins.hpp"

#include "uart.hpp"
#include "gpio.hpp"
#include "dma.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "math.h"
#include "delay.hpp"
#include <string>
#include <vector>

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



uint8_t result[32]={0x77,},n=0,m=0;
uint32_t k3[32]={0,};
uint8_t pin_map[32][32]={{0},{0}};
uint32_t ob[32]={0,};
uint32_t kz[32]={0,};
uint32_t obr[32]={0,};
uint8_t error[32]={0x77,};
uint8_t result_buff[32]={0x88,};
uint8_t temp_=0;
uint8_t state_pin[32]={0,};
uint8_t ignore[32]={0,};


lcd oled;
gpio gpio_stm32f103RC;
gpio_lcd_oled gpio_lcds;
Led mcu_led;
usart usart1;
dma_usart dma_usart1;


void breakpoint(const char * data);
void SettingsSPI(SPI_TypeDef*SPIx ,RegCR1 SPE,
                      RegCR1 MS,
                      double frequency,
                      RegCR1 Type,
                      RegCR1 WordSize,
                      RegCR1 LsbMsbFirst);

uint32_t resolve(uint32_t val);
uint32_t find_K3(uint32_t *val);
uint8_t check_K3(uint8_t value);
uint8_t check_num_0(uint8_t value);
void check_km_1(uint8_t num,uint8_t num_cable);
void check_km_2(uint8_t num,uint8_t num_cable);
void check_DOF(uint8_t num,uint8_t num_cable);
void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable);
void check_PKU_NKK_3(uint8_t num,uint8_t num_cable);
void check_eth(uint8_t num,uint8_t num_cable);
void check_ext_fridge(uint8_t num,uint8_t num_cable);
void check_SD_SC2(uint8_t num,uint8_t num_cable);
void check_UART();


#endif