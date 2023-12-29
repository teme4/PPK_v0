#ifndef CABLE_TEST
#define CABLE_TEST
#include <stm32f1xx.h>
#include "74hc595.hpp"
#include "74hc165d.hpp"
#include "uart.hpp"
#include "connectors_pins.hpp"

extern usart usart1;

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