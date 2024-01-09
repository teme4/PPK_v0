
#ifndef _PIN_STM32
#define _PIN_STM32
#pragma once

#include <stm32f1xx.h>
#include <vector>
#include <string>
#include "hardware_config.hpp"
#include "uart.hpp"
#include "gpio.hpp"
#include "delay.hpp"

void start_menu(void);
void adc_init(void);
uint8_t adc1_scan();

extern void check_SD_SC2(uint8_t num,uint8_t num_cable);
extern void check_DOF(uint8_t num,uint8_t num_cable);
extern void check_eth(uint8_t num,uint8_t num_cable);
extern void check_PKU_NKK_3(uint8_t num,uint8_t num_cable);
extern void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable);
extern void check_ext_fridge(uint8_t num,uint8_t num_cable);
extern void check_km_1(uint8_t num,uint8_t num_cable);
extern void check_km_2(uint8_t num,uint8_t num_cable);
extern void check_UART();

/*
std::vector<std::string> cables_list
{
"NKK-MLI",
"NKK- PP",
"PP1-PP2",
"PKU-KM(A)",
"PKU-KM(B)",
"PKU-SD/SC",
"NKK-DISP",
"PKU-NKK(UART)",
"PKU-NKK(AOM/BAT)",
"PKU-DOF",
"DOF-fri",
"Ethernet",
"NKK-Oswitch",
};*/

#endif//_PIN_STM32