
#ifndef _PIN_STM32
#define _PIN_STM32
#include <main.hpp>
#include <vector>

void start_menu(void);
void adc_init(void);
uint8_t adc1_scan();


#pragma once
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
};
std::vector<std::string> cables_list_ru
{
"���-���",
"���-��������.",
"��1-��2",
"���-��(�����)",
"���-��(���)",
"���-SD/SC",
"���-�������.",
"���-���(UART)",
"���-���(���/���)",
"���-���",
"���-������.",
"Ethernet",
"���-���.������.",
};

#endif//_PIN_STM32