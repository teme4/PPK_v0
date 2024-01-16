#ifndef LCD
#define LCD

//#include <stm32f1xx.h>
#include <map>
#include <string>
#include "gpio.hpp"
#include "hardware_config.hpp"
#include "delay.hpp"
#pragma once

extern gpio_lcd_oled gpio_lcds;
extern gpio gpio_stm32f103RC;

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00


class lcd
{
private:
static const std::map<char,uint8_t> _cyrillic;
uint16_t _temp;
public:
void delay(int a);
void PulseLCD(void);
void SendByte(char ByteToSend, int IsData);
void ClearLCDScreen(void);
void busy_flag(void);
void InitializeLCD(void);
void Cursor(char Row, char Col);
void PrintStr(char *Text);
void LCD_String(char* st);
void fake_ClearLCD(void);
void LCD_String_Cyrilic(std::string st);
};

/*
extern void check_SD_SC2(uint8_t num,uint8_t num_cable);
extern void check_DOF(uint8_t num,uint8_t num_cable);
extern void check_eth(uint8_t num,uint8_t num_cable);
extern void check_PKU_NKK_3(uint8_t num,uint8_t num_cable);
extern void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable);
extern void check_ext_fridge(uint8_t num,uint8_t num_cable);
extern void check_km_1(uint8_t num,uint8_t num_cable);
extern void check_km_2(uint8_t num,uint8_t num_cable);
extern void check_UART();
*/
#endif