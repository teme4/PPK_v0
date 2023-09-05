#include <stm32f1xx.h>

//volatile uint8_t data[32];



#define BLACK 0x0000
#define BLUE 0x001F
#define RED 0x0F800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF


void Pulse_CS();
void parallel_cmd(uint8_t cmd);
void TFT9341_Write8(unsigned char dt);
void TFT_Send_Cmd(uint8_t cmd);
void TFT_Write_Data(uint8_t data);
void Send_cmd(char ByteToSend);
void Send_data(char ByteToSend);
void TFT9341_reset(void);
void TFT9341_SetRotation(unsigned char r);
void TFT9341_Flood(uint16_t color,uint32_t len);
void TFT9341_WriteRegister32(unsigned char r, unsigned long d);
void TFT9341_SetAddrWindow(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2);
void TFT9341_FillScreen(unsigned int color);
void TFT9341_DrawPixel(int x, int y, unsigned int color);
uint8_t RD_REG(char Reg,gpio &stm32f103 );
void TFT1520_InvertDisplay(uint8_t i);
// Формирование команды на порту
void pushData(unsigned char data);
void TFT1520_SendCommand(uint8_t index);
void TFT1520_SendData(uint8_t data);
void TFT1520_Reset(void);
void TFT1520_SetRotation(uint8_t r);
void TFT1520_init();
void TFT1520_SetAddrWindow(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1);
void TFT1520_DrawPixel(uint16_t x, uint16_t y,uint16_t color);
/*
void TFT9341_SendCommand(unsigned char cmd)
{
  CD_COMMAND;//лапка в состоянии посылки команды//RS
  RD_IDLE;//отключим чтение
  CS_ACTIVE;//выбор дисплея
  DATA_PORT=cmd;
  WR_STROBE;
  CS_IDLE;
}*/

//—————————————————————

/**/


/*
void TFT9341_SendData(unsigned char dt)
{
  CD_DATA;//лапка в состоянии посылки данных RS=1
  RD_IDLE;//отключим чтение rD=1
  CS_ACTIVE;//выбор дисплея
  DATA_PORT=dt;
  WR_STROBE;
  CS_IDLE;
}*/
