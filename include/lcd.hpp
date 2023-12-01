#include <stm32f1xx.h>
/*void delay_ns(uint32_t ns);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);*/

class lcd
{
private:
uint16_t _temp;
public:
void delay(int a);
void PulseLCD(void);
void SendByte(char ByteToSend, int IsData);
void ClearLCDScreen(void);
//void busy_flag(void);
void InitializeLCD(void);
void Cursor(char Row, char Col);
void PrintStr(char *Text);
void LCD_String(char* st);
void fake_ClearLCD(void);
};