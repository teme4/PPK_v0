#include <stm32f1xx.h>

#define TFTWIDTH   320
#define TFTHEIGHT  240
// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

void setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void fillScreen(uint16_t color) ;
void DrawPixel(uint16_t x, uint16_t y,uint16_t color); 
void LCD_DrawPixel(uint32_t x, uint32_t y, uint32_t color);
void lcdFillRGB(uint16_t color);

uint32_t LCD_Putchar(uint32_t x, uint32_t y, char c);
void LCD_DrawString(uint32_t x, uint32_t y, char *str);
void begin(); 
void flood(uint16_t color, uint32_t len);
void reset();
void writeRegister32(uint8_t r, uint32_t d);
void write8(uint8_t data);
void writeRegister8(uint8_t a,uint8_t d); 
void writeRegister16(uint16_t a, uint16_t d);
void port_data(uint8_t cmd);

  