#include <stm32f1xx.h>
#include "gpio.hpp"
#include "string.h"
#include "hardware_config.hpp"
#include "delay.hpp"

#include "registers.hpp"
#include "tft2.hpp"

unsigned int X_SIZE = 0;
unsigned int Y_SIZE = 0;


extern gpio stm32f103;

void Pulse_CS()
{
    GPIOA->BSRR=(1<<CS+16);//0
    delay_us(1);
    GPIOA->BSRR=(1<<CS);//1
    delay_us(1);
    GPIOA->BSRR=(1<<CS+16);//0
}



void parallel_cmd(uint8_t cmd)
{
uint8_t cmd_temp=0,mask[8]={1,2,4,8,16,32,64,128},state=0;  
GPIO_TypeDef *ports[8]={GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA,GPIOA};
uint8_t gpio[8]={0,1,2,3,4,5,6,7};
for(int i=0;i<8;i++)
{
  cmd_temp=cmd;
  cmd_temp=cmd_temp&mask[i];
  if(cmd_temp==0)
  state=0;
  else
  state=1;
 stm32f103.set_pin_state(ports[i],gpio[i],state);
}
}

void TFT9341_Write8(unsigned char dt)
{
  parallel_cmd(dt);
  stm32f103.set_pin_state(GPIOA,WR,0);
  stm32f103.set_pin_state(GPIOA,WR,1);
}

void TFT_Send_Cmd(uint8_t cmd)
{   
	stm32f103.set_pin_state(GPIOA,RS,0);  //будем слать команду
	stm32f103.set_pin_state(GPIOA,RD,1);	//выставляем на ножке, отвечающей за чтение 1
	stm32f103.set_pin_state(GPIOA,CS,0);   //активируем чип
	stm32f103.set_pin_state(GPIOA,WR,0);
	parallel_cmd(cmd);
	//delay_ms(1);
	stm32f103.set_pin_state(GPIOA,WR,1);
	stm32f103.set_pin_state(GPIOA,CS,1);    //деактивируем чип
}

void TFT_Write_Data(uint8_t data)
{
	stm32f103.set_pin_state(GPIOA,RS,1);	//будем слать ДАННЫЕ
	stm32f103.set_pin_state(GPIOA,RD,1);	//выставляем на ножке, отвечающей за чтение 1
	stm32f103.set_pin_state(GPIOA,CS,0);   //активируем чип
	stm32f103.set_pin_state(GPIOA,WR,0);	//стробируем битом записи
	parallel_cmd(data);
	delay_us(5);
	stm32f103.set_pin_state(GPIOA,WR,1);
	stm32f103.set_pin_state(GPIOA,CS,1); //деактивируем чип
}


/*
uint8_t TFT_Read_Data(void)
{
uint32_t data2;
uint8_t temp1=0,temp2=0,temp3=0,temp4=0;
stm32f103.gpio_conf(GPIOA,D0,stm32f103.input_mode_pull_down); //stm32f103.input_mode_floating
  stm32f103.gpio_conf(GPIOA,D1,stm32f103.input_mode_pull_down);
    stm32f103.gpio_conf(GPIOA,D2,stm32f103.input_mode_pull_down);
      stm32f103.gpio_conf(GPIOA,D3,stm32f103.input_mode_pull_down);
        stm32f103.gpio_conf(GPIOA,D4,stm32f103.input_mode_pull_down);
          stm32f103.gpio_conf(GPIOA,D5,stm32f103.input_mode_pull_down);
            stm32f103.gpio_conf(GPIOA,D6,stm32f103.input_mode_pull_down);
              stm32f103.gpio_conf(GPIOA,D7,stm32f103.input_mode_pull_down);

	stm32f103.set_pin_state(GPIOA,RS,1);	 //будем читать ДАННЫЕ
	stm32f103.set_pin_state(GPIOA,WR,1);	 //выставляем на ножке, отвечающей за запись 1
	stm32f103.set_pin_state(GPIOA,CS,0);   //активируем чип

//delay_ms(5);
   data[0]=stm32f103.get_state_pin(GPIOA,D0);
     data[1]=stm32f103.get_state_pin(GPIOA,D1);
      data[2]=stm32f103.get_state_pin(GPIOA,D2);
       data[3]=stm32f103.get_state_pin(GPIOA,D3);
        data[4]=stm32f103.get_state_pin(GPIOA,D4);
         data[5]=stm32f103.get_state_pin(GPIOA,D5);
          data[6]=stm32f103.get_state_pin(GPIOA,D6);
           data[7]=stm32f103.get_state_pin(GPIOA,D7);
	stm32f103.set_pin_state(GPIOA,RD,0);    //стробируем битом чтения
  delay_ms(1);
  stm32f103.set_pin_state(GPIOA,RD,1);  	

  data[8]=stm32f103.get_state_pin(GPIOA,D0);
     data[9]=stm32f103.get_state_pin(GPIOA,D1);
      data[10]=stm32f103.get_state_pin(GPIOA,D2);
       data[11]=stm32f103.get_state_pin(GPIOA,D3);
        data[12]=stm32f103.get_state_pin(GPIOA,D4);
         data[13]=stm32f103.get_state_pin(GPIOA,D5);
          data[14]=stm32f103.get_state_pin(GPIOA,D6);
           data[15]=stm32f103.get_state_pin(GPIOA,D7);
	stm32f103.set_pin_state(GPIOA,RD,0);    //стробируем битом чтения
   delay_ms(1);
  stm32f103.set_pin_state(GPIOA,RD,1);  	

   data[16]=stm32f103.get_state_pin(GPIOA,D0);
     data[17]=stm32f103.get_state_pin(GPIOA,D1);
      data[18]=stm32f103.get_state_pin(GPIOA,D2);
       data[19]=stm32f103.get_state_pin(GPIOA,D3);
        data[20]=stm32f103.get_state_pin(GPIOA,D4);
         data[21]=stm32f103.get_state_pin(GPIOA,D5);
          data[22]=stm32f103.get_state_pin(GPIOA,D6);
           data[23]=stm32f103.get_state_pin(GPIOA,D7);
	stm32f103.set_pin_state(GPIOA,RD,0);    //стробируем битом чтения
    delay_ms(1);
  stm32f103.set_pin_state(GPIOA,RD,1);
 
     data[24]=stm32f103.get_state_pin(GPIOA,D0);
     data[25]=stm32f103.get_state_pin(GPIOA,D1);
      data[26]=stm32f103.get_state_pin(GPIOA,D2);
       data[27]=stm32f103.get_state_pin(GPIOA,D3);
        data[28]=stm32f103.get_state_pin(GPIOA,D4);
         data[29]=stm32f103.get_state_pin(GPIOA,D5);
          data[30]=stm32f103.get_state_pin(GPIOA,D6);
           data[31]=stm32f103.get_state_pin(GPIOA,D7);
	stm32f103.set_pin_state(GPIOA,RD,0);    //стробируем битом чтения
    delay_ms(1);
  stm32f103.set_pin_state(GPIOA,RD,1);  

	stm32f103.set_pin_state(GPIOA,CS,1);    //деактивируем чип
	// Перенастраиваем порты на выход
stm32f103.gpio_conf(GPIOA,D0,stm32f103.gpio_mode_pp_50);
  stm32f103.gpio_conf(GPIOA,D1,stm32f103.gpio_mode_pp_50);
    stm32f103.gpio_conf(GPIOA,D2,stm32f103.gpio_mode_pp_50);
      stm32f103.gpio_conf(GPIOA,D3,stm32f103.gpio_mode_pp_50);
        stm32f103.gpio_conf(GPIOA,D4,stm32f103.gpio_mode_pp_50);
          stm32f103.gpio_conf(GPIOA,D5,stm32f103.gpio_mode_pp_50);
            stm32f103.gpio_conf(GPIOA,D6,stm32f103.gpio_mode_pp_50);
              stm32f103.gpio_conf(GPIOA,D7,stm32f103.gpio_mode_pp_50);
	//data2=data[0];
  for(int k=0;k<8;k++)
  {
  temp1=data2<<1;
  }

  for(int k=0;k<32;k++)
  {
  data2=data2<<1;
  data2=data2+data[k];
  }
  return data2;
}*/

void Send_cmd(char ByteToSend)
{
  stm32f103.set_pin_state(GPIOA,RS,0);
  stm32f103.set_pin_state(GPIOA,RD,1);
  stm32f103.set_pin_state(GPIOA,CS,0);
    parallel_cmd(ByteToSend);
 // delay_us(5); 
 stm32f103.set_pin_state(GPIOA,WR,0);
  stm32f103.set_pin_state(GPIOA,WR,1);
  stm32f103.set_pin_state(GPIOA,CS,1);          
}
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
void Send_data(char ByteToSend)
{
  stm32f103.set_pin_state(GPIOA,RS,1);
  stm32f103.set_pin_state(GPIOA,RD,1);
  stm32f103.set_pin_state(GPIOA,CS,0);

  parallel_cmd(ByteToSend);
    stm32f103.set_pin_state(GPIOA,WR,0);
  stm32f103.set_pin_state(GPIOA,WR,1);
  stm32f103.set_pin_state(GPIOA,CS,1);       
}




void TFT9341_reset(void)
{
  stm32f103.set_pin_state(GPIOA,CS,1); 
  stm32f103.set_pin_state(GPIOA,WR,1);
  stm32f103.set_pin_state(GPIOA,RD,1);
  delay_us(5);
  delay_us(15);
  delay_us(15);
}

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



void TFT9341_SetRotation(unsigned char r)
{
  Send_cmd(0x36);
  switch(r)
  {
    case 0:
    Send_cmd(0x48);
    X_SIZE = 240;
    Y_SIZE = 320;
    break;
    case 1:
    Send_cmd(0x28);
    X_SIZE = 320;
    Y_SIZE = 240;
    break;
    case 2:
    Send_cmd(0x88);
    X_SIZE = 240;
    Y_SIZE = 320;
    break;
    case 3:
    Send_cmd(0xE8);
    X_SIZE = 320;
    Y_SIZE = 240;
    break;
  }
}



void TFT9341_Flood(uint16_t color,uint32_t len)
{
	uint16_t blocks;
	uint8_t i,hi=color>>8,lo=color;
	Send_cmd(0x2C);
	Send_data(hi);
	delay_us(1);
	Send_cmd(lo);
	len--;
	blocks=(uint16_t)(len/64);//64pixels/blocks
	while(blocks--)
	{
		i=16;
		do
		{
			Send_data(hi);
			Send_data(lo);
			Send_data(hi);
			Send_data(lo);
			Send_data(hi);
			Send_data(lo);
			Send_data(hi);
			Send_data(lo);
			Send_data(hi);
			Send_data(lo);
		} while(--i);
	}
	//Fill any remaining pixels (1 to 64);
	for(i=(uint8_t)len&63;i--;)
	{
		Send_data(hi);
		Send_data(lo);
	}
}

void TFT9341_WriteRegister32(unsigned char r, unsigned long d)
{
  stm32f103.set_pin_state(GPIOA,CS,0); 
   stm32f103.set_pin_state(GPIOA,RS,0); 
  Send_data(r);
  stm32f103.set_pin_state(GPIOA,RS,1); 
  delay_us(1);
  Send_data(d>>24);
  delay_us(1);
  Send_data(d>>16);
  delay_us(1);
  Send_data(d>>8);
  delay_us(1);
  Send_data(d);
  stm32f103.set_pin_state(GPIOA,CS,1); 
}

void TFT9341_SetAddrWindow(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2)
{
  unsigned long t;
  stm32f103.set_pin_state(GPIOA,CS,0); 
  t = x1;
  t<<=16;
  t |= x2;
  TFT9341_WriteRegister32(0x2A,t);//Column Addres Set
  t = y1;
  t<<=16;
  t |= y2;
  TFT9341_WriteRegister32(0x2B,t);//Page Addres Set
   stm32f103.set_pin_state(GPIOA,CS,1); 
}

void TFT9341_FillScreen(unsigned int color)
{
  TFT9341_SetAddrWindow(0,0,X_SIZE-1,Y_SIZE-1);
  delay_ms(1);
  TFT9341_Flood(color,(long)X_SIZE*(long)Y_SIZE);
}



void TFT9341_DrawPixel(int x, int y, unsigned int color)
{
  if((x<0)||(y<0)||(x>=X_SIZE)||(y>=Y_SIZE)) return;
  stm32f103.set_pin_state(GPIOA,CS,0);
  TFT9341_SetAddrWindow(x,y,X_SIZE-1,Y_SIZE-1);
  stm32f103.set_pin_state(GPIOA,CS,0);
  stm32f103.set_pin_state(GPIOA,RS,0);
  Send_data(0x2C);
  stm32f103.set_pin_state(GPIOA,RS,1);
  Send_data(color>>8);Send_data(color);
  stm32f103.set_pin_state(GPIOA,CS,1);
}



uint8_t RD_REG(char Reg,gpio &stm32f103 )
{
  uint8_t data[8];
  // Пишем регистр или команду (строб записи)
  Send_cmd(Reg);
  // Перенастраиваем порт на вход
  stm32f103.gpio_conf(GPIOA,D0,stm32f103.input_mode_floating);
  stm32f103.gpio_conf(GPIOA,D1,stm32f103.input_mode_floating);
    stm32f103.gpio_conf(GPIOA,D2,stm32f103.input_mode_floating);
      stm32f103.gpio_conf(GPIOA,D3,stm32f103.input_mode_floating);
        stm32f103.gpio_conf(GPIOA,D4,stm32f103.input_mode_floating);
          stm32f103.gpio_conf(GPIOA,D5,stm32f103.input_mode_floating);
            stm32f103.gpio_conf(GPIOA,D6,stm32f103.input_mode_floating);
              stm32f103.gpio_conf(GPIOA,D7,stm32f103.input_mode_floating);


 stm32f103.set_pin_state(GPIOA,RS,1);
  stm32f103.set_pin_state(GPIOA,WR,1);
  stm32f103.set_pin_state(GPIOA,CS,0);
    stm32f103.set_pin_state(GPIOA,RD,0);

    data[0]=stm32f103.get_state_pin(GPIOA,D0);
     data[1]=stm32f103.get_state_pin(GPIOA,D1);
      data[2]=stm32f103.get_state_pin(GPIOA,D2);
       data[3]=stm32f103.get_state_pin(GPIOA,D3);
        data[4]=stm32f103.get_state_pin(GPIOA,D4);
         data[5]=stm32f103.get_state_pin(GPIOA,D5);
          data[6]=stm32f103.get_state_pin(GPIOA,D6);
           data[7]=stm32f103.get_state_pin(GPIOA,D7);

   
    stm32f103.set_pin_state(GPIOA,RD,1);
    stm32f103.set_pin_state(GPIOA,CS,1);
   // Перенастраиваем порты на выход
stm32f103.gpio_conf(GPIOA,D0,stm32f103.gpio_mode_pp_50);
  stm32f103.gpio_conf(GPIOA,D1,stm32f103.gpio_mode_pp_50);
    stm32f103.gpio_conf(GPIOA,D2,stm32f103.gpio_mode_pp_50);
      stm32f103.gpio_conf(GPIOA,D3,stm32f103.gpio_mode_pp_50);
        stm32f103.gpio_conf(GPIOA,D4,stm32f103.gpio_mode_pp_50);
          stm32f103.gpio_conf(GPIOA,D5,stm32f103.gpio_mode_pp_50);
            stm32f103.gpio_conf(GPIOA,D6,stm32f103.gpio_mode_pp_50);
              stm32f103.gpio_conf(GPIOA,D7,stm32f103.gpio_mode_pp_50);
  return *data;
}


void TFT1520_InvertDisplay(uint8_t i) 
{
 TFT_Send_Cmd(i ? 0x21 : 0x20);
}

// Формирование команды на порту
void pushData(unsigned char data)
 {
  parallel_cmd(data);
  stm32f103.set_pin_state(GPIOA,WR,0);
  delay_ms(1);
  stm32f103.set_pin_state(GPIOA,WR,1);
 }


void TFT1520_SendCommand(uint8_t index)
 {
 stm32f103.set_pin_state(GPIOA,CS,0);
 stm32f103.set_pin_state(GPIOA,RS,0);
 pushData(index);
 stm32f103.set_pin_state(GPIOA,CS,1);
 }

void TFT1520_SendData(uint8_t data)
 {
 stm32f103.set_pin_state(GPIOA,CS,0);
 stm32f103.set_pin_state(GPIOA,RS,1);
 pushData(data);
 stm32f103.set_pin_state(GPIOA,CS,1);
 } 

void TFT1520_Reset(void) 
{
  stm32f103.set_pin_state(GPIOA,CS,1); 
  stm32f103.set_pin_state(GPIOA,RD,1);
  stm32f103.set_pin_state(GPIOA,WR,1);
  delay_us(15);
} 


void TFT1520_SetRotation(uint8_t r) 
{
 TFT1520_SendCommand(0x36);
 switch(r) {
 case 0:
 TFT1520_SendData(0x48);
 X_SIZE = 240; Y_SIZE = 320;
 break;
 case 1:
 TFT1520_SendData(0x28);
 X_SIZE = 320; Y_SIZE = 240;
 break;
 case 2: 
TFT1520_SendData(0x88);
 X_SIZE = 240; Y_SIZE = 320;
 break;
 case 3:
 TFT1520_SendData(0xE8);
 X_SIZE = 320; Y_SIZE = 240;
 break;
  }
  }


void TFT1520_init()
{
 TFT1520_Reset();
 TFT1520_SendCommand(0x01); 
 delay_us(3000);
 TFT1520_SendCommand(0x28);
 TFT1520_SendCommand(0xB0);
 TFT1520_SendData(0x00);
 TFT1520_SendCommand(0xC0);
 TFT1520_SendData(0x0A);
 TFT1520_SendCommand(0x11); 
 delay_us(2000);
 TFT1520_SendCommand(0x29);
 TFT1520_SendCommand(0x3A);
 TFT1520_SendData(0x55); 
 TFT1520_SetRotation(0); 
 TFT1520_InvertDisplay(0);  
}



void TFT1520_SetAddrWindow(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1) 
{
 TFT1520_SendCommand(0x2A);
 TFT1520_SendData(x >> 8);
 TFT1520_SendData(x);
 TFT1520_SendData(x1 >> 8);
 TFT1520_SendData(x1);
 TFT1520_SendCommand(0x2B);
 TFT1520_SendData(y >> 8);
 TFT1520_SendData(y);
 TFT1520_SendData(y1 >> 8);
 TFT1520_SendData(y1); 
 }
void TFT1520_DrawPixel(uint16_t x, uint16_t y,uint16_t color) 
{
 if((x<0)||(y<0)||(x>=X_SIZE)||(y>=Y_SIZE))
return;
 TFT1520_SetAddrWindow(x, y, x, y);
 TFT1520_SendCommand(0x2C);
 TFT1520_SendData(color >> 8);
 TFT1520_SendData(color & 0xFF);
  }