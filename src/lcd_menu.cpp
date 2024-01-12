#include "lcd_menu.hpp"

extern lcd oled;
extern Led mcu_led;

void adc_init()
{
  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN; //˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜
  ADC1->CR2 |= ADC_CR2_CAL; //˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜
  while (!(ADC1->CR2 & ADC_CR2_CAL)); //˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜
  ADC1->SMPR2 |= (ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0); //˜˜˜˜˜˜
                                                                                            // ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜
  ADC1->CR2 |= ADC_CR2_JEXTSEL; //˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
                                                       //˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜ JSWSTART
  ADC1->CR2 |= ADC_CR2_JEXTTRIG; //˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
  ADC1->CR2 |= ADC_CR2_CONT; //˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜ ˜˜ ˜˜˜˜˜˜
  ADC1->CR1 |= ADC_CR1_JAUTO; //˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜
                                     //˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜. ˜˜ ˜˜˜˜˜˜˜ ˜˜˜˜˜, ˜˜ ˜˜˜ ˜˜˜˜˜ ˜˜ ˜˜˜˜˜˜˜˜
  ADC1->JSQR |= (10<<15); //˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜ (˜˜˜˜˜˜ ADC1)
  ADC1->CR2 |= ADC_CR2_ADON;//˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜ ˜˜˜
  ADC1->CR2 |= ADC_CR2_JSWSTART; //˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜
  while (!(ADC1->SR & ADC_SR_JEOC));//˜˜˜˜ ˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜
  //˜˜˜˜˜˜ ˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜ JDR1
  uint32_t adc_res; //˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜ ˜˜˜˜˜˜˜. ˜˜˜˜˜ ˜ ˜˜˜ ˜˜
}

std::vector<std::string> cables_list_ru
{
"ÍÊÊ-ÌËÈ",
"ÍÊÊ-ïåðåêëþ÷.",
"ÏÏ1-ÏÏ2",
"ÏÊÓ-ÊÌ(Àëèñà)",
"ÏÊÓ-ÊÌ(Áîá)",
"ÏÊÓ-SD/SC",
"ÍÊÊ-êîìïåíñ.",
"ÏÊÓ-ÍÊÊ(UART)",
"ÏÊÓ-ÍÊÊ(Áàò/ÀÎÌ)",
"ÏÊÓ-ÄÎÔ",
"ÄÎÔ-õîîëîä.",
"Ethernet",
"ÍÊÊ-Îïò.ïåðåêë.",
};


uint8_t adc1_scan()
{
uint32_t adc_res=ADC1->JDR1;
uint8_t sw=0;
if(adc_res>3690 && adc_res<4096)
{
sw=1;
}
if(adc_res>50 && adc_res<150)
{
sw=2;
}
if(adc_res>1900 && adc_res<2100)
{
sw=3;
}
return sw;
}

void start_menu()
{
oled.Cursor(0,0);
oled.LCD_String_Cirilic("Âûáîð êàáåëÿ:");
for (int i=0;i<13;i++)
{
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
if(adc1_scan()==3)
{
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,0);
switch (i)
{
case 0x01:
  //  check_SD_SC2(16,0x01,1);
    break;
case 0x02:
  //  check_SD_SC2(20,0x02,1);
    break;
case 0x03:
//check_SD_SC2(8,0x03,1);
    break;
case 0x04:
    //check_km_1(20,0x04,1);
    break;
case 0x05:
    //check_km_2(20,0x05,1);
    break;
case 0x06:
  //  check_SD_SC2(14,0x06,1);
    break;
case 0x07:
//check_SD_SC2(10,0x07,1);
    break;
case 0x08:
    //check_PKU_NKK_3(20,0x08,1);
    break;
case 0x09:
   // check_PKU_NKK_2_1(20,0x09,1);
    break;
case 0x10:
    // check_DOF(20,0x10,1);
    break;
case 0x11:
    //check_ext_fridge(16,0x11,1);
    break;
case 0x12:
   //check_eth(8,0x12,1);
   break;
case 0x13:
 //  check_SD_SC2(20,0x13,1);
   break;
}
oled.ClearLCDScreen();
return;
}
oled.Cursor(1,0);
oled.LCD_String_Cirilic(cables_list_ru.at(i));
delay_ms(1000);
oled.Cursor(1,0);
oled.PrintStr("                ");
}
oled.ClearLCDScreen();
}