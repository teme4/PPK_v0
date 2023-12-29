#include "lcd_menu.hpp"
#include "lcd.hpp"

extern lcd oled;
extern Led mcu_led;

void adc_init()
{
  RCC->APB2ENR|=RCC_APB2ENR_ADC1EN; //˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜˜˜ ˜˜˜
  ADC1->CR2 |= ADC_CR2_CAL; //˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜ ˜˜˜
  while (!(ADC1->CR2 & ADC_CR2_CAL))
    ; //˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜ ˜˜˜˜˜˜˜˜˜˜
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

uint8_t adc1_scan()
{
uint32_t adc_res=ADC1->JDR1;
uint8_t sw=0;

if(adc_res>50 && adc_res<150)
{
sw=2;
}
if(adc_res>1900 && adc_res<2100)
{
sw=3;
}
if(adc_res>3690 && adc_res<4096)
{
sw=1;
}
return sw;
}

void start_menu()
{
oled.Cursor(0,0);
oled.LCD_String_Cirilic("Âûáîð êàáåëÿ:");
for (int i=0;i<16;i++)
{
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
if(adc1_scan()==3)
{
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,0);
switch (i)
{
case 0x01:
    check_SD_SC2(16,0x01);
    break;
case 0x02:
    check_SD_SC2(20,0x02);
    break;
case 0x03:
    check_SD_SC2(8,0x03);
    break;
case 0x04:
    check_km_1(20,0x04);
    break;
case 0x05:
    check_km_2(20,0x05);
    break;
case 0x06:
    check_SD_SC2(14,0x06);
    break;
case 0x07:
    check_SD_SC2(10,0x07);
    break;
case 0x08:
    check_PKU_NKK_3(20,0x08);
    break;
case 0x09:
    check_PKU_NKK_2_1(20,0x09);
    break;
case 0x10:
     check_DOF(20,0x10);
    break;
case 0x11:
    check_ext_fridge(16,0x11);
    break;
case 0x12:
   check_eth(8,0x12);
   break;
case 0x13:
   check_SD_SC2(20,0x13);
   break;
}
}
oled.Cursor(1,0);
oled.LCD_String_Cirilic(cables_list_ru[i]);
delay_ms(500);
oled.Cursor(1,0);
oled.PrintStr("                ");
}
}