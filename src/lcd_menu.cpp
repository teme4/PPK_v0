#include "lcd_menu.hpp"

extern lcd oled;
extern Led mcu_led;

extern "C" void TIM1_IRQHandler(void){

	if(TIM1->SR & TIM_SR_TIF){

		/* Toggle LED (PA4 pin) */
	//	GPIOA->ODR ^= GPIO_ODR_4;
    gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);


		/* Interrupt enabled */
		TIM1->SR &= ~TIM_SR_TIF;
	}

}

void Enc_Trig_Int(void){

	/* Trigger Edge Detector */

	/* 0000: No filter, sampling is done at fDTS */
	TIM1->SMCR &= ~(TIM_SMCR_ETF);

	/* 100: TI1 Edge Detector (TI1F_ED) */
	TIM1->SMCR &= ~(TIM_SMCR_TS_0 | TIM_SMCR_TS_1);
	TIM1->SMCR |= TIM_SMCR_TS_2;

	/* 1: Trigger interrupt enabled. */
	TIM1->DIER |= TIM_DIER_TIE;

	NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void tim1_init()
{

RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
/* 01: CC1 channel is configured as input, IC1 is mapped on TI1
 * 01: CC2 channel is configured as input, IC2 is mapped on TI2 */
TIM1->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;
TIM1->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
TIM1->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
TIM1->ARR = 13;
TIM1->CR1 = TIM_CR1_CEN;
/*
TIM1->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);
TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S_1 | TIM_CCMR1_CC2S_1);

	// 00: noninverted/rising edge //
	TIM1->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
	TIM1->CCER &= ~(TIM_CCER_CC2NP | TIM_CCER_CC2NP);

	// 001: Encoder mode 1 - Counter counts up/down on TI2FP1 edge depending on TI1FP2 level //
	TIM1->SMCR |= TIM_SMCR_SMS_0;
	TIM1->SMCR &= ~TIM_SMCR_SMS_1;
	TIM1->SMCR &= ~TIM_SMCR_SMS_2;

	// 1111: fSAMPLING = fDTS / 32, N = 8 //
	TIM1->CCMR1 |= (TIM_CCMR1_IC1F_0 | TIM_CCMR1_IC1F_1 | TIM_CCMR1_IC1F_2 | TIM_CCMR1_IC1F_3);
	TIM1->CCMR1 |= (TIM_CCMR1_IC2F_0 | TIM_CCMR1_IC2F_1 | TIM_CCMR1_IC2F_2 | TIM_CCMR1_IC2F_3);

	// Auto-Reload Register (MAX counter number) //
	TIM1->ARR = 30;

	//Enc_Trig_Int();

	// 1: Counter enabled //
	TIM1->CR1 |= TIM_CR1_CEN;*/
}



uint8_t encoder_check()
{

}


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
uint8_t last;
oled.Cursor(0,0);
oled.LCD_String_Cyrilic("Âûáîð êàáåëÿ:");
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,0);
while(1)
{
gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,1);
//gpio_stm32f103RC.set_pin_state(GPIOC,mcu_led.green,0);*/
/*
switch (TIM1->CNT )
{
case 1:
  //  check_SD_SC2(16,0x01,1);
    break;
case 2:
  //  check_SD_SC2(20,0x02,1);
    break;
case 3:
//check_SD_SC2(8,0x03,1);
    break;
case 4:
    //check_km_1(20,0x04,1);
    break;
case 5:
    //check_km_2(20,0x05,1);
    break;
case 6:
  //  check_SD_SC2(14,0x06,1);
    break;
case 7:
//check_SD_SC2(10,0x07,1);
    break;
case 8:
    //check_PKU_NKK_3(20,0x08,1);
    break;
case 9:
   // check_PKU_NKK_2_1(20,0x09,1);
    break;
case 10:
    // check_DOF(20,0x10,1);
    break;
case 11:
    //check_ext_fridge(16,0x11,1);
    break;
case 12:
   //check_eth(8,0x12,1);
   break;
case 13:
 //  check_SD_SC2(20,0x13,1);
   break;
default:
break;
}*/

oled.Cursor(1,0);
if(last!=TIM1->CNT)
{
oled.PrintStr("                ");
}
last=TIM1->CNT;
oled.LCD_String_Cyrilic(cables_list_ru.at(TIM1->CNT));
delay_ms(500);
oled.Cursor(1,0);
//oled.PrintStr("                ");
}
oled.ClearLCDScreen();
}