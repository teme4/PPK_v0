#include "rcc.hpp"
//Настраиваем тактирование системы от внешнего кварца
//через PLL на саксимально возможных частотах.
//Внешний кварц должен быть на 8МГц
//Возвращает:
//  0 - завершено успешно
//  1 - не запустился кварцевый генератор
//  2 - не запустился PLL

int ClockInit(void)
{
  __IO int StartUpCounter;
  
  ////////////////////////////////////////////////////////////
  //Запускаем кварцевый генератор
  ////////////////////////////////////////////////////////////
  
  RCC->CR |= (1<<RCC_CR_HSEON_Pos); //Запускаем генератор HSE
  
  //Ждем успешного запуска или окончания тайм-аута
  for(StartUpCounter=0; ; StartUpCounter++)
  {
    //Если успешно запустилось, то 
    //выходим из цикла
    if(RCC->CR & (1<<RCC_CR_HSERDY_Pos))
      break;
    
    //Если не запустилось, то
    //отключаем все, что включили
    //и возвращаем ошибку
    if(StartUpCounter > 0x1000)
    {
      RCC->CR &= ~(1<<RCC_CR_HSEON_Pos); //Останавливаем HSE
      return 1;
    }
  }
  
  ////////////////////////////////////////////////////////////
  //Настраиваем и запускаем PLL
  ////////////////////////////////////////////////////////////
  
  //Настраиваем PLL
  RCC->CFGR |= (0x07<<RCC_CFGR_PLLMULL_Pos) //PLL множитель равен 9
            | (0x01<<RCC_CFGR_PLLSRC_Pos); //Тактирование PLL от HSE
  
  
  RCC->CR |= (1<<RCC_CR_PLLON_Pos); //Запускаем PLL
  
  //Ждем успешного запуска или окончания тайм-аута
  for(StartUpCounter=0; ; StartUpCounter++)
  {
    //Если успешно запустилось, то 
    //выходим из цикла
    if(RCC->CR & (1<<RCC_CR_PLLRDY_Pos))
      break;
    
    //Если по каким-то причинам не запустился PLL, то
    //отключаем все, что включили
    //и возвращаем ошибку
    if(StartUpCounter > 0x1000)
    {
      RCC->CR &= ~(1<<RCC_CR_HSEON_Pos); //Останавливаем HSE
      RCC->CR &= ~(1<<RCC_CR_PLLON_Pos); //Останавливаем PLL
      return 2;
    }
  }
  
  ////////////////////////////////////////////////////////////
  //Настраиваем FLASH и делители
  ////////////////////////////////////////////////////////////
  
  //Устанавливаем 2 цикла ожидания для Flash
  //так как частота ядра у нас будет 48 MHz < SYSCLK <= 72 MHz
  FLASH->ACR |= (0x02<<FLASH_ACR_LATENCY_Pos); 
  
  //Делители
  RCC->CFGR |= (0x00<<RCC_CFGR_PPRE2_Pos) //Делитель шины APB2 отключен
            | (0x04<<RCC_CFGR_PPRE1_Pos) //Делитель нишы APB1 равен 2
            | (0x00<<RCC_CFGR_HPRE_Pos); //Делитель AHB отключен
  
  
  RCC->CFGR |= (0x02<<RCC_CFGR_SW_Pos); //Переключаемся на работу от PLL
  
  //Ждем, пока переключимся
  while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x02<<RCC_CFGR_SWS_Pos))
  {
  }
  
  //После того, как переключились на
  //внешний источник такирования
  //отключаем внутренний RC-генератор
  //для экономии энергии
  RCC->CR &= ~(1<<RCC_CR_HSION_Pos);
  
  //Настройка и переклбючение сисемы
  //на внешний кварцевый генератор
  //и PLL запершилось успехом.
  //Выходим
  return 0;
}

//----------------------------------------------------------

void RCC_init()
{
   BKP->CR &=~ BKP_CR_TPE;                                  //The TAMPER pin is free for general purpose I/O
   BKP->CR |= BKP_CR_TPAL;                                  //0: A high level on the TAMPER pin resets all data backup registers (if TPE bit is set).
   BKP->CSR =0;                                             //������ �������
   PWR->CR |= PWR_CR_DBP;                                   //1: ������� ������ � RTC � ��������� ���������
   RCC->BDCR &=~ RCC_BDCR_BDRST;
   RCC->BDCR &=~RCC_BDCR_RTCEN;                              //0: RTC clock disabled
   RCC->BDCR &=~ RCC_BDCR_LSEON;                            // ��������� LSE
   while (RCC->BDCR & RCC_BDCR_LSERDY){}                    // ���� ���� ��������� ����������

   RCC->CR &=~ RCC_CR_HSEON;                                // ��������� HSE
   RCC->CR |= RCC_CR_HSION;                                 // �������� HSI ���������
   while (!(RCC->CR & RCC_CR_HSIRDY)){}                     // ���� ���� ��������� �� ���������

   RCC->CFGR |= RCC_CFGR_SW_HSI;                            // ������� HSI � �������� ���������� ������������
   RCC->CFGR &=~ RCC_CFGR_PLLSRC;
   RCC->CFGR &=~ RCC_CFGR_PLLMULL_0;                        // �������� ��� ����������� PLL !! PLL=x8 (4��� *8 =32 ���)
   RCC->CFGR|= RCC_CFGR_PLLMULL_1 |RCC_CFGR_PLLMULL_2;

   RCC->CR |= RCC_CR_PLLON;                                 // �������� PLL
   while (!(RCC->CR & RCC_CR_PLLRDY)){}                     // ���� ���� ���������� �� ���������

   RCC->CFGR |= RCC_CFGR_SW_PLL;                            // ������� PLL � �������� ���������� ������������
   RCC->CFGR |= RCC_CFGR_HPRE_DIV1;                         // ABH Prescaler ���������� � ������� �� 1
   RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 Prescaler ���������� � ������� �� 1
   RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 Prescaler ���������� � ������� �� 1
}