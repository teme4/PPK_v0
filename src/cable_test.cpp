#include "cable_test.hpp"


IC_74HC165 IC__74HC165;
IC_74hc595 IC__74hc595;


uint32_t resolve(uint32_t val)
{
for(int i=0;i<32;i++)
{
    if((val & 1<<i)==val)
    {
      return i+1;
    }
}
}

uint32_t find_K3(uint32_t *val)
{
for(int i=0;i<32;i++)
{
  for(int j=0;j<32;j++)
  {
     if(val[i]==val[j] && i!=j)
     {

     }
  }
}
}

uint8_t check_K3(uint8_t value)
{
  uint8_t count=0;
for (int i{0}; i < 8; i ++)
        count += static_cast<bool>(value & (1<<i));
        return count;
}

uint8_t check_num_0(uint8_t value)
{
uint8_t count=0;
for (uint8_t i=0; i<8; i++)
{
count += static_cast<bool>(value & (1<<i));
}
return count;
}

void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable,uint8_t functions)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
     IC__74HC165.Read_pin_state();
    k3[i]=IC__74HC165.res[1];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
     IC__74HC165.Read_pin_state();
    ob[i]=IC__74HC165.res[1];
  }
if(i>15)
{
    k=i-16;
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,0);
     IC__74HC165.Read_pin_state();
    k3[i]=IC__74HC165.res[1];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,1);
     IC__74HC165.Read_pin_state();
    ob[i]=IC__74HC165.res[1];
}
}
 for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j && kz[i]!=0)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
         if(ob[x]==0)
        {
              state_pin[x]=0x02; //OB
        }
         if(kz[x]==pku_nkk_21[x])
         {
              state_pin[x]=0x00; //OK
         }
         if(kz[x]!=pku_nkk_21[x] && ob[x]!=0)
         {
              state_pin[x]=0x03; //HP
         }
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int i=0;i<20;i++)
{
  if(pku_nkk_21[i]==0)
  state_pin[i]=0x04;//NC
}

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);
if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}


void check_PKU_NKK_3(uint8_t num,uint8_t num_cable,uint8_t functions)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
     IC__74HC165.Read_pin_state();
    k3[i]=IC__74HC165.res[1];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
     IC__74HC165.Read_pin_state();
    ob[i]=IC__74HC165.res[1];
  }
if(i>15)
{
    k=i-16;
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,0);
     IC__74HC165.Read_pin_state();
    k3[i]=IC__74HC165.res[1];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,1);
     IC__74HC165.Read_pin_state();
    ob[i]=IC__74HC165.res[1];
}
}
 for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j && kz[i]!=0)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
        if(kz[x]==0)
        {
              state_pin[x]=0x02; //OB
        }
          if(kz[x]==pku_nkk_uart[x])
         {
              state_pin[x]=0x00; //OK
         }
         if(kz[x]!=pku_nkk_uart[x] && kz[x]!=0)
         {
              state_pin[x]=0x03; //HP
         }
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int i=0;i<20;i++)
{
  if(pku_nkk_uart[i]==0)
  state_pin[i]=0x04;//NC
}



for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}

void check_eth(uint8_t num,uint8_t num_cable,uint8_t functions)
{
uint8_t count=0;
for(int i=0;i<num;i++)
{
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
    IC__74HC165.Read_pin_state();
    k3[i]=IC__74HC165.res[6];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
    IC__74HC165.Read_pin_state();
    ob[i]=IC__74HC165.res[6];
}
  for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
    if(ob[x]==0)
     {
        state_pin[x]=0x02;//OB
        kz[x]=100;
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++)//K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j)
                  {
                     state_pin[i]=0x01;//K3
                  }
              }
        }
        }
   }
          if(kz[x]==x+1)
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(ignore[x]!=0x77)
              {
                state_pin[x]=0x03; //OB
                ignore[x]=0x99;
              }
        }


}
for(int i=0;i<32;i++)
{
if(ob[i]==0)
  state_pin[i]=0x02;//OB
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}

void check_ext_fridge(uint8_t num,uint8_t num_cable,uint8_t functions)
{
   uint8_t count=0,k;
//////////////////////////////// 
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////  

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
     IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];
  }
if(i>15)
{
    k=i-16;
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,1);
    IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];
}
}
  for(int x=0;x<num;x++)
   {
     count=0;
  for(int z=0;z<num;z++)
   {
        kz[z]=k3[z] & 1<<x;
        if(kz[z]>0)
        count++;
   }
  if(count==0)
  {
    state_pin[x]=0x02; //OBR
  }
  if(count==num-1)
  {
      if(kz[x]==0)
      state_pin[x]=0x00; //OK
      else
      state_pin[x]=0x03; //OK
  }
  if(count<num-1 && count!=0)
   state_pin[x]=0x01; //K3
   }

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

 state_pin[0]=0x04; //NC
 state_pin[1]=0x04; //NC
 state_pin[12]=0x04; //NC
 state_pin[13]=0x04; //NC
 state_pin[14]=0x04; //NC
 state_pin[15]=0x04; //NC

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}

void check_UART()
{
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=0x95;
result_buff[3]=0x95;
result_buff[4]=0x95;
result_buff[5]=0x95;
result_buff[6]=0x95;
result_buff[7]=0x95;
result_buff[8]=0x95;
result_buff[9]=0x95;
result_buff[10]=0x95;
result_buff[11]=0x95;
result_buff[12]=0x95;
result_buff[13]=0x95;
result_buff[14]=gencrc(result_buff,14);
for(int k=0;k<15;k++)
{
  usart1.uart_tx_byte(result_buff[k]);
}
}


void check_univers(std::vector<uint16_t> vec,uint8_t num_cable,uint8_t functions)
{
uint8_t count=0,k,q=0;
uint8_t num=20;
uint8_t res1,res2,res3,res4;
//Выбор сдвигового регистра для KM1 KM2
if(num_cable==0x04||num_cable==0x05)
{
  res1=12;
  res2=11;
  res3=14;
  res4=13;
}
//Выбор сдвигового регистра для DOF
if(num_cable==0x10)
{
  res1=2;
  res2=3;
  res3=4;
  res4=5;
}
//Выставляем логические 0 и 1.
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[res1]<<24)|(IC__74HC165.res[res2]<<16)|(IC__74HC165.res[res3]<<8)|IC__74HC165.res[res4];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
     IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[res1]<<24)|(IC__74HC165.res[res2]<<16)|(IC__74HC165.res[res3]<<8)|IC__74HC165.res[res4];
  }
if(i>15)
{
    k=i-16;
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[res1]<<24)|(IC__74HC165.res[res2]<<16)|(IC__74HC165.res[res3]<<8)|IC__74HC165.res[res4];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,1);
     IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[res1]<<24)|(IC__74HC165.res[res2]<<16)|(IC__74HC165.res[res3]<<8)|IC__74HC165.res[res4];
}
}
for(int x=0;x<num;x++)
   {
      if(ob[x]!=0)
      kz[x]=resolve(ob[x]);
      if(kz[x]>15)
      kz[x]=kz[x]-16;
    }
count++;
/****************************************************/
for(int x=0;x<num;x++)
   {
         if(kz[x]==vec.at(x))
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(num_cable==0x05)
              {
                  if(kz[1]==1 || kz[1]==16)// ||
                  {
                       ignore[x]=0x77;
                       state_pin[x]=0x00;
                  }
                  if(ignore[x]!=0x77)
                  {
                  state_pin[x]=0x03; //HP
                  ignore[x]=0x99;
                  }
              }
        }
   }
for(int i=0;i<20;i++)
{
if(ob[i]==0)
{
  state_pin[i]=0x02; //OB
}
if(vec.at(i)==0)
{
  state_pin[i]=0x04;//NC
}
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;
for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);
if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}

void check_flex_cables(std::vector<uint16_t> vec,uint8_t num,uint8_t num_cable,uint8_t functions)
{
uint8_t count=0,k=0;

//////////////////////////////// 

for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////  

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,1<<i,0x0000,1);
    IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];
  }
if(i>15)
{
    k=i-16;
    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,0);
     IC__74HC165.Read_pin_state();
    k3[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];

    IC__74hc595.HC74_595_SET(IC__74hc595.gpio_stm32f103,0x0000,1<<k,1);
     IC__74HC165.Read_pin_state();
    ob[i]=(IC__74HC165.res[8]<<16)|(IC__74HC165.res[9]<<8)|IC__74HC165.res[10];
}
}
  for(int x=0;x<num;x++)
   {
     count=0;
  for(int z=0;z<num;z++)
   {
        kz[z]=k3[z] & 1<<x;
        if(kz[z]>0)
        count++;
   }
  if(count==0)
  {
    if(x<num)
    state_pin[x]=0x02; //OBR
      // k3[x]=k3[x] & 1<<x;
  }
  if(count==num-1)
  {
      if(kz[x]==0)
      state_pin[x]=0x00; //OK
      else
      state_pin[x]=0x03; //OK
  }
  if(count<num-1 && count!=0)
   state_pin[x]=0x01; //K3
   }

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

if (num==14)
{
  num=20;
    state_pin[14]=0x04; //NC
    state_pin[15]=0x04; //NC
    state_pin[16]=0x04; //NC
    state_pin[17]=0x04; //NC
    state_pin[18]=0x04; //NC
    state_pin[19]=0x04; //NC
}

if (num==16)
{
  num=20;
    state_pin[16]=0x04; //NC
    state_pin[17]=0x04; //NC
    state_pin[18]=0x04; //NC
    state_pin[19]=0x04; //NC
}

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);
if(functions==0)
{
for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}
}
