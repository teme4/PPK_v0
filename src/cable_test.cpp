#include "cable_test.hpp"
#include "uart.hpp"
#include "connectors_pins.hpp"

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

void check_km_1(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
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
          if(kz[x]==km1[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1 || kz[1]==16)
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
for(int i=0;i<20;i++)
{
if(ob[i]==0)
{
  state_pin[i]=0x02; //OB
}
if(km1[i]==0)
{
  state_pin[i]=0x04;//NC
}
}

result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;
//result_buff[4]=0x00;
for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}

void check_km_2(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[12]<<24)|(res[11]<<16)|(res[14]<<8)|res[13];
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
          if(kz[x]==km2[x])
         {
              state_pin[x]=0x00; //OK
              ignore[x]=0x77;
         }
         else
         {
              if(kz[1]==1 || kz[1]==16)
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
for(int i=0;i<20;i++)
{
if(ob[i]==0)
{
  state_pin[i]=0x02; //OB
}
if(km2[i]==0)
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

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
}

void check_DOF(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[2]<<24)|(res[3]<<16)|(res[4]<<8)|res[5];
}
}
  for(int x=0;x<num;x++)
   {
      kz[x]=resolve(ob[x]);
    if(ob[x]==0)
     {
        state_pin[x]=0x02; //OB
        kz[x]=100;
        ignore[x]=0x88;
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j)
                  {
                     state_pin[i]=0x01; //K3
                  }
              }
        }
        }
   }
 

          if(kz[x]==dof_pins_2[x])
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
     state_pin[i]=0x02; //OB
}

 if(state_pin[7]==state_pin[9])
{
state_pin[7]=0x04;
state_pin[9]=0x04;
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
}

void check_PKU_NKK_2_1(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[1];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=res[1];
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

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}


void check_PKU_NKK_3(uint8_t num,uint8_t num_cable)
{
uint8_t count=0,k,q=0;
for(int i=0;i<num;i++)
{
  if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[1];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=res[1];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=res[1];
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

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_eth(uint8_t num,uint8_t num_cable)
{
uint8_t count=0;
for(int i=0;i<num;i++)
{
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=res[6];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=res[6];
}
  for(int x=0;x<num;x++)
   {
     if(ob[x]!=0)
     {
      kz[x]=resolve(ob[x]);
     }
    if(ob[x]==0)
     {
        state_pin[x]=0x02; //OB
        kz[x]=100;
     }
  for(int z=0;z<num;z++)
   {
        if(kz[z]!=0 && kz[z]<32)
        {
        for(int i=0;i<32;i++) //K3
        {
          for(int j=0;j<32;j++)
              {
               if(kz[i]==kz[j] && i!=j)
                  {
                     state_pin[i]=0x01; //K3
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
  state_pin[i]=0x02; //OB
}
result_buff[0]=0xAA;
result_buff[1]=0x55;
result_buff[2]=num_cable;

for(int g=0;g<num+4;g++)
{
  result_buff[3+g]=state_pin[g];
}
result_buff[num+3]=gencrc(result_buff, num+3);

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}
result[21]=0x77;
count++;
}

void check_ext_fridge(uint8_t num,uint8_t num_cable)
{
   uint8_t count=0,k;
////////////////////////////////˜˜˜˜˜˜˜ ˜˜˜˜˜˜
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
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

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}

result[21]=0x77;
}

void check_SD_SC2(uint8_t num,uint8_t num_cable)
{
   uint8_t count=0,k;
////////////////////////////////˜˜˜˜˜˜˜ ˜˜˜˜˜˜
for(int i=0;i<num;i++)
{
k3[i]=0;
ob[i]=0;
result[i]=0x77;
}////////////////////////////////˜˜˜˜˜˜˜ ˜˜˜˜˜˜ ˜˜˜

for(int i=0;i<num;i++)
{
    if(i<16)
  {
    HC74_595_SET(1<<i,0x0000,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(1<<i,0x0000,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
  }
if(i>15)
{
    k=i-16;
    HC74_595_SET(0x0000,1<<k,0);
    flex_cable();
    k3[i]=(res[8]<<16)|(res[9]<<8)|res[10];

    HC74_595_SET(0x0000,1<<k,1);
    flex_cable();
    ob[i]=(res[8]<<16)|(res[9]<<8)|res[10];
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

for(int k=0;k<num+4;k++)
{
usart1.uart_tx_byte(result_buff[k]);
}

result[21]=0x77;
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
