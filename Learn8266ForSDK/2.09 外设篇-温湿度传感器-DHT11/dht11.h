#ifndef __DHT11_H_
#define __DHT11_H_

#ifndef _DHT11_C_
#define _DHT11_C_ extern
#else
#define _DHT11_C_
#endif
  
#include "c_types.h"

#define OW_MASTER_MUX PERIPHS_IO_MUX_GPIO4_U
#define OW_MASTER_GPIO 4
#define OW_MASTER_FUNC FUNC_GPIO4


#define	DHT11_DQ_IN  GPIO_INPUT_GET(OW_MASTER_GPIO)


_DHT11_C_ u8 DHT11Data[4];//��ʪ������(�¶ȸ�λ,�¶ȵ�λ,ʪ�ȸ�λ,ʪ�ȵ�λ)

void DHT11_start(void);
void DHT11_Receive(void);     //����40λ������


void DHT11_Read_Data(void); 
u8 DHT11_Read_Byte(void);//??????
u8 DHT11_Read_Bit(void);//?????
u8 DHT11_Check(void);//??????DHT11
void DHT11_Rst(void);//??DHT11  



#endif

