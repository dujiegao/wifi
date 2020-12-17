#define _DHT11_C_

#include "eagle_soc.h"
#include "../include/driver/dht11.h"
#include "osapi.h"
#include "gpio.h"

/**
* ����:  DHT11�ɼ�Դ�ļ�
* ˵��:  �ɼ�������Ϊ:PB9 ����DHT11.h�ļ����޸�
* ˵��:  �ɼ�����ʪ�����ݴ洢--DHT11Data[4]
* ˵��:  None
* ˵��:  None
* ֧��:  QQ946029359 --Ⱥ 607064330
* �Ա�:  https://shop411638453.taobao.com/
* ����:  С��
**/


u8 DHT11Data[4]={0};//��ʪ������(ʪ�ȸ�λ,ʪ�ȵ�λ,�¶ȸ�λ,�¶ȵ�λ)
u8 GatherRrrorCnt = 0;//�ɼ�����Ĵ���
u8 LastT=0,LastR=0;//��¼��һ�ε���ʪ��

/**
* @brief  DHT11��ʼ�ź�
* @param  
* @param  None
* @param  None
* @retval None
* @example 
**/
void DHT11_Rst(void)	   
{                 
	PIN_FUNC_SELECT(OW_MASTER_MUX, OW_MASTER_FUNC);//��ͨIO
	GPIO_OUTPUT_SET(OW_MASTER_GPIO, 0);
	os_delay_us(10000);
	os_delay_us(10000);
	GPIO_OUTPUT_SET(OW_MASTER_GPIO, 1);
	os_delay_us(30);
}



u8 DHT11_Check(void) 	   
{   
	u8 retry=0;

	GPIO_DIS_OUTPUT(OW_MASTER_GPIO);
	PIN_PULLUP_EN(OW_MASTER_MUX);//��������


	while (DHT11_DQ_IN&&retry<100){
		retry++;
		os_delay_us(1);
	} 
	
	if(retry>=100){
		return 1;
	}
	
	retry=0;
	while (!DHT11_DQ_IN&&retry<100){
		retry++;
		os_delay_us(1);
	}
	
	if(retry>=100){
		return 1;	 
	}   
	return 0;
}



u8 DHT11_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(DHT11_DQ_IN&&retry<100)
	{
		retry++;
		os_delay_us(1);
	}
	retry=0;
	while(!DHT11_DQ_IN&&retry<100)
	{
		retry++;
		os_delay_us(1);
	}
	os_delay_us(35);
	if(DHT11_DQ_IN)return 1;
	else return 0;		   
}

u8 DHT11_Read_Byte(void)    
{
	u8 i,dat;
	dat=0;
	for (i=0;i<8;i++) {
	dat<<=1; 
	dat|=DHT11_Read_Bit();
	}						    
	return dat;
}

void DHT11_Read_Data(void)    
{        
 	u8 buf[5];
	u8 i;
	DHT11_Rst();
	if(DHT11_Check()==0)
	{
		for(i=0;i<5;i++){
			buf[i]=DHT11_Read_Byte();
		}
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4]){
			DHT11Data[0]=buf[0];
			DHT11Data[1]=buf[1];
			DHT11Data[2]=buf[2];
			DHT11Data[3]=buf[3];
			LastT = DHT11Data[2];
			LastR = DHT11Data[0];
		}
		else{
			if(abs(LastT-DHT11Data[2])<3 && abs(LastR-DHT11Data[0])<3)//������Ǻܴ�Ҳ��Ϊ����ȷ������
			{
				DHT11Data[0]=buf[0];
				DHT11Data[1]=buf[1];
				DHT11Data[2]=buf[2];
				DHT11Data[3]=buf[3];
				GatherRrrorCnt = 0;
			}
			else
			{
				GatherRrrorCnt++;
			}
		}
		if(GatherRrrorCnt>5)//����5�βɼ�����
		{
			GatherRrrorCnt = 0;
			DHT11Data[0]=0xff;
			DHT11Data[1]=0xff;
			DHT11Data[2]=0xff;
			DHT11Data[3]=0xff;
		}
	}
}
