/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include "esp_common.h"
#include "gpio.h"
#include "esp_timer.h"
#include "hw_timer.h"
#include "uart.h"
#include "pwm.h"


extern char Usart0ReadBuff[Usart0ReadLen];//�������ݻ���
extern u32  Usart0ReadCnt;//���ڽ��յ����ݸ���
extern u32  Usart0ReadCntCopy;//���ڿ������ڽ��յ����ݸ���
extern u32  Usart0IdleCnt;//����ʱ���ۼӱ���

int i;

os_timer_t os_timer_one;//����һ����ʱ���ṹ�����
int LightValue = 1;

                       //���żĴ�����ַ                                  ����ֵ(��ͨIO)  �������
uint32 io_info[1][3] = {PERIPHS_IO_MUX_GPIO2_U, FUNC_GPIO2, 2};
int duty[1]={1023/1000*0};//�ߵ�ƽʱ��Լ��0us



/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

/**
* @brief   ��ʱ���ص�����
* @param   parg:���������os_timer_setfn�������Ĳ���
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example
**/
void os_timer_one_function(void *parg)
{
	pwm_set_duty(duty[0],0);//����0ͨ����PWMʱ��
	pwm_start();//����PWM
	//���LightValue��1,��ÿ���ۼ�5;���LightValue��-1,��ÿ�μ�5
	duty[0] = duty[0] + 5*LightValue;
	if(duty[0]>1023){//�ﵽ���ֵ
		duty[0] = 1023;//����Ϊ���ֵ
		LightValue = -1;//��ʼ��С
	}
	if(duty[0]<0){//�ﵽ��Сֵ
		duty[0] = 0;//����Ϊ��Сֵ
		LightValue=1;//��ʼ�ۼ�
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
	uart_init_new();
	//1000:����1000us duty:�ߵ�ƽʱ��100us   1:��������һ���ܽ�,��Ϊ������[1][3]   io_info:io_info����
	pwm_init(1000, duty, 1, io_info);
	pwm_start();//����PWM

    //���ö�ʱ��
    os_timer_setfn(&os_timer_one,os_timer_one_function,"yang");//os_timer_one:��ʱ���ṹ�����    os_timer_one_function:�ص�����    yang:�����ص������Ĳ���
    //ʹ�ܶ�ʱ��
    //ÿ��10ms�ı�LED�Ƶ�����(���ۿ�������С��20ms��˲��仯,�ῴ��������)
    os_timer_arm(&os_timer_one,10,1);//os_timer_one:��ʱ������        10:10ms��һ��    1:ѭ��
}

