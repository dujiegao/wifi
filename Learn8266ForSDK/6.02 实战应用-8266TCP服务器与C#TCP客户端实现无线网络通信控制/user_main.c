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
#include "spi_interface.h"


#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"
#include "lwip/igmp.h"
#include "lwip/tcp.h"


extern char Usart0ReadBuff[Usart0ReadLen];//�������ݻ���
extern u32  Usart0ReadCnt;//���ڽ��յ����ݸ���
extern u32  Usart0ReadCntCopy;//���ڿ������ڽ��յ����ݸ���
extern u32  Usart0IdleCnt;//����ʱ���ۼӱ���


/*���ڷ��ؼ̵�����״̬*/
u8 RelayOn[4]={0x55,0xaa,0x01,0x01};//�̵�������
u8 RelayOff[4]={0x55,0xaa,0x01,0x00};//�̵����Ͽ�


struct tcp_pcb *tcp_pcb_server;//����һ��TCP���ƿ�
#define TcpServerBuffLen 1460
u8 TcpServerBuff[TcpServerBuffLen];//���ջ���
u8 tcp_pcb_server_state = 0;//��¼����״̬

/**
* @brief   TCP��������
* @param   arg:tcp_arg��������Ĳ���
* * @param   p:���յ����ݻ���
* @param   err:������Ϣ
* @param   None
* @retval  None
* @warning None
* @example
**/
static err_t net_tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
	struct pbuf *q;
	u32 length = 0,i=0;
	tcp_pcb_server = tpcb;
	tcp_pcb_server_state = 1;
	if (!p || err!=ERR_OK) {
		if(p){
			pbuf_free(p);
		}
		tcp_pcb_server_state = 0;
		tcp_close(tcp_pcb_server);//�ر�����
		return ERR_CLSD;
	}
	//����TCP����(�̶�)
	for(q=p;q!=NULL;q=q->next){
		if(q->len > (TcpServerBuffLen-length))//���յ����ݸ���������������Խ��յ����ݸ���
			memcpy(TcpServerBuff+length,q->payload,(TcpServerBuffLen-length));//ֻ����������Խ��յ����ݸ���
		else
			memcpy(TcpServerBuff+length,q->payload,q->len);//����TCP��������
		length += q->len;
		if(length > TcpServerBuffLen) break;
	}

	if(TcpServerBuff[0] == 0xaa && TcpServerBuff[1] == 0x55){
		if(TcpServerBuff[2] == 0x01){
			if(TcpServerBuff[3] == 0x01){
				GPIO_OUTPUT_SET(5, 1);//����GPIO5����ߵ�ƽ
				tcp_write(tcp_pcb_server, RelayOn, 4, TCP_WRITE_FLAG_COPY);//TCP_WRITE_FLAG_COPY:���������ͻ���
				tcp_output(tcp_pcb_server);//��������
			}
			else if(TcpServerBuff[3] == 0x00){
				GPIO_OUTPUT_SET(5, 0);//����GPIO5����͵�ƽ
				tcp_write(tcp_pcb_server, RelayOff, 4, TCP_WRITE_FLAG_COPY);//TCP_WRITE_FLAG_COPY:���������ͻ���
				tcp_output(tcp_pcb_server);//��������
			}
		}
	}

	//�̶�����
	tcp_recved(tcp_pcb_server, p->tot_len);/*���½���,���ߵײ���Խ��Ż���������*/
	pbuf_free(p);//�ͷ�����
	return ERR_OK;
}


/**
* @brief   TCP���Ӵ���
* @param   arg:tcp_arg��������Ĳ���
* @param   err:������Ϣ
* @param   None
* @param   None
* @retval  None
* @warning None
* @example
**/
static void net_err_cb(void *arg, err_t err) {
	tcp_pcb_server = (struct tcp_pcb*)arg; //tcp_arg�����˸ò���
	tcp_pcb_server_state = 0;
    tcp_close(tcp_pcb_server);//�ر�����
    tcp_pcb_server = NULL;//���
}

/**
* @brief   �ͻ������ӻص�
* @param   arg:tcp_arg��������Ĳ���
* @param   newpcb:���ӵ�TCP���ƿ�
* @param   err:������Ϣ
* @param   None
* @retval  None
* @warning None
* @example
**/
static err_t net_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err) {
	tcp_pcb_server = newpcb;//��ֵ������Ŀ��ƿ�
	tcp_pcb_server_state = 1;
	tcp_arg(newpcb, newpcb);//���ݵ�arg����Ϊ tcp_pcb_server
	tcp_err(newpcb, net_err_cb);//����ص�
	tcp_recv(newpcb, net_tcp_recv_cb);//�������ݻص�

	printf("�ͻ������� \n");
	return ERR_OK;
}



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
* @brief   Ӳ����ʱ���жϻص�����
* @param   None
* @param   None
* @param   None
* @param   None
* @retval  None
* @warning None
* @example
**/
void hw_test_timer_cb(void)
{
	if(Usart0ReadCnt!=0){//���ڽ��յ�����
		Usart0IdleCnt++;//����ʱ���ۼ�
		if(Usart0IdleCnt>Usart0IdleTime){//�ۼӵ�����ֵ(10ms)
			Usart0IdleCnt=0;
			Usart0ReadCntCopy = Usart0ReadCnt;//�������յ����ݸ���
			Usart0ReadCnt=0;
			/*��������
			 * ���ݻ�������:Usart0ReadBuff
			 * ���ݳ���:Usart0ReadCntCopy
			 * */
			if(tcp_pcb_server_state == 1){//�пͻ�������
				tcp_write(tcp_pcb_server, Usart0ReadBuff, Usart0ReadCntCopy, TCP_WRITE_FLAG_COPY);//TCP_WRITE_FLAG_COPY:���������ͻ���
				tcp_output(tcp_pcb_server);//��������
			}
		}
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
	err_t err = ERR_OK;//���շ��صĴ�����Ϣ

	uart_init_new();


    /*����GPIO5Ϊ��ͨ����*/
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U , FUNC_GPIO5);

	struct tcp_pcb *tcp_pcb1 = tcp_new();//����һ��TCP���ƿ�

	//���ƿ��IP��ַ�Ͷ˿ں�
	err = tcp_bind(tcp_pcb1, IP_ADDR_ANY, 8080);//IP_ADDR_ANY:�󶨱�ģ��IP  8080:��8080�˿�
	if (err == ERR_OK) {//û�д���
		struct tcp_pcb *pcb1 = tcp_listen(tcp_pcb1);//��������

		if (pcb1) {
			tcp_accept(pcb1, net_accept_cb);//���ü������ͻ������ӵĻص�����
		}
	}

    //��ʱ����ʼ��
    hw_timer_init(1);//1:ѭ��
    //���ö�ʱ���ص�����
    hw_timer_set_func(hw_test_timer_cb);//hw_test_timer_cb:Ӳ����ʱ���жϻص�����
    hw_timer_arm(1000);//1000:1000us��ʱ�����жϺ���
}

