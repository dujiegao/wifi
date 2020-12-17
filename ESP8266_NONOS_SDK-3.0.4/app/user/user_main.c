/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ets_sys.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "sntp.h"
#include "upgrade.h"
#include "espconn.h"
#include "http_client.h"
#include "uart.h"
#define  PRE_VERSION  "1.0.1"
#define  NEXT_VERSION "1.0.3"
#define SPI_FLASH_SIZE_MAP 3
#if ((SPI_FLASH_SIZE_MAP == 0) || (SPI_FLASH_SIZE_MAP == 1))
#error "The flash map is not supported"
#elif (SPI_FLASH_SIZE_MAP == 2)
#define SYSTEM_PARTITION_OTA_SIZE							0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR							0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR						0xfb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0xfc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0xfd000
#elif (SPI_FLASH_SIZE_MAP == 3)
#define SYSTEM_PARTITION_OTA_SIZE							0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR							0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR						0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0x1fd000
#elif (SPI_FLASH_SIZE_MAP == 4)
#define SYSTEM_PARTITION_OTA_SIZE							0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR							0x81000
#define SYSTEM_PARTITION_RF_CAL_ADDR						0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0x3fd000
#elif (SPI_FLASH_SIZE_MAP == 5)
#define SYSTEM_PARTITION_OTA_SIZE							0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR							0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR						0x1fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0x1fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0x1fd000
#elif (SPI_FLASH_SIZE_MAP == 6)
#define SYSTEM_PARTITION_OTA_SIZE							0x6A000
#define SYSTEM_PARTITION_OTA_2_ADDR							0x101000
#define SYSTEM_PARTITION_RF_CAL_ADDR						0x3fb000
#define SYSTEM_PARTITION_PHY_DATA_ADDR						0x3fc000
#define SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR				0x3fd000
#else
#error "The flash map is not supported"
#endif

MQTT_Client mqttClient;
typedef unsigned long u32_t;
static ETSTimer sntp_timer;
void ICACHE_FLASH_ATTR ota_start_Upgrade(const char *server_ip, uint16_t port,const char *path);

void sntpfn()
{
    u32_t ts = 0;
    ts = sntp_get_current_timestamp();
    os_printf("current time : %s\n", sntp_get_real_time(ts));
    if (ts == 0) {
        //os_printf("did not get a valid time from sntp server\n");
    } else {
            os_timer_disarm(&sntp_timer);
            MQTT_Connect(&mqttClient);
    }
}

void wifiConnectCb(uint8_t status)
{
	uint8 ip[4] = {HTTP_IP1,HTTP_IP2,HTTP_IP3,HTTP_IP4};
    if(status == STATION_GOT_IP){
        sntp_setservername(0, "pool.ntp.org");        // set sntp server after got ip address
        sntp_init();
        os_timer_disarm(&sntp_timer);
        os_timer_setfn(&sntp_timer, (os_timer_func_t *)sntpfn, NULL);
        os_timer_arm(&sntp_timer, 1000, 1);//1s
		INFO("wifiConnectCb\r\n");
    } else {
          MQTT_Disconnect(&mqttClient);
    }
}

void mqttConnectedCb(uint32_t *args)
{
    MQTT_Client* client = (MQTT_Client*)args;
    INFO("MQTT: Connected\r\n");
    MQTT_Subscribe(client, "/mqtt/topic/0", 0);
    MQTT_Subscribe(client, "/mqtt/topic/1", 1);
    MQTT_Subscribe(client, "/mqtt/topic/2", 2);

    MQTT_Publish(client, "/mqtt/topic/0", "hello0", 6, 0, 0);
    MQTT_Publish(client, "/mqtt/topic/1", "hello1", 6, 1, 0);
    MQTT_Publish(client, "/mqtt/topic/2", "hello2", 6, 2, 0);

}

void mqttDisconnectedCb(uint32_t *args)
{
    MQTT_Client* client = (MQTT_Client*)args;
    INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
    MQTT_Client* client = (MQTT_Client*)args;
    INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
    char *topicBuf = (char*)os_zalloc(topic_len+1),
            *dataBuf = (char*)os_zalloc(data_len+1);

    MQTT_Client* client = (MQTT_Client*)args;

    os_memcpy(topicBuf, topic, topic_len);
    topicBuf[topic_len] = 0;

    os_memcpy(dataBuf, data, data_len);
    dataBuf[data_len] = 0;

    INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
    os_free(topicBuf);
    os_free(dataBuf);
}

static const partition_item_t at_partition_table[] = {
    { SYSTEM_PARTITION_BOOTLOADER, 						0x0, 												0x1000},
    { SYSTEM_PARTITION_OTA_1,   						0x1000, 											SYSTEM_PARTITION_OTA_SIZE},
    { SYSTEM_PARTITION_OTA_2,   						SYSTEM_PARTITION_OTA_2_ADDR, 						SYSTEM_PARTITION_OTA_SIZE},
    { SYSTEM_PARTITION_RF_CAL,  						SYSTEM_PARTITION_RF_CAL_ADDR, 						0x1000},
    { SYSTEM_PARTITION_PHY_DATA, 						SYSTEM_PARTITION_PHY_DATA_ADDR, 					0x1000},
    { SYSTEM_PARTITION_SYSTEM_PARAMETER, 				SYSTEM_PARTITION_SYSTEM_PARAMETER_ADDR, 			0x3000},
};

void ICACHE_FLASH_ATTR user_pre_init(void)
{
    if(!system_partition_table_regist(at_partition_table, sizeof(at_partition_table)/sizeof(at_partition_table[0]),SPI_FLASH_SIZE_MAP)) {
		os_printf("system_partition_table_regist fail\r\n");
		while(1);
	}
}

void ota_finished_callback(struct upgrade_server_info *arg)
{
    struct upgrade_server_info *update = arg;
	if (update->upgrade_flag == true){
		os_printf("OTA  Success ! rebooting!\n");
		system_upgrade_reboot();
	}else{
		os_printf("OTA failed!\n");
	}
	os_free(update->pespconn);
	os_free(update->url);
	os_free(update);

}

/**
 * server_ip: 服务器地址
 * port:服务器端口
 * path:文件路径
 */

void ICACHE_FLASH_ATTR ota_start_Upgrade(const char *server_ip, uint16_t port,const char *path) {
	const char* file;
	uint8 pre_version[16]=PRE_VERSION;
    uint8 upgrade_version[16]=NEXT_VERSION;
	//获取系统的目前加载的是哪个bin文件
	uint8_t userBin = system_upgrade_userbin_check();
	file = (char*) os_zalloc(20);

	switch (userBin) {

	//如果检查当前的是处于user1的加载文件，那么拉取的就是user2.bin
	case UPGRADE_FW_BIN1:
		file = "user2.2048.new.3.bin";		
		break;

		//如果检查当前的是处于user2的加载文件，那么拉取的就是user1.bin
	case UPGRADE_FW_BIN2:
		file = "user1.2048.new.3.bin";
		break;

		//如果检查都不是，可能此刻不是OTA的bin固件
	default:
		os_printf("Fail read system_upgrade_userbin_check! \n\n");
		return;
	}

	struct upgrade_server_info* update =(struct upgrade_server_info *) os_zalloc(
					sizeof(struct upgrade_server_info));
	update->pespconn = (struct espconn *) os_zalloc(sizeof(struct espconn));
	//设置服务器地址
	os_memcpy(update->ip, server_ip, 4);
	//设置服务器端口
	update->port = port;
	//设置OTA回调函数
	update->check_cb = ota_finished_callback;
	//设置定时回调时间
	update->check_times = 10000;
	//从 2M *1024 =2048申请内存
	update->url = (uint8 *)os_zalloc(2048);
	os_memcpy(update->pre_version,pre_version,sizeof(pre_version));
	os_memcpy(update->upgrade_version,upgrade_version,sizeof(upgrade_version)); 
	//打印下請求地址
	os_printf("Http Server Address:%d.%d.%d.%d ,port: %d,filePath: %s,fileName: %s \n",
			IP2STR(update->ip), update->port, path, file);

	//拼接完整的 URL去请求服务器
	os_sprintf((char*) update->url, "GET /%s%s HTTP/1.1\r\n"
			"Host: "IPSTR":%d\r\n"
	"Connection: keep-alive\r\n"
	"\r\n", path, file, IP2STR(update->ip), update->port);

	if (system_upgrade_start(update) == false) {
		os_printf(" Could not start upgrade\n");
		//释放资源
		os_free(update->pespconn);
		os_free(update->url);
		os_free(update);
		os_free(file);	

	} else {
		os_printf(" Upgrading...\n");
	}

}


#if 1
//**********************************************************************************/

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
uint32 ICACHE_FLASH_ATTR
user_rf_cal_sector_set(void) {
	enum flash_size_map size_map = system_get_flash_size_map();
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



void ICACHE_FLASH_ATTR
print_chip_info(void) {
	u8 macAddr[6] = { 0 };
	os_printf("\n*********************************\r\n");
	os_printf("user bin:%d\r\n", system_upgrade_userbin_check());
	os_printf("SDK version:%s\r\n", system_get_sdk_version());
	os_printf("Build Datetime:%s %s\r\n", __DATE__, __TIME__);
	os_printf("Chip ID:%d\r\n", system_get_chip_id());
	os_printf("CPU freq:%d\r\n", system_get_cpu_freq());
	os_printf("Free heap size:%d\r\n", system_get_free_heap_size());
	if (wifi_get_macaddr(STATION_IF, macAddr)) {
		os_printf("MAC:"MACSTR"\r\n", MAC2STR(macAddr));
	} else {
		os_printf("Get MAC fail!\r\n");
	}

	os_printf("meminfo:\r\n");
	system_print_meminfo();
	os_printf("*********************************\r\n");
}

void ICACHE_FLASH_ATTR
init_done_cb_init(void) {
	print_chip_info();

#if 1
	user_wifi_init();
#else
	// parameter: STATION_MODE/SOFTAP_MODE/STATIONAP_MODE
	wifi_set_opmode(STATION_MODE);
	user_set_station_config();
#endif
	wifi_station_disconnect();
	wifi_station_connect();
	//http_client_init(HOST, PORT);
}
#endif
void user_init(void)
{
	
    uart_init_2(BIT_RATE_115200,BIT_RATE_74880);
	//uart_init(BIT_RATE_115200,BIT_RATE_74880);
	system_uart_swap();
    os_delay_us(60000);
#if 1
    CFG_Load();
	//os_printf("\t Project:\t%s\r\n", PRE_VERSION);//项目名称可带版本号测试观察
    os_printf("\t SDK version:\t%s", system_get_sdk_version());//SDK版本
    os_printf("\r\n\tuser bin:%d\r\n", system_upgrade_userbin_check());
    INFO("[IMALIUBO] system_get_free_heap_size=%d\r\n",system_get_free_heap_size());

    MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
    //MQTT_InitConnection(&mqttClient, "192.168.11.122", 1880, 0);

    MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
    //MQTT_InitClient(&mqttClient, "client_id", "user", "pass", 120, 1);

    MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
    MQTT_OnConnected(&mqttClient, mqttConnectedCb);
    MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
    MQTT_OnPublished(&mqttClient, mqttPublishedCb);
    MQTT_OnData(&mqttClient, mqttDataCb);
    WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);
	
#else	
	//system_init_done_cb(init_done_cb_init);
	INFO("\r\nSystem started ...\r\n");
	
#endif
    
}
