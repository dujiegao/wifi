#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#define USE_OPTIMIZE_PRINTF
#define AP_CACHE           0

#if AP_CACHE
#define AP_CACHE_NUMBER    5
#endif

#define WIFI_SSID 		"JYD_2.4G"
#define WIFI_PASSWORD	"guobingg810920"

#define HOST	"192.168.11.181"
#define PORT	8080

#define FILE_URL "/esp8266/newbin/user1.2048.new.3.bin"
#define FILE_URL2 "/esp8266/newbin/user2.2048.new.3.bin"

#define HTTP_REQUEST "GET "FILE_URL" HTTP/1.1\r\nHost: "HOST"\r\n\r\n"
#define HTTP_REQUEST2 "GET "FILE_URL2" HTTP/1.1\r\nHost: "HOST"\r\n\r\n"

// @0x01000
#define USER1_BIN_FLASH_SECTOR	0x01
// @0x81000
#define USER2_BIN_FLASH_SECTOR	0x81

// 是否使用SSL
#define SSL_CLIENT_ENABLE		0

#if SSL_CLIENT_ENABLE
#define SSL_CLIENT_KEY_ADDR		0x9A
#define SSL_CA_ADDR				0x9B
#endif
#endif

