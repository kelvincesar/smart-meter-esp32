// Inclusion guard, to prevent multiple includes of the same header
#ifndef WIFI_H
#define WIFI_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "driver/gpio.h"
#include <esp_http_server.h>

#include <sys/param.h>
#include "tcpip_adapter.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"

#define LED_R 0
#define LED_G 2
#define LED_B 4

#define SSID "r3358"
#define PASSPHARSE "kelvin321"
#define MESSAGE "HelloTCPServer"
#define TCP_SERVER_IP "192.168.43.129"
#define TCP_SERVER_PORT (8090)


void wifi_init_softap(void);
int socket_tcp_connect();
void socket_tcp_close_connection(int s);
uint16_t socket_tcp_send_data(int s, uint8_t data[], uint16_t data_size);

// End of the inclusion guard
#endif