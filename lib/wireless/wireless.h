// Inclusion guard, to prevent multiple includes of the same header
#ifndef WIFI_H
#define WIFI_H


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "tcpip_adapter.h"

#define WIFI_SSID "kelvin_esp"
#define WIFI_PASS "kelvin321"

#define TCP_SERVER_IP "192.168.137.1"
#define TCP_SERVER_PORT (8090)


void setup_wifi(void);

int socket_tcp_connect();
void socket_tcp_close_connection(int s);
uint16_t socket_tcp_send_data(int s, uint8_t data[], uint16_t data_size);


#endif