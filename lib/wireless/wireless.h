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
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "tcpip_adapter.h"
// Libraries used on MQTT publish
#include "cJSON.h"
#include "data_buffers.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/semphr.h"
#include "freertos/queue.h"


#include "mqtt_client.h"


#define WIFI_SSID "r3358"
#define WIFI_PASS "kelvin3210"

#define TCP_SERVER_IP "192.168.137.1"
#define TCP_SERVER_PORT (8090)

// WEGnology credentials.
#define WNOLOGY_URI "mqtt://broker.app.wnology.io"
#define WNOLOGY_PORT (1883)
#define WNOLOGY_DEVICE_ID "5faf350f106a3a000746277c"
#define WNOLOGY_ACCESS_KEY "6ce8fbe0-ff33-45b5-87d4-4898a62d8867"
#define WNOLOGY_ACCESS_SECRET "2d8d1bbc6cde4d99f07566d4d6ffbca063653f4b478d70410c176b5dde4d578c"
#define WNOLOGY_STATE_TOPIC "losant/5faf350f106a3a000746277c/state"

void setup_wifi();

int socket_tcp_connect();
void socket_tcp_close_connection(int s);
void publish_mqtt_payload();
void mqtt_app_start();
uint16_t socket_tcp_send_data(int s, uint8_t data[], uint16_t data_size);


#endif