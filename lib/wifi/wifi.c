/*  WiFi connection over AP by Moritz Boesenberg
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "wifi.h"
static EventGroupHandle_t s_wifi_event_group;

static EventGroupHandle_t wifi_event_group;

const int CONNECTED_BIT = BIT0;

static const char *TAG = "ESP32_Server";


static RTC_DATA_ATTR char __SSID[32] = SSID;
static RTC_DATA_ATTR char __PWD[64] = PASSPHARSE;
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station " MACSTR " leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }

    ESP_LOGI(TAG, "wifi_event_handler wifi_event_handler wifi_event_handler");
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP ");
        ESP_LOGI(TAG, "Login Success");
        gpio_set_level(LED_R, 1);
        gpio_set_level(LED_G, 1);
        gpio_set_level(LED_B, 1);
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:

        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED %d ", esp_wifi_connect());

		gpio_set_level(LED_R, 0);
        gpio_set_level(LED_G, 0);
        gpio_set_level(LED_B, 0);

        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}
void wifi_init_softap()
{

    if (strlen(__PWD) && strlen(__SSID) != 0)
    {
        tcpip_adapter_init();
        wifi_event_group = xEventGroupCreate();
        ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

        wifi_config_t wifi_config = {};
        strcpy((char *)wifi_config.sta.ssid, __SSID);
        strcpy((char *)wifi_config.sta.password, __PWD);

        ESP_LOGI(TAG, "WiFi %s ", wifi_config.sta.ssid);
        ESP_LOGI(TAG, "PSW %s ", wifi_config.sta.password);
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    }
    else
    {
        s_wifi_event_group = xEventGroupCreate();

        tcpip_adapter_init();

        ESP_ERROR_CHECK(esp_event_loop_create_default());
        ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL)); //-> just here to get rid of startup error
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();       //= WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

        wifi_config_t wifi_config = {
            .ap = {
                .ssid = "SM Server",
                .ssid_len = strlen("SM Server"),
                .password = "",
                .max_connection = 1,
                .authmode = WIFI_AUTH_OPEN},
        };



        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));

        tcpip_adapter_ip_info_t ip_info;
        IP4_ADDR(&ip_info.ip, 192, 168, 1, 1);
        IP4_ADDR(&ip_info.gw, 192, 168, 1, 1);
        IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

        tcpip_adapter_ap_start((uint8_t *)"9C:B6:D0:E7:30:0F", &ip_info);

        ESP_ERROR_CHECK(esp_wifi_start());
    }


}
// # Handle TCP socket connection
int socket_tcp_connect(){
    int s;
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = inet_addr(TCP_SERVER_IP);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons( TCP_SERVER_PORT );
    ESP_LOGI(TAG,"- TCP Connection started. \n");

    xEventGroupWaitBits(wifi_event_group,CONNECTED_BIT,false,true,portMAX_DELAY);
    // # Allocate socket
    s = socket(AF_INET, SOCK_STREAM, 0);
    if(s < 0) {
        ESP_LOGE(TAG, " * Failed to allocate socket.\n");
        return -1;
    }
    ESP_LOGI(TAG, " * Socket allocated\n");
    if(connect(s, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
        ESP_LOGE(TAG, " * Socket connect failed errno=%d \n", errno);
        close(s);
        return -1;
    }
    ESP_LOGI(TAG, " * Connected! \n");

    return s;
}
void socket_tcp_close_connection(int s){
    close(s);
    ESP_LOGI(TAG, " * Connection closed! \n");
}
// # Send data over TCP Socket
uint16_t socket_tcp_send_data(int s, uint8_t data[], uint16_t data_size){
    printf("Size of data : %d", data_size);
    if(s >= 0){
        if( write(s, data , data_size) < 0)
        {
            ESP_LOGE(TAG, " * Send failed \n");
            return 0;
        }
        //ESP_LOGI(TAG, "... socket send success");
        return 1; 
    } else return 0;
}