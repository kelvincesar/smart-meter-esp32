
#include "wireless.h"
SM_Payload sm_payload;
esp_mqtt_client_handle_t client;
// Event group
EventGroupHandle_t event_group;

int WIFI_CONNECTED_BIT = BIT0;
int MQTT_CONNECTED_BIT = BIT1;

static const char *TAG = "MQTT";

cJSON *json_root, *json_data;

// Wifi event handler
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
		
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    
	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(event_group, WIFI_CONNECTED_BIT);
        break;
    
	case SYSTEM_EVENT_STA_DISCONNECTED:
		xEventGroupClearBits(event_group, WIFI_CONNECTED_BIT);
        setup_wifi();
        break;
    
	default:
        break;
    }
   
	return ESP_OK;
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{

    
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            xEventGroupSetBits(event_group, MQTT_CONNECTED_BIT);

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            xEventGroupClearBits(event_group, MQTT_CONNECTED_BIT);
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

// Main application
void setup_wifi()
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    


	// disable the default wifi logging
	esp_log_level_set("wifi", ESP_LOG_NONE);
	
	// initialize NVS
	ESP_ERROR_CHECK(nvs_flash_init());
	
	// create the event group to handle wifi events
	event_group = xEventGroupCreate();
		
	// initialize the tcp stack
	tcpip_adapter_init();

	// initialize the wifi event handler
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	
	// initialize the wifi stack in STAtion mode with config in RAM
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	// configure the wifi connection and start the interface
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
	printf("Connecting to %s\n", WIFI_SSID);
	
	// wait for connection
	printf("Main task: waiting for connection to the wifi network... ");
	xEventGroupWaitBits(event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
	printf("connected!\n");
	
	// print the local IP address
	tcpip_adapter_ip_info_t ip_info;
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
	printf("IP Address:  %s\n", ip4addr_ntoa(&ip_info.ip));
	printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
    
    if((xEventGroupGetBits( event_group ) & BIT1) >> 1 == 0x0){
        mqtt_app_start();
    }
    vTaskDelete(NULL);
}
void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = WNOLOGY_URI,
        .port = WNOLOGY_PORT,
        .client_id = WNOLOGY_DEVICE_ID,
        .username = WNOLOGY_ACCESS_KEY,
        .password = WNOLOGY_ACCESS_SECRET
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}
// # JSON creator
void publish_mqtt_payload(){
    /*  Create the following JSON payload:
    "data": {
        "vRms": 123.12,
        "iRms": 123.12,
        "sPower": 123.12,
        "aPower": 123.12,
        "rPower": 123.12,
        "freq": 123.12,
        "fp": 123.12,
        "THDV": 123.12,
        "THDI": 123.12,
        "THDP": 123.12,
        "H_vRms": 123.12,
        "H_iRms": 123.12,
        "H_sPower": 123.12,
        "H_aPower": 123.12,
        "H_rPower": 123.12,
        "H_freq": 123.12,
        "H_fp": 123.12,
        "H_THDV": 123.12,
        "H_THDI": 123.12,
        "H_THDP": 123.12,
        "L_vRms": 123.12,
        "L_iRms": 123.12,
        "L_sPower": 123.12,
        "L_aPower": 123.12,
        "L_rPower": 123.12,
        "L_freq": 123.12,
        "L_fp": 123.12,
        "L_THDV": 123.12,
        "L_THDI": 123.12,
        "L_THDP": 123.12,
        "vAmp": 123.12,
        "vFreq": 123.12,
        "vPhase": 123.12,
        "cAmp": 123.12,
        "cPhase": 123.12,
        "cFreq": 123.12,
        "E_P": 12313.1,
        "E_Q": 12313.12,

    }, "ts": 12313412313    (epochtimestamp)
    */
    // Verifica se está conectado na WI-FI e no servidor MQTT
    //if(xEventGroupGetBits( event_group ) == 0x3){
        int msg_id;
        sm_compute_payload(&sm_payload);
        json_root = cJSON_CreateObject();
        cJSON_AddItemToObject(json_root, "data", json_data=cJSON_CreateObject());
        // Add values to json object data
        cJSON_AddNumberToObject(json_data, "vRms"     , sm_payload.vRms    );
        cJSON_AddNumberToObject(json_data, "iRms"     , sm_payload.iRms    );
        cJSON_AddNumberToObject(json_data, "sPower"   , sm_payload.sPower  );
        cJSON_AddNumberToObject(json_data, "aPower"   , sm_payload.aPower  );
        cJSON_AddNumberToObject(json_data, "rPower"   , sm_payload.rPower  );
        cJSON_AddNumberToObject(json_data, "freq"     , sm_payload.freq    );
        cJSON_AddNumberToObject(json_data, "fp"       , sm_payload.fp      );
        cJSON_AddNumberToObject(json_data, "THDV"     , sm_payload.THDV    );
        cJSON_AddNumberToObject(json_data, "THDI"     , sm_payload.THDI    );
        cJSON_AddNumberToObject(json_data, "THDP"     , sm_payload.THDP    );
        cJSON_AddNumberToObject(json_data, "vAmp"     , sm_payload.vAmp    );
        cJSON_AddNumberToObject(json_data, "vFreq"    , sm_payload.vFreq   );
        cJSON_AddNumberToObject(json_data, "vPhase"   , sm_payload.vPhase  );
        cJSON_AddNumberToObject(json_data, "cAmp"     , sm_payload.cAmp    );
        cJSON_AddNumberToObject(json_data, "cPhase"   , sm_payload.cPhase  );
        cJSON_AddNumberToObject(json_data, "cFreq"    , sm_payload.cFreq   );
        cJSON_AddNumberToObject(json_data, "L_vRms"   , sm_payload.L_vRms  );
        cJSON_AddNumberToObject(json_data, "L_iRms"   , sm_payload.L_iRms  );
        cJSON_AddNumberToObject(json_data, "L_sPower" , sm_payload.L_sPower);
        cJSON_AddNumberToObject(json_data, "L_aPower" , sm_payload.L_aPower);
        cJSON_AddNumberToObject(json_data, "L_rPower" , sm_payload.L_rPower);
        cJSON_AddNumberToObject(json_data, "L_freq"   , sm_payload.L_freq  );
        cJSON_AddNumberToObject(json_data, "L_fp"     , sm_payload.L_fp    );
        cJSON_AddNumberToObject(json_data, "L_THDV"   , sm_payload.L_THDV  );
        cJSON_AddNumberToObject(json_data, "L_THDI"   , sm_payload.L_THDI  );
        cJSON_AddNumberToObject(json_data, "L_THDP"   , sm_payload.L_THDP  );
        cJSON_AddNumberToObject(json_data, "H_vRms"   , sm_payload.H_vRms  );
        cJSON_AddNumberToObject(json_data, "H_iRms"   , sm_payload.H_iRms  );
        cJSON_AddNumberToObject(json_data, "H_sPower" , sm_payload.H_sPower);
        cJSON_AddNumberToObject(json_data, "H_aPower" , sm_payload.H_aPower);
        cJSON_AddNumberToObject(json_data, "H_rPower" , sm_payload.H_rPower);
        cJSON_AddNumberToObject(json_data, "H_freq"   , sm_payload.H_freq  );
        cJSON_AddNumberToObject(json_data, "H_fp"     , sm_payload.H_fp    );
        cJSON_AddNumberToObject(json_data, "H_THDV"   , sm_payload.H_THDV  );
        cJSON_AddNumberToObject(json_data, "H_THDI"   , sm_payload.H_THDI  );
        cJSON_AddNumberToObject(json_data, "H_THDP"   , sm_payload.H_THDP  );
        cJSON_AddNumberToObject(json_data, "E_P"      , sm_payload.E_P     );
        cJSON_AddNumberToObject(json_data, "E_Q"      , sm_payload.E_Q     ); 

        char *payload_string = cJSON_Print(json_root);
        msg_id = esp_mqtt_client_publish(client, WNOLOGY_STATE_TOPIC, payload_string, 0, 1, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    //}
    vTaskDelete(NULL);
}   

// # Handle TCP socket connection
int socket_tcp_connect(){
    int s;
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = inet_addr(TCP_SERVER_IP);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons( TCP_SERVER_PORT );
    ESP_LOGI(TAG,"- TCP Connection started. \n");

    xEventGroupWaitBits(event_group,WIFI_CONNECTED_BIT,false,true,portMAX_DELAY);
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