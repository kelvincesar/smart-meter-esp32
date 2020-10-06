// Include used libraries
#include "smartmeter.h"					// Smartmeter header


// # Current sensor configuration:
#define S_AMP_RATIO         1           // Current sensor ratio;

// # Voltage sensor configuration:
#define S_VOL_RATIO         1           // Voltage sensor ratio;
#define MAIN_FREQ           60          // Main frequency in Hz;

// # ADC configuration:
#define I2S_NUM 0                       // I2S drive number;
#define ADC_NUM_OF_CH 2                 // Number of channels that are read;
#define ADC_SAMPLING_RATE   30000       // Sampling rate in Hz
#define ADC_BUFFER_SIZE    (ADC_SAMPLING_RATE*ADC_NUM_OF_CH)/MAIN_FREQ

#define ADC_DMA_COUNT       8           // Number of DMA buffers

#define ADC_GET_MEASURE(s)  (s & 0xFFF) // Macro used to get 12 bit part from adc read;
#define ADC_V_REF           3300        // ADC Voltage reference in mV;
#define ADC_RESOLUTION      4096        // ADC resolution
#define ADC_SIGNAL_IS_AC    true        // Define that signal read is AC;
#define ADC_SIGNAL_OFFSET   2048        // Define offset for AC signal;


static void adc_i2s_init(void)
{
    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = ADC_SAMPLING_RATE*ADC_NUM_OF_CH,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL3,
        .dma_buf_count = ADC_DMA_COUNT,
        .dma_buf_len = ADC_BUFFER_SIZE,
        .use_apll = true};
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

    //init ADC pad
    static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_6},
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_7
        },
    };

    i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));
}

static void testAdc(void *p)
{
    	// Local variables
	int64_t start_time = 0;        			// Start time tick [us]
	int64_t end_time = 0;          			// End time tick [us]
	float time_spent = 0;					// Time spent in the execution [s]
    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_V_REF, &cal);

    adc_i2s_init();

    adc_sample_t buf[ADC_BUFFER_SIZE];



    //read data from I2S bus, in this case, from ADC.
    size_t got;
    // Obtain current time
    i2s_adc_enable(I2S_NUM);
    for (int k = 0; k < 2; ++k) {
        start_time = esp_timer_get_time();
        esp_err_t err = i2s_read(I2S_NUM, buf, sizeof(buf), &got, portMAX_DELAY);
        // Obtain current time
        end_time = esp_timer_get_time();
        if (err != ESP_OK)
        {
            printf("i2s_read: %d\n", err);
        }

        //DBG_ASSERT(got % sizeof(adc_sample_t) == 0);
        got /= sizeof(adc_sample_t);

        uint32_t data_converted = 0;  // Voltage read;

        for (size_t i = 0; i < got; i += ADC_NUM_OF_CH)
        {
            for (size_t j = 0; j < ADC_NUM_OF_CH; ++j)
            {

                data_converted = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i + j]), &cal);
                printf("{Data %d} - [CH-%d]%d mV\n", i + j,buf[i + j] >> 12, data_converted);
            }

            printf("\n");
        }
        
        printf("[GOT VARIABLE] = %d\n", got);
        // Calculating time spent
        time_spent = (float)(end_time - start_time) / 1000000;
        // Printing after execution
        printf("execution time (one block at a time) = %f s\n", time_spent);
    }
    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    vTaskDelete(NULL);
    
}

void app_main(void)
{
    // start the main task
    //xTaskCreate(&connect_to_wifi, "con2WIFI", 2048, NULL, 5, NULL);
    //set RGB PINs
    gpio_pad_select_gpio(LED_R);
    gpio_pad_select_gpio(LED_G);
    gpio_pad_select_gpio(LED_B);
    
    gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);


    wifi_init_softap();
    //xTaskCreate(testAdc, "testAdc", 1024 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
}
