// Include used libraries
#include "smartmeter.h"					// Smartmeter header


static void adc_i2s_init(void)
{
    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = ADC_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL3,
        .dma_buf_count = ADC_DMA_COUNT,
        .dma_buf_len = ADC_BUFFER_SIZE,
        .use_apll = false,
    };
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);

    //init ADC pad
    static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_6,
        },
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_7
        }
    };

    i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));


    // Delay necessary or reliable start;
    vTaskDelay(1000/portTICK_RATE_MS);
}
// # ----------------------------------------------
// # SIGNAL GENERATOR 
// # ----------------------------------------------


void signal_generator_task(void *parameters)
{
    static uint8_t table_sin_wave[SIN_WAVE_NUM] = {
    // Sin wave
        128,131,134,137,141,144,147,150,153,156,159,162,165,168,171,174,
        177,180,183,186,188,191,194,196,199,202,204,207,209,212,214,216,218,221,223,225,227,229,231,233,234,236,
        238,239,241,242,243,245,246,247,248,249,250,251,252,253,253,254,254,255,255,255,255,255,255,
        255,255,255,255,254,254,253,253,252,251,251,250,249,248,247,245,244,243,241,240,238,
        237,235,233,232,230,228,226,224,222,220,217,215,213,210,208,205,203,200,198,195,192,190,187,184,181,178,176,
        173,170,167,164,161,158,155,151,148,145,142,139,136,133,130,126,123,120,117,114,111,108,105,101,
        98,95,92,89,86,83,80,78,75,72,69,66,64,61,58,56,53,51,48,46,43,41,39,36,34,32,30,28,26,24,23,21,
        19,18,16,15,13,12,11,9,8,7,6,5,5,4,3,3,2,2,1,1,1,1,1,1,1,1,1,1,2,2,3,3,4,5,6,7,8,9,10,
        11,13,14,15,17,18,20,22,23,25,27,29,31,33,35,38,40,42,44,47,49,52,54,
        57,60,62,65,68,70,73,76,79,82,85,88,91,94,97,100,103,106,109,112,115,119,122,125
    };

    uint16_t i = 0;
    // Enable DAC CHANNEL 1 - GPIO25
    dac_output_enable(DAC_CHANNEL_1);
    int64_t start_time = 0;
    while (true){

        // To not hog the CPU
        if( (esp_timer_get_time() - start_time >= 60) || start_time == 0){   // Check delta in uS
            dac_output_voltage(DAC_CHANNEL_1, table_sin_wave[i]);
            i++;
            if (i >= SIN_WAVE_NUM) i = 0;

            start_time = esp_timer_get_time();
        }
        // Feed watchdog
        TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
        TIMERG0.wdt_feed=1;
        TIMERG0.wdt_wprotect=0;

    }
}

void compute_goertzel(){

    // - Goertzel
	float dft[3][2] = {                 
		{0, 59},                        // {amplitude, frequency}
		{0, 60},                        // {amplitude, frequency}
		{0, 61}                         // {amplitude, frequency}
	};
    // Compute goertzel 
	for (int i = 0; i < G_MAIN_FREQ_NUM; i++){
		goertzel_handle = goertzel(&buf_voltage, &g_state, dft[i][1], buf_voltage.size);
        printf("g_state.DFT_m %f\n", g_state.DFT_m);
		dft[i][0] = S_VOL_RATIO * g_state.DFT_m;
	}
	// Debug
	printf("%f, %f\n", dft[0][0], dft[0][1]);
	printf("%f, %f\n", dft[1][0], dft[1][1]);
	printf("%f, %f\n", dft[2][0], dft[2][1]);

	// Interpolation
	int epsilon = (dft[2][0] >= dft[0][0]) ? 1 : -1;
	printf("%d epsilon\n", epsilon);
	float alpha = dft[1 + epsilon][0] / dft[1][0];
	float delta =  (2*alpha - 1) / (alpha + 1);

	float frequency = (60 + epsilon*delta);

	printf("frequency: %f\n", frequency);
}

void sample_read_task(void *parameters)
{
    // # Define ADC Characteristics for convertion
    esp_adc_cal_characteristics_t cal;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, ADC_V_REF, &cal);

    // # Declare I2S variables
    adc_sample_t buf[ADC_BUFFER_SIZE];  // Raw buffer with measures from ADC;  
    size_t measures_read;               // Receive number of measures read from I2S - ADC
    
    // # Declare local variables
    int16_t voltage_converted = 0;      // Voltage read;
    int16_t current_converted = 0;      // Current read;

    u16_t current_ch_off = 0;           // Offset applied to get current measure from buffer
    u16_t voltage_ch_off = 0;           // Offset applied to get voltage measure from buffer
    u16_t num_samples = 0;  

    // # Debug
	int64_t start_time = 0;             // Start time tick [us]
	int64_t end_time = 0;               // End time tick [us]
	float time_spent = 0;	            // Time spent in the execution [s]

    // # Start ADC I2S Setup
    adc_i2s_init();

    while (true) {
    //for (int k = 0; k < 1; ++k) {
        // Verify buffer size
        if(buf_voltage.size >= BUFFER_SIZE){
            /*
            // Debug
            for(uint16_t x = 0; x < buf_voltage.size ; x++){
                printf("Buffer [%d] = %d\n", x, buf_voltage.data[x]);
                //vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            */
            // # Apply goertzel:
            start_time = esp_timer_get_time();      // Debug
            compute_goertzel();
            // Obtain current time
            end_time = esp_timer_get_time();

            // Calculating time spent
            time_spent = (float)(end_time - start_time) / 1000000;
            printf("compute_goertzel execution time = %f us", time_spent);
            // Clean buffer
            buffer_clean(&buf_voltage);
            break;
        }
        // # Read data from i2s.
        esp_err_t err = i2s_read(I2S_NUM, buf, sizeof(buf), &measures_read, portMAX_DELAY);

        if (err != ESP_OK)
        {
            printf("i2s_read: %d\n", err);
        }

        measures_read /= sizeof(adc_sample_t);
        num_samples = measures_read/2;          // 2 channels

        

        // Get channels offset on buffer
        switch (buf[0] >> 12){
            case ADC_CURRENT_CHANNEL:
                current_ch_off = 0;
                voltage_ch_off = 1;
            break;
            case ADC_VOLTAGE_CHANNEL:
                current_ch_off = 1;
                voltage_ch_off = 0;                
            break; 

            default:
                ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[0]>>12);

        }

        // Loop on measures to convert and save on buffer
        for (u16_t i = 0; i < measures_read; i += ADC_NUM_OF_CH)
        {
            voltage_converted = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i+voltage_ch_off]), &cal) - ADC_V_REF / 2 ;  //       
            current_converted = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i+current_ch_off]), &cal) - ADC_V_REF / 2 ;  // - ADC_V_REF / 2

            //printf("%d voltage_converted\n", voltage_converted);
            // Insert data into buffers
            buffer_handle = buffer_push(&buf_voltage, voltage_converted);
            buffer_handle = buffer_push(&buf_current, current_converted);

           //sum_voltage += (uint32_t) (voltage_converted*voltage_converted);
           //sum_current += (uint32_t) (current_converted*current_converted);
           //sum_active_power += (uint32_t) (voltage_converted*current_converted);
        }




    }
    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    vTaskDelete(NULL);
    
}

void app_main(void)
{

    //set RGB PINs
    //gpio_pad_select_gpio(LED_R);
    //gpio_pad_select_gpio(LED_G);
    //gpio_pad_select_gpio(LED_B);
//
    //gpio_set_direction(LED_R, GPIO_MODE_OUTPUT);
    //gpio_set_direction(LED_G, GPIO_MODE_OUTPUT);
    //gpio_set_direction(LED_B, GPIO_MODE_OUTPUT);

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }



    ESP_ERROR_CHECK(ret);
    //wifi_init_softap();


    // Local variables
	BaseType_t task_create_ret;				// Return of task create
	// Print info to show main task is running
	ESP_LOGI(TAG_SM, "# Running app_main in core %d", xPortGetCoreID());

    
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating signal_generator task");

	// Create task to generate sine wave
	task_create_ret = xTaskCreatePinnedToCore(
		signal_generator_task,				// Function executed in the task
		"SGT",					            // Task name (for debug)
		4096,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&signal_generator_task_handle,		// Used to pass back a handle by which the created task can be referenced
		1);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating signal_generator task"); }
	// Delay 
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating sample_read_task task");

    
	// Create task to write audio to wav file in SD card
	task_create_ret = xTaskCreatePinnedToCore(
		sample_read_task,					// Function executed in the task
		"SRT",					            // Task name (for debug)
		4096,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&sample_read_task_handle,			// Used to pass back a handle by which the created task can be referenced
		0);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating sample_read task"); }


    
    
}
