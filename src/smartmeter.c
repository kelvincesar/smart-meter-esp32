// Include used libraries
#include "smartmeter.h"					// Smartmeter header

// Global variable


SmartMeter sm_data;                     // Store smart meter main variables data;    
IpDFT ip_dft_data;                      // Store IpDFT algorithm return values



static void adc_i2s_init(void)
{
    
    static const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = ADC_SAMPLE_RATE*2,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = ADC_DMA_COUNT,
        .dma_buf_len = ADC_BUFFER_SIZE,
        .use_apll = false,
    };
    //install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    // ADC-I2S Scanning does not normal start (more badly) after the reset of ESP32. 
    // It is necessary to add a delay of 5 seconds before switching on the ADC for a reliable start, this is issue.
    vTaskDelay(5000/portTICK_RATE_MS);
    //init ADC pad
    static const adc_i2s_pattern_t adc_i2s_pattern[] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_0,
        },
        {
            .atten = ADC_ATTEN_DB_11,
            .bits = ADC_WIDTH_BIT_12,
            .channel = ADC1_CHANNEL_3
        }
    };

    i2s_set_adc_mode(ADC_UNIT_1, adc_i2s_pattern, sizeof(adc_i2s_pattern));

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

uint8_t do_thd(){
    // Compute the following parameters from harmonic = 2 ... 7;
    // - Frequency
    // - Amplitude
    // - Phase

    u16_t i = 0;
    // Config to compute applying complete Goertzel calculus;
    //u8_t gtz_config = 0x3;        
    
    
    float delta_phi = 0;
    float main_active_power = 0;
    float main_cosphi = 0;
    float power_aux = 0;

    float harm_active_power = 0;
    float aux_thd_i_v  =   0;           // Store Sqrt(1 - THD_v^2)*Sqrt(1 - THD_i^2)
    // {amp, freq, phase, rms}
    float v_harm_data[7][4] = {
		{sm_data.voltage_amp, sm_data.voltage_freq, sm_data.voltage_phase}, // Main 
		{0, 0, 0, 0},          // 2nd order                 
		{0, 0, 0, 0},          // 3rd order
		{0, 0, 0, 0},          // 4th order
    	{0, 0, 0, 0},          // 5th order
    	{0, 0, 0, 0},          // 6th order
    	{0, 0, 0, 0},          // 7th order
    };
    float i_harm_data[7][4] = {
		{sm_data.current_amp, sm_data.current_freq, sm_data.current_phase}, // Main 
		{0, 0, 0, 0},          // 2nd order
		{0, 0, 0, 0},          // 3rd order
		{0, 0, 0, 0},          // 4th order
    	{0, 0, 0, 0},          // 5th order
    	{0, 0, 0, 0},          // 6th order
    	{0, 0, 0, 0},          // 7th order
    };
    // # Compute voltage and current RMS considering a pure sine wave
    float v_main_rms = (float) (v_harm_data[0][H_AMP] / M_SQRT2);
    float i_main_rms = (float) (i_harm_data[0][H_AMP] / M_SQRT2);

    // # Compute values for the main frequency
    delta_phi = (float) fabs(v_harm_data[0][H_PHASE] - i_harm_data[0][H_PHASE]);
    main_cosphi = cosf(delta_phi);

    sm_data.active_power   = (float) (v_main_rms * i_main_rms * main_cosphi);
    main_active_power = sm_data.active_power;  // used to compute Pn/P1

    sm_data.reactive_power = (float) (v_main_rms * i_main_rms * sinf(delta_phi));

    sm_data.THD_V = 0;
    sm_data.THD_I = 0;

    //printf("\n Debug delta_phi=%f, main_cosphi=%f, v_main_rms=%f, i_main_rms=%f, main_active_power=%f, sm_data.reactive_power=%f\n", delta_phi, main_cosphi, v_main_rms, i_main_rms,main_active_power, sm_data.reactive_power);

    // # Compute harmonics amplitude and phase using Goertzel.
    // Start from i=1 because first position stores main signal values.
    for (i = 1; i < ARRAY_LEN(v_harm_data); i++){
        printf("\n- Harmonic number [%d] \n", i+1);
       
        ipdft_handle = do_ipdft(&buf_voltage, &ip_dft_data, 60*(i+1));
        if(ipdft_handle != 0){
            // # Store result into main data block
            v_harm_data[i][H_AMP] = ip_dft_data.amplitude;
            v_harm_data[i][H_PHASE] = ip_dft_data.phase_ang;
            v_harm_data[i][H_FREQ] = ip_dft_data.frequency;

            
            ipdft_handle = do_ipdft(&buf_current, &ip_dft_data, 60*(i+1));
            if(ipdft_handle != 0){
                i_harm_data[i][H_AMP] = ip_dft_data.amplitude;
                i_harm_data[i][H_PHASE] = ip_dft_data.phase_ang;
                i_harm_data[i][H_FREQ] = ip_dft_data.frequency;


                // Compute harmonic RMS value
                v_harm_data[i][H_RMS] = (float) (v_harm_data[i][H_AMP] / M_SQRT2);
                i_harm_data[i][H_RMS] = (float) (i_harm_data[i][H_AMP] / M_SQRT2);

                // # Compute active and reactive power for this harmonic
                delta_phi = (float) fabs(v_harm_data[i][H_PHASE] - i_harm_data[i][H_PHASE]);    // Use aux to compute delta phi
                harm_active_power = (float) (v_harm_data[i][H_RMS] * i_harm_data[i][H_RMS]);// Used as aux to compute V*I
                sm_data.reactive_power += (float) (harm_active_power * sinf(delta_phi));    // Sum harmonic piece into total active power 

                sm_data.active_power += (float) (harm_active_power * cosf(delta_phi));                                  // Sum harmonic piece into total active power 
                

                // Compute sum of (Vrms_n / Vrms_1) and (Irms_n / Irms_1)
                power_aux = (float) (v_harm_data[i][H_RMS] / v_main_rms);
                sm_data.THD_V += (float) (power_aux*power_aux);

                power_aux = (float) (i_harm_data[i][H_RMS] / i_main_rms);
                sm_data.THD_I += (float) (power_aux*power_aux);

                sm_data.THD_P += (float) (harm_active_power / main_active_power);
                #if DEBUG_EN == true 
                //printf("*** CURRENT *** \n");
                //printf("- Amplitude: %f \n", i_harm_data[i][H_AMP]);
                //printf("- Angle: %f \n", i_harm_data[i][H_PHASE]);
                //printf("- RMS: %f \n", i_harm_data[i][H_RMS]);
                //printf("- Frequency: %f \n", i_harm_data[i][H_FREQ]);
                //printf("*** Voltage *** \n");
                //printf("- Amplitude: %f \n", v_harm_data[i][H_AMP]);
                //printf("- Angle: %f \n", v_harm_data[i][H_PHASE]);
                //printf("- RMS: %f \n", v_harm_data[i][H_RMS]);
                //printf("- Frequency: %f \n", v_harm_data[i][H_FREQ]);
                //printf("Delta phi: %f \n\n", delta_phi);

                printf("[C_%d] - %f; %f; %f; %f\n", i+1, i_harm_data[i][H_AMP], i_harm_data[i][H_PHASE], i_harm_data[i][H_RMS], i_harm_data[i][H_FREQ]);
                printf("[V_%d] - %f; %f; %f; %f\n", i+1, v_harm_data[i][H_AMP], v_harm_data[i][H_PHASE], v_harm_data[i][H_RMS], v_harm_data[i][H_FREQ]);
                printf("[P_%d] - %f; %f\n", i+1,(float) (harm_active_power * cosf(delta_phi)), (float) (harm_active_power * sinf(delta_phi)));
                printf("Delta phi: %f \n\n", delta_phi);
                #endif
                
                


            } else {
                ESP_LOGE(TAG_SM, "# Error when computing IpDFT for current");
            }
        } else {
            ESP_LOGE(TAG_SM, "# Error when computing IpDFT for voltage");
        }


        // Compute 
        // Compute active and reactive power (P and Q)
    }



    // # Compute THD_v and THD_i
    sm_data.THD_V = sqrtf(sm_data.THD_V);
    sm_data.THD_I = sqrtf(sm_data.THD_I);

    // # Compute Sqrt(1 - THD_v^2)*Sqrt(1 - THD_i^2) 
    aux_thd_i_v  = (float) (sqrtf(1+sm_data.THD_V * sm_data.THD_V));
    aux_thd_i_v *= (float) (sqrtf(1+sm_data.THD_I * sm_data.THD_I));
    
    // # Compute aparent power
    sm_data.aparent_power  = (float) (sm_data.v_rms * sm_data.i_rms * aux_thd_i_v);

    // # Compute FP
    sm_data.fp  = (float) (main_cosphi * (1 + sm_data.THD_P));
    sm_data.fp /= (float) (aux_thd_i_v);

    sm_data.frequency = (sm_data.voltage_freq + sm_data.current_freq) / 2;

    //save_to_flash();
    sm_data_converter(&sm_data);
    show_sm_values();
    
    sm_push(&sm_data);
    SmartMeter_Struct_Reset(&sm_data);

    return 1;
}
void show_sm_values(){
    printf("\n===========[ Smart Meter values ]===========\n ");
    printf(" - v_rms           = %f\n", sm_data.v_rms);
    printf(" - i_rms           = %f\n", sm_data.i_rms);         
    printf(" - aparent_power   = %f\n", sm_data.aparent_power);
    printf(" - active_power    = %f\n", sm_data.active_power);
    printf(" - reactive_power  = %f\n", sm_data.reactive_power);
    printf(" - frequency       = %f\n", sm_data.frequency);
    printf(" - voltage_amp     = %f\n", sm_data.voltage_amp);   
    printf(" - voltage_freq    = %f\n", sm_data.voltage_freq); 
    printf(" - voltage_phase   = %f\n", sm_data.voltage_phase);
    printf(" - current_amp     = %f\n", sm_data.current_amp);
    printf(" - current_phase   = %f\n", sm_data.current_phase); 
    printf(" - current_freq    = %f\n", sm_data.current_freq);
    printf(" - fp              = %f\n", sm_data.fp);
    printf(" - THD_V           = %f\n", sm_data.THD_V);         
    printf(" - THD_I           = %f\n", sm_data.THD_I);       
    printf(" - THD_P           = %f\n", sm_data.THD_P);
    printf("============================================\n");      
}
void get_flash_data(){
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");
    }
    int32_t aux;
    err = nvs_get_i32(my_handle, "v_rms", &aux);
    sm_data.v_rms           = (float) (aux);
    err = nvs_get_i32(my_handle, "i_rms", &aux);
    sm_data.i_rms           = (float) (aux);
    err = nvs_get_i32(my_handle, "aparent_power", &aux);
    sm_data.aparent_power   = (float) (aux);
    err = nvs_get_i32(my_handle, "active_power", &aux);
    sm_data.active_power    = (float) (aux);
    err = nvs_get_i32(my_handle, "reactive_power", &aux);
    sm_data.reactive_power  = (float) (aux);
    err = nvs_get_i32(my_handle, "frequency", &aux);
    sm_data.frequency       = (float) (aux);
    err = nvs_get_i32(my_handle, "voltage_amp", &aux);
    sm_data.voltage_amp     = (float) (aux);
    err = nvs_get_i32(my_handle, "voltage_freq", &aux);
    sm_data.voltage_freq    = (float) (aux);
    err = nvs_get_i32(my_handle, "voltage_phase", &aux);
    sm_data.voltage_phase   = (float) (aux);
    err = nvs_get_i32(my_handle, "current_amp", &aux);
    sm_data.current_amp     = (float) (aux);
    err = nvs_get_i32(my_handle, "current_phase", &aux);
    sm_data.current_phase   = (float) (aux);
    err = nvs_get_i32(my_handle, "current_freq", &aux);
    sm_data.current_freq    = (float) (aux);
    err = nvs_get_i32(my_handle, "fp", &aux);
    sm_data.fp              = (float) (aux);
    err = nvs_get_i32(my_handle, "THD_V", &aux);
    sm_data.THD_V           = (float) (aux);
    err = nvs_get_i32(my_handle, "THD_I", &aux);
    sm_data.THD_I           = (float) (aux);
    err = nvs_get_i32(my_handle, "THD_P", &aux); 
    sm_data.THD_P           = (float) (aux);
   
    show_sm_values();
}
void save_to_flash(){
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        
        
        // Write
        int32_t aux = 0;
        aux = (int32_t) (sm_data.v_rms);
        err = nvs_set_i32(my_handle, "v_rms", sm_data.v_rms);
        aux = (int32_t) (sm_data.i_rms);
        err = nvs_set_i32(my_handle, "i_rms", sm_data.i_rms);
        aux = (int32_t) (sm_data.aparent_power);
        err = nvs_set_i32(my_handle, "aparent_power", sm_data.aparent_power);
        aux = (int32_t) (sm_data.active_power);
        err = nvs_set_i32(my_handle, "active_power", sm_data.active_power);
        aux = (int32_t) (sm_data.reactive_power);
        err = nvs_set_i32(my_handle, "reactive_power", sm_data.reactive_power);
        aux = (int32_t) (sm_data.frequency);
        err = nvs_set_i32(my_handle, "frequency", sm_data.frequency);
        aux = (int32_t) (sm_data.voltage_amp);
        err = nvs_set_i32(my_handle, "voltage_amp", sm_data.voltage_amp);
        aux = (int32_t) (sm_data.voltage_freq);
        err = nvs_set_i32(my_handle, "voltage_freq", sm_data.voltage_freq);
        aux = (int32_t) (sm_data.voltage_phase);
        err = nvs_set_i32(my_handle, "voltage_phase", sm_data.voltage_phase);
        aux = (int32_t) (sm_data.current_amp);
        err = nvs_set_i32(my_handle, "current_amp", sm_data.current_amp);
        aux = (int32_t) (sm_data.current_phase);
        err = nvs_set_i32(my_handle, "current_phase", sm_data.current_phase);
        aux = (int32_t) (sm_data.current_freq);
        err = nvs_set_i32(my_handle, "current_freq", sm_data.current_freq);
        aux = (int32_t) (sm_data.fp);
        err = nvs_set_i32(my_handle, "fp", sm_data.fp);
        aux = (int32_t) (sm_data.THD_V);
        err = nvs_set_i32(my_handle, "THD_V", sm_data.THD_V);
        aux = (int32_t) (sm_data.THD_I);
        err = nvs_set_i32(my_handle, "THD_I", sm_data.THD_I);
        aux = (int32_t) (sm_data.THD_P);
        err = nvs_set_i32(my_handle, "THD_P", sm_data.THD_P);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }




}

uint8_t do_ipdft(Buffer *buf, IpDFT *ipdft, uint16_t target_freq){

    
    u16_t i = 0;
    u8_t epsilon;   
    float delta, delta_pi;

    // Frequecies and amplitudes around 60 Hz where Goertzel algorithm will be applied.
	float dft[5][3] = {      
		{0, target_freq-10, 0},                        // {amp, freq, phase}           
		{0, target_freq-5, 0},                        // {amp, freq, phase}
		{0, target_freq, 0},                        // {amp, freq, phase}
		{0, target_freq+5, 0},                        // {amp, freq, phase}
    	{0, target_freq+10, 0},                        // {amp, freq, phase}
	};

    // Config to compute applying Hanning Window and complete Goertzel calculus;
    u8_t gtz_config = 0x3;           

    // Compute goertzel for frequencies arround main frequency
    for (i = 0; i < G_MAIN_FREQ_NUM; i++){
        // # Apply Goertzel to current frequency
        
        //printf("Harmonic [%d] f\n", (uint16_t) (dft[i][1]));
        
        goertzel_handle = goertzel(buf, &g_state, (uint16_t) (dft[i][1]), ADC_SAMPLE_RATE, gtz_config);
        
        dft[i][0] = 1 * g_state.DFT_m;  // # Store Goertzel magnitude
        dft[i][2] = g_state.DFT_arg;    // # Store Goertzel angle
        //printf("%f, %f\n", dft[i][0], dft[i][1]);
    }
    

    // ## Interpolation ##
    u8_t k = 0;         // Index from the highest amplitude found using Goertzel algo
    u16_t k_m = 0;       // Index of DFT
    // # Find highest amplitude in DFT vector:
    for (i = 1; i < G_MAIN_FREQ_NUM-1; i++){
        if(dft[i][0] > dft[i-1][0] && dft[i][0] > dft[i+1][0]){
            k = i;
        }
    }
    
    // # Compute IpDFT algorithm for Hanning Window
    if(k > 0){
        epsilon = (dft[k+1][0] >= dft[k-1][0]) ? 1 : -1;
        //delta = epsilon * (2 * dft[k+epsilon][0] - dft[k][0]) / (dft[k][0] + dft[k+epsilon][0]);
        delta =  (float) (dft[k + epsilon][0] - dft[k - epsilon][0]);
        delta /= (float) (dft[k - epsilon][0] + 2* dft[k][0] + dft[k + epsilon][0]);
        delta *= (float) (2 * epsilon);


        // # Delta deve estar entre 0.5 <= δ < 0.5 
        if(delta >= 0.5) delta = 0.5;
        if(delta < -0.5) delta = -0.5;
        // # Compute k_m index
        k_m = (u16_t) floor((0.5 + (buf->size * dft[k][1] / ADC_SAMPLE_RATE)));
        // # Interpolate to get main frequency
        ipdft->frequency = (k_m + delta)* (1 / SAMPLING_WINDOW_TIME);

        // # Compute real signal amplitude
        delta_pi = M_PI * delta;
        ipdft->amplitude = fabs (dft[k][0]);
        ipdft->amplitude *= fabs ( (float) (delta_pi) / (float) sinf(delta_pi));
        ipdft->amplitude *= fabs ( (float) (delta*delta - 1));

        // # Compute signal phase
        ipdft->phase_ang = dft[k][2] - delta_pi;
        #if DEBUG_EN == true 
        printf("\nMaior amplitude = %f, k = %d\n", dft[k][0], k);
        printf("- Epsilon: %d \n", epsilon);
        printf("- delta: %f \n", delta);
        printf("- k_m: %d \n", k_m);
        printf("- delta_pi: %f \n", delta_pi);

        #endif
    
 
        return 1;
    } else {
        return 0;
    }
    

}
// Compute rms and remove dc offset
void do_rms(Buffer *voltage, Buffer *current){
    u32_t sum_voltage = 0;
    u32_t sum_current = 0;

    // # Calcula o valor de offset do sinal
    u16_t voltage_offset = (voltage->sum / voltage->size);
    u16_t current_offset = (current->sum / current->size);
    
    // Remove o offset do sinal e computa o somatório da sua potência
    for (u16_t i = 0; i < BUFFER_SIZE; i++){
        // Atualiza o valor do buffer removendo o offset
        voltage->data[i] -= voltage_offset;
        current->data[i] -= current_offset;
        // Função quadrada
        sum_voltage += (uint32_t) (voltage->data[i]*voltage->data[i]);
        sum_current += (uint32_t) (current->data[i]*current->data[i]);
    }
    // Cálculo do RMS
    sm_data.v_rms = sqrtf(sum_voltage / BUFFER_SIZE);
    sm_data.i_rms = sqrtf(sum_current / BUFFER_SIZE);
    
    printf("==================\n");
    printf("v_rms = %f\n", sm_data.v_rms );
    printf("i_rms = %f\n", sm_data.i_rms );
    printf("voltage_offset = %d\n", voltage_offset );
    printf("current_offset = %d\n", current_offset );
    printf("==================\n");
    //#if DEBUG_EN == true 
    //#endif

}


// # Verify buffers state
bool buffers_verify(){
    return (buf_voltage.size >= BUFFER_SIZE || buf_current.size >= BUFFER_SIZE || buffer_handle == -1) ? true : false;
}


/*
    Apply linear equation to convert binary to mV
    y = 0.8165x + 143.21
*/ 
uint16_t convert_to_voltage(uint16_t value){
    return (float) (value * 0.8165 + 143.21);
    //return (value * 3300 / 4095);  // To compare with generated values
}

void sample_read_task(void *parameters)
{
    // # Define ADC Characteristics for convertion
    esp_adc_cal_characteristics_t cal;
    // Local variables
	BaseType_t task_create_ret;				// Return of task create

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, ADC_V_REF, &cal);
//ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 3300, adc_chars);
    // # Struct to moving average
    FilterTypeDef filterStruct;
    Moving_Average_Init(&filterStruct);



    // # Declare I2S variables
    adc_sample_t buf[ADC_BUFFER_SIZE];  // Raw buffer with measures from ADC;  
    size_t measures_read;               // Receive number of measures read from I2S - ADC
    
    // # Declare local variables
    u16_t sample = 0;                   // Generic variable to momently store sample value converted to mV
    u16_t loop_ctrl = 0;                // Used to control while loop

    // # Start ADC I2S Setup
    ESP_LOGI(TAG_SM, "# Starting ADC configuration");
    adc_i2s_init();
    ESP_LOGI(TAG_SM, "# ADC setup successfuly");
    buffer_clean(&buf_current);
    buffer_clean(&buf_voltage);
    ESP_LOGI(TAG_SM, "# Buffers cleaned");

    while (loop_ctrl == 0) {
        // # Read data from i2s.      
        esp_err_t err = i2s_read(I2S_NUM, buf, sizeof(buf), &measures_read, portMAX_DELAY);

        if (err != ESP_OK)
        {
            printf("i2s_read: %d\n", err);

        }
        // # Compute vector size
        measures_read /= sizeof(adc_sample_t);  

        // # Loop on measures to convert and save on buffer
        for (u16_t i = 0; i < measures_read; i ++)
        {
            // Get channels offset on buffer
            switch (buf[i] >> 12){
                case ADC_CURRENT_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    //sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                    sample = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i]), &cal);
                    sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_current, sample);
                    break;
                case ADC_VOLTAGE_CHANNEL:
                    // Realiza a conversão da amostra para tensão de acordo com calibração
                    //sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                    sample = esp_adc_cal_raw_to_voltage(ADC_GET_MEASURE(buf[i]), &cal);
                    sample = Moving_Average_Compute(sample, &filterStruct);
                    // Salva no buffer de tensão
                    buffer_handle = buffer_push(&buf_voltage, sample);
                    break; 
                default:
                    ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[i]>>12);

            }
            //printf("[%d]: %d\n", buf[i] >> 12, ADC_GET_MEASURE(buf[i]));
        }
        if(buf_voltage.size != buf_current.size){
            ESP_LOGE(TAG_SM, "# Voltage buffer and current doesn't have the same size [%d/%d]", buf_voltage.size, buf_current.size);
        } else {

            // Verifica se os buffers chegaram ao limite
            if(buffers_verify()){
                // Debug
                int64_t start_time = 0;             // Start time tick [us]
                int64_t end_time = 0;               // End time tick [us]
                float time_spent = 0;	            // Time spent in the execution [s]
                start_time = esp_timer_get_time();      // Debug
                // # Compute RMS and remove DC offset from signals;
                //for(uint16_t x = 0; x < buf_voltage.size ; x++){
                //    printf("%u,%d;", x, buf_voltage.data[x]);
                //}
                do_rms(&buf_voltage, &buf_current);
                // # Apply goertzel to voltage signal
                #if DEBUG_EN == true 
                    ESP_LOGI(TAG_SM, "- Executing IPDFT for voltage"); 
                #endif
                ipdft_handle = do_ipdft(&buf_voltage, &ip_dft_data, 60);
                if(ipdft_handle != 0){
                    // # Store result into main data block
                    sm_data.voltage_amp = ip_dft_data.amplitude;
                    sm_data.voltage_phase = ip_dft_data.phase_ang;
                    sm_data.voltage_freq = ip_dft_data.frequency;
                    #if DEBUG_EN == true 
                        ESP_LOGI(TAG_SM, "- Executing IPDFT for current"); 
                    #endif
                    // # Apply goertzel to current signal
                    ipdft_handle = do_ipdft(&buf_current, &ip_dft_data, 60);
                    if(ipdft_handle != 0){
                        // # Store result into main data block
                        sm_data.current_amp = ip_dft_data.amplitude;
                        sm_data.current_phase = ip_dft_data.phase_ang;
                        sm_data.current_freq = ip_dft_data.frequency;
                        #if DEBUG_EN == true 
                            ESP_LOGI(TAG_SM, "- Executing THD and general calculations");
                        #endif
                        
                        do_thd();

                    } else {
                        ESP_LOGE(TAG_SM, "# Error when computing IpDFT for current");
                    }
                } else {
                    ESP_LOGE(TAG_SM, "# Error when computing IpDFT for voltage");
                }

                buffer_clean(&buf_current);
                buffer_clean(&buf_voltage);
                // # Debug 
                // Obtain current time
                end_time = esp_timer_get_time();

                // Calculating time spent
                time_spent = (float)(end_time - start_time) / 1000;
                printf("smart meter execution time = %f ms\n", time_spent);
                break;
                //#if ENABLE_IOT == true
                if( sm_get_counter() >= SM_TIMESERIE_SIZE ){
                    // Create task to publish data to IoT platform
                    task_create_ret = xTaskCreatePinnedToCore(
                    publish_mqtt_payload,		// Function executed in the task
                    "MQTTPUB",					// Task name (for debug)
                    8000,						// Stack size in bytes
                    NULL,						// Parameter to pass to the function
                    1,							// Task priority
                    &publish_task_handle,		// Used to pass back a handle by which the created task can be referenced
                    1);							// CPU core ID

                }
                //#endif
                //loop_ctrl = 1;
            }
        }
            

    }
    // Debug
    /*
    int s = 0;
    s = socket_tcp_connect();
    if(s >= 0){
        uint8_t* tcp_buffer;
        tcp_buffer = (uint8_t*) calloc(8192, sizeof(uint8_t)); 

        for(uint16_t x = 0; x < buf_voltage.size ; x++){
            if(x % 2 == 0){
                tcp_buffer[x] = buf_voltage.data[x] & 0xFF;
            } else { 
                tcp_buffer[x] = buf_voltage.data[x] >> 8; 
            }

            //printf("Buffer [%d] - %d (%d)\n", x, tcp_buffer[x], buf_voltage.data[x]);
        }
            //printf("Buffer [%d] = %d\n", x, buf_voltage.data[x]);
        socket_tcp_send_data(s, tcp_buffer, 8192);
        free(tcp_buffer); 
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
    }
    socket_tcp_close_connection(s);
    */

    i2s_adc_disable(I2S_NUM);
    i2s_stop(I2S_NUM);
    vTaskDelete(NULL);
    
}
void test_functions(void *parameters) {
    BaseType_t task_create_ret;				// Return of task create
    FilterTypeDef filterStruct;
    Moving_Average_Init(&filterStruct);
    adc_sample_t buf[2048*2] = {1692, 14254, 1704, 14275, 1714, 14294, 1725, 14310, 1734, 14325, 1743, 14337, 1751, 14348, 1759, 14358, 1766, 14367, 1773, 14375, 1780, 14384, 1787, 14392, 1795, 14402, 1802, 14411, 1810, 14422, 1818, 14433, 1826, 14446, 1834, 14458, 1842, 14471, 1851, 14485, 1859, 14498, 1866, 14510, 1873, 14522, 1880, 14532, 1886, 14541, 1891, 14548, 1895, 14553, 1898, 14556, 1901, 14557, 1902, 14555, 1902, 14552, 1902, 14548, 1901, 14541, 1900, 14534, 1898, 14525, 1896, 14516, 1894, 14506, 1893, 14496, 1891, 14485, 1889, 14475, 1888, 14465, 1886, 14454, 1885, 14444, 1884, 14433, 1883, 14423, 1881, 14412, 1880, 14400, 1878, 14388, 1876, 14375, 1873, 14361, 1869, 14347, 1865, 14331, 1860, 14315, 1854, 14297, 1847, 14280, 1840, 14261, 1833, 14243, 1825, 14224, 1816, 14205, 1808, 14187, 1799, 14168, 1790, 14150, 1782, 14133, 1773, 14115, 1765, 14098, 1757, 14081, 1749, 14064, 1741, 14047, 1734, 14029, 1726, 14010, 1718, 13990, 1710, 13969, 1701, 13947, 1692, 13924, 1683, 13899, 1673, 13872, 1662, 13844, 1651, 13815, 1640, 13786, 1627, 13756, 1615, 13725, 1602, 13695, 1589, 13665, 1576, 13636, 1562, 13608, 1549, 13582, 1537, 13558, 1524, 13535, 1513, 13514, 1501, 13495, 1491, 13478, 1480, 13463, 1471, 13449, 1461, 13437, 1453, 13426, 1444, 13416, 1436, 13406, 1428, 13396, 1421, 13387, 1414, 13378, 1406, 13368, 1399, 13359, 1392, 13349, 1386, 13339, 1379, 13330, 1373, 13320, 1367, 13311, 1361, 13302, 1356, 13294, 1352, 13287, 1347, 13281, 1344, 13276, 1341, 13272, 1339, 13270, 1337, 13269, 1335, 13269, 1334, 13270, 1333, 13272, 1333, 13276, 1333, 13280, 1333, 13285, 1332, 13290, 1332, 13296, 1332, 13303, 1332, 13310, 1332, 13318, 1332, 13326, 1333, 13334, 1333, 13344, 1334, 13354, 1334, 13364, 1336, 13376, 1338, 13388, 1340, 13401, 1343, 13415, 1347, 13430, 1351, 13446, 1356, 13462, 1362, 13479, 1368, 13496, 1375, 13514, 1382, 13531, 1389, 13549, 1396, 13566, 1404, 13584, 1412, 13601, 1420, 13619, 1428, 13636, 1436, 13654, 1444, 13672, 1452, 13691, 1461, 13711, 1469, 13731, 1478, 13753, 1488, 13777, 1498, 13802, 1508, 13828, 1519, 13856, 1531, 13886, 1543, 13917, 1556, 13949, 1569, 13982, 1583, 14016, 1597, 14050, 1611, 14083, 1626, 14116, 1640, 14147, 1654, 14177, 1667, 14205, 1680, 14231, 1693, 14255, 1704, 14276, 1715, 14295, 1725, 14311, 1734, 14325, 1743, 14338, 1751, 14349, 1759, 14358, 1766, 14367, 1774, 14376, 1781, 14384, 1788, 14393, 1795, 14402, 1803, 14412, 1810, 14422, 1818, 14434, 1826, 14446, 1834, 14459, 1843, 14472, 1851, 14485, 1859, 14498, 1867, 14511, 1874, 14522, 1880, 14532, 1886, 14541, 1891, 14548, 1895, 14553, 1898, 14556, 1901, 14557, 1902, 14555, 1902, 14552, 1902, 14547, 1901, 14541, 1900, 14533, 1898, 14525, 1896, 14515, 1894, 14505, 1892, 14495, 1891, 14485, 1889, 14475, 1887, 14464, 1886, 14454, 1885, 14443, 1884, 14433, 1883, 14422, 1881, 14411, 1880, 14400, 1878, 14387, 1876, 14374, 1873, 14360, 1869, 14346, 1865, 14330, 1859, 14314, 1854, 14296, 1847, 14279, 1840, 14260, 1832, 14242, 1824, 14223, 1816, 14204, 1807, 14186, 1799, 14167, 1790, 14149, 1781, 14132, 1773, 14114, 1765, 14097, 1757, 14080, 1749, 14063, 1741, 14046, 1733, 14028, 1726, 14009, 1718, 13989, 1709, 13968, 1701, 13946, 1692, 13922, 1683, 13897, 1673, 13871, 1662, 13843, 1651, 13814, 1639, 13784, 1627, 13754, 1614, 13724, 1601, 13693, 1588, 13664, 1575, 13635, 1562, 13607, 1549, 13581, 1536, 13556, 1524, 13534, 1512, 13513, 1501, 13494, 1490, 13478, 1480, 13462, 1470, 13449, 1461, 13437, 1452, 13425, 1444, 13415, 1436, 13405, 1428, 13396, 1421, 13386, 1413, 13377, 1406, 13368, 1399, 13358, 1392, 13349, 1385, 13339, 1379, 13329, 1372, 13320, 1367, 13310, 1361, 13302, 1356, 13294, 1351, 13287, 1347, 13281, 1344, 13276, 1341, 13272, 1338, 13270, 1336, 13269, 1335, 13269, 1334, 13270, 1333, 13272, 1333, 13276, 1333, 13280, 1333, 13285, 1332, 13290, 1332, 13297, 1332, 13303, 1332, 13310, 1332, 13318, 1332, 13326, 1333, 13335, 1333, 13344, 1334, 13354, 1335, 13365, 1336, 13376, 1338, 13389, 1340, 13402, 1343, 13416, 1347, 13431, 1351, 13447, 1356, 13463, 1362, 13480, 1368, 13497, 1375, 13514, 1382, 13532, 1389, 13550, 1397, 13567, 1405, 13585, 1412, 13602, 1420, 13620, 1428, 13637, 1436, 13655, 1445, 13673, 1453, 13692, 1461, 13712, 1470, 13732, 1479, 13754, 1488, 13778, 1498, 13803, 1509, 13829, 1520, 13858, 1531, 13887, 1544, 13919, 1557, 13951, 1570, 13984, 1584, 14018, 1598, 14051, 1612, 14085, 1626, 14117, 1641, 14149, 1654, 14179, 1668, 14207, 1681, 14232, 1693, 14256, 1705, 14277, 1716, 14296, 1726, 14312, 1735, 14326, 1744, 14338, 1752, 14349, 1759, 14359, 1767, 14368, 1774, 14376, 1781, 14385, 1788, 14393, 1795, 14402, 1803, 14412, 1811, 14423, 1818, 14435, 1827, 14447, 1835, 14460, 1843, 14473, 1851, 14486, 1859, 14499, 1867, 14511, 1874, 14523, 1881, 14533, 1886, 14541, 1891, 14548, 1895, 14553, 1899, 14556, 1901, 14557, 1902, 14555, 1902, 14552, 1902, 14547, 1901, 14541, 1900, 14533, 1898, 14524, 1896, 14515, 1894, 14505, 1892, 14495, 1891, 14484, 1889, 14474, 1887, 14464, 1886, 14453, 1885, 14443, 1884, 14432, 1883, 14422, 1881, 14411, 1880, 14399, 1878, 14387, 1875, 14374, 1872, 14360, 1869, 14345, 1864, 14329, 1859, 14313, 1853, 14296, 1847, 14278, 1840, 14259, 1832, 14241, 1824, 14222, 1815, 14203, 1807, 14185, 1798, 14166, 1789, 14148, 1781, 14131, 1772, 14114, 1764, 14096, 1756, 14079, 1748, 14062, 1741, 14045, 1733, 14027, 1725, 14008, 1717, 13988, 1709, 13967, 1701, 13945, 1692, 13921, 1682, 13896, 1672, 13869, 1661, 13841, 1650, 13813, 1638, 13783, 1626, 13753, 1614, 13722, 1601, 13692, 1587, 13662, 1574, 13633, 1561, 13606, 1548, 13580, 1536, 13555, 1523, 13533, 1511, 13512, 1500, 13494, 1490, 13477, 1479, 13462, 1470, 13448, 1461, 13436, 1452, 13425, 1444, 13415, 1435, 13405, 1428, 13395, 1420, 13386, 1413, 13377, 1406, 13367, 1399, 13358, 1392, 13348, 1385, 13338, 1378, 13329, 1372, 13319, 1366, 13310, 1361, 13301, 1356, 13293, 1351, 13286, 1347, 13280, 1344, 13276, 1341, 13272, 1338, 13270, 1336, 13269, 1335, 13269, 1334, 13270, 1333, 13273, 1333, 13276, 1333, 13280, 1333, 13285, 1332, 13291, 1332, 13297, 1332, 13304, 1332, 13311, 1332, 13318, 1332, 13327, 1333, 13335, 1333, 13345, 1334, 13355, 1335, 13365, 1336, 13377, 1338, 13389, 1340, 13403, 1344, 13417, 1347, 13432, 1352, 13448, 1357, 13464, 1362, 13481, 1369, 13498, 1375, 13515, 1382, 13533, 1390, 13550, 1397, 13568, 1405, 13586, 1413, 13603, 1421, 13621, 1429, 13638, 1437, 13656, 1445, 13674, 1453, 13693, 1462, 13713, 1470, 13734, 1479, 13756, 1489, 13779, 1499, 13804, 1509, 13831, 1520, 13859, 1532, 13889, 1544, 13920, 1557, 13953, 1571, 13986, 1584, 14019, 1599, 14053, 1613, 14086, 1627, 14119, 1641, 14150, 1655, 14180, 1669, 14208, 1681, 14234, 1694, 14257, 1705, 14278, 1716, 14296, 1726, 14313, 1735, 14327, 1744, 14339, 1752, 14350, 1760, 14359, 1767, 14368, 1774, 14377, 1781, 14385, 1789, 14394, 1796, 14403, 1803, 14413, 1811, 14424, 1819, 14435, 1827, 14447, 1835, 14460, 1844, 14473, 1852, 14487, 1860, 14500, 1867, 14512, 1875, 14523, 1881, 14533, 1887, 14542, 1892, 14548, 1896, 14553, 1899, 14556, 1901, 14556, 1902, 14555, 1902, 14552, 1902, 14547, 1901, 14540, 1900, 14533, 1898, 14524, 1896, 14514, 1894, 14504, 1892, 14494, 1890, 14484, 1889, 14473, 1887, 14463, 1886, 14453, 1885, 14442, 1884, 14432, 1883, 14421, 1881, 14410, 1880, 14398, 1878, 14386, 1875, 14373, 1872, 14359, 1868, 14344, 1864, 14329, 1859, 14312, 1853, 14295, 1846, 14277, 1839, 14258, 1832, 14240, 1823, 14221, 1815, 14202, 1806, 14184, 1798, 14165, 1789, 14148, 1780, 14130, 1772, 14113, 1764, 14096, 1756, 14079, 1748, 14061, 1740, 14044, 1732, 14026, 1725, 14007, 1717, 13987, 1709, 13966, 1700, 13944, 1691, 13920, 1682, 13895, 1672, 13868, 1661, 13840, 1650, 13811, 1638, 13781, 1626, 13751, 1613, 13721, 1600, 13690, 1587, 13661, 1574, 13632, 1560, 13604, 1548, 13578, 1535, 13554, 1523, 13532, 1511, 13511, 1500, 13493, 1489, 13476, 1479, 13461, 1469, 13448, 1460, 13435, 1451, 13424, 1443, 13414, 1435, 13404, 1427, 13395, 1420, 13386, 1412, 13376, 1405, 13367, 1398, 13357, 1391, 13348, 1385, 13338, 1378, 13328, 1372, 13319, 1366, 13309, 1360, 13301, 1355, 13293, 1351, 13286, 1347, 13280, 1343, 13275, 1341, 13272, 1338, 13270, 1336, 13269, 1335, 13269, 1334, 13270, 1333, 13273, 1333, 13276, 1333, 13280, 1332, 13285, 1332, 13291, 1332, 13297, 1332, 13304, 1332, 13311, 1332, 13319, 1332, 13327, 1333, 13336, 1333, 13345, 1334, 13355, 1335, 13366, 1336, 13378, 1338, 13390, 1341, 13403, 1344, 13418, 1347, 13433, 1352, 13448, 1357, 13465, 1363, 13481, 1369, 13499, 1376, 13516, 1383, 13534, 1390, 13551, 1398, 13569, 1405, 13586, 1413, 13604, 1421, 13621, 1429, 13639, 1437, 13657, 1445, 13675, 1454, 13694, 1462, 13714, 1471, 13735, 1480, 13757, 1489, 13780, 1499, 13805, 1510, 13832, 1521, 13861, 1533, 13891, 1545, 13922, 1558, 13954, 1571, 13987, 1585, 14021, 1599, 14055, 1614, 14088, 1628, 14121, 1642, 14152, 1656, 14182, 1669, 14209, 1682, 14235, 1694, 14258, 1706, 14279, 1717, 14297, 1727, 14313, 1736, 14327, 1744, 14340, 1753, 14350, 1760, 14360, 1768, 14369, 1775, 14377, 1782, 14385, 1789, 14394, 1796, 14403, 1804, 14413, 1811, 14424, 1819, 14436, 1827, 14448, 1836, 14461, 1844, 14474, 1852, 14487, 1860, 14500, 1868, 14512, 1875, 14524, 1881, 14534, 1887, 14542, 1892, 14549, 1896, 14553, 1899, 14556, 1901, 14556, 1902, 14555, 1902, 14552, 1902, 14547, 1901, 14540, 1900, 14532, 1898, 14523, 1896, 14514, 1894, 14504, 1892, 14494, 1890, 14483, 1889, 14473, 1887, 14463, 1886, 14452, 1885, 14442, 1884, 14431, 1883, 14421, 1881, 14409, 1880, 14398, 1878, 14385, 1875, 14372, 1872, 14358, 1868, 14343, 1864, 14328, 1859, 14311, 1853, 14294, 1846, 14276, 1839, 14258, 1831, 14239, 1823, 14220, 1815, 14201, 1806, 14183, 1797, 14165, 1789, 14147, 1780, 14129, 1772, 14112, 1763, 14095, 1755, 14078, 1748, 14061, 1740, 14043, 1732, 14025, 1724, 14006, 1716, 13986, 1708, 13965, 1700, 13943, 1691, 13919, 1681, 13893, 1671, 13867, 1660, 13839, 1649, 13810, 1637, 13780, 1625, 13749, 1612, 13719, 1599, 13689, 1586, 13659, 1573, 13630, 1560, 13603, 1547, 13577, 1534, 13553, 1522, 13531, 1510, 13510, 1499, 13492, 1488, 13475, 1478, 13460, 1469, 13447, 1460, 13435, 1451, 13424, 1443, 13414, 1435, 13404, 1427, 13394, 1419, 13385, 1412, 13376, 1405, 13366, 1398, 13357, 1391, 13347, 1384, 13337, 1378, 13328, 1372, 13318, 1366, 13309, 1360, 13300, 1355, 13293, 1351, 13286, 1347, 13280, 1343, 13275, 1340, 13272, 1338, 13270, 1336, 13269, 1335, 13269, 1334, 13270, 1333, 13273, 1333, 13276, 1333, 13281, 1332, 13286, 1332, 13291, 1332, 13298, 1332, 13304, 1332, 13311, 1332, 13319, 1332, 13327, 1333, 13336, 1333, 13346, 1334, 13356, 1335, 13367, 1336, 13378, 1338, 13391, 1341, 13404, 1344, 13418, 1348, 13433, 1352, 13449, 1357, 13465, 1363, 13482, 1369, 13500, 1376, 13517, 1383, 13535, 1390, 13552, 1398, 13570, 1406, 13587, 1414, 13605, 1422, 13622, 1430, 13640, 1438, 13658, 1446, 13676, 1454, 13695, 1462, 13715, 1471, 13736, 1480, 13758, 1490, 13782, 1500, 13807, 1510, 13834, 1521, 13862, 1533, 13892, 1546, 13923, 1558, 13956, 1572, 13989, 1586, 14023, 1600, 14056, 1614, 14090, 1629, 14122, 1643, 14153, 1657, 14183, 1670, 14211, 1683, 14236, 1695, 14259, 1706, 14280, 1717, 14298, 1727, 14314, 1736, 14328, 1745, 14340, 1753, 14351, 1761, 14360, 1768, 14369, 1775, 14377, 1782, 14386, 1789, 14395, 1797, 14404, 1804, 14414, 1812, 14425, 1820, 14436, 1828, 14449, 1836, 14462, 1844, 14475, 1853, 14488, 1861, 14501, 1868, 14513, 1875, 14524, 1882, 14534, 1887, 14543, 1892, 14549, 1896, 14554, 1899, 14556, 1901, 14556, 1902, 14555, 1902, 14551, 1902, 14546, 1901, 14540, 1900, 14532, 1898, 14523, 1896, 14513, 1894, 14503, 1892, 14493, 1890, 14483, 1889, 14472, 1887, 14462, 1886, 14452, 1885, 14441, 1884, 14431, 1882, 14420, 1881, 14409, 1880, 14397, 1877, 14385, 1875, 14372, 1872, 14358, 1868, 14343, 1864, 14327, 1858, 14310, 1852, 14293, 1846, 14275, 1838, 14257, 1831, 14238, 1823, 14219, 1814, 14200, 1805, 14182, 1797, 14164, 1788, 14146, 1780, 14128, 1771, 14111, 1763, 14094, 1755, 14077, 1747, 14060, 1739, 14042, 1732, 14024, 1724, 14005, 1716, 13985, 1708, 13964, 1699, 13941, 1690, 13917, 1681, 13892, 1670, 13865, 1660, 13837, 1648, 13808, 1637, 13778, 1624, 13748, 1612, 13717, 1599, 13687, 1585, 13658, 1572, 13629, 1559, 13602, 1546, 13576, 1534, 13552, 1521, 13530, 1510, 13509, 1499, 13491, 1488, 13474, 1478, 13460, 1468, 13446, 1459, 13434, 1451, 13423, 1442, 13413, 1434, 13403, 1427, 13394, 1419, 13385, 1412, 13375, 1405, 13366, 1397, 13356, 1391, 13347, 1384, 13337, 1377, 13327, 1371, 13318, 1365, 13309, 1360, 13300, 1355, 13292, 1350, 13285, 1347, 13280, 1343, 13275, 1340, 13272, 1338, 13270, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13273, 1333, 13277, 1333, 13281, 1332, 13286, 1332, 13292, 1332, 13298, 1332, 13305, 1332, 13312, 1332, 13320, 1332, 13328, 1333, 13337, 1333, 13346, 1334, 13356, 1335, 13367, 1336, 13379, 1338, 13391, 1341, 13405, 1344, 13419, 1348, 13434, 1352, 13450, 1358, 13466, 1363, 13483, 1370, 13500, 1376, 13518, 1383, 13535, 1391, 13553, 1398, 13571, 1406, 13588, 1414, 13606, 1422, 13623, 1430, 13641, 1438, 13659, 1446, 13677, 1454, 13696, 1463, 13716, 1472, 13737, 1481, 13759, 1490, 13783, 1500, 13808, 1511, 13835, 1522, 13864, 1534, 13894, 1546, 13925, 1559, 13957, 1573, 13991, 1586, 14024, 1601, 14058, 1615, 14091, 1629, 14124, 1643, 14155, 1657, 14184, 1671, 14212, 1683, 14237, 1696, 14260, 1707, 14281, 1718, 14299, 1728, 14315, 1737, 14329, 1745, 14341, 1753, 14351, 1761, 14361, 1768, 14369, 1775, 14378, 1782, 14386, 1790, 14395, 1797, 14404, 1804, 14414, 1812, 14425, 1820, 14437, 1828, 14449, 1837, 14462, 1845, 14475, 1853, 14489, 1861, 14501, 1868, 14514, 1876, 14525, 1882, 14535, 1888, 14543, 1892, 14549, 1896, 14554, 1899, 14556, 1901, 14556, 1902, 14555, 1902, 14551, 1902, 14546, 1901, 14539, 1900, 14531, 1898, 14522, 1896, 14513, 1894, 14503, 1892, 14493, 1890, 14482, 1889, 14472, 1887, 14462, 1886, 14451, 1885, 14441, 1884, 14430, 1882, 14419, 1881, 14408, 1879, 14397, 1877, 14384, 1875, 14371, 1872, 14357, 1868, 14342, 1863, 14326, 1858, 14309, 1852, 14292, 1845, 14274, 1838, 14256, 1830, 14237, 1822, 14218, 1814, 14200, 1805, 14181, 1796, 14163, 1788, 14145, 1779, 14127, 1771, 14110, 1763, 14093, 1755, 14076, 1747, 14059, 1739, 14041, 1731, 14023, 1724, 14004, 1716, 13984, 1707, 13963, 1699, 13940, 1690, 13916, 1680, 13891, 1670, 13864, 1659, 13836, 1648, 13807, 1636, 13777, 1624, 13746, 1611, 13716, 1598, 13686, 1585, 13656, 1572, 13628, 1558, 13600, 1546, 13575, 1533, 13551, 1521, 13528, 1509, 13508, 1498, 13490, 1487, 13474, 1477, 13459, 1468, 13446, 1459, 13434, 1450, 13423, 1442, 13413, 1434, 13403, 1426, 13393, 1419, 13384, 1411, 13375, 1404, 13365, 1397, 13356, 1390, 13346, 1384, 13336, 1377, 13327, 1371, 13317, 1365, 13308, 1360, 13300, 1355, 13292, 1350, 13285, 1346, 13279, 1343, 13275, 1340, 13271, 1338, 13269, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13273, 1333, 13277, 1333, 13281, 1332, 13286, 1332, 13292, 1332, 13298, 1332, 13305, 1332, 13312, 1332, 13320, 1332, 13328, 1333, 13337, 1333, 13347, 1334, 13357, 1335, 13368, 1336, 13379, 1338, 13392, 1341, 13406, 1344, 13420, 1348, 13435, 1353, 13451, 1358, 13467, 1364, 13484, 1370, 13501, 1377, 13519, 1384, 13536, 1391, 13554, 1399, 13572, 1407, 13589, 1414, 13607, 1422, 13624, 1430, 13642, 1438, 13660, 1447, 13678, 1455, 13697, 1463, 13717, 1472, 13738, 1481, 13760, 1491, 13784, 1501, 13809, 1511, 13836, 1523, 13865, 1534, 13895, 1547, 13927, 1560, 13959, 1573, 13992, 1587, 14026, 1601, 14060, 1616, 14093, 1630, 14125, 1644, 14156, 1658, 14186, 1671, 14213, 1684, 14239, 1696, 14261, 1707, 14282, 1718, 14300, 1728, 14316, 1737, 14329, 1746, 14341, 1754, 14352, 1761, 14361, 1769, 14370, 1776, 14378, 1783, 14387, 1790, 14395, 1797, 14405, 1805, 14415, 1813, 14426, 1820, 14438, 1829, 14450, 1837, 14463, 1845, 14476, 1853, 14489, 1861, 14502, 1869, 14514, 1876, 14525, 1882, 14535, 1888, 14543, 1892, 14550, 1896, 14554, 1899, 14556, 1901, 14556, 1902, 14555, 1902, 14551, 1902, 14546, 1901, 14539, 1899, 14531, 1898, 14522, 1896, 14512, 1894, 14502, 1892, 14492, 1890, 14482, 1888, 14471, 1887, 14461, 1886, 14451, 1885, 14440, 1883, 14430, 1882, 14419, 1881, 14408, 1879, 14396, 1877, 14383, 1875, 14370, 1872, 14356, 1868, 14341, 1863, 14325, 1858, 14309, 1852, 14291, 1845, 14273, 1838, 14255, 1830, 14236, 1822, 14217, 1813, 14199, 1805, 14180, 1796, 14162, 1787, 14144, 1779, 14126, 1770, 14109, 1762, 14092, 1754, 14075, 1746, 14058, 1739, 14040, 1731, 14022, 1723, 14003, 1715, 13983, 1707, 13962, 1698, 13939, 1689, 13915, 1680, 13889, 1669, 13863, 1659, 13834, 1647, 13805, 1635, 13775, 1623, 13745, 1610, 13714, 1597, 13684, 1584, 13655, 1571, 13626, 1558, 13599, 1545, 13573, 1532, 13549, 1520, 13527, 1509, 13507, 1498, 13489, 1487, 13473, 1477, 13458, 1467, 13445, 1458, 13433, 1450, 13422, 1441, 13412, 1434, 13402, 1426, 13393, 1418, 13384, 1411, 13374, 1404, 13365, 1397, 13355, 1390, 13346, 1383, 13336, 1377, 13326, 1371, 13317, 1365, 13308, 1359, 13299, 1354, 13292, 1350, 13285, 1346, 13279, 1343, 13275, 1340, 13271, 1338, 13269, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13273, 1333, 13277, 1333, 13281, 1332, 13286, 1332, 13292, 1332, 13298, 1332, 13305, 1332, 13313, 1332, 13320, 1332, 13329, 1333, 13338, 1333, 13347, 1334, 13357, 1335, 13368, 1336, 13380, 1338, 13393, 1341, 13406, 1344, 13421, 1348, 13436, 1353, 13452, 1358, 13468, 1364, 13485, 1370, 13502, 1377, 13520, 1384, 13537, 1391, 13555, 1399, 13572, 1407, 13590, 1415, 13607, 1423, 13625, 1431, 13643, 1439, 13660, 1447, 13679, 1455, 13698, 1464, 13718, 1473, 13739, 1482, 13761, 1491, 13785, 1501, 13811, 1512, 13838, 1523, 13866, 1535, 13897, 1547, 13928, 1560, 13961, 1574, 13994, 1588, 14028, 1602, 14061, 1616, 14095, 1631, 14127, 1645, 14158, 1659, 14187, 1672, 14215, 1685, 14240, 1697, 14263, 1708, 14283, 1719, 14301, 1728, 14316, 1738, 14330, 1746, 14342, 1754, 14352, 1762, 14362, 1769, 14370, 1776, 14379, 1783, 14387, 1790, 14396, 1798, 14405, 1805, 14415, 1813, 14426, 1821, 14438, 1829, 14451, 1837, 14464, 1846, 14477, 1854, 14490, 1862, 14503, 1869, 14515, 1876, 14526, 1883, 14536, 1888, 14544, 1893, 14550, 1896, 14554, 1899, 14556, 1901, 14556, 1902, 14554, 1902, 14551, 1902, 14545, 1901, 14538, 1899, 14530, 1898, 14522, 1896, 14512, 1894, 14502, 1892, 14492, 1890, 14481, 1888, 14471, 1887, 14461, 1886, 14450, 1885, 14440, 1883, 14429, 1882, 14418, 1881, 14407, 1879, 14395, 1877, 14383, 1875, 14370, 1871, 14355, 1867, 14340, 1863, 14324, 1857, 14308, 1851, 14290, 1845, 14272, 1837, 14254, 1830, 14235, 1821, 14216, 1813, 14198, 1804, 14179, 1795, 14161, 1787, 14143, 1778, 14126, 1770, 14108, 1762, 14091, 1754, 14074, 1746, 14057, 1738, 14040, 1731, 14021, 1723, 14002, 1715, 13982, 1707, 13961, 1698, 13938, 1689, 13914, 1679, 13888, 1669, 13861, 1658, 13833, 1647, 13804, 1635, 13774, 1622, 13743, 1610, 13713, 1597, 13683, 1583, 13653, 1570, 13625, 1557, 13598, 1544, 13572, 1532, 13548, 1520, 13526, 1508, 13506, 1497, 13488, 1486, 13472, 1476, 13457, 1467, 13444, 1458, 13433, 1449, 13422, 1441, 13412, 1433, 13402, 1425, 13393, 1418, 13383, 1411, 13374, 1403, 13364, 1396, 13355, 1390, 13345, 1383, 13335, 1376, 13326, 1370, 13316, 1365, 13307, 1359, 13299, 1354, 13291, 1350, 13285, 1346, 13279, 1343, 13274, 1340, 13271, 1338, 13269, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13274, 1333, 13277, 1333, 13282, 1332, 13287, 1332, 13293, 1332, 13299, 1332, 13306, 1332, 13313, 1332, 13321, 1332, 13329, 1333, 13338, 1333, 13348, 1334, 13358, 1335, 13369, 1337, 13381, 1339, 13393, 1341, 13407, 1345, 13421, 1349, 13436, 1353, 13452, 1358, 13469, 1364, 13486, 1370, 13503, 1377, 13521, 1384, 13538, 1392, 13556, 1400, 13573, 1407, 13591, 1415, 13608, 1423, 13626, 1431, 13643, 1439, 13661, 1447, 13680, 1456, 13699, 1464, 13719, 1473, 13740, 1482, 13763, 1492, 13786, 1502, 13812, 1512, 13839, 1524, 13868, 1536, 13898, 1548, 13930, 1561, 13962, 1575, 13996, 1589, 14029, 1603, 14063, 1617, 14096, 1631, 14129, 1645, 14159, 1659, 14189, 1673, 14216, 1685, 14241, 1697, 14264, 1709, 14284, 1719, 14302, 1729, 14317, 1738, 14331, 1747, 14342, 1755, 14353, 1762, 14362, 1769, 14371, 1776, 14379, 1784, 14388, 1791, 14396, 1798, 14406, 1806, 14416, 1813, 14427, 1821, 14439, 1829, 14451, 1838, 14464, 1846, 14477, 1854, 14491, 1862, 14503, 1870, 14515, 1877, 14526, 1883, 14536, 1888, 14544, 1893, 14550, 1897, 14554, 1899, 14556, 1901, 14556, 1902, 14554, 1902, 14550, 1902, 14545, 1901, 14538, 1899, 14530, 1898, 14521, 1896, 14511, 1894, 14501, 1892, 14491, 1890, 14481, 1888, 14470, 1887, 14460, 1886, 14450, 1885, 14439, 1883, 14429, 1882, 14418, 1881, 14407, 1879, 14395, 1877, 14382, 1874, 14369, 1871, 14355, 1867, 14340, 1863, 14324, 1857, 14307, 1851, 14289, 1844, 14271, 1837, 14253, 1829, 14234, 1821, 14215, 1812, 14197, 1804, 14178, 1795, 14160, 1786, 14142, 1778, 14125, 1770, 14108, 1761, 14091, 1753, 14074, 1746, 14056, 1738, 14039, 1730, 14020, 1722, 14001, 1714, 13981, 1706, 13960, 1697, 13937, 1688, 13913, 1679, 13887, 1668, 13860, 1658, 13831, 1646, 13802, 1634, 13772, 1622, 13742, 1609, 13711, 1596, 13681, 1583, 13652, 1570, 13623, 1557, 13596, 1544, 13571, 1531, 13547, 1519, 13525, 1508, 13505, 1496, 13487, 1486, 13471, 1476, 13457, 1466, 13444, 1457, 13432, 1449, 13421, 1441, 13411, 1433, 13401, 1425, 13392, 1418, 13383, 1410, 13373, 1403, 13364, 1396, 13354, 1389, 13345, 1383, 13335, 1376, 13325, 1370, 13316, 1364, 13307, 1359, 13298, 1354, 13291, 1350, 13284, 1346, 13279, 1343, 13274, 1340, 13271, 1338, 13269, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13274, 1333, 13277, 1333, 13282, 1332, 13287, 1332, 13293, 1332, 13299, 1332, 13306, 1332, 13313, 1332, 13321, 1332, 13330, 1333, 13338, 1333, 13348, 1334, 13358, 1335, 13369, 1337, 13381, 1339, 13394, 1341, 13408, 1345, 13422, 1349, 13437, 1353, 13453, 1359, 13470, 1364, 13487, 1371, 13504, 1378, 13521, 1385, 13539, 1392, 13557, 1400, 13574, 1408, 13592, 1416, 13609, 1424, 13627, 1432, 13644, 1440, 13662, 1448, 13681, 1456, 13700, 1465, 13720, 1473, 13741, 1483, 13764, 1492, 13788, 1502, 13813, 1513, 13841, 1524, 13869, 1536, 13900, 1549, 13931, 1562, 13964, 1575, 13997, 1589, 14031, 1604, 14065, 1618, 14098, 1632, 14130, 1646, 14161, 1660, 14190, 1673, 14217, 1686, 14242, 1698, 14265, 1709, 14285, 1720, 14302, 1729, 14318, 1738, 14331, 1747, 14343, 1755, 14353, 1762, 14363, 1770, 14371, 1777, 14380, 1784, 14388, 1791, 14397, 1798, 14406, 1806, 14417, 1814, 14428, 1822, 14439, 1830, 14452, 1838, 14465, 1846, 14478, 1855, 14491, 1862, 14504, 1870, 14516, 1877, 14527, 1883, 14536, 1889, 14544, 1893, 14550, 1897, 14554, 1899, 14556, 1901, 14556, 1902, 14554, 1902, 14550, 1902, 14545, 1901, 14538, 1899, 14530, 1897, 14521, 1895, 14511, 1894, 14501, 1892, 14491, 1890, 14480, 1888, 14470, 1887, 14459, 1886, 14449, 1884, 14439, 1883, 14428, 1882, 14417, 1881, 14406, 1879, 14394, 1877, 14382, 1874, 14368, 1871, 14354, 1867, 14339, 1862, 14323, 1857, 14306, 1851, 14289, 1844, 14270, 1837, 14252, 1829, 14233, 1820, 14214, 1812, 14196, 1803, 14177, 1795, 14159, 1786, 14141, 1777, 14124, 1769, 14107, 1761, 14090, 1753, 14073, 1745, 14055, 1737, 14038, 1730, 14019, 1722, 14000, 1714, 13980, 1706, 13959, 1697, 13936, 1688, 13911, 1678, 13885, 1668, 13858, 1657, 13830, 1646, 13801, 1634, 13771, 1621, 13740, 1608, 13710, 1595, 13680, 1582, 13650, 1569, 13622, 1556, 13595, 1543, 13570, 1531, 13546, 1519, 13524, 1507, 13504, 1496, 13487, 1485, 13471, 1475, 13456, 1466, 13443, 1457, 13431, 1448, 13421, 1440, 13411, 1432, 13401, 1425, 13392, 1417, 13382, 1410, 13373, 1403, 13364, 1396, 13354, 1389, 13344, 1382, 13334, 1376, 13325, 1370, 13315, 1364, 13306, 1359, 13298, 1354, 13291, 1349, 13284, 1346, 13278, 1342, 13274, 1340, 13271, 1337, 13269, 1336, 13269, 1335, 13269, 1334, 13271, 1333, 13274, 1333, 13278, 1333, 13282, 1332, 13287, 1332, 13293, 1332, 13299, 1332, 13306, 1332, 13314, 1332, 13322, 1332, 13330, 1333, 13339, 1333, 13349, 1334, 13359, 1335, 13370, 1337, 13382, 1339, 13395, 1342, 13408, 1345, 13423, 1349, 13438, 1354, 13454, 1359, 13470, 1365, 13487, 1371, 13505, 1378, 13522, 1385, 13540, 1393, 13558, 1400, 13575, 1408, 13593, 1416, 13610, 1424, 13628, 1432, 13645, 1440, 13663, 1448, 13682, 1457, 13701, 1465, 13721, 1474, 13742, 1483, 13765, 1493, 13789, 1503, 13815, 1514, 13842, 1525, 13871, 1537, 13901, 1549, 13933, 1562, 13966, 1576, 13999, 1590, 14033, 1604, 14066, 1619, 14100, 1633, 14132, 1647, 14162, 1661, 14192, 1674, 14219, 1686, 14243, 1698, 14266, 1710, 14286, 1720, 14303, 1730, 14319, 1739, 14332, 1747, 14344, 1755, 14354, 1763, 14363, 1770, 14372, 1777, 14380, 1784, 14388, 1791, 14397, 1799, 14407, 1806, 14417, 1814, 14428, 1822, 14440, 1830, 14453, 1839, 14466, 1847, 14479, 1855, 14492, 1863, 14505, 1870, 14517, 1877, 14527, 1883, 14537, 1889, 14545, 1893, 14551, 1897, 14555, 1900, 14556, 1901, 14556, 1902, 14554, 1902, 14550, 1902, 14544, 1901, 14537, 1899, 14529, 1897, 14520, 1895, 14510, 1893, 14500, 1892, 14490, 1890, 14480, 1888, 14469, 1887, 14459, 1886, 14449, 1884, 14438, 1883, 14428, 1882, 14417, 1881, 14405, 1879, 14393, 1877, 14381, 1874, 14367, 1871, 14353, 1867, 14338, 1862, 14322, 1857, 14305, 1850, 14288, 1844, 14270, 1836, 14251, 1828, 14232, 1820, 14214, 1812, 14195, 1803, 14176, 1794, 14158, 1786, 14140, 1777, 14123, 1769, 14106, 1761, 14089, 1753, 14072, 1745, 14055, 1737, 14037, 1729, 14019, 1722, 13999, 1714, 13979, 1705, 13957, 1697, 13934, 1687, 13910, 1678, 13884, 1667, 13857, 1656, 13829, 1645, 13799, 1633, 13769, 1621, 13739, 1608, 13708, 1595, 13678, 1581, 13649, 1568, 13621, 1555, 13594, 1542, 13568, 1530, 13545, 1518, 13523, 1506, 13504, 1495, 13486, 1485, 13470, 1475, 13455, 1466, 13443, 1457, 13431, 1448, 13420, 1440, 13410, 1432, 13400, 1424, 13391, 1417, 13382, 1410, 13373, 1402, 13363, 1395, 13353, 1389, 13344, 1382, 13334, 1376, 13324, 1369, 13315, 1364, 13306, 1358, 13298, 1354, 13290, 1349, 13284, 1345, 13278, 1342, 13274, 1340, 13271, 1337, 13269, 1336, 13269, 1334, 13269, 1334, 13271, 1333, 13274, 1333, 13278, 1333, 13282, 1332, 13288, 1332, 13293, 1332, 13300, 1332, 13307, 1332, 13314, 1332, 13322, 1332, 13330, 1333, 13339, 1333, 13349, 1334, 13359, 1335, 13371, 1337, 13383, 1339, 13395, 1342, 13409, 1345, 13424, 1349, 13439, 1354, 13455, 1359, 13471, 1365, 13488, 1371, 13506, 1378, 13523, 1386, 13541, 1393, 13558, 1401, 13576, 1409, 13593, 1416, 13611, 1424, 13628, 1432, 13646, 1440, 13664, 1449, 13683, 1457, 13702, 1465, 13722, 1474, 13743, 1483, 13766, 1493, 13790, 1503, 13816, 1514, 13843, 1525, 13872, 1537, 13903, 1550, 13935, 1563, 13967, 1577, 14001, 1591, 14035, 1605, 14068, 1619, 14101, 1634, 14133, 1648, 14164, 1661, 14193, 1674, 14220, 1687, 14244, 1699, 14267, 1710, 14287, 1721, 14304, 1730, 14319, 1739, 14333, 1748, 14344, 1756, 14354, 1763, 14363};
    size_t measures_read = 2048*2;      
    u16_t sample = 0;                   // Generic variable to momently store sample value converted to mV
    u16_t loop_ctrl = 0;                // Used to control while loop
    while(loop_ctrl == 0){
        // # Loop on measures to convert and save on buffer
        for (u16_t i = 0; i < measures_read; i ++)
        {
            // Get channels offset on buffer
                //printf("Channel %d \n", buf[i] >> 12);
                switch (buf[i] >> 12){
                    case ADC_CURRENT_CHANNEL:
                        // Realiza a conversão da amostra para tensão de acordo com calibração
                        sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                        //sample = Moving_Average_Compute(sample, &filterStruct);
                        // Salva no buffer de tensão
                        buffer_handle = buffer_push(&buf_current, sample);
                        break;
                    case ADC_VOLTAGE_CHANNEL:
                        // Realiza a conversão da amostra para tensão de acordo com calibração
                        sample = convert_to_voltage(ADC_GET_MEASURE(buf[i]));
                        //sample = Moving_Average_Compute(sample, &filterStruct);
                        // Salva no buffer de tensão
                        buffer_handle = buffer_push(&buf_voltage, sample);
                        break; 
                    default:
                        ESP_LOGE(TAG_SM, "# Unknown channel %d", buf[i]>>12);

                }
        }
        if(buf_voltage.size != buf_current.size){
            ESP_LOGE(TAG_SM, "# Voltage buffer and current doesn't have the same size [%d/%d]", buf_voltage.size, buf_current.size);
        } else {

            // Verifica se os buffers chegaram ao limite
            if(buffers_verify()){
                // Debug
                int64_t start_time = 0;             // Start time tick [us]
                int64_t end_time = 0;               // End time tick [us]
                float time_spent = 0;	            // Time spent in the execution [s]
                start_time = esp_timer_get_time();      // Debug
                // # Compute RMS and remove DC offset from signals;
                /*for(uint16_t x = 0; x < buf_voltage.size ; x++){
                    printf("%u,%d;", x, buf_voltage.data[x]);
                }*/
                do_rms(&buf_voltage, &buf_current);
                // # Apply goertzel to voltage signal
                #if DEBUG_EN == true 
                    ESP_LOGI(TAG_SM, "- Executing IPDFT for voltage"); 
                #endif
                ipdft_handle = do_ipdft(&buf_voltage, &ip_dft_data, 60);
                if(ipdft_handle != 0){
                    // # Store result into main data block
                    sm_data.voltage_amp = ip_dft_data.amplitude;
                    sm_data.voltage_phase = ip_dft_data.phase_ang;
                    sm_data.voltage_freq = ip_dft_data.frequency;
                    #if DEBUG_EN == true 
                        ESP_LOGI(TAG_SM, "- Executing IPDFT for current"); 
                    #endif
                    // # Apply goertzel to current signal
                    ipdft_handle = do_ipdft(&buf_current, &ip_dft_data, 60);
                    if(ipdft_handle != 0){
                        // # Store result into main data block
                        sm_data.current_amp = ip_dft_data.amplitude;
                        sm_data.current_phase = ip_dft_data.phase_ang;
                        sm_data.current_freq = ip_dft_data.frequency;
                        #if DEBUG_EN == true 
                            ESP_LOGI(TAG_SM, "- Executing THD and general calculations");
                        #endif
                        
                        do_thd();

                    } else {
                        ESP_LOGE(TAG_SM, "# Error when computing IpDFT for current");
                    }
                } else {
                    ESP_LOGE(TAG_SM, "# Error when computing IpDFT for voltage");
                }

                buffer_clean(&buf_current);
                buffer_clean(&buf_voltage);
                // # Debug 
                // Obtain current time
                end_time = esp_timer_get_time();

                // Calculating time spent
                time_spent = (float)(end_time - start_time) / 1000;
                printf("smart meter execution time = %f ms\n", time_spent);
                
                if( sm_get_counter() >= SM_TIMESERIE_SIZE ){
                    // Create task to publish data to IoT platform
                    task_create_ret = xTaskCreatePinnedToCore(
                    publish_mqtt_payload,		// Function executed in the task
                    "MQTTPUB",					// Task name (for debug)
                    8000,						// Stack size in bytes
                    NULL,						// Parameter to pass to the function
                    1,							// Task priority
                    &publish_task_handle,		// Used to pass back a handle by which the created task can be referenced
                    1);							// CPU core ID

                }
                if(time_spent <= 200) time_spent = 200 - time_spent;
                vTaskDelay((time_spent)/ portTICK_PERIOD_MS);
                loop_ctrl = 1;
            }
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    /*
    //Configure ADC
    if (ADC_UNIT_1 == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_12Bit);
        adc1_config_channel_atten(ADC_CHANNEL_0, ADC_ATTEN_11db);
    } else {
        adc2_config_channel_atten((adc2_channel_t)ADC_CHANNEL_0, ADC_ATTEN_11db);
    }
    //Characterize ADC]
    static esp_adc_cal_characteristics_t *adc_chars;
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db, ADC_WIDTH_12Bit, 3300, adc_chars);
    

    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
        //Multisampling
        for (int i = 0; i < 1; i++) {
            if (ADC_UNIT_1 == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)ADC_CHANNEL_0);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)ADC_CHANNEL_0, ADC_WIDTH_12Bit, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= 1;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    */
    // Local variables
	BaseType_t task_create_ret;				// Return of task create
	// Print info to show main task is running
	ESP_LOGI(TAG_SM, "# Running app_main in core %d", xPortGetCoreID());
    #if ENABLE_IOT == true
        // Create task to generate sine wave
        task_create_ret = xTaskCreatePinnedToCore(
            setup_wifi,				// Function executed in the task
            "SWIF",					            // Task name (for debug)
            4096,								// Stack size in bytes
            NULL,								// Parameter to pass to the function
            1,									// Task priority
            &setup_wifi_task_handle,		// Used to pass back a handle by which the created task can be referenced
            1);									// CPU core ID

        // Check task creation error
        if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating setup_wifi task"); }
    #endif
    
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
	vTaskDelay(10000 / portTICK_PERIOD_MS);
    /**/
    //get_flash_data();

    //vTaskDelay(10000 / portTICK_PERIOD_MS);
    // Log task creation
	ESP_LOGI(TAG_SM, "# Creating sample_read_task");

    
	// Create main task to sample and process signals
	task_create_ret = xTaskCreatePinnedToCore(
		sample_read_task,					// Function executed in the task
		"SRT",					            // Task name (for debug)
		32000,								// Stack size in bytes
		NULL,								// Parameter to pass to the function
		1,									// Task priority
		&sample_read_task_handle,			// Used to pass back a handle by which the created task can be referenced
		0);									// CPU core ID

    // Check task creation error
	if (task_create_ret != pdPASS){ ESP_LOGE(TAG_SM, "Error creating sample_read task"); }


    
    
}
