#include "data_buffers.h"             // Include buffer header

TimeSerieSM sm_ts;
uint16_t sm_get_counter(){
    return sm_ts.counter;
}
// Function used to insert data into timeseries struct 
int sm_push (SmartMeter *data){
    // Store data into timeseries data block
    sm_ts.data.vRms   += data->v_rms;
    sm_ts.data.iRms   += data->i_rms;
    sm_ts.data.sPower += data->aparent_power;
    sm_ts.data.aPower += data->active_power;
    sm_ts.data.rPower += data->reactive_power;
    sm_ts.data.freq   += data->frequency;
    sm_ts.data.fp     += data->fp;
    sm_ts.data.THDV   += data->THD_V;
    sm_ts.data.THDI   += data->THD_I;
    sm_ts.data.THDP   += data->THD_P;
    sm_ts.data.vAmp   += data->voltage_amp;
    sm_ts.data.vFreq  += data->voltage_freq;
    sm_ts.data.vPhase += data->voltage_phase;
    sm_ts.data.cAmp   += data->current_amp;
    sm_ts.data.cPhase += data->current_phase;
    sm_ts.data.cFreq  += data->current_freq;
    sm_ts.data.E_P    += data->active_power * WINDOW_LENGTH;
    sm_ts.data.E_Q    += data->reactive_power * WINDOW_LENGTH;
    
    // Compute new max
    if(sm_ts.data.H_vRms   < data->v_rms)          sm_ts.data.H_vRms   =  data->v_rms;
    if(sm_ts.data.H_iRms   < data->i_rms)          sm_ts.data.H_iRms   =  data->i_rms;
    if(sm_ts.data.H_sPower < data->aparent_power)  sm_ts.data.H_sPower =  data->aparent_power;
    if(sm_ts.data.H_aPower < data->active_power)   sm_ts.data.H_aPower =  data->active_power;
    if(sm_ts.data.H_rPower < data->reactive_power) sm_ts.data.H_rPower =  data->reactive_power;
    if(sm_ts.data.H_freq   < data->frequency)      sm_ts.data.H_freq   =  data->frequency;
    if(sm_ts.data.H_fp     < data->fp)             sm_ts.data.H_fp     =  data->fp;
    if(sm_ts.data.H_THDV   < data->THD_V)          sm_ts.data.H_THDV   =  data->THD_V;
    if(sm_ts.data.H_THDI   < data->THD_I)          sm_ts.data.H_THDI   =  data->THD_I;
    if(sm_ts.data.H_THDP   < data->THD_P)          sm_ts.data.H_THDP   =  data->THD_P;

    // Compute new min
    if(sm_ts.data.L_vRms   > data->v_rms)          sm_ts.data.L_vRms   =  data->v_rms;
    if(sm_ts.data.L_iRms   > data->i_rms)          sm_ts.data.L_iRms   =  data->i_rms;
    if(sm_ts.data.L_sPower > data->aparent_power)  sm_ts.data.L_sPower =  data->aparent_power;
    if(sm_ts.data.L_aPower > data->active_power)   sm_ts.data.L_aPower =  data->active_power;
    if(sm_ts.data.L_rPower > data->reactive_power) sm_ts.data.L_rPower =  data->reactive_power;
    if(sm_ts.data.L_freq   > data->frequency)      sm_ts.data.L_freq   =  data->frequency;
    if(sm_ts.data.L_fp     > data->fp)             sm_ts.data.L_fp     =  data->fp;
    if(sm_ts.data.L_THDV   > data->THD_V)          sm_ts.data.L_THDV   =  data->THD_V;
    if(sm_ts.data.L_THDI   > data->THD_I)          sm_ts.data.L_THDI   =  data->THD_I;
    if(sm_ts.data.L_THDP   > data->THD_P)          sm_ts.data.L_THDP   =  data->THD_P;
    sm_ts.counter++;

    return 1;
}
void sm_compute_payload(SM_Payload *payload){
    payload->vRms      = sm_ts.data.vRms     / sm_ts.counter;
    payload->iRms      = sm_ts.data.iRms     / sm_ts.counter;
    payload->sPower    = sm_ts.data.sPower   / sm_ts.counter;
    payload->aPower    = sm_ts.data.aPower   / sm_ts.counter;
    payload->rPower    = sm_ts.data.rPower   / sm_ts.counter;
    payload->freq      = sm_ts.data.freq     / sm_ts.counter;
    payload->fp        = sm_ts.data.fp       / sm_ts.counter;
    payload->THDV      = sm_ts.data.THDV     / sm_ts.counter;
    payload->THDI      = sm_ts.data.THDI     / sm_ts.counter;
    payload->THDP      = sm_ts.data.THDP     / sm_ts.counter;
    payload->vAmp      = sm_ts.data.vAmp     / sm_ts.counter;
    payload->vFreq     = sm_ts.data.vFreq    / sm_ts.counter;
    payload->vPhase    = sm_ts.data.vPhase   / sm_ts.counter;
    payload->cAmp      = sm_ts.data.cAmp     / sm_ts.counter;
    payload->cPhase    = sm_ts.data.cPhase   / sm_ts.counter;
    payload->cFreq     = sm_ts.data.cFreq    / sm_ts.counter;
    payload->L_vRms    = sm_ts.data.L_vRms   ;
    payload->L_iRms    = sm_ts.data.L_iRms   ;
    payload->L_sPower  = sm_ts.data.L_sPower ;
    payload->L_aPower  = sm_ts.data.L_aPower ;
    payload->L_rPower  = sm_ts.data.L_rPower ;
    payload->L_freq    = sm_ts.data.L_freq   ;
    payload->L_fp      = sm_ts.data.L_fp     ;
    payload->L_THDV    = sm_ts.data.L_THDV   ;
    payload->L_THDI    = sm_ts.data.L_THDI   ;
    payload->L_THDP    = sm_ts.data.L_THDP   ;
    payload->H_vRms    = sm_ts.data.H_vRms   ;
    payload->H_iRms    = sm_ts.data.H_iRms   ;
    payload->H_sPower  = sm_ts.data.H_sPower ;
    payload->H_aPower  = sm_ts.data.H_aPower ;
    payload->H_rPower  = sm_ts.data.H_rPower ;
    payload->H_freq    = sm_ts.data.H_freq   ;
    payload->H_fp      = sm_ts.data.H_fp     ;
    payload->H_THDV    = sm_ts.data.H_THDV   ;
    payload->H_THDI    = sm_ts.data.H_THDI   ;
    payload->H_THDP    = sm_ts.data.H_THDP   ;
    payload->E_P       = sm_ts.data.E_P      ;
    payload->E_Q       = sm_ts.data.E_Q      ;
    // Reset data;
    sm_init_data();
}
void sm_init_data () {
    // Reset data
    sm_ts.data.L_vRms   = 65535.0;
    sm_ts.data.L_iRms   = 65535.0;
    sm_ts.data.L_sPower = 65535.0;
    sm_ts.data.L_aPower = 65535.0;
    sm_ts.data.L_rPower = 65535.0;
    sm_ts.data.L_freq   = 65535.0;
    sm_ts.data.L_fp     = 65535.0;
    sm_ts.data.L_THDV   = 65535.0;
    sm_ts.data.L_THDI   = 65535.0;
    sm_ts.data.L_THDP   = 65535.0;
    sm_ts.data.H_vRms   = 0;
    sm_ts.data.H_iRms   = 0;
    sm_ts.data.H_sPower = 0;
    sm_ts.data.H_aPower = 0;
    sm_ts.data.H_rPower = 0;
    sm_ts.data.H_freq   = 0;
    sm_ts.data.H_fp     = 0;
    sm_ts.data.H_THDV   = 0;
    sm_ts.data.H_THDI   = 0;
    sm_ts.data.H_THDP   = 0;
    sm_ts.data.vRms     = 0;
    sm_ts.data.iRms     = 0;
    sm_ts.data.sPower   = 0;
    sm_ts.data.aPower   = 0;
    sm_ts.data.rPower   = 0;
    sm_ts.data.freq     = 0;
    sm_ts.data.fp       = 0;
    sm_ts.data.THDV     = 0;
    sm_ts.data.THDI     = 0;
    sm_ts.data.THDP     = 0;
    sm_ts.data.vAmp     = 0;
    sm_ts.data.vFreq    = 0;
    sm_ts.data.vPhase   = 0;
    sm_ts.data.cAmp     = 0;
    sm_ts.data.cPhase   = 0;
    sm_ts.data.cFreq    = 0;
    sm_ts.counter = 0;
}

// Function used to insert data into buffer and contabilize it size;
int buffer_push (Buffer *buf, int16_t value){
    // Verify if buffer size is not full
    if(buf->size == BUFFER_SIZE){
        return -1;
    }
    // Insert data into buffer and increment it counter
    buf->data[buf->size] = value;
    buf->sum += value;
    buf->size++;


    return 1;
}
void buffer_clean (Buffer *buf){
    for (uint16_t i = 0; i < buf->size ; i++){
        buf->data[i] = 0;
    }
    buf->sum = 0;
    buf->size = 0;
}
int is_buffer_full(Buffer *buf){
    // Verify if buffer size is not full
    if(buf->size == BUFFER_SIZE){
        return 1;
    } 
    return 0;
}
/* // Test routine
int main(){
    Buffer voltage;
	int buffer_handle = 0;


	for(uint16_t k = 0; k < BUFFER_SIZE+1; k++){
		buffer_handle = buffer_push(&voltage, k);
		if(buffer_handle >= 0){
			printf("Buffer [%d] = %d [Buffer size %d]\n", k, voltage.data[k], voltage.size);
		} else {
			printf("Error while inserting\n");
		}
	}
	buffer_clean(&voltage);
	printf("Buffer size: %d\n", voltage.size);
	for(uint16_t k = 0; k < BUFFER_SIZE; k++){
		printf("Buffer [%d] = %d\n", k, voltage.data[k]);
	}
}
*/