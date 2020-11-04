// Include header
#include "goertzel.h"          


float fast_sqrt(float x){
	float halfnumber = 0.5f * x;
	int i = *(int*) &x;
	i = 0x5F376908 - (i>>1);
	x = *(float*)&i;
	x = x*(1.5008789f - halfnumber*x*x);
	x = x*(1.5000006f - halfnumber*x*x);
	return 1/x ;
}

// # Compute goertzel algorithm for desired frequency
uint8_t goertzel (Buffer *buf, GoertzelState *goertz, uint16_t target_freq, uint16_t sample_rate, uint8_t config){
    /*
		buf = buffer com as amostras;
		goertz = variável onde será armazenado o estado calculado
		target_freq = frequencia desejada
		sample_rate = frequencia de amostragem do sinal
		config = configuração do cálculo, onde:
			- bit0 = habilita hanning window;
			- bit1 = habilita cálculo completo da DFT
    */

   	// # Declara variáveis;
	uint16_t k;
	uint8_t enable_hanning = config & 0x1;
	uint8_t enable_full_dft = config > 0x1;
	float w, c_real = 0, c_img = 0, coeff = 0;
	float hann_const = 1;
	float scale_factor = buf->size / 4;

	// # Calculate k constant
	k = (unsigned int) floor((0.5 + (buf->size*target_freq/sample_rate)));


	// # Calculate omega
	w = (float) (k * 2.0 * M_PI) / (buf->size);	// Omega

	// # Goertzel coeffiencts
	c_real = cosf(w);		// Real coeff
	coeff = 2 * c_real;		// Pre-compute coeff;

	// Verifica se o cálculo completo da DFT está ativo
	if(enable_full_dft){ 
		c_img =  sinf(w);		// Img coeff
	}
	// Cálcula a constante de Hanning para utilizar aplicar no sinal
	if(enable_hanning){ 
		hann_const =  2 * M_PI / (buf->size-1);		// Compute hanning constant
		printf(" # Hanning window enabled");
	} 


	// # Init goertzel state variables;
	float y = 0;
	float y_1 = 0;
	float y_2 = 0;
	
	// # Process samples
	for (uint16_t n = 0; n < buf->size; n++){
		if(enable_hanning)
			y = (float) buf->data[n] * (1 - cosf(hann_const*n)) / 2;
		else	
			y = (float) buf->data[n];
		y += coeff*y_1 - y_2;
		y_2 = y_1;
		y_1 = y;
	}


	if(enable_full_dft){ 
		// # Calculate real, imaginary and amplitude
		goertz->DFT_r = (y_1 * c_real - y_2);
		goertz->DFT_i = (y_1 * c_img);
		goertz->DFT_m = fast_sqrt(goertz->DFT_r*goertz->DFT_r + goertz->DFT_i*goertz->DFT_i)  / (scale_factor);

		// # Calculate the angle <!>
		if(goertz->DFT_r > 0) {
			goertz->DFT_arg = atanf(goertz->DFT_i / goertz->DFT_r);
		} else if(goertz->DFT_r < 0) {
			goertz->DFT_arg = M_PI + atanf(goertz->DFT_i/goertz->DFT_r);
		} else if(goertz->DFT_r == 0 && goertz->DFT_i > 0) {
			goertz->DFT_arg = M_PI_2;
		} else if(goertz->DFT_r == 0 && goertz->DFT_i < 0) {
			goertz->DFT_arg = -M_PI_2;
		}
	} else {
		goertz->DFT_r = 0;
		goertz->DFT_i = 0;
		goertz->DFT_m = fast_sqrt(y_1*y_1 + y_2*y_2 - y_1*y_2*coeff) / scale_factor;
		goertz->DFT_arg = 0;
	}

	// # DEBUG
	//printf("- Sample rate: %d\n", sample_rate);
	//printf("- Target freq: %d\n", target_freq);
	//printf("- Num samples: %d\n", buf->size);
	//printf("- k %d\n", k);
	//printf("- Omega %f\n", w);
	//printf("- c_real %f\n", c_real);
	//printf("- c_img %f\n", c_img);
	//printf("- y %f\n", y);
	//printf("- y-1 %f\n", y_1);
	printf(" - DFT_r: %f\n", goertz->DFT_r);
	printf(" - DFT_i: %f\n", goertz->DFT_i);
	printf(" - DFT_m: %f\n", goertz->DFT_m);
	printf(" - DFT_arg: %f\n\n", goertz->DFT_arg);

	return 1;
}