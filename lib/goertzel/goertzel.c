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
int goertzel (Buffer *buf, GoertzelState *goertz, uint16_t target_freq, uint16_t sample_rate){
    /*
     * k = (int) [ 0.5 + ( samples*target_freq/sample_rate ) ]
     * omega = (2*PI/samples) * k
     * coeff = 2*cos(omega)
    */

	// # Calculate k constant
	uint16_t k = (unsigned int) (0.5 + (buf->size*target_freq/sample_rate));


	// # Calculate omega
	float w = (k * 2.0 * PI_VALUE) / (buf->size);	// Omega

	// # Goertzel coeffiencts
	float c_real = cosf(w);		// Real coeff
	float c_img =  sinf(w);		// Img coeff
	float coeff = 2 * c_real;	// Pre-compute coeff;


	// # Init goertzel state variables;
	float y = 0;
	float y_1 = 0;
	float y_2 = 0;

	// # Process samples
	for (uint16_t n = 0; n < buf->size; n++){
		y = buf->data[n] + coeff*y_1 - y_2;
		y_2 = y_1;
		y_1 = y;
	}



	// # Calculate real, imaginary and amplitude
	goertz->DFT_r = (y_1 - y_2 * c_real) / (buf->size / 2);
	goertz->DFT_i = (y_2 * c_img)  / (buf->size / 2);
	goertz->DFT_m = fast_sqrt(goertz->DFT_r*goertz->DFT_r + goertz->DFT_i*goertz->DFT_i);

	// # Calculate the angle <!>
	if(goertz->DFT_r > 0) {
		goertz->DFT_arg = atanf(goertz->DFT_i / goertz->DFT_r);
	} else if(goertz->DFT_r < 0) {
		goertz->DFT_arg = PI_VALUE + atanf(goertz->DFT_i/goertz->DFT_r);
	} else if(goertz->DFT_r == 0 && goertz->DFT_i > 0) {
		goertz->DFT_arg = PI_2;
	} else if(goertz->DFT_r == 0 && goertz->DFT_i < 0) {
		goertz->DFT_arg = -PI_2;
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

	return 1;
}