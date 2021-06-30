/*
 * MT_current_sense.c
 *
 * Created: 21/05/2021 18:41:34
 *  Author: mtorres
 */ 

/* File inclusion */
#include "current_sense.h"


/* Variable definition */
static float instant_current_amp = 0.0;
static float peak_current_amp = 0.0;
static float rms_current_amp = 0.0; 


/* Function definition */

void Current_Measure(void){
	
	uint16_t adc_bin_val;
	float adc_volt;
	float input_volt;
	float rectified_current;	 
	
	/* Select the proper channel */
	ADC_Seleccionar_Canal(CURRENT_MEASURE_ADC_CHANNEL);
	/* Start conversion */
    ADCSRA |= (1 << ADSC);
	while(!(ADCSRA & (1 << ADIF))){
		/* Wait until the conversion ends */
	}
	/* Clean the ADC flag */
	ADCSRA |= (1 << ADIF);	
	
	/* Measure voltage in the ADC input */
	adc_bin_val = ADC;
	adc_volt = (adc_bin_val * ADC_REF_VOLTAGE) / 1024.0;
	input_volt = ((adc_volt - 0.5) / CURRENT_SENSE_CIRCUIT_GAIN_02) + 0.5;
	instant_current_amp = (CURRENT_SENSE_CIRCUIT_GAIN_01 * (0.5 - input_volt)) / CURRENT_SENSE_RESISTOR_OHM;	
	instant_current_amp = (CURRENT_SENSE_SPAN * instant_current_amp) + CURRENT_SENSE_OFFSET;

	
	/* Rectify the instant current */
	if(instant_current_amp < 0.0){
		rectified_current = -instant_current_amp;
	}else{
		rectified_current = instant_current_amp;
	}
	
	/* Update the max instant current value (peak value) */
	if(rectified_current > peak_current_amp){
		peak_current_amp = rectified_current;
	}

	
}


void RMS_Current_Calculate(void){
	rms_current_amp = peak_current_amp * 0.7071;
}


void Peak_Current_Reset(void){
	peak_current_amp = 0.0;
}


float Instant_Current_Get(void){
	return instant_current_amp;
}


float Peak_Current_Get(void){
	return peak_current_amp;
}


float RMS_Current_Get(void){
	return rms_current_amp;
}

void Current_Clear(void){
	instant_current_amp = 0;
	peak_current_amp = 0;
	rms_current_amp = 0;
}