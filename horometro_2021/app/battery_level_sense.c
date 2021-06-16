/*
 * battery_level_sense.c
 *
 * Created: 15/06/2021 14:50:13
 *  Author: mtorres
 */ 

/* File inclusion */
#include "battery_level_sense.h"

/* Variable definition */
static uint8_t battery_charge_level = BATTERY_LEVEL_HIGH;


/* Function definition */

void Battery_Level_Measure(void){
	
	uint16_t adc_bin_val;
	float	battery_volt;
	float battery_percent;
	
	/* Select the proper channel */
	ADC_Seleccionar_Canal(VBAT_MEASURE_ADC_CHANNEL);
	/* Start conversion */
	ADCSRA |= (1 << ADSC);
	while(!(ADCSRA & (1 << ADIF))){
		/* Wait until the conversion ends */
	}
	/* Clean the ADC flag */
	ADCSRA |= (1 << ADIF);
	
	/* Measure voltage in the ADC input */
	adc_bin_val = ADC;
	battery_volt = BATTERY_VSENSE_GAIN	* ((adc_bin_val * ADC_REF_VOLTAGE) / 1024.0);
	
	/* Calculate the percentage of the battery that has been charged */
	battery_percent = 100.0 * (battery_volt - BATT_MIN_OUT_V) / (BATT_MAX_OUT_V - BATT_MIN_OUT_V);	
	
	if(battery_percent > 100.0){
		battery_percent = 99.0;
	}else if(battery_percent < 0.0){
		battery_percent = 0.0;
	}else{
		// Does nothing
	}	
	

	if(battery_percent >= BATT_MED_TO_HIGH_LEVEL_PERCENT_THRESHOLD){
		battery_charge_level = BATTERY_LEVEL_HIGH;
	}else if((battery_percent < BATT_HIGH_TO_MED_LEVEL_PERCENT_THRESHOLD) && (battery_percent > BATT_LOW_TO_MED_LEVEL_PERCENT_THRESHOLD)){
		battery_charge_level = BATTERY_LEVEL_MED;
	}else if(battery_percent <= BATT_MED_TO_LOW_LEVEL_PERCENT_THRESHOLD){
		battery_charge_level = BATTERY_LEVEL_LOW;
	}else{
		// Does nothing
	}
	
}


uint8_t Battery_Level_Get(void){
	return battery_charge_level;
}
