/*
 * low_power_horometer.c
 *
 * Created: 18/05/2021 09:17:42
 *  Author: mtorres
 */ 


/* File inclusion */
#include "low_power_horometer.h"


/* Variable defintion */
volatile uint16_t system_flags = 0;
static uint8_t system_mode = VIBRATION_SENSOR_ONLY_MODE;
static uint8_t stored_system_mode = VIBRATION_SENSOR_ONLY_MODE;
static uint32_t esp32_time_counter = 0;


/* Function definition */

void WDT_Off(void){
	
	cli();
	wdt_reset();
	MCUSR &= ~(1 << WDRF);
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = 0x00;
	//sei();
	
}

void System_Initialize(void){
	
	/* Turn-off all power supplies */
	Enabling_Switches_Initialize();
	All_PSU_Switches_Off();
	/* Initialize peripherals */
	LEDs_Initialize();
	Soft_RTC_Initialize();
	ADC_Initialize();
	ESP32_Comm_Interface_Initialize();
	Timer0_Initialize();
	Timer2_Initialize();
	Piezoelectric_Sensor_Initialize();
	Magnetic_Pickup_Initialize();
	/* Disable unused peripherals */
	Analog_Comparator_Disable();
	Timer0_Interrupt_Disable();
	Magnetic_Pickup_Disable();
}


void LEDs_Initialize(void){
	DDR_ACCEL_SENSING_LED |= (1 << ACCEL_SENSING_LED);
	PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
}


void Timer0_Initialize(void){
	
	Timer_8b_t my_timer;
	
	my_timer.clock = Timer_Clk_Disabled;
	my_timer.mode = Timer_8b_CTC_Mode;
	my_timer.OCRA = 99;
	my_timer.OCRB = 0;
	my_timer.OCA = OC_Disabled;
	my_timer.OCB = OC_Disabled;
	my_timer.interrupt_mask = Timer_Interrupts_Disabled;
	
	Timer0_Configurar(&my_timer);
}


void Timer2_Initialize(void){
	
	Async_Timer_8b_t my_timer;
	
	/* Disable interrupts */
	TIMSK2 &= ~((1<< OCIE2A) | (1 << OCIE2B) | (1 << TOIE2));
	/* Select asynchronous clock source (external crystal 32.768 kHz) */
	ASSR &= ~(1 << EXCLK);
	ASSR |= (1 << AS2);
	
	/* Configure Timer2 operation */
	my_timer.clock = Async_Timer_Clk_PS128;
	my_timer.mode = Timer_8b_CTC_Mode;
	my_timer.OCRA = 7; // Interrupt every 1/32 seconds (31.250 ms)
	my_timer.OCRB = 0;
	my_timer.OCA = OC_Disabled;
	my_timer.OCB = OC_Disabled;
	my_timer.interrupt_mask = Timer_Interrupts_Disabled;
	
	Timer2_Configurar(&my_timer);
	
	while(ASSR & (1 << TCN2UB)){
		/* Wait until the TCNT2 value has been loaded */
	}
	
	while(ASSR & (1 << OCR2AUB)){
		/* Wait until the OCR2A value has been loaded */
	}
	
	while(ASSR & (1 << OCR2BUB)){
		/* Wait until the OCR2B value has been loaded */
	}
	
	while(ASSR & (1 << TCR2AUB)){
		/* Wait until the TCCR2A value has been loaded */
	}
	
	while(ASSR & (1 << TCR2BUB)){
		/* Wait until the TCCR2B value has been loaded */
	}
	
	/* Clear the interrupt flags */
	TIFR2 |= (1 << OCF2B) | (1 << OCF2A) | (1 << TOV2);
	
	/* Enable Interrupt by comparing with OCRA */
	TIMSK2 |= (1 << OCIE2A);
	
}


void ADC_Initialize(void){
	
	ADC_t my_adc;

	my_adc.prescaler = ADC_Prescaler_2;
	my_adc.vref = Internal_1p1_Bandgap;
	my_adc.channel = VBAT_MEASURE_ADC_CHANNEL;
	my_adc.auto_trigger = ADC_Autotrigger_Disabled;
	my_adc.trigger_source = ADC_Free_Running_Mode;
	my_adc.interrupt_mask = ADC_Interrupt_Disabled;
	
	ADC_Configurar(&my_adc);
	
	/* Disable digital I/Os in the pins that will be used with the ADC (for power saving) */ 
	DIDR0 |= ((1 << CURRENT_MEASURE_ADC_CHANNEL) | (1 << VBAT_MEASURE_ADC_CHANNEL));
	
}

void ADC_Disable(void){
	ADCSRA &= ~(1 << ADEN);
}


void Analog_Comparator_Disable(void){
	ACSR |= (1 << ACD);
}


void Timer0_Interrupt_Enable(void){
	
	cli();
	TCNT0 = 0;
	TIFR0 |= (1 << OCF0A);
	TIMSK0 |= (1 << OCIE0A);
	TCCR0B |= (1 << CS00); 
	sei();
	
}

void Timer0_Interrupt_Disable(void){
	
	cli();
	TIMSK0 &= ~(1 << OCIE0A);
	TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
	TCNT0 = 0;
	TIFR0 |= (1 << OCF0A);
	sei();
	
}


void System_Mode_Save(void){
	stored_system_mode = system_mode;
}


void System_Mode_Load(void){
	system_mode = stored_system_mode;
}



void System_Sequence(void){
	
	uint8_t hora, minuto, segundo;
	uint8_t dia, mes, anio;
	
	char texto[30];	
	
	switch(system_mode){
		
		case VIBRATION_SENSOR_ONLY_MODE:
			Vibration_Sense_Only_Sequence();
			break;
			
		case VIBRATION_CURRENT_PICKUP_SENSOR_MODE:
			Vibration_Sense_Current_Sense_And_Motor_Speed_Sequence();
			break;
			
		case VIBRATION_SENSOR_CALIBRATION_MODE:
			Vibration_Sense_Calibration_Sequence();
		break;			
			
		default:
			break;				
	}
	
	if(system_flags & (1 << SERIAL_MSG_FLAG)){
		
		cli();
		system_flags &= ~(1 << SERIAL_MSG_FLAG);
		sei();
		
		sprintf(texto, "%d", system_mode);
		UARTn_Tx_String(UART0, texto);
		
	}
	
}


void Vibration_Sense_Calibration_Sequence(void){
	
	static uint8_t sequence_state = 0;
	
	switch(sequence_state){
	
	case 0:
	
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		/* Wait until the power supply sets up */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);
	
		/* Start with the LED off */
		PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
		
		sequence_state++;
	 	
		break;
		
	case 1:
	
		if(system_flags & (1 << ESP32_COMM_CHECK_FLAG)){
			
			cli();
			system_flags &= ~(1 << ESP32_COMM_CHECK_FLAG);
			sei();
			
			if(ESP32_Operation_Mode_And_Display_Update() == SEQUENCE_COMPLETE){
	
				cli();
				/* Clear the INT0 flag */
				EIFR |= (1 << INTF0);
				sei();
			
				/* Reset the calibration counter */
				Calib_Time_Reset();
			
				sequence_state++;		
			}		
		}
	
		break;
	
	case 2:	

		/* Ignore current sensor */
		if(system_flags & (1 << CURRENT_SENSE_FLAG)){
			cli();
			system_flags &= ~(1 << CURRENT_SENSE_FLAG);
			sei();
		}

		/* Read vibration sensor (piezoelectric) */
		if(system_flags & (1 << VIBRATION_SENSE_FLAG)){
			
			cli();
			system_flags &= ~(1 << VIBRATION_SENSE_FLAG);
			sei();
			
			Calib_Time_Count_Update();
		}

		/* Read DIP switch (ignore buttons) */
		if(system_flags & (1 << BUTTON_READ_FLAG)){
			
			cli();
			system_flags &= ~(1 << BUTTON_READ_FLAG);
			sei();
						
			if(!(g1_button_state & (1 << DIP_SW_CALIB_MODE))){
				sequence_state++;
			}
		}
		
		/* Measure the battery charge level */
		if(system_flags & (1 << ONE_SECOND_ELAPSED_FLAG)){
			
			cli();
			system_flags &= ~(1 << ONE_SECOND_ELAPSED_FLAG);
			sei();
			
			Battery_Level_Measure();
		}
		
		break;
	
	case 3:	
	
		/* Turn-off calibration LED */
		PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
		/* Go back to the previous mode */
		System_Mode_Load();
		
		sequence_state = 0;
		
		break;


	default:
	
		break;
	
	}

}


void Vibration_Sense_Only_Sequence(void){
	
	static uint8_t sequence_state = 0;	
	
	switch(sequence_state){
		
	case 0:
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		/* Wait until the power supply sets up and reset the calibration counter */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);
									
		sequence_state++;
			
		break;	

	case 1:
	
		if(system_flags & (1 << ESP32_COMM_CHECK_FLAG)){
		
			cli();
			system_flags &= ~(1 << ESP32_COMM_CHECK_FLAG);
			sei();
		
			if(ESP32_Operation_Mode_And_Display_Update() == SEQUENCE_COMPLETE){
			
				cli();
				/* Clear the INT0 flag */
				EIFR |= (1 << INTF0);
				sei();
						
				sequence_state++;
			}
		}
	
		break;

		
	case 2:

		/* Ignore current sensor */
		if(system_flags & (1 << CURRENT_SENSE_FLAG)){
			cli();
			system_flags &= ~(1 << CURRENT_SENSE_FLAG);
			sei();
		}
		
		/* Read vibration sensor (piezoelectric) */
		if(system_flags & (1 << VIBRATION_SENSE_FLAG)){
			
			cli();
			system_flags &= ~(1 << VIBRATION_SENSE_FLAG);
			sei();
			
			Working_Time_Count_Update();
		}

		/* Read DIP switch and buttons */
		if(system_flags & (1 << BUTTON_READ_FLAG)){
			
			cli();
			system_flags &= ~(1 << BUTTON_READ_FLAG);
			sei();
			
			if(!(system_flags & (1 << WIFI_COMM_EN_FLAG))){
			
				if(g1_button_state & (1 << DIP_SW_CALIB_MODE)){
					sequence_state = 2;
				}else if(G1_Get_Button_Press(1 << MODE_BUTTON)){
					sequence_state = 3;
				}else if(G1_Get_Button_Press(1 << WIFI_BUTTON)){
					cli();
					system_flags |= (1 << WIFI_COMM_EN_FLAG);
					sei();
				}else{
					// Does nothing
				}
			
			}else{
				Wifi_Connection_Sequence();
			}
			
		}
		
		/* Measure the battery charge level */
		if(system_flags & (1 << ONE_SECOND_ELAPSED_FLAG)){
			
			cli();
			system_flags &= ~(1 << ONE_SECOND_ELAPSED_FLAG);
			sei();
			
			Battery_Level_Measure();
		}	
	
		break;
		
	case 3:
	
		/* Save current state and go to testing mode */
		System_Mode_Save();
		system_mode = VIBRATION_SENSOR_CALIBRATION_MODE;	
		sequence_state = 0;	
	
		break;	
		
	case 4:
	
		/* Go to Vibration + Current Mode */
		system_mode = VIBRATION_CURRENT_PICKUP_SENSOR_MODE;	
		sequence_state = 0;
	
		break;
	
	default:
	
		break;	
		
	}
	
}




void Vibration_Sense_Current_Sense_And_Motor_Speed_Sequence(void){
	
	static uint8_t sequence_state = 0;
	static uint8_t current_sense_sample_counter = 0;
	
	switch(sequence_state){
		
	case 0:
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		Current_Sense_Circuit_PSU_On();
		Magnetic_Pickup_Circuit_PSU_On();
		/* Wait until the power supply sets up and reset the calibration counter */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);
	
		sequence_state++;
		
		break;


	case 1:
	
	if(system_flags & (1 << ESP32_COMM_CHECK_FLAG)){
		
		cli();
		system_flags &= ~(1 << ESP32_COMM_CHECK_FLAG);
		sei();
		
		if(ESP32_Operation_Mode_And_Display_Update() == SEQUENCE_COMPLETE){
			
			cli();
			/* Clear the INT0 flag */
			EIFR |= (1 << INTF0);
			/* Prepare for current measurement */
			Timer0_Interrupt_Enable();
			current_sense_sample_counter = 0;
			/* Prepare to measure motor speed */
			Magnetic_Pickup_Enable();
			sei();
			
			current_sense_sample_counter = 0;
			sequence_state++;
		}
	}
	
	break;

		
	case 2:

		/* Read current sensor */
		if(system_flags & (1 << CURRENT_SENSE_FLAG)){
			cli();
			system_flags &= ~(1 << CURRENT_SENSE_FLAG);
			sei();
			
			Current_Measure();
			current_sense_sample_counter++;
			if(current_sense_sample_counter >= CURRENT_RMS_CALC_N_SAMPLES){
				current_sense_sample_counter = 0;
				RMS_Current_Calculate();
				Peak_Current_Reset();
			}
		}
		
		/* Read vibration sensor (piezoelectric) */
		if(system_flags & (1 << VIBRATION_SENSE_FLAG)){
			
			cli();
			system_flags &= ~(1 << VIBRATION_SENSE_FLAG);
			sei();
			
			Working_Time_Count_Update();
		}

		/* Read DIP switch and buttons */
		if(system_flags & (1 << BUTTON_READ_FLAG)){
			
			cli();
			system_flags &= ~(1 << BUTTON_READ_FLAG);
			sei();
			
			if(!(system_flags & (1 << WIFI_COMM_EN_FLAG))){
				
				if(g1_button_state & (1 << DIP_SW_CALIB_MODE)){
					sequence_state = 2;
				}else if(G1_Get_Button_Press(1 << MODE_BUTTON)){
					sequence_state = 3;
				}else if(G1_Get_Button_Press(1 << WIFI_BUTTON)){
					cli();
					system_flags |= (1 << WIFI_COMM_EN_FLAG);
					sei();
				}else{
					// Does nothing
				}
				
			}else{
				Wifi_Connection_Sequence();
			}
			
		}
		
		/* Measure the battery charge level */
		if(system_flags & (1 << ONE_SECOND_ELAPSED_FLAG)){
			
			cli();
			system_flags &= ~(1 << ONE_SECOND_ELAPSED_FLAG);
			sei();
			
			Battery_Level_Measure();
		}
		
		break;
		
	case 3:
	
		/* Disable current measurement and magnetic pick-up circuit */
		cli();
		Timer0_Interrupt_Disable();
		Magnetic_Pickup_Disable();
		sei();		
		Current_Sense_Circuit_PSU_Off();
		Magnetic_Pickup_Circuit_PSU_Off();		
		
		/* Save current state and go to testing mode */
		System_Mode_Save();
		system_mode = VIBRATION_SENSOR_CALIBRATION_MODE;
		sequence_state = 0;
		
		break;
		
	case 4:
		
		/* Disable current measurement and magnetic pick-up circuit */
		cli();
		Timer0_Interrupt_Disable();
		Magnetic_Pickup_Disable();
		sei();		
		Current_Sense_Circuit_PSU_Off();
		Magnetic_Pickup_Circuit_PSU_Off();		
		
		/* Save current state and go to Vibration Only mode */
		system_mode = VIBRATION_SENSOR_ONLY_MODE;
		sequence_state = 0;
		
		break;
		
	default:
		
		break;
		
	}	
	
}


void Wifi_Connection_Sequence(void){
	cli();
	system_flags &= ~(1 << WIFI_COMM_EN_FLAG);
	sei();
}


uint8_t ESP32_Operation_Mode_And_Display_Update(void){
	
	static uint8_t seq_state = 0;
	uint8_t result = SEQUENCE_IN_PROCESS;
	uint8_t temp;
	
	switch(seq_state){
		
	case 0:
		ESP32_Buffer_Operation_Mode_Set(system_mode);
		seq_state++;
		break;		
		
	case 1:
		temp = ESP32_Turn_On();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}
		break;
		
	case 2:
		temp = ESP32_Operation_Mode_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state = 0;
			result = SEQUENCE_COMPLETE;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 0;
			result = SEQUENCE_COMPLETE;
		}else{
			//Does nothing
		}
		break;	
	
	default:
		break;
		
	}

	return result;
}