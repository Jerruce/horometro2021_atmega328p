/*
 * low_power_horometer.c
 *
 * Created: 18/05/2021 09:17:42
 *  Author: mtorres
 */ 


/* File inclusion */
#include "low_power_horometer.h"


/* Variable defintion */

volatile uint32_t system_flags = 0;
volatile uint16_t parameter_status_flag = 0;
volatile uint8_t alarm_status_flags= 0;

static uint8_t previous_alarm_flags = 0;
static uint8_t new_alarm_flags = 0;
static uint8_t alarm_change_flags = 0;
static uint8_t alarm_event_flags = 0; 
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
	uint32_t calib_counter;
	
	uint32_t motor_hours;
	uint8_t motor_min, motor_sec;
	
	uint32_t al1_hours;
	uint8_t al1_min, al1_sec;
	
	uint32_t al2_hours;
	uint8_t al2_min, al2_sec;
	
	uint32_t al3_hours;
	uint8_t al3_min, al3_sec;	
	
	float corriente;
	uint32_t corriente_ent;
	
	uint16_t frecuencia;
	
	uint16_t  my_param_status;
	
	char texto[50];	
	
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
	
	if(system_flags & ((uint32_t)1 << SERIAL_MSG_FLAG)){
		
		cli();
		system_flags &= ~((uint32_t)1 << SERIAL_MSG_FLAG);
		sei();
		
		cli();
		Soft_RTC1_Get_Date(&dia, &mes, &anio);
		sei();
		sprintf(texto, "Fecha:%02d/%02d/%02d\r\n", dia, mes, anio);		
		UARTn_Tx_String(UART0, texto);
		
		cli();
		Soft_RTC1_Get_Time(&hora, &minuto, &segundo);
		sei();
		sprintf(texto, "Hora:%02d:%02d:%02d\r\n", hora, minuto, segundo);
		UARTn_Tx_String(UART0, texto);
		
		sprintf(texto, "Modo:%d\r\n", system_mode);
		UARTn_Tx_String(UART0, texto);
		
		sprintf(texto, "Calib t:%u\r\n", Calib_Time_Get());
		UARTn_Tx_String(UART0, texto);		
		
		Working_Time_Get(&motor_hours, &motor_min, &motor_sec);
		sprintf(texto, "Motor t:%u\r\n", motor_sec);
		UARTn_Tx_String(UART0, texto);		
		
		Alarm1_Time_Get(&al1_hours, &al1_min, &al1_sec);
		sprintf(texto, "AL1 t:%u\r\n", al1_sec);
		UARTn_Tx_String(UART0, texto);	
		
		Alarm2_Time_Get(&al2_hours, &al2_min, &al2_sec);
		sprintf(texto, "AL2 t:%u\r\n", al2_sec);
		UARTn_Tx_String(UART0, texto);	
		
		Alarm3_Time_Get(&al3_hours, &al3_min, &al3_sec);
		sprintf(texto, "AL3 t:%u\r\n", al3_sec);
		UARTn_Tx_String(UART0, texto);				
		
		corriente = RMS_Current_Get();
		corriente_ent = (uint32_t)corriente;
		sprintf(texto, "I(mA):%d\r\n", corriente_ent);
		UARTn_Tx_String(UART0, texto);
		
		frecuencia = Magnetic_Pickup_Get_Freq_Hz();
		sprintf(texto, "Freq(Hz): %d\r\n", frecuencia);
		UARTn_Tx_String(UART0, texto);
		
		//my_param_status = ESP32_Buffer_Parameters_Status_Get();
		sprintf(texto, "param_status: 0x%04x\r\n", parameter_status_flag);
		UARTn_Tx_String(UART0, texto);
		
		UARTn_Tx_Byte(UART0, '\n');
		
	}
	
}


void Vibration_Sense_Calibration_Sequence(void){
	
	static uint8_t sequence_state = 0;
	static uint8_t measure_enable = 0;

	if(measure_enable){

		/* Ignore current sensor */
		if(system_flags & ((uint32_t)1 << CURRENT_SENSE_FLAG)){
			cli();
			system_flags &= ~((uint32_t)1 << CURRENT_SENSE_FLAG);
			sei();
		}

		/* Read vibration sensor (piezoelectric) */
		if(system_flags & ((uint32_t)1 << VIBRATION_SENSE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << VIBRATION_SENSE_FLAG);
			sei();
			
			if(Piezoelectric_Sensor_Read()){
				PORT_ACCEL_SENSING_LED |= (1 << ACCEL_SENSING_LED);
				Calib_Time_Count_Update();	
			}else{
				PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
			}	
			
		}		
		
		
		/* Measure the battery charge level */
		if(system_flags & ((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG);
			sei();
		
			Battery_Level_Measure();
		}	
		
		
		/* Read DIP switch (ignore buttons) */
		if(system_flags & ((uint32_t)1 << BUTTON_READ_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BUTTON_READ_FLAG);
			sei();
		
			if(!(g1_button_state & (1 << DIP_SW_CALIB_MODE))){
				cli();
				system_flags |= ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
				sei();
			}
		}		
			
		
	}


	switch(sequence_state){
	
	case 0:
	
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		/* Wait until the power supply sets up */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);
	
		Magnetic_Pickup_Clear_Freq();
	
		/* Start with the LED off */
		PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
		
		cli();
		/* Clear the INT0 flag */
		EIFR |= (1 << INTF0);
		system_flags |= ((uint32_t)1 << SHOW_CALIBRATION_SCREEN_FLAG);
		sei();
				
		/* Reset the calibration counter */
		Calib_Time_Reset();		
		
		sequence_state++;
		measure_enable = 1;
	 	
		break;
			
	case 1:	

		if(system_flags & ((uint32_t)1 << SHOW_CALIBRATION_SCREEN_FLAG)){
			
			sequence_state = 3;
			
		}else if(system_flags & ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG)){	
			
			sequence_state++;
		}
		
		break;
	
	case 2:	

		/* Clean buttons */
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);
	
		/* Turn-off calibration LED */
		PORT_ACCEL_SENSING_LED &= ~(1 << ACCEL_SENSING_LED);
		/* Go back to the previous mode */
		System_Mode_Load();
		cli();
		system_flags &= ~((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
		sei();
		sequence_state = 0;
		measure_enable = 0;
		
		break;


	case 3:
	
		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
				
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();
	
			if(ESP32_Calibration_Screen_Display_Update() == SEQUENCE_COMPLETE){
				/* Print the alarm screen for 10 seconds (and clear the flag in the end) */
				cli();
				system_flags &= ~((uint32_t)1 << SHOW_CALIBRATION_SCREEN_FLAG);
				sei();
				/* Go back to the normal mode */
				sequence_state = 1;
			}
		
		}
	
		break;

	default:
	
		break;
	
	}
	
}


void Vibration_Sense_Only_Sequence(void){
	
	static uint8_t sequence_state = 0;	
	static uint8_t measure_enable = 0;


	if(measure_enable){
		
		/* Ignore current sensor */
		if(system_flags & ((uint32_t)1 << CURRENT_SENSE_FLAG)){
			cli();
			system_flags &= ~((uint32_t)1 << CURRENT_SENSE_FLAG);
			sei();
		}
	
		/* Read vibration sensor (piezoelectric) */
		if(system_flags & ((uint32_t)1 << VIBRATION_SENSE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << VIBRATION_SENSE_FLAG);
			sei();
		
			if(Piezoelectric_Sensor_Read()){
				
				Working_Time_Count_Update();
				if(alarm_status_flags & (1 << E1_FLAG)){
					Alarm1_Time_Count_Update();
				}
				if(alarm_status_flags & (1 << E2_FLAG)){
					Alarm2_Time_Count_Update();
				}
				if(alarm_status_flags & (1 << E3_FLAG)){
					Alarm3_Time_Count_Update();
				}
				
			}		
				
		}
	
	
		/* Measure the battery charge level */
		if(system_flags & ((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG);
			sei();
		
			Battery_Level_Measure();
		}	
	
		Check_For_Alarm_Events();
		if(alarm_event_flags){
			cli();			
			system_flags &= ~((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG);
			system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);			
			sei();
		}		
		
		
		/* Read DIP switch and buttons */
		if(system_flags & ((uint32_t)1 << BUTTON_READ_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BUTTON_READ_FLAG);
			sei();
		
			if(!(system_flags & ((uint32_t)1 << WIFI_COMM_EN_FLAG))){
			
				if(g1_button_state & (1 << DIP_SW_CALIB_MODE)){
					cli();
					system_flags |= ((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG);
					sei();
				}else if(G1_Get_Button_Long(1 << MODE_BUTTON)){
					cli();
					system_flags |= ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
					sei();
				}else if(G1_Get_Button_Short(1 << MODE_BUTTON)){
					cli();
					system_flags ^= ((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG);
					system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
					sei();					
				}else if(G1_Get_Button_Press(1 << WIFI_BUTTON)){
					cli();
					system_flags |= ((uint32_t)1 << WIFI_COMM_EN_FLAG);
					sei();
				}else{
					// Does nothing
				}
			
			}
		
		}
		
	}


	switch(sequence_state){
		
	case 0:
	
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		/* Wait until the power supply sets up and reset the calibration counter */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);

		Magnetic_Pickup_Clear_Freq();

		cli();
		/* Clear the INT0 flag */
		EIFR |= (1 << INTF0);
		system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
		sei();

		measure_enable = 1;							
		sequence_state++;
			
		break;	
		
	case 1:
	
		if(system_flags & ((uint32_t)1 << WIFI_COMM_EN_FLAG)){
			
			sequence_state = 6;
			
		}else if(system_flags & ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG)){
			
			if(system_flags & ((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG)){
				sequence_state = 5;
			}else{
				sequence_state = 4;			
			}
			
		}else if(system_flags & ((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG)){
			sequence_state++;
		}else if(system_flags & ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG)){
			sequence_state = 3;
		}else{
			//Does nothing
		}
	
		break;
		
	case 2:

		/* Clean buttons */
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);	
	
		/* Save current state and go to testing mode */
		System_Mode_Save();
		system_mode = VIBRATION_SENSOR_CALIBRATION_MODE;	
		cli();
		system_flags &= ~((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG);
		sei();		
		sequence_state = 0;	
		measure_enable = 0;
	
		break;	
		
	case 3:
	
		/* Clean buttons */
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);
	
		/* Go to Vibration + Current Mode */
		system_mode = VIBRATION_CURRENT_PICKUP_SENSOR_MODE;		
		cli();
		system_flags &= ~((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
		sei();		
		sequence_state = 0;
		measure_enable = 0;
	
		break;
		
	case 4:
	
		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
			
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();			
			
			/* Print the main screen */
			if(ESP32_Main_Screen_Display_Update() == SEQUENCE_COMPLETE){
				/* Go back to the normal mode */
				cli();
				system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
				sei();
				sequence_state = 1;			
			}
		}
	
		break;
		
	case 5:
	
		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
			
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();			
			
			/* Print the alarm screen */
			if(ESP32_Alarm_Screen_Display_Update() == SEQUENCE_COMPLETE){
				cli();
				system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
				sei();
				/* Go back to the normal mode */
				sequence_state = 1;
			}
	
		}
		
		break;	
				
	case 6:

		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
			
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();
			
			/* Print the alarm screen */
			if(Wifi_Connection_Sequence() == SEQUENCE_COMPLETE){
				
				cli();
				system_flags &= ~((uint32_t)1 << WIFI_COMM_EN_FLAG);
				sei();
				
				/* Go back to the normal mode */
				sequence_state = 1;
		
			}
			
		}
		
		break;	
				
	default:
		break;	
		
	}
		
}




void Vibration_Sense_Current_Sense_And_Motor_Speed_Sequence(void){
	
	static uint8_t sequence_state = 0;
	static uint8_t current_sense_sample_counter = 0;
	float new_current;
	uint16_t new_freq_rpm;
	static uint8_t measure_enable = 0;


	if(measure_enable){
		
		/* Read current sensor */
		if(system_flags & ((uint32_t)1 << CURRENT_SENSE_FLAG)){
			
			cli();
			system_flags &= ~((uint32_t)1 << CURRENT_SENSE_FLAG);
			sei();
			
			Current_Measure();
			current_sense_sample_counter++;
			if(current_sense_sample_counter >= CURRENT_RMS_CALC_N_SAMPLES){
				current_sense_sample_counter = 0;
				RMS_Current_Calculate();
				Peak_Current_Reset();
			
				new_freq_rpm = Magnetic_Pickup_Get_Freq_RPM();
				new_current = RMS_Current_Get();
			
				if(new_current > OVERCURRENT_UPPER_THRESHOLD){
					system_flags |= ((uint32_t)1 << OVERCURRENT_ALARM_FLAG);
				}else if(new_current < OVERCURRENT_LOWER_THRESHOLD){
					system_flags &= ~((uint32_t)1 << OVERCURRENT_ALARM_FLAG);
				}else{
					//Does nothing
				}
			
				if(system_flags & ((uint32_t)1 << MAG_PICKUP_TIMEOUT_FLAG)){
				
					if(new_current > MOTOR_STUCK_CURRENT_UPPER_THRESHOLD){
						cli();
						system_flags |= ((uint32_t)1 << MOTOR_STUCK_ALARM_FLAG);
						sei();
					}else if(new_current < MOTOR_STUCK_CURRENT_LOWER_THRESHOLD){
						cli();
						system_flags &= ~((uint32_t)1 << MOTOR_STUCK_ALARM_FLAG);
						sei();
					}else{
						//Does nothing
					}
				
				}else{
					cli();
					system_flags &= ~((uint32_t)1 << MOTOR_STUCK_ALARM_FLAG);
					sei();
				}
			}
		}
	
		/* Read vibration sensor (piezoelectric) */
		if(system_flags & ((uint32_t)1 << VIBRATION_SENSE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << VIBRATION_SENSE_FLAG);
			sei();
		
			if(Piezoelectric_Sensor_Read()){
				
				Working_Time_Count_Update();
				if(alarm_status_flags & (1 << E1_FLAG)){
					Alarm1_Time_Count_Update();
				}
				if(alarm_status_flags & (1 << E2_FLAG)){
					Alarm2_Time_Count_Update();
				}
				if(alarm_status_flags & (1 << E3_FLAG)){
					Alarm3_Time_Count_Update();
				}
				
			}

		}
		
		/* Measure the battery charge level */
		if(system_flags & ((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BATTERY_LEVEL_MEASURE_FLAG);
			sei();
		
			Battery_Level_Measure();
		}		
	
		Check_For_Alarm_Events();
		if(alarm_event_flags){
			system_flags &= ~((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG);
			system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
		}		
		

		/* Read DIP switch and buttons */
		if(system_flags & ((uint32_t)1 << BUTTON_READ_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << BUTTON_READ_FLAG);
			sei();
		
			if(!(system_flags & ((uint32_t)1 << WIFI_COMM_EN_FLAG))){
			
				if(g1_button_state & (1 << DIP_SW_CALIB_MODE)){
					cli();
					system_flags |= ((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG);
					sei();
				}else if(G1_Get_Button_Long(1 << MODE_BUTTON)){
					cli();
					system_flags |= ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
					sei();
				}else if(G1_Get_Button_Short(1 << MODE_BUTTON)){
					cli();
					system_flags ^= ((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG);
					system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
					sei();					
				}else if(G1_Get_Button_Press(1 << WIFI_BUTTON)){
					cli();
					system_flags |= ((uint32_t)1 << WIFI_COMM_EN_FLAG);
					sei();
				}else{
					// Does nothing
				}
			
			}
		
		}

	}

	
	switch(sequence_state){
		
	case 0:
	
		/* Enable the vibration sensor */
		General_Power_Supply_Circuit_On();
		Piezoelectric_Circuit_PSU_On();
		Current_Sense_Circuit_PSU_On();
		Magnetic_Pickup_Circuit_PSU_On();
		/* Wait until the power supply sets up and reset the calibration counter */
		_delay_ms(PSU_SW_WAIT_PERIOD_MS);
	
		cli();
		/* Clear the INT0 flag */
		EIFR |= (1 << INTF0);
		/* Prepare for current measurement */
		Timer0_Interrupt_Enable();
		current_sense_sample_counter = 0;
		/* Prepare to measure motor speed */
		Magnetic_Pickup_Enable();
		system_flags |= ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
		sei();	
	
		sequence_state++;
		measure_enable = 1;
		
		break;

	case 1:
	
		if(system_flags & ((uint32_t)1 << WIFI_COMM_EN_FLAG)){
			
			sequence_state = 6;
			
		}else if(system_flags & ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG)){
			if(system_flags & ((uint32_t)1 << TOGGLE_SCREEN_INDEX_FLAG)){
				sequence_state = 5;
			}else{
				sequence_state = 4;	
			}
		}else if(system_flags & ((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG)){
			sequence_state++;
		}else if(system_flags & ((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG)){
			sequence_state = 3;
		}else{
			//Does nothing
		}
		
		break;
		
	case 2:
	
		/* Clean buttons */
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);
	
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
		cli();
		system_flags &= ~((uint32_t)1 << CHANGE_TO_CALIB_MODE_FLAG);
		sei();		
		sequence_state = 0;
		measure_enable = 0;
		current_sense_sample_counter = 0;
		
		break;
		
	case 3:
		
		/* Clean buttons */
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);
		
		/* Disable current measurement and magnetic pick-up circuit */
		cli();
		Timer0_Interrupt_Disable();
		Magnetic_Pickup_Disable();
		sei();		
		Current_Sense_Circuit_PSU_Off();
		Magnetic_Pickup_Circuit_PSU_Off();		
		
		/* Save current state and go to Vibration Only mode */
		system_mode = VIBRATION_SENSOR_ONLY_MODE;
		cli();
		system_flags &= ~((uint32_t)1 << CHANGE_OPERATION_MODE_FLAG);
		sei();		
		sequence_state = 0;
		measure_enable = 0;
		current_sense_sample_counter = 0;
		
		break;
	
	case 4:
	
		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();
		
			/* Print the main screen */
			if(ESP32_Main_Screen_Display_Update() == SEQUENCE_COMPLETE){

				cli();
				system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
				sei();
				/* Go back to the normal mode */
				sequence_state = 1;
			}
		}
	
		break;
	
	case 5:
	
		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();
		
			/* Print the alarm screen */
			if(ESP32_Alarm_Screen_Display_Update() == SEQUENCE_COMPLETE){

				cli();
				system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
				sei();			
				/* Go back to the normal mode */
				sequence_state = 1;
			}
		
		}
	
		break;

	case 6:

		if(system_flags & ((uint32_t)1 << ESP32_COMM_CHECK_FLAG)){
		
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_COMM_CHECK_FLAG);
			sei();
		
			/* Print the alarm screen */
			if(Wifi_Connection_Sequence() == SEQUENCE_COMPLETE){
			
				cli();
				system_flags &= ~((uint32_t)1 << WIFI_COMM_EN_FLAG);
				sei();
			
				/* Go back to the normal mode */
				sequence_state = 1;
			}
		
		}
	
		break;
	
		
	default:
		
		break;
		
	}	
	
}


void Check_For_Alarm_Events(void){
	
	previous_alarm_flags = new_alarm_flags;
	
	if(system_flags & ((uint32_t)1 << ALARM_01_REACHED_FLAG)){
		alarm_status_flags |= (1 << AL1_FLAG);
		new_alarm_flags |= (1 << PROG_ALARM1_EVENT_FLAG);
	}else{
		alarm_status_flags &= ~(1 << AL1_FLAG);
		new_alarm_flags &= ~(1 << PROG_ALARM1_EVENT_FLAG);
		previous_alarm_flags &= ~(1 << PROG_ALARM1_EVENT_FLAG);	
	}

	if(system_flags & ((uint32_t)1 << ALARM_02_REACHED_FLAG)){
		alarm_status_flags |= (1 << AL2_FLAG);
		new_alarm_flags |= (1 << PROG_ALARM2_EVENT_FLAG);
	}else{
		alarm_status_flags &= ~(1 << AL2_FLAG);
		new_alarm_flags &= ~(1 << PROG_ALARM2_EVENT_FLAG);
		previous_alarm_flags &= ~(1 << PROG_ALARM2_EVENT_FLAG);		
	}
	
	if(system_flags & ((uint32_t)1 << ALARM_03_REACHED_FLAG)){
		alarm_status_flags |= (1 << AL3_FLAG);
		new_alarm_flags |= (1 << PROG_ALARM3_EVENT_FLAG);
	}else{
		alarm_status_flags &= ~(1 << AL3_FLAG);
		new_alarm_flags &= ~(1 << PROG_ALARM3_EVENT_FLAG);
		previous_alarm_flags &= ~(1 << PROG_ALARM3_EVENT_FLAG);	
	}	
	
	
	if(system_flags & ((uint32_t)1 << MOTOR_STUCK_ALARM_FLAG)){
		alarm_status_flags |= (1 << ALS_FLAG);
		new_alarm_flags |= (1 << MOTOR_STUCK_ALARM_EVENT_FLAG);
	}else{
		alarm_status_flags &= ~(1 << ALS_FLAG);
		new_alarm_flags &= ~(1 << MOTOR_STUCK_ALARM_EVENT_FLAG);
		previous_alarm_flags &= ~(1 << MOTOR_STUCK_ALARM_EVENT_FLAG);
	}


	if(system_flags & ((uint32_t)1 << OVERCURRENT_ALARM_FLAG)){
		alarm_status_flags |= (1 << ALO_FLAG);
		new_alarm_flags |= (1 << MOTOR_OVERCURRENT_ALARM_EVENT_FLAG);
	}else{
		alarm_status_flags &= ~(1 << ALO_FLAG);
		new_alarm_flags &= ~(1 << MOTOR_OVERCURRENT_ALARM_EVENT_FLAG);
		previous_alarm_flags &= ~(1 << MOTOR_OVERCURRENT_ALARM_EVENT_FLAG);
	}	
	
	alarm_change_flags = previous_alarm_flags ^ new_alarm_flags; 
	alarm_event_flags |= alarm_change_flags;
	
}


uint8_t ESP32_Operation_Mode_Update(void){
	
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
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state++;
			result = SEQUENCE_COMPLETE;
		}else{
			//Does nothing
		}
		break;	
			
	case 3:
	
		//temp = ESP32_Turn_Off();
		//if(temp == DATA_COMM_SUCCESS){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else if(temp == DATA_COMM_FAIL){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else{
			////Does nothing
		//}
		seq_state = 0;
		result = SEQUENCE_COMPLETE;
		break;	
			
	default:
		break;
		
	}

	return result;
}



uint8_t ESP32_Main_Screen_Display_Update(void){
	
	static uint8_t seq_state = 0;
	uint8_t result = SEQUENCE_IN_PROCESS;
	uint8_t temp;
	uint32_t new_hh;
	uint8_t new_mm, new_ss;
	uint8_t new_date[3];
	uint8_t new_time[3];
	
	switch(seq_state){
		
	case 0:
		/* System mode */
		ESP32_Buffer_Operation_Mode_Set(system_mode);
		/* Battery level */
		ESP32_Buffer_Battery_Level_Set(Battery_Level_Get());			
		/* Alarm setpoints */
		ESP32_Buffer_Alarm1_Setpoint_Set(Alarm1_Setpoint_Get());
		ESP32_Buffer_Alarm2_Setpoint_Set(Alarm2_Setpoint_Get());
		ESP32_Buffer_Alarm3_Setpoint_Set(Alarm3_Setpoint_Get());
		/* Alarm counters */
		Alarm1_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm1_Counter_Set(new_hh);
		Alarm2_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm2_Counter_Set(new_hh);
		Alarm3_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm3_Counter_Set(new_hh);					
		/* General motor counter */
		Working_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Motor_Counter_Set(new_hh);
		/* Date and time */
		cli();
		Soft_RTC1_Get_Date(new_date, new_date + 1, new_date + 2);
		Soft_RTC1_Get_Time(new_time, new_time + 1, new_time + 2);
		sei();
		ESP32_Buffer_Date_And_Time_Set(new_date, new_time);
		/* Alarm status */
		ESP32_Buffer_Alarms_Status_Set(alarm_status_flags);		
		/* Alarm events */
		ESP32_Buffer_Alarms_Events_Set(alarm_event_flags);
				
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
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;		

	
	case 3:
	
		temp = ESP32_Battery_Level_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;	
	
	case 4:
	
		temp = ESP32_Alarm1_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;		
		
	case 5:
	
		temp = ESP32_Alarm1_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;		
			
	case 6:
	
		temp = ESP32_Alarm2_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
	
	case 7:
	
		temp = ESP32_Alarm2_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
			}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
			}else{
			//Does nothing
		}
		break;		
		
	case 8:
	
		temp = ESP32_Alarm3_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
	
	case 9:
	
		temp = ESP32_Alarm3_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;		
		
	case 10:
		
		temp = ESP32_Motor_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;	
		
	case 11:
	
		temp = ESP32_Date_And_Time_Write();
		if(temp == DATA_COMM_SUCCESS){
			alarm_event_flags = 0;
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;	
			
	case 12:
	
		temp = ESP32_Alarms_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;		
	
	case 13:
		temp = ESP32_Epaper_Screen01_Update();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state++;
		}else{
			//Does nothing
		}
		break;
		
	case 14:
		
		//temp = ESP32_Turn_Off();
		//if(temp == DATA_COMM_SUCCESS){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else if(temp == DATA_COMM_FAIL){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else{
			////Does nothing
		//}
		
		seq_state = 0;
		result = SEQUENCE_COMPLETE;
		
		break;
		
	default:
		break;
		
	}

	return result;
}


uint8_t ESP32_Alarm_Screen_Display_Update(void){
	
	static uint8_t seq_state = 0;
	uint8_t result = SEQUENCE_IN_PROCESS;
	uint8_t temp;
	uint32_t new_hh;
	uint8_t new_mm, new_ss;
	uint8_t new_date[3];
	uint8_t new_time[3];
	
	switch(seq_state){
		
	case 0:
		/* System mode */
		ESP32_Buffer_Operation_Mode_Set(system_mode);
		/* Battery level */
		ESP32_Buffer_Battery_Level_Set(Battery_Level_Get());
		/* Alarm setpoints */
		ESP32_Buffer_Alarm1_Setpoint_Set(Alarm1_Setpoint_Get());
		ESP32_Buffer_Alarm2_Setpoint_Set(Alarm2_Setpoint_Get());
		ESP32_Buffer_Alarm3_Setpoint_Set(Alarm3_Setpoint_Get());
		/* Alarm counters */
		Alarm1_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm1_Counter_Set(new_hh);
		Alarm2_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm2_Counter_Set(new_hh);
		Alarm3_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Alarm3_Counter_Set(new_hh);
		/* General motor counter */
		Working_Time_Get(&new_hh, &new_mm, &new_ss);
		ESP32_Buffer_Motor_Counter_Set(new_hh);
		/* Date and time */
		cli();
		Soft_RTC1_Get_Date(new_date, new_date + 1, new_date + 2);
		Soft_RTC1_Get_Time(new_time, new_time + 1, new_time + 2);
		sei();
		ESP32_Buffer_Date_And_Time_Set(new_date, new_time);
		/* Alarm status */
		ESP32_Buffer_Alarms_Status_Set(alarm_status_flags);
		/* Alarm events */
		ESP32_Buffer_Alarms_Events_Set(alarm_event_flags);
		
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
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;

		
	case 3:
		
		temp = ESP32_Battery_Level_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
			}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
			}else{
			//Does nothing
		}
		break;
		
	case 4:
		
		temp = ESP32_Alarm1_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
			}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
			}else{
			//Does nothing
		}
		break;
		
	case 5:
		
		temp = ESP32_Alarm1_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 6:
		
		temp = ESP32_Alarm2_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
			}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
			}else{
			//Does nothing
		}
		break;
		
	case 7:
		
		temp = ESP32_Alarm2_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 8:
		
		temp = ESP32_Alarm3_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 9:
		
		temp = ESP32_Alarm3_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 10:
		
		temp = ESP32_Motor_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 11:
		
		temp = ESP32_Date_And_Time_Write();
		if(temp == DATA_COMM_SUCCESS){
			alarm_event_flags = 0;
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 12:
		
		temp = ESP32_Alarms_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 14;
		}else{
			//Does nothing
		}
		break;
		
	case 13:
	
		temp = ESP32_Epaper_Screen02_Update();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state++;
		}else{
			//Does nothing
		}
		break;
		
	case 14:
		
		//temp = ESP32_Turn_Off();
		//if(temp == DATA_COMM_SUCCESS){
		//seq_state = 0;
		//result = SEQUENCE_COMPLETE;
		//}else if(temp == DATA_COMM_FAIL){
		//seq_state = 0;
		//result = SEQUENCE_COMPLETE;
		//}else{
		////Does nothing
		//}
		
		seq_state = 0;
		result = SEQUENCE_COMPLETE;
		
		break;
		
	default:
		break;
		
	}

	return result;
}


uint8_t ESP32_Calibration_Screen_Display_Update(void){

	static uint8_t seq_state = 0;
	uint8_t result = SEQUENCE_IN_PROCESS;
	uint8_t temp;
	
	switch(seq_state){
		
	case 0:
		ESP32_Buffer_Operation_Mode_Set(system_mode);
		ESP32_Buffer_Battery_Level_Set(Battery_Level_Get());
		ESP32_Buffer_Calibration_Counter_Set(Calib_Time_Get());
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
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 6;
			result = SEQUENCE_COMPLETE;
		}else{
			//Does nothing
		}
		break;
		
	case 3:
	
		temp = ESP32_Battery_Level_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 6;
			result = SEQUENCE_COMPLETE;
		}else{
			//Does nothing
		}
		break;			
		
	case 4:
		temp = ESP32_Calibration_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 6;
			result = SEQUENCE_COMPLETE;
		}else{
			//Does nothing
		}
	break;		
		
	case 5:
		temp = ESP32_Epaper_Screen03_Update();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state++;
		}else{
			//Does nothing
		}
		break;
		
	case 6:
		
		//temp = ESP32_Turn_Off();
		//if(temp == DATA_COMM_SUCCESS){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else if(temp == DATA_COMM_FAIL){
			//seq_state = 0;
			//result = SEQUENCE_COMPLETE;
		//}else{
			////Does nothing
		//}
		
		seq_state = 0;
		result = SEQUENCE_COMPLETE;
		break;
		
	default:
		break;
		
	}	
	
	
	return result;	
	
}


uint8_t Wifi_Connection_Sequence(void){
	
	static uint8_t seq_state = 0;
	uint8_t result = SEQUENCE_IN_PROCESS;
	uint8_t temp;
	uint32_t new_hh;
	uint8_t new_mm, new_ss;
	uint8_t new_date[3];
	uint8_t new_time[3];
	
	static uint8_t wifi_connected = 0;
	
	
	switch(seq_state){
		
	case 0:

		/* Date and time */
		cli();
		Soft_RTC1_Get_Date(new_date, new_date + 1, new_date + 2);
		Soft_RTC1_Get_Time(new_time, new_time + 1, new_time + 2);
		sei();
		ESP32_Buffer_Date_And_Time_Set(new_date, new_time);
		/* Alarm status */
		ESP32_Buffer_Alarms_Status_Set(alarm_status_flags);
		/* Alarm events */
		ESP32_Buffer_Alarms_Events_Set(alarm_event_flags);
	
		if(!wifi_connected){
			/* System mode */
			ESP32_Buffer_Operation_Mode_Set(system_mode);
			/* Battery level */
			ESP32_Buffer_Battery_Level_Set(Battery_Level_Get());
			/* Alarm setpoints */
			ESP32_Buffer_Alarm1_Setpoint_Set(Alarm1_Setpoint_Get());
			ESP32_Buffer_Alarm2_Setpoint_Set(Alarm2_Setpoint_Get());
			ESP32_Buffer_Alarm3_Setpoint_Set(Alarm3_Setpoint_Get());
			/* Alarm counters */
			Alarm1_Time_Get(&new_hh, &new_mm, &new_ss);
			ESP32_Buffer_Alarm1_Counter_Set(new_hh);
			Alarm2_Time_Get(&new_hh, &new_mm, &new_ss);
			ESP32_Buffer_Alarm2_Counter_Set(new_hh);
			Alarm3_Time_Get(&new_hh, &new_mm, &new_ss);
			ESP32_Buffer_Alarm3_Counter_Set(new_hh);
			/* General motor counter */
			Working_Time_Get(&new_hh, &new_mm, &new_ss);
			ESP32_Buffer_Motor_Counter_Set(new_hh);			
		}
	
		seq_state++;
	
		break;
		
	case 1:
		temp = ESP32_Turn_On();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}
		break;


	case 2:
	
		temp = ESP32_Date_And_Time_Write();
		if(temp == DATA_COMM_SUCCESS){
			alarm_event_flags = 0;
			
			if(wifi_connected){
				seq_state = 14;
			}else{
				seq_state++;		
			}
		
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;

	case 3:
		
		temp = ESP32_Operation_Mode_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;

		
	case 4:
		
		temp = ESP32_Battery_Level_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 5:
		
		temp = ESP32_Alarm1_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 6:
		
		temp = ESP32_Alarm1_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 7:
		
		temp = ESP32_Alarm2_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 8:
		
		temp = ESP32_Alarm2_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 9:
		
		temp = ESP32_Alarm3_Setpoint_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 10:
		
		temp = ESP32_Alarm3_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 11:
		
		temp = ESP32_Motor_Counter_Write();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
			
	case 12:
		
		temp = ESP32_Alarms_Status_Write();
		if(temp == DATA_COMM_SUCCESS){
			if(wifi_connected){
				seq_state = 14;
			}else{
				seq_state++;
			}
						
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
			
	case 13:
	
		temp = ESP32_WiFi_Enable();
		if(temp == DATA_COMM_SUCCESS){
			wifi_connected = 1;
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;		
	
	
	case 14:
	
		temp = ESP32_Epaper_Screen01_Update();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
			cli();
			system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
			sei();
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
			cli();
			system_flags &= ~((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG);
			sei();
		}else{
		//Does nothing
		}
		break;	
	
	case 15:
			
		if(system_flags & ((uint32_t)1 << SHOW_MAIN_OR_ALARM_SCREEN_FLAG)){
			seq_state = 0;
		}else if(system_flags & ((uint32_t)1 << ESP32_WEB_PARAMETERS_CHECK_FLAG)){
			seq_state++;
		}else{
			//Does nothing
		}		
		
		break;	
	
		
	case 16:
	
		temp = ESP32_Parameters_Status_Read();
		if(temp == DATA_COMM_SUCCESS){
			parameter_status_flag = ESP32_Buffer_Parameters_Status_Get();
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_WEB_PARAMETERS_CHECK_FLAG);
			sei();			
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			cli();
			system_flags &= ~((uint32_t)1 << ESP32_WEB_PARAMETERS_CHECK_FLAG);
			sei();
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;			
		
	case 17:
	
		if(parameter_status_flag & (1 << PARAM_STATUS_AL1_EN_BIT)){
			alarm_status_flags |= (1 << E1_FLAG);
		}else{
			alarm_status_flags &= ~(1 << E1_FLAG);
			cli();
			system_flags &= ~(1 << ALARM_01_REACHED_FLAG);
			Alarm1_Time_Reset();
			sei();			
		}

		if(parameter_status_flag & (1 << PARAM_STATUS_AL2_EN_BIT)){
			alarm_status_flags |= (1 << E2_FLAG);
		}else{
			alarm_status_flags &= ~(1 << E2_FLAG);
			cli();
			system_flags &= ~(1 << ALARM_02_REACHED_FLAG);
			Alarm2_Time_Reset();
			sei();			
		}		
		
		if(parameter_status_flag & (1 << PARAM_STATUS_AL3_EN_BIT)){
			alarm_status_flags |= (1 << E3_FLAG);
		}else{
			alarm_status_flags &= ~(1 << E3_FLAG);
			cli();
			system_flags &= ~(1 << ALARM_03_REACHED_FLAG);
			Alarm3_Time_Reset();
			sei();			
		}
		
		if(parameter_status_flag & (1 << PARAM_STATUS_AL1_RST_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL1_RST_BIT);			
			cli();
			system_flags &= ~(1 << ALARM_01_REACHED_FLAG);
			Alarm1_Time_Reset();	
			sei();
		}
		
		if(parameter_status_flag & (1 << PARAM_STATUS_AL2_RST_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL2_RST_BIT);
			cli();
			system_flags &= ~(1 << ALARM_02_REACHED_FLAG);
			Alarm2_Time_Reset();
			sei();
		}
		
		if(parameter_status_flag & (1 << PARAM_STATUS_AL3_RST_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL3_RST_BIT);
			cli();
			system_flags &= ~(1 << ALARM_03_REACHED_FLAG);
			Alarm3_Time_Reset();
			sei();
		}			
		
		if(parameter_status_flag & (1 << PARAM_STATUS_MOT_RST_BIT)){
			cli();
			parameter_status_flag &= ~(1 << PARAM_STATUS_MOT_RST_BIT);
			Working_Time_Reset();
			sei();
		}
			
		if(parameter_status_flag & (1 << PARAM_STATUS_MODE_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_MODE_BIT);
			seq_state = 18;
		}else if(parameter_status_flag & (1 << PARAM_STATUS_D_AND_T_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_D_AND_T_BIT);
			seq_state = 19;
		}else if(parameter_status_flag & (1 << PARAM_STATUS_AL1_SP_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL1_SP_BIT);
			seq_state = 20;		
		}else if(parameter_status_flag & (1 << PARAM_STATUS_AL2_SP_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL2_SP_BIT);
			seq_state = 21;
		}else if(parameter_status_flag & (1 << PARAM_STATUS_AL3_SP_BIT)){
			parameter_status_flag &= ~(1 << PARAM_STATUS_AL3_SP_BIT);
			seq_state = 22;
		}else if(parameter_status_flag & (1 << PARAM_WIFI_OFF_BIT)){
			parameter_status_flag &= ~(1 << PARAM_WIFI_OFF_BIT);
			seq_state = 23;
		}else{
			seq_state = 15;
		}
		break;		
		
	case 18:	

		temp = ESP32_Operation_Mode_Read();
		if(temp == DATA_COMM_SUCCESS){
			system_mode = ESP32_Buffer_Operation_Mode_Get();
			seq_state = 17;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;
		
	case 19:

		temp = ESP32_Date_And_Time_Read();
		if(temp == DATA_COMM_SUCCESS){
			ESP32_Buffer_Date_And_Time_Get(new_date, new_time);
			cli();
			Soft_RTC1_Set_Date(new_date[0], new_date[1], new_date[2]);
			Soft_RTC1_Set_Time(new_time[0], new_time[1], new_time[2]);
			sei();
			seq_state = 17;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;	
		
	case 20:	
		
		temp = ESP32_Alarm1_Setpoint_Read();
		if(temp == DATA_COMM_SUCCESS){
			cli();
			Alarm1_Setpoint_Set(ESP32_Buffer_Alarm1_Setpoint_Get());
			sei();
			seq_state = 17;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;	
		
	case 21:
		
		temp = ESP32_Alarm2_Setpoint_Read();
		if(temp == DATA_COMM_SUCCESS){
			cli();
			Alarm2_Setpoint_Set(ESP32_Buffer_Alarm2_Setpoint_Get());
			sei();		
			seq_state = 17;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;	
		
	case 22:
	
		temp = ESP32_Alarm3_Setpoint_Read();
		if(temp == DATA_COMM_SUCCESS){
			cli();
			Alarm3_Setpoint_Set(ESP32_Buffer_Alarm3_Setpoint_Get());
			sei();			
			seq_state = 17;
		}else if(temp == DATA_COMM_FAIL){
			seq_state = 24;
		}else{
			//Does nothing
		}
		break;

	case 23:
	
		temp = ESP32_Epaper_Screen01_Update();
		if(temp == DATA_COMM_SUCCESS){
			seq_state++;
		}else if(temp == DATA_COMM_FAIL){
			seq_state++;
		}else{
			//Does nothing
		}
		break;


	case 24:

		temp = 0;
		
		//temp = ESP32_Turn_Off();
		//if(temp == DATA_COMM_SUCCESS){
			//seq_state = 0;
			//wifi_connected = 0;
			//result = SEQUENCE_COMPLETE;
			//G1_Get_Button_Press(1 << MODE_BUTTON);
			//G1_Get_Button_Long(1 << MODE_BUTTON);
			//G1_Get_Button_Press(1 << WIFI_BUTTON);
			//
		//}else if(temp == DATA_COMM_FAIL){
			//seq_state = 0;
			//wifi_connected = 0;
			//result = SEQUENCE_COMPLETE;
			//G1_Get_Button_Press(1 << MODE_BUTTON);
			//G1_Get_Button_Long(1 << MODE_BUTTON);
			//G1_Get_Button_Press(1 << WIFI_BUTTON);		
		//}else{
			////Does nothing
		//}
		
		seq_state = 0;
		wifi_connected = 0;
		result = SEQUENCE_COMPLETE;
		G1_Get_Button_Press(1 << MODE_BUTTON);
		G1_Get_Button_Long(1 << MODE_BUTTON);
		G1_Get_Button_Press(1 << WIFI_BUTTON);
		
		break;
		
	default:
		break;
		
	}

	return result;
}