/*
 * horometro_2021.c
 *
 * Created: 21/04/2021 08:51:25
 * Author : mtorres
 */ 

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "project_defines.h" 
#include <util/delay.h>
#include "global.h" 
#include "low_power_horometer.h"
#include "system_test.h"
#include "mt_debounce.h"
#include "MT_magnetic_pickup.h"



int main(void)
{

	WDT_Off();
	UART_Initialize();
	System_Initialize();
	//Soft_RTC1_Set_Date(20, 5, 21);
	//Soft_RTC1_Set_Time(17, 5, 0);
	sei();
	
    /* Replace with your application code */
    while (1) 
    {	
		System_Sequence();
    }
	
	return  0;
}



/* Interrupt Service Routines (ISRs) */


/* Timer0 Interrupts every 1ms */
ISR(TIMER0_COMPA_vect){
		
	/* Measure the instant current value every 1ms */
	system_flags |= (1 << CURRENT_SENSE_FLAG);
		
}


/* Falling edge detection for magnetic pick-up */
ISR(INT1_vect){
	Magnetic_Pickup_State_Machine();
}


/* Timer1 interrupts if there is a Timeout for magnetic pick-up */
ISR(TIMER1_COMPA_vect){
	Magnetic_Pickup_Timeout();		
}


/* Timer2 interrupts every 1/32 sec (it uses a 32.768 KHz crystal) */
ISR(TIMER2_COMPA_vect){
	
	static uint8_t counter_1s = 0;
	
	static uint8_t counter_1_div_32_sec = 0;
	
	system_flags |= (1 << ESP32_COMM_CHECK_FLAG);
	
	/* Read the vibration sensor every 1/32 sec (31.25ms) */
	system_flags |= (1 << VIBRATION_SENSE_FLAG);
	
	/* Scan buttons every 1/32 sec (31.25ms) */
	G1_Button_Scan();
	system_flags |= (1 << BUTTON_READ_FLAG);	
	
	/* Update software RTC every 1 second */
	counter_1_div_32_sec++;
	if(counter_1_div_32_sec >= 32){
		counter_1_div_32_sec = 0;
		Soft_RTC1_Update();
		system_flags |= (1 << ONE_SECOND_ELAPSED_FLAG);
			
		counter_1s++;
		if(counter_1s >= 5){
			counter_1s = 0;
			system_flags |= (1 << SERIAL_MSG_FLAG);
		}
	}	
}

