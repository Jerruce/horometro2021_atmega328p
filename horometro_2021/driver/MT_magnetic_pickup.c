/*
 * magnetic_pick_up.c
 *
 * Created: 19/05/2021 11:56:12
 *  Author: mtorres
 */ 

/* File inclusion */
#include "MT_magnetic_pickup.h"

/* Variable definition */

volatile uint16_t period_bin_average_value = PICKUP_WAITING_BEGINNING;
volatile uint8_t cycle_counter = 0;
volatile uint32_t accumulator = 0;
volatile uint8_t pickup_state = 0;		// pickup state (1 or 0)


/* Function definition */


void Timer1_Initialize(void){
	
	Timer_16b_t my_timer;
	
	my_timer.clock = Timer_Clk_Disabled;
	my_timer.mode = Timer_16b_CTC_OCRA_Mode;
	my_timer.OCRA = TIMER1_TIMEOUT_VALUE;
	my_timer.OCRB = 0;
	my_timer.ICR = 0;
	my_timer.OCA = OC_Disabled;
	my_timer.OCB = OC_Disabled;
	my_timer.ic_edge_selector = Timer_IC_Falling_Edge;
	my_timer.ic_noise_canceler = Timer_ICNC_Disabled;
	my_timer.interrupt_mask = Timer_Interrupts_Disabled;
	
	Timer1_Configurar(&my_timer);
}


void Magnetic_Pickup_Initialize(void){
	
	/* Configure Timer1 */
	Timer1_Initialize();
	
	/* Configure pick-up pin as INPUT and enable the internal pull-up resistor if required */
	PIN_PICK_UP_SENSOR &= ~(1 << PICK_UP_SENSOR);
	PORT_PICK_UP_SENSOR |= (1 << PICK_UP_SENSOR);
	
	/* Configure external interrupt INT1 to activate with every falling edge */
	EICRA &= ~(1 << ISC10);
	EICRA |= (1 << ISC11);
	
	/* Clear the INT1 flag */
	EIFR |= (1 << INTF1);	
		
}

void Magnetic_Pickup_Enable(void){

	accumulator = 0;
	cycle_counter = 0;
	period_bin_average_value = TIMER1_TIMEOUT_VALUE;
	pickup_state = PICKUP_WAITING_BEGINNING;
	
	/* Clear INT1 interrupt flag */
	EIFR |= (1 << INTF1);
	/* Enable INT1 external interrupt */
	EIMSK |= (1 << INT1);
	
	/* Restart TIMER1 and enable COMPA interrupt */
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
	TCNT1 = 0;
	TIFR1 |= (1 << OCF1A);
	TIMSK1 |= (1 << OCIE0A);
	
	#if (TIMER1_PRESCALER_VALUE == 1)
		TCCR1B |= (1 << CS10);
	#elif (TIMER1_PRESCALER_VALUE == 8)
		TCCR1B |= (1 << CS11);
	#elif (TIMER1_PRESCALER_VALUE == 64)
		TCCR1B |= ((1 << CS11) | (1 << CS10));
	#elif (TIMER1_PRESCALER_VALUE == 256)
		TCCR1B |= (1 << CS12);
	#elif (TIMER1_PRESCALER_VALUE == 1024)	
		TCCR1B |= (1 << CS12) | (1 << CS10);
	#else
		#error	Prescaler de Timer1 invalido
	#endif		
}

void Magnetic_Pickup_Disable(void){
	
	/* Disable INT1 external interrupt and clear the flag */
	EIMSK &= ~(1 << INT1);
	EIFR |= (1 << INTF1);
	
	/* Stop TIMER1 */
	TIMSK1 &= ~(1 << OCIE0A);
	TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
	TCNT1 = 0;
	TIFR1 |= (1 << OCF1A);
	
}

uint16_t Magnetic_Pickup_Get_Freq_Hz(void){
	
	uint16_t bin_value;
	uint32_t freq_value_hz;
	
	cli();
	bin_value = period_bin_average_value;
	sei();
	
	freq_value_hz = (uint32_t)ACTUAL_F_CPU_HZ / ((uint32_t)bin_value * TIMER1_PRESCALER_VALUE);

	return freq_value_hz; 
}


uint16_t Magnetic_Pickup_Get_Freq_RPM(void){
	
	uint16_t bin_value;
	uint32_t freq_value_rpm;

	cli();
	bin_value = period_bin_average_value;
	sei();

	freq_value_rpm = ((uint32_t)ACTUAL_F_CPU_HZ * 60) / ((uint32_t)bin_value * TIMER1_PRESCALER_VALUE);
	
	return freq_value_rpm;	

}

