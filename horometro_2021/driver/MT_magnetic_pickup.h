/*
 * magnetic_pickup.h
 *
 * Created: 19/05/2021 11:55:48
 *  Author: mtorres
 */ 


#ifndef MT_MAGNETIC_PICKUP_H_
#define MT_MAGNETIC_PICKUP_H_

/* File inclusion */
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "project_defines.h"
#include "global.h"
#include "MT_timer.h"

/* Pin definition */
//#define PIN_PICK_UP_SENSOR				PIND
//#define PICK_UP_SENSOR					3

/* Constant definition */

//#define TIMER1_PRESCALER_VALUE			1
//#define F_CPU_HZ						128000
//#define FREQ_CORRECTION_FACTOR			0.78125			
//#define MAG_PICKUP_MAX_FREQ_HZ			60
//#define MAG_PICKUP_MIN_FREQ_HZ			5
//#define PICKUP_N_SAMPLES				16

#define ACTUAL_F_CPU_HZ					((uint32_t)((float)F_CPU_HZ * FREQ_CORRECTION_FACTOR))					
#define TIMER1_MINIMUM_VALUE			(ACTUAL_F_CPU_HZ / (MAG_PICKUP_MAX_FREQ_HZ	 * TIMER1_PRESCALER_VALUE))// 16.666 ms --> 60 Hz
#define TIMER1_TIMEOUT_VALUE			(ACTUAL_F_CPU_HZ / (MAG_PICKUP_MIN_FREQ_HZ * TIMER1_PRESCALER_VALUE)) // 0.2 sec --> 5 Hz
#define PICKUP_WAITING_BEGINNING		0
#define PICKUP_WAITING_END				1

/* Variable declaration */

extern volatile uint16_t period_bin_average_value;
extern volatile uint8_t cycle_counter;
extern volatile uint32_t accumulator; 
extern volatile uint8_t pickup_state;		// pickup state (1 or 0)

/* Function declaration */

void Timer1_Initialize(void);
void Magnetic_Pickup_Initialize(void);
void Magnetic_Pickup_Enable(void);
void Magnetic_Pickup_Disable(void);

uint16_t Magnetic_Pickup_Get_Freq_Hz(void);
uint16_t Magnetic_Pickup_Get_Freq_RPM(void);
void Magnetic_Pickup_Clear_Freq(void);

/* Inline functions */

static inline void Magnetic_Pickup_State_Machine(void){
	
	uint16_t new_period_value;
	
	switch(pickup_state){
	
		case PICKUP_WAITING_BEGINNING:
	
			/* Restart TIMER1 */
			TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
			TCNT1 = 0;
			TIFR1 |= (1 << OCF1A);
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
			
			/* Update the pick-up state */
			pickup_state = PICKUP_WAITING_END;
			
			break;
	
		case PICKUP_WAITING_END:
			
			/* Restart TIMER1 and record time measurement */
			TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
			new_period_value = TCNT1;
			TCNT1 = 0;
			TIFR1 |= (1 << OCF1A);
			
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
			
			/* Accumulate several samples and calculate the average value */
			accumulator += new_period_value;
			cycle_counter++;
			if(cycle_counter >= PICKUP_N_SAMPLES){
				cycle_counter = 0;
				period_bin_average_value = accumulator / PICKUP_N_SAMPLES;
				accumulator = 0;
				
				if(period_bin_average_value <= TIMER1_MINIMUM_VALUE){
					period_bin_average_value = TIMER1_MINIMUM_VALUE;
					system_flags |= (1 << MAG_PICKUP_TOO_FAST_FLAG);
					system_flags &= ~(1 << MAG_PICKUP_TIMEOUT_FLAG);
				}else{
					system_flags &= ~((1 << MAG_PICKUP_TOO_FAST_FLAG) | (1 << MAG_PICKUP_TIMEOUT_FLAG));
				}
				
			}
			
			/* Update the pick-up state */
			pickup_state = PICKUP_WAITING_BEGINNING;
			
			break;
	
		default:
		
			break;
		
	}
		
}



static inline void Magnetic_Pickup_Timeout(void){
			
	accumulator = 0;
	cycle_counter = 0;
	period_bin_average_value = TIMER1_TIMEOUT_VALUE;
	pickup_state = PICKUP_WAITING_BEGINNING;

	system_flags |= (1 << MAG_PICKUP_TIMEOUT_FLAG);
}


#endif /* MAGNETIC_PICKUP_H_ */