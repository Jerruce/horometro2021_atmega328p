/*
 * Timer.c
 *
 *  Created on: 26 ene. 2018
 *      Author: MiguelAngel
 */

#include <stdint.h>
#include <avr/io.h>
#include "MT_timer.h"

/* Definici�n de funciones */


/****************************************************************************
* Nombre de la funci�n: Timer0_Configurar
* retorna : Nada
* arg : timer_param
* Descripci�n : Recibe un puntero hacia una estructura de tipo Timer_8b_t,
*               (la cual contiene los par�metros de configuraci�n) y configura
* 				el m�dulo Timer0.
* Notas : Si se desea emplear el generador de se�ales con los pines OC0A u
* 		  OC0B, antes es necesario configurar estos pines como salida.
*****************************************************************************/
void Timer0_Configurar(Timer_8b_t *timer_param){

	/* Inicializar los registros de configuraci?n */
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;
    /* Cargar los valores de OCRA y OCRB */
    OCR0A = timer_param->OCRA;
    OCR0B = timer_param->OCRB;
	/* Configurar la fuente de reloj del temporizador */
	TCCR0B |= (timer_param->clock) & 0x07;
	/* Configurar el modo de operacion del temporizador */
	TCCR0B |= ((timer_param->mode) << 1) & (1 << WGM02);
	TCCR0A |= (timer_param->mode) & 0x03;
	/* Configurar las salida por comparaci?n OC0A */
	TCCR0A |= (timer_param->OCA) << COM0A0;
	/* Configurar la salida por comparaci?n OC0B */
	TCCR0A |= (timer_param->OCB) << COM0B0;
	/* Habilitar las interrupciones que sean necesarias */
	TIMSK0 |= timer_param->interrupt_mask;
}


/****************************************************************************
* Nombre de la funci�n: Timer2_Configurar
* retorna : Nada
* arg : timer_param
* Descripci�n : Recibe un puntero hacia una estructura de tipo Timer_8b_t,
*               (la cual contiene los par�metros de configuraci�n) y configura
* 				el m�dulo Timer2.
* Notas : Si se desea emplear el generador de se�ales con los pines OC2A u
* 		  OC2B, antes es necesario configurar estos pines como salida.
*****************************************************************************/
void Timer2_Configurar(Async_Timer_8b_t *timer_param){

	/* Inicializar los registros de configuraci?n */
	TCCR2A = 0;
	TCCR2B = 0;
	TIMSK2 = 0;
	
    /* Cargar los valores de OCRA y OCRB */
    OCR2A = timer_param->OCRA;
    OCR2B = timer_param->OCRB;
	/* Configurar la fuente de reloj del temporizador */
	TCCR2B |= (timer_param->clock) & 0x07;
	/* Configurar el modo de operacion del temporizador */
	TCCR2B |= ((timer_param->mode) << 1) & (1 << WGM22);
	TCCR2A |= (timer_param->mode) & 0x03;
	/* Configurar las salida por comparaci?n OC0A */
	TCCR2A |= (timer_param->OCA) << COM2A0;
	/* Configurar la salida por comparaci?n OC0B */
	TCCR2A |= (timer_param->OCB) << COM2B0;
	/* Habilitar las interrupciones que sean necesarias */
	TIMSK2 |= timer_param->interrupt_mask;
}


/****************************************************************************
* Nombre de la funci�n: Timer1_Configurar
* retorna : Nada
* arg : timer_param
* Descripci�n : Recibe un puntero hacia una estructura de tipo Timer_16b_t,
*               (la cual contiene los par�metros de configuraci�n) y configura
* 				el m�dulo Timer1.
* Notas : Si se desea emplear el generador de se�ales con los pines OC1A,
* 		  OC1B u OC1C, antes es necesario configurar estos pines como salida.
*****************************************************************************/
void Timer1_Configurar(Timer_16b_t *timer_param){

	/* Inicializar los registros de configuraci?n */
	TCCR1A = 0;
	TCCR1B = 0;
	TIMSK1 = 0;
    /* Cargar los valores de OCRA, OCRB, OCRC e ICR */
    OCR1A = timer_param->OCRA;
    OCR1B = timer_param->OCRB;
    ICR1 =  timer_param->ICR;
	/* Configurar la fuente de reloj del temporizador */
	TCCR1B |= (timer_param->clock) & 0x07;
	/* Configurar el modo de operaci?n del temporizador */
	TCCR1B |= ((timer_param->mode) << 1) & (1 << WGM12);
	TCCR1A |= (timer_param->mode) & 0x03;
	/* Configurar las salida por comparaci?n OC0A */
	TCCR1A |= (timer_param->OCA) << COM1A0;
	/* Configurar la salida por comparaci?n OC0B */
	TCCR1A |= (timer_param->OCB) << COM1B0;
	/* Habilitar el cancelador de ruido de la entrada de captura */
	TCCR1B |= (timer_param->ic_noise_canceler) << ICNC1;
	/* Configurar selector de flancos de la entrada de captura */
	TCCR1B |= (timer_param->ic_edge_selector) << ICES1;
	/* Habilitar las interrupciones que sean necesarias */
	TIMSK1 |= timer_param->interrupt_mask;
}




void Timer3_Configurar(Timer_16b_t *timer_param){

	/* Inicializar los registros de configuraci?n */
	TCCR3A = 0;
	TCCR3B = 0;
	TIMSK3 = 0;
	/* Cargar los valores de OCRA, OCRB, OCRC e ICR */
	OCR3A = timer_param->OCRA;
	OCR3B = timer_param->OCRB;
	ICR3 =  timer_param->ICR;
	/* Configurar la fuente de reloj del temporizador */
	TCCR3B |= (timer_param->clock) & 0x07;
	/* Configurar el modo de operaci?n del temporizador */
	TCCR3B |= ((timer_param->mode) << 1) & (1 << WGM32);
	TCCR3A |= (timer_param->mode) & 0x03;
	/* Configurar las salida por comparaci?n OC0A */
	TCCR3A |= (timer_param->OCA) << COM3A0;
	/* Configurar la salida por comparaci?n OC0B */
	TCCR3A |= (timer_param->OCB) << COM3B0;
	/* Habilitar el cancelador de ruido de la entrada de captura */
	TCCR3B |= (timer_param->ic_noise_canceler) << ICNC3;
	/* Configurar selector de flancos de la entrada de captura */
	TCCR3B |= (timer_param->ic_edge_selector) << ICES3;
	/* Habilitar las interrupciones que sean necesarias */
	TIMSK3 |= timer_param->interrupt_mask;
}



void Timer4_Configurar(Timer_16b_t *timer_param){

	/* Inicializar los registros de configuraci?n */
	TCCR4A = 0;
	TCCR4B = 0;
	TIMSK4 = 0;
	/* Cargar los valores de OCRA, OCRB, OCRC e ICR */
	OCR4A = timer_param->OCRA;
	OCR4B = timer_param->OCRB;
	ICR4 =  timer_param->ICR;
	/* Configurar la fuente de reloj del temporizador */
	TCCR4B |= (timer_param->clock) & 0x07;
	/* Configurar el modo de operaci?n del temporizador */
	TCCR4B |= ((timer_param->mode) << 1) & (1 << WGM42);
	TCCR4A |= (timer_param->mode) & 0x03;
	/* Configurar las salida por comparaci?n OC0A */
	TCCR4A |= (timer_param->OCA) << COM4A0;
	/* Configurar la salida por comparaci?n OC0B */
	TCCR4A |= (timer_param->OCB) << COM4B0;
	/* Habilitar el cancelador de ruido de la entrada de captura */
	TCCR4B |= (timer_param->ic_noise_canceler) << ICNC4;
	/* Configurar selector de flancos de la entrada de captura */
	TCCR4B |= (timer_param->ic_edge_selector) << ICES4;
	/* Habilitar las interrupciones que sean necesarias */
	TIMSK4 |= timer_param->interrupt_mask;
	
}