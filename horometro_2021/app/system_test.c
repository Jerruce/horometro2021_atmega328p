/*
 * system_test.c
 *
 * Created: 18/05/2021 11:17:04
 *  Author: mtorres
 */ 

/* File inclusion */
#include "system_test.h"


void UART_Initialize(void){
		
	UART_t my_uart;	
	
	my_uart.baud_rate = 2400;//2700;//2800;//Baud_2400_bps;
	my_uart.data_bits = UART_8_Data_Bits;
	my_uart.parity_mode = UART_Parity_Disabled;
	my_uart.stop_bits = UART_1_Stop_Bit;
	my_uart.tx_status = UART_Tx_Enabled;
	my_uart.rx_status = UART_Rx_Disabled;
	my_uart.u2x = U2X_Enabled;
	my_uart.interrupt_mask = UART_Interrupts_Disabled;
	
	UART0_Configurar(&my_uart);
	
}