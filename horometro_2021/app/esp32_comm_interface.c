/*
 * esp32_comm_interface.c
 *
 * Created: 17/06/2021 12:40:04
 *  Author: mtorres
 */ 

/* File inclusion */
#include "esp32_comm_interface.h"

/* Variable definition */

static uint32_t esp32_timeout_counter = 0;

// Data buffers for ESP32
static uint8_t esp32_buffer_operation_mode = VIBRATION_SENSOR_ONLY_MODE;
static uint8_t esp32_buffer_date[3] = {0, 0, 0};
static uint8_t esp32_buffer_time[3] = {0, 0, 0};
static uint8_t esp32_buffer_alarms_status = 0;
static uint32_t esp32_buffer_alarm1_setpoint = 0;
static uint32_t esp32_buffer_alarm2_setpoint = 0;
static uint32_t esp32_buffer_alarm3_setpoint = 0;
static uint32_t esp32_buffer_alarm1_counter = 0;
static uint32_t esp32_buffer_alarm2_counter = 0;
static uint32_t esp32_buffer_alarm3_counter = 0;
static uint32_t esp32_buffer_motor_counter = 0;
static uint16_t esp32_buffer_motor_speed = 0;
static uint32_t esp32_buffer_motor_current = 0;
static uint8_t esp32_buffer_battery_level = 0; 

static uint16_t esp32_buffer_param_status = 0;

// Buffers for SPI bus
static uint8_t spi_tx_buffer[SPI_TX_BUFF_SIZE];
static uint8_t spi_rx_buffer[SPI_RX_BUFF_SIZE];


/* Function definition */

void SPI1_Initialize(void){

	spi_config_t my_spi;
	
	/* Configurar el módulo SPI para trabajar en Modo 0 a 64 KHz */
	my_spi.mode = SPI_Master;
	my_spi.sck_prescaler = SCK_Fosc_2;
	my_spi.cpol = CPOL_0;
	my_spi.cpha = CPHA_0;
	my_spi.data_order = MSB_First;
	my_spi.interrupt = SPI_Int_Disabled;

	SPI1_Configurar(&my_spi);
}


void ESP32_Comm_Interface_Initialize(void){
	
	/* Configure as input and enable pull-up of the handshaking (mcu feedback) pin */
	DDR_MCU_FEEDBACK_HANDSHAKE &= ~(1 << MCU_FEEDBACK_HANDSHAKE);
	PORT_MCU_FEEDBACK_HANDSHAKE |= (1 << MCU_FEEDBACK_HANDSHAKE);
	
	/* Configure the chip select pin as output and initialize in HIGH */
	DDR_MCU_TO_MCU_CS |= (1 << MCU_TO_MCU_CS);
	PORT_MCU_TO_MCU_CS |= (1 << MCU_TO_MCU_CS);
	
	/* Configure the SPI in Master mode */
	SPI1_Initialize();
	
 }



uint8_t ESP32_Turn_On(void){
	
	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		ESP32_Microcontroller_PSU_Off();
		seq_state++;
		esp32_timeout_counter = 0;
		break;		
		
	case 1:
	
		esp32_timeout_counter++;
		if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
			esp32_timeout_counter = 0;
			seq_state = 0;
			result = DATA_COMM_SUCCESS;
		}	
		
		break;
		 	
	default:
		break;	
		
	}
	
	return result;
	
}


uint8_t ESP32_Turn_Off(void){
	
	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		esp32_timeout_counter = 0;
		seq_state++;
		break;
		
	case 1:	
	
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}	
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 2:
	
		SPI1_Master_Tx_Byte(ESP32_TURN_OFF_WRITE_CMD);
		seq_state++;
	
		break;	
	
	
	case 3:

		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 4:
		
		ESP32_Microcontroller_PSU_Off();
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
					
		break;
		
	case 5:
	
		ESP32_Microcontroller_PSU_Off();
		seq_state = 0;
		result = DATA_COMM_FAIL;
		
		break;		
		
	default:
		break;	
		
	}
		
	return result;
}


uint8_t ESP32_WiFi_Enable(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		esp32_timeout_counter = 0;
		seq_state++;
		break;
		
	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_WIFI_ENABLE_WRITE_CMD);
		seq_state++;
		
		break;
		
		
	case 3:

		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 4:
		
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		
		break;
		
	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		
		break;
		
		default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_WiFi_Disable(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		esp32_timeout_counter = 0;
		seq_state++;
		break;
		
	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_WIFI_DISABLE_WRITE_CMD);
		seq_state++;
		
		break;
		
		
	case 3:

		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		
		break;
		
	case 4:
		
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		
		break;
		
	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		
		break;
		
	default:
		break;
		
	}
	
	return result;
}


uint8_t ESP32_Epaper_Display_Update(void){
	
	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
		case 0:
		esp32_timeout_counter = 0;
		seq_state++;
		break;
		
	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_EPAPER_UPDATE_WRITE_CMD);
		seq_state++;
		break;
		
	case 3:

		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 4:
		
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;
		
	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;	
	
	
}


uint8_t ESP32_Operation_Mode_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
	
		esp32_timeout_counter = 0;
		seq_state++;
		break;		

	case 1:
	
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
	
		SPI1_Master_Tx_Byte(ESP32_OP_MODE_WRITE_CMD);
		seq_state++;
	
		break;
		
	case 3:
	
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}		
		break;	

	case 4:
	
		esp32_buffer_operation_mode = SPI1_Master_Tx_Byte(0x00);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;	

	case 5:
	
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;	
		
	}
	
	return result;	
}


uint8_t ESP32_Date_And_Time_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_DATE_AND_TIME_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_date[0];
		spi_tx_buffer[1] = esp32_buffer_date[1];
		spi_tx_buffer[2] = esp32_buffer_date[2];
		spi_tx_buffer[3] = esp32_buffer_time[0];
		spi_tx_buffer[4] = esp32_buffer_time[1];
		spi_tx_buffer[5] = esp32_buffer_time[2];
		
		SPI1_Master_Tx_Bitstream(6, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}	
	
	return result;
}


uint8_t ESP32_Alarms_Status_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;

	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARMS_STATUS_WRITE_CMD);
		seq_state++;
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
	
		SPI1_Master_Tx_Byte(esp32_buffer_alarms_status);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	
	return result;
}


uint8_t ESP32_Alarm1_Setpoint_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM1_SETPOINT_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm1_setpoint & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm1_setpoint >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm1_setpoint >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm1_setpoint >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm2_Setpoint_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM2_SETPOINT_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm2_setpoint & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm2_setpoint >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm2_setpoint >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm2_setpoint >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm3_Setpoint_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM3_SETPOINT_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm3_setpoint & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm3_setpoint >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm3_setpoint >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm3_setpoint >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm1_Counter_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM1_COUNTER_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm1_counter & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm1_counter >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm1_counter >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm1_counter >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm2_Counter_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM2_COUNTER_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm2_counter & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm2_counter >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm2_counter >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm2_counter >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm3_Counter_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM3_COUNTER_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_alarm3_counter & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_alarm3_counter >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_alarm3_counter >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_alarm3_counter >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
}


uint8_t ESP32_Motor_Counter_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_MOTOR_COUNTER_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_motor_counter & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_motor_counter >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_motor_counter >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_motor_counter >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;	
	
}


uint8_t ESP32_Motor_Speed_Status_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_MOTOR_SPEED_STATUS_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_motor_speed & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_motor_speed >> 8) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(2, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Motor_Current_Status_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_MOTOR_CURRENT_STATUS_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		spi_tx_buffer[0] = esp32_buffer_motor_current & 0x000000FF;
		spi_tx_buffer[1] = (esp32_buffer_motor_current >> 8) & 0x000000FF;
		spi_tx_buffer[2] = (esp32_buffer_motor_current >> 16) & 0x000000FF;
		spi_tx_buffer[3] = (esp32_buffer_motor_current >> 24) & 0x000000FF;
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Battery_Level_Status_Write(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_BATTERY_LEVEL_STATUS_WRITE_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Byte(esp32_buffer_battery_level);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Parameters_Status_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_PARAM_STATUS_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Bitstream(2, spi_tx_buffer, spi_rx_buffer);
		esp32_buffer_param_status = (((uint16_t)spi_rx_buffer[1]) << 8) | (spi_rx_buffer[0]); 
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}



uint8_t ESP32_Operation_Mode_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
			}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_OP_MODE_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		esp32_buffer_operation_mode = SPI1_Master_Tx_Byte(0x00);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}



uint8_t ESP32_Date_And_Time_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_DATE_AND_TIME_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Bitstream(6, spi_tx_buffer, spi_rx_buffer);
		esp32_buffer_date[0] = spi_rx_buffer[0];
		esp32_buffer_date[1] = spi_rx_buffer[1];
		esp32_buffer_date[2] = spi_rx_buffer[2];
		esp32_buffer_time[0] = spi_rx_buffer[3];
		esp32_buffer_time[1] = spi_rx_buffer[4];
		esp32_buffer_time[2] = spi_rx_buffer[5];
		
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm1_Setpoint_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM1_SETPOINT_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		esp32_buffer_alarm1_setpoint = (((uint32_t)spi_rx_buffer[3]) << 24) | (((uint32_t)spi_rx_buffer[2]) << 16) | (((uint32_t)spi_rx_buffer[1]) << 8) | (spi_rx_buffer[0]);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm2_Setpoint_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM2_SETPOINT_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		esp32_buffer_alarm2_setpoint = (((uint32_t)spi_rx_buffer[3]) << 24) | (((uint32_t)spi_rx_buffer[2]) << 16) | (((uint32_t)spi_rx_buffer[1]) << 8) | (spi_rx_buffer[0]);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


uint8_t ESP32_Alarm3_Setpoint_Read(void){

	uint8_t result = DATA_COMM_IN_PROGESS;
	static uint8_t seq_state = 0;
	
	switch(seq_state){
		
	case 0:
		
		esp32_timeout_counter = 0;
		seq_state++;
		break;

	case 1:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;
		
	case 2:
		
		SPI1_Master_Tx_Byte(ESP32_ALARM3_SETPOINT_READ_CMD);
		seq_state++;
		
		break;
		
	case 3:
		
		if(PIN_MCU_FEEDBACK_HANDSHAKE & (1 << MCU_FEEDBACK_HANDSHAKE)){
			esp32_timeout_counter++;
			if(esp32_timeout_counter >= ESP32_COMM_TIMEOUT_VALUE){
				esp32_timeout_counter = 0;
				seq_state = 5;
			}
		}else{
			esp32_timeout_counter = 0;
			seq_state++;
		}
		break;

	case 4:
		
		SPI1_Master_Tx_Bitstream(4, spi_tx_buffer, spi_rx_buffer);
		esp32_buffer_alarm3_setpoint = (((uint32_t)spi_rx_buffer[3]) << 24) | (((uint32_t)spi_rx_buffer[2]) << 16) | (((uint32_t)spi_rx_buffer[1]) << 8) | (spi_rx_buffer[0]);
		seq_state = 0;
		result = DATA_COMM_SUCCESS;
		break;

	case 5:
		
		seq_state = 0;
		result = DATA_COMM_FAIL;
		break;
		
	default:
		break;
		
	}
	
	return result;
	
}


void ESP32_Buffer_Operation_Mode_Set(uint8_t new_mode){
	esp32_buffer_operation_mode = new_mode;
}


void ESP32_Buffer_Date_And_Time_Set(uint8_t *new_date, uint8_t *new_time){
	esp32_buffer_date[0] = *new_date;
	esp32_buffer_date[1] = *(new_date + 1);
	esp32_buffer_date[2] = *(new_date + 2);
	
	esp32_buffer_time[0] = *new_time;
	esp32_buffer_time[1] = *(new_time + 1);
	esp32_buffer_time[2] = *(new_time + 2);
}


void ESP32_Buffer_Alarms_Status_Set(uint8_t new_alarms_status){
	esp32_buffer_alarms_status = new_alarms_status;
}


void ESP32_Buffer_Alarm1_Setpoint_Set(uint32_t new_setpoint){
	esp32_buffer_alarm1_setpoint = new_setpoint;
}


void ESP32_Buffer_Alarm2_Setpoint_Set(uint32_t new_setpoint){
	esp32_buffer_alarm2_setpoint = new_setpoint;
}


void ESP32_Buffer_Alarm3_Setpoint_Set(uint32_t new_setpoint){
	esp32_buffer_alarm3_setpoint = new_setpoint;
}


void ESP32_Buffer_Alarm1_Counter_Set(uint32_t new_counter){
	esp32_buffer_alarm1_counter = new_counter;
}


void ESP32_Buffer_Alarm2_Counter_Set(uint32_t new_counter){
	esp32_buffer_alarm2_counter = new_counter;
}


void ESP32_Buffer_Alarm3_Counter_Set(uint32_t new_counter){
	esp32_buffer_alarm3_counter = new_counter;
}


void ESP32_Buffer_Motor_Counter_Set(uint32_t new_counter){
	esp32_buffer_motor_counter = new_counter;
}


void ESP32_Buffer_Motor_Speed_Set(uint32_t new_speed){
	esp32_buffer_motor_speed = new_speed;
}


void ESP32_Buffer_Motor_Current_Set(uint32_t new_current){
	esp32_buffer_motor_current = new_current;
}


void ESP32_Buffer_Battery_Level_Set(uint32_t new_level){
	esp32_buffer_battery_level = new_level;
}


uint16_t ESP32_Buffer_Parameters_Status_Get(void){
	return esp32_buffer_param_status;
}


uint8_t ESP32_Buffer_Operation_Mode_Get(void){
	return esp32_buffer_operation_mode;
}


void ESP32_Buffer_Date_And_Time_Get(uint8_t *new_date, uint8_t *new_time){
	*new_date = esp32_buffer_date[0];
	*(new_date + 1) = esp32_buffer_date[1];
	*(new_date + 2) = esp32_buffer_date[2];
	
	*new_time = esp32_buffer_time[0];
	*(new_time + 1) = esp32_buffer_time[1];
	*(new_time + 2) = esp32_buffer_time[2];	
}


uint32_t ESP32_Buffer_Alarm1_Setpoint_Get(void){
	return esp32_buffer_alarm1_setpoint;
}


uint32_t ESP32_Buffer_Alarm2_Setpoint_Get(void){
	return esp32_buffer_alarm2_setpoint;
}


uint32_t ESP32_Buffer_Alarm3_Setpoint_Get(void){
	return esp32_buffer_alarm3_setpoint;
}
