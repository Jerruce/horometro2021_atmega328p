/*
 * esp32_spi_interface.h
 *
 * Created: 17/06/2021 12:38:52
 *  Author: mtorres
 */ 


#ifndef ESP32_COMM_INTERFACE_H_
#define ESP32_COMM_INTERFACE_H_

/* File inclusion */
#include "project_defines.h"
#include <avr/io.h>
#include <stdint.h>
#include "global.h"
#include "MT_spi_master.h"
#include "psu_sw_manager.h"

// Commands for TRANSMITTING information to the ESP32
#define ESP32_TURN_OFF_WRITE_CMD					0x01
#define ESP32_WIFI_ENABLE_WRITE_CMD					0x02
#define ESP32_WIFI_DISABLE_WRITE_CMD				0x03
#define ESP32_EPAPER_UPDATE_WRITE_CMD				0x04
#define ESP32_OP_MODE_WRITE_CMD						0x05
#define ESP32_DATE_AND_TIME_WRITE_CMD				0x06
#define ESP32_ALARMS_STATUS_WRITE_CMD				0x07
#define ESP32_ALARM1_SETPOINT_WRITE_CMD				0x08
#define ESP32_ALARM2_SETPOINT_WRITE_CMD				0x09
#define ESP32_ALARM3_SETPOINT_WRITE_CMD				0x0A
#define ESP32_ALARM1_COUNTER_WRITE_CMD				0x0B
#define ESP32_ALARM2_COUNTER_WRITE_CMD				0x0C
#define ESP32_ALARM3_COUNTER_WRITE_CMD				0x0D
#define ESP32_MOTOR_COUNTER_WRITE_CMD				0x0E
#define ESP32_MOTOR_SPEED_STATUS_WRITE_CMD			0x0F
#define ESP32_MOTOR_CURRENT_STATUS_WRITE_CMD		0x10
#define ESP32_BATTERY_LEVEL_STATUS_WRITE_CMD		0x11


// Size in bytes of the frames TRANSMITTED to the ESP32
#define OP_MODE_WRITE_FRAME_SIZE				4//1
#define DATE_AND_TIME_WRITE_FRAME_SIZE			8//6
#define ALARMS_STATUS_WRITE_FRAME_SIZE			4//1
#define ALARM1_SETPOINT_WRITE_FRAME_SIZE		4
#define ALARM2_SETPOINT_WRITE_FRAME_SIZE		4
#define ALARM3_SETPOINT_WRITE_FRAME_SIZE		4
#define ALARM1_COUNTER_WRITE_FRAME_SIZE			4
#define ALARM2_COUNTER_WRITE_FRAME_SIZE			4
#define ALARM3_COUNTER_WRITE_FRAME_SIZE			4
#define MOTOR_COUNTER_WRITE_FRAME_SIZE			4
#define MOTOR_SPEED_STATUS_WRITE_FRAME_SIZE		4//2
#define CURRENT_STATUS_WRITE_FRAME_SIZE			4
#define BATTERY_LEVEL_WRITE_FRAME_SIZE			4//1


// Commands for RECEIVING information from the ESP32
#define ESP32_PARAM_STATUS_READ_CMD				0x21
#define ESP32_OP_MODE_READ_CMD					0x22
#define ESP32_DATE_AND_TIME_READ_CMD			0x23
#define ESP32_ALARM1_SETPOINT_READ_CMD			0x24
#define ESP32_ALARM2_SETPOINT_READ_CMD			0x25
#define ESP32_ALARM3_SETPOINT_READ_CMD			0x26

// Size in bytes of the frames RECEIVED from the ESP32
#define PARAM_STATUS_READ_FRAME_SIZE			4//2
#define OP_MODE_READ_FRAME_SIZE					4//1
#define DATE_AND_TIME_READ_FRAME_SIZE			8//6
#define ALARM1_SETPOINT_READ_FRAME_SIZE			4
#define ALARM2_SETPOINT_READ_FRAME_SIZE			4
#define ALARM3_SETPOINT_READ_FRAME_SIZE			4

// Parameter status flags
#define PARAM_STATUS_MODE_BIT					0
#define PARAM_STATUS_D_AND_T_BIT				1
#define PARAM_STATUS_AL1_EN_BIT					2
#define PARAM_STATUS_AL2_EN_BIT					3
#define PARAM_STATUS_AL3_EN_BIT					4
#define PARAM_STATUS_AL1_SP_BIT					5
#define PARAM_STATUS_AL2_SP_BIT					6
#define PARAM_STATUS_AL3_SP_BIT					7
#define PARAM_STATUS_AL1_RST_BIT				8
#define PARAM_STATUS_AL2_RST_BIT				9
#define PARAM_STATUS_AL3_RST_BIT				10
#define PARAM_STATUS_MOT_RST_BIT				11

/* Timeout value */
#define ESP32_COMM_TIMEOUT_SEC					1//10
#define ESP32_COMM_TIMEOUT_VALUE				((32 * ESP32_COMM_TIMEOUT_SEC) + 1)

/* Buffers' size */
#define SPI_TX_BUFF_SIZE						8
#define SPI_RX_BUFF_SIZE						8

/* Data transfer states */
#define DATA_COMM_IN_PROGESS					0				
#define DATA_COMM_SUCCESS						1
#define DATA_COMM_FAIL							2

/* Function declaration */

void SPI1_Initialize(void);
void ESP32_Comm_Interface_Initialize(void);

uint8_t ESP32_Turn_On(void);
uint8_t ESP32_Turn_Off(void);
uint8_t ESP32_WiFi_Enable(void);
uint8_t ESP32_WiFi_Disable(void);
uint8_t ESP32_Epaper_Display_Update(void);
uint8_t ESP32_Operation_Mode_Write(void);
uint8_t ESP32_Date_And_Time_Write(void);
uint8_t ESP32_Alarms_Status_Write(void);
uint8_t ESP32_Alarm1_Setpoint_Write(void);
uint8_t ESP32_Alarm2_Setpoint_Write(void);
uint8_t ESP32_Alarm3_Setpoint_Write(void);
uint8_t ESP32_Alarm1_Counter_Write(void);
uint8_t ESP32_Alarm2_Counter_Write(void);
uint8_t ESP32_Alarm3_Counter_Write(void);
uint8_t ESP32_Motor_Counter_Write(void);
uint8_t ESP32_Motor_Speed_Status_Write(void);
uint8_t ESP32_Motor_Current_Status_Write(void);
uint8_t ESP32_Battery_Level_Status_Write(void);

uint8_t ESP32_Parameters_Status_Read(void);
uint8_t ESP32_Operation_Mode_Read(void);
uint8_t ESP32_Date_And_Time_Read(void);
uint8_t ESP32_Alarm1_Setpoint_Read(void);
uint8_t ESP32_Alarm2_Setpoint_Read(void);
uint8_t ESP32_Alarm3_Setpoint_Read(void);

void ESP32_Buffer_Operation_Mode_Set(uint8_t new_mode);
void ESP32_Buffer_Date_And_Time_Set(uint8_t *new_date, uint8_t *new_time);
void ESP32_Buffer_Alarms_Status_Set(uint8_t new_alarms_status);
void ESP32_Buffer_Alarm1_Setpoint_Set(uint32_t new_setpoint);
void ESP32_Buffer_Alarm2_Setpoint_Set(uint32_t new_setpoint);
void ESP32_Buffer_Alarm2_Setpoint_Set(uint32_t new_setpoint);
void ESP32_Buffer_Alarm1_Counter_Set(uint32_t new_counter);
void ESP32_Buffer_Alarm2_Counter_Set(uint32_t new_counter);
void ESP32_Buffer_Alarm3_Counter_Set(uint32_t new_counter);
void ESP32_Buffer_Motor_Counter_Set(uint32_t new_counter);
void ESP32_Buffer_Motor_Speed_Set(uint32_t new_speed);
void ESP32_Buffer_Motor_Current_Set(uint32_t new_current);
void ESP32_Buffer_Battery_Level_Set(uint32_t new_level);

uint16_t ESP32_Buffer_Parameters_Status_Get(void);
uint8_t ESP32_Buffer_Operation_Mode_Get(void);
void ESP32_Buffer_Date_And_Time_Get(uint8_t *new_date, uint8_t *new_time);
uint32_t ESP32_Buffer_Alarm1_Setpoint_Get(void);
uint32_t ESP32_Buffer_Alarm2_Setpoint_Get(void);
uint32_t ESP32_Buffer_Alarm3_Setpoint_Get(void);



#endif /* ESP32_SPI_INTERFACE_H_ */