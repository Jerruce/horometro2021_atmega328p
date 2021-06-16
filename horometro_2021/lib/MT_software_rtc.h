/*
 * MT_software_rtc.h
 *
 * Created: 17/05/2021 14:10:42
 *  Author: mtorres
 */ 


#ifndef MT_SOFTWARE_RTC_H_
#define MT_SOFTWARE_RTC_H_

/* File inclusion */
#include <avr/io.h>
#include <stdint.h> 
#include "MT_timer.h"

/* Macro definition */
#define SOFT_RTC_SEC_INDEX		0
#define SOFT_RTC_MIN_INDEX		1
#define SOFT_RTC_HOUR_INDEX		2
#define SOFT_RTC_DAY_INDEX		3	
#define SOFT_RTC_MONTH_INDEX	4
#define SOFT_RTC_YEAR_INDEX		5

/* Type definition */

typedef enum{
	JANUARY = 0,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY, 
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER
}Soft_RTC_Month;


/* Variable declaration */
extern volatile uint8_t rtc1_data[6];
extern volatile uint8_t rtc1_leap_year_index;
extern const uint8_t	month_size[2][12];

/* Function declaration */
void Soft_RTC_Initialize(void);
void Soft_RTC1_Set_Time(uint8_t hh, uint8_t mm, uint8_t ss);
void Soft_RTC1_Set_Date(uint8_t dd, uint8_t mm, uint8_t yy);
void Soft_RTC1_Get_Time(uint8_t *hh, uint8_t *mm, uint8_t *ss);
void Soft_RTC1_Get_Date(uint8_t *dd, uint8_t *mm, uint8_t *yy);


static inline void Soft_RTC1_Update(void){
	
	rtc1_data[SOFT_RTC_SEC_INDEX]++;
	if(rtc1_data[SOFT_RTC_SEC_INDEX] > 59){
		rtc1_data[SOFT_RTC_SEC_INDEX] = 0;
		rtc1_data[SOFT_RTC_MIN_INDEX]++;
		if(rtc1_data[SOFT_RTC_MIN_INDEX] > 59){
			rtc1_data[SOFT_RTC_MIN_INDEX] = 0;
			rtc1_data[SOFT_RTC_HOUR_INDEX]++;
			if(rtc1_data[SOFT_RTC_HOUR_INDEX] > 23){
				rtc1_data[SOFT_RTC_HOUR_INDEX] = 0;
				rtc1_data[SOFT_RTC_DAY_INDEX]++;
				if(rtc1_data[SOFT_RTC_DAY_INDEX] > month_size[rtc1_leap_year_index][rtc1_data[SOFT_RTC_MONTH_INDEX]]){
					rtc1_data[SOFT_RTC_DAY_INDEX] = 1;
					rtc1_data[SOFT_RTC_MONTH_INDEX]++;
					if(rtc1_data[SOFT_RTC_MONTH_INDEX] >= 12){
						rtc1_data[SOFT_RTC_MONTH_INDEX] = 0;
						rtc1_data[SOFT_RTC_YEAR_INDEX]++;
						if(rtc1_data[SOFT_RTC_YEAR_INDEX] > 99){
							rtc1_data[SOFT_RTC_YEAR_INDEX] = 0;
						}
					}
				}
			}
		}
	}
		
	if(rtc1_data[SOFT_RTC_YEAR_INDEX] % 4){
		rtc1_leap_year_index = 0;
	}else{
		rtc1_leap_year_index = 1;
	}
		
}



#endif /* MT_SOFTWARE_RTC_H_ */