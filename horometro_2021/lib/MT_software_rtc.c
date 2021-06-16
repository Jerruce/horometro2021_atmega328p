/*
 * MT_software_rtc.c
 *
 * Created: 17/05/2021 14:11:00
 *  Author: mtorres
 */ 

/* File inclusion */
#include "MT_software_rtc.h"

/* Variable definition */
volatile uint8_t rtc1_data[6] = {0, 0, 0, 1, 0, 0};
volatile uint8_t rtc1_leap_year_index = 1;

const uint8_t	month_size[2][12] =	{
										{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
										{31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
									};

/* Function definition */

void Soft_RTC_Initialize(void){
	Soft_RTC1_Set_Time(0, 0, 0);
	Soft_RTC1_Set_Date(1, 0, 0);
}


void Soft_RTC1_Set_Time(uint8_t hh, uint8_t mm, uint8_t ss){
	
	if(hh > 23){
		hh = 23;
	}
	
	if(mm > 59){
		mm = 59;
	}
	
	if(ss > 59){
		ss = 59;
	}
	
	rtc1_data[SOFT_RTC_HOUR_INDEX] = hh;
	rtc1_data[SOFT_RTC_MIN_INDEX] = mm;
	rtc1_data[SOFT_RTC_SEC_INDEX] = ss;
}


void Soft_RTC1_Set_Date(uint8_t dd, uint8_t mm, uint8_t yy){
	
	if(yy > 99){
		yy = 99;
	}
	
	if(yy % 4){
		rtc1_leap_year_index = 0;
	}else{
		rtc1_leap_year_index = 1;
	}
	
	
	if(mm <= 1){
		mm = 0;
	}else if(mm >= 12){
		mm = 11;
	}else{
		mm--;
	}
	
	
	if(dd < 1){
		dd = 1;
	}else if(dd > month_size[rtc1_leap_year_index][mm]){
		dd = month_size[rtc1_leap_year_index][mm];
	}
	
	rtc1_data[SOFT_RTC_DAY_INDEX] = dd;
	rtc1_data[SOFT_RTC_MONTH_INDEX] = mm;
	rtc1_data[SOFT_RTC_YEAR_INDEX] = yy;
}



void Soft_RTC1_Get_Time(uint8_t *hh, uint8_t *mm, uint8_t *ss){
	*hh = rtc1_data[SOFT_RTC_HOUR_INDEX];
	*mm = rtc1_data[SOFT_RTC_MIN_INDEX];
	*ss = rtc1_data[SOFT_RTC_SEC_INDEX]; 
}


void Soft_RTC1_Get_Date(uint8_t *dd, uint8_t *mm, uint8_t *yy){
	*dd = rtc1_data[SOFT_RTC_DAY_INDEX];
	*mm = rtc1_data[SOFT_RTC_MONTH_INDEX] + 1;
	*yy = rtc1_data[SOFT_RTC_YEAR_INDEX];	
}