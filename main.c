
/************************************************************************************************\
	
	IRBeam v2.0 - PIC12F629/675
	www.ajk.altervista.com
 	
	Copyright (c) 2013, Alberto Garbui aka AjK
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification, 
	are permitted provided that the following conditions are met:

	-Redistributions of source code must retain the above copyright notice, this list 
	 of conditions and the following disclaimer.
	-Redistributions in binary form must reproduce the above copyright notice, this list 
	 of conditions and the following disclaimer in the documentation and/or other 
	 materials provided with the distribution.
	-Neither the name of the AjK Elettronica Digitale nor the names of its contributors may be 
	 used to endorse or promote products derived from this software without specific prior 
	 written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
	OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
	SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
	PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
	INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

\************************************************************************************************/

#ifndef _XTAL_FREQ
	#define _XTAL_FREQ 4000000
#endif

#include	<htc.h>

//fuses
__CONFIG(FOSC_INTRCIO & WDTE_OFF & PWRTE_OFF & BOREN_OFF & CPD_OFF & CP_OFF & MCLRE_OFF);

//pin connections
#define SENS1	GPIO5 //input sensors [1..3]
#define SENS2	GPIO4
#define SENS3	GPIO3
#define BUZZ	GPIO2 //output buzzer
#define RELE	GPIO1 //output alarm

//configurations stuff
#define ALARM_PULSE		1000	//1000ms -> 1sec alarm beep

//debug TEST (decommentare per ottenere la funzione debug)
//#define _DEBUG_TEST
//#define _DEBUG_TEST_BOUNCE

#define PULSE_TRIGGER	5		//min pulses to trigger a missing pulse status
								//lato lungo 3pulse su 65ms
								//lato corto 2pulse su 60ms
								//lato corto 100ms, 7,8pulse libero, 5pulse occupato
								//lato lungo 100ms, 7,8,9pulse libero, 5pulse occupato

#define WINDOW_WIDTH	100		//100ms

/*
	_BOUNCE MODE:
	Bounce mode checks if there are enough misses pulses inside a time window. Checking more then one missing pulse
	will prevent false alarm triggering due to occasionally disturbs that may affects the IR sensors.
	WINDOW_WIDTH defines how much often the pulses are checked (100ms default)
	BOUNCE_WIDTH defines how many WINDOW_WIDTH to check. 
	BOUNCE_TIMEOUT defines the width of the total time window to be checked (5 * 100ms = 500ms)

	 1 2 3 4 5 	
	|x|-|x|x|-|  if we got enough misses inside a BOUNCE_TIMEOUT an alarm will be triggered (misses >= BOUNCE_MISS)
	|x|x|-|-|-|  in other cases nothing will happen (misses < BOUNCE_MISS)

*/
#define _BOUNCEMODE				//enable/disable BOUNCEMODE
#define BOUNCE_MISS		2		//min missCount pulse missing during antibounce window to trigger an alarm status
#define BOUNCE_WIDTH	4		//5times WINDOW_WIDTH antibounce window (WINDOW_WIDTH=100ms -> BOUNCE_WIDTH=5*100ms = 500ms)
							
//**********DONT MODIFY ANY VALUES UNDER THIS LINE!*******************//

#ifdef  _DEBUG_TEST_BOUNCE		//force BOUNCEMODE if DEBUG_TEST_BOUNCE is active
	#define _BOUNCEMODE
#endif
#define BOUNCE_TIMEOUT	(BOUNCE_WIDTH*WINDOW_WIDTH-50)
#define TMR0_PRESET		8		//Timer0 preset (prescaler 1:4 -> overflow every 1ms)

//global variables
unsigned short timerTimeout,delay,alarmTimeout;	//time counter
unsigned short C1,C2,C3;						//pulse counters

//BOUNCE TIMEOUT STRUCT
typedef struct
{
	unsigned short time;
	unsigned char miss;
} TIMEOUT;

TIMEOUT bounceTimeout[BOUNCE_WIDTH];

void countersReset()
{
	C1=0;C2=0;C3=0;					//counters reset
	timerTimeout=WINDOW_WIDTH;		//start windowWidth timerTimeout
}

void startAlarm()
{
	RELE=0;							//active LOW
	BUZZ=1;							//active HIGH
	alarmTimeout=ALARM_PULSE;		//start counter
}

void resetAlarm()
{
	RELE=1;				//ALARM active LOW (default HIGH)
	BUZZ=0;				//BUZZER active HIGH (default LOW)
	#ifdef _BOUNCEMODE
	unsigned char i;
	for(i=0;i<BOUNCE_WIDTH;i++)	//reset BOUNCETIMEOUTs
	{
		bounceTimeout[i].time=0;
		bounceTimeout[i].miss=0;
	}
	#endif
}

void systemInit()
{
	// I/O init
	TRISIO=0b111000;
	GPIO=0b000010;
	WPU=0;				//pullUp disabled
	CMCON=7;			//comparator disabled
	//ANSEL=0;			//all digital pins (only PIC12F675)
	IOC=0b111000;		//InterruptOnChange pins (GP3..5)

	resetAlarm();

	//Timer0 - prescaler 1:4 - 4us steps - overflow every 1ms (with PRESET=8)
	OPTION_REG=0b10000001;

	TMR0=TMR0_PRESET;			
	INTCON=0b10101000;			//Timer0 interrupt + IOC interrupt

}

void delay_ms(unsigned short ms)
{
	delay=ms;
	while(delay>0);
}

static void interrupt isr(void)					
{
	unsigned char i;
	if(T0IF)								//timer0 overflow?	
	{
		if(timerTimeout>0)timerTimeout--;
		if(alarmTimeout>0)alarmTimeout--;	
		if(delay>0)delay--;	
		for(i=0;i<BOUNCE_WIDTH;i++)		
			if(bounceTimeout[i].time>0)bounceTimeout[i].time--;
		TMR0=TMR0_PRESET;					//timer0 reset
		T0IF=0;								//flag reset
	}

	if(GPIF)								//pin changed?
	{
		if(timerTimeout!=0)					//incremento C1,C2,C3 solo se non sono in timeOut
		{	
			C1+=(!SENS1);
			C2+=(!SENS2);
			C3+=(!SENS3);
		}
		GPIF=0;								//flag reset
	}
}

void main()
{
	#ifdef _BOUNCEMODE
	unsigned char ii,j=0;
	#endif
	systemInit();
	
	BUZZ=1;				//start beep
	delay_ms(500);
	BUZZ=0;
	alarmTimeout=0;	
	resetAlarm();
	countersReset();	//reset counters
	
	while(1)
	{
		//alarmTimeout reset the alarm status
		if(alarmTimeout==0)
		{
			resetAlarm();		//alarm off
		}
		
		#ifdef _BOUNCEMODE
		for(ii=0;ii<BOUNCE_WIDTH;ii++)			//for every timeout
		{
			//if there is a timeout with enough misses, trigger an alarm
			if(bounceTimeout[ii].time==0 &&	bounceTimeout[ii].miss>=BOUNCE_MISS)
				startAlarm();
		}
		#endif

		//window width reached 
		if(timerTimeout==0) 			//timeout reached
		{	

			#ifdef _DEBUG_TEST
				short i;
				for(i=0;i<C2;i++)		//sensore centrale C2
				{
					BUZZ=1;				//start beep
					delay_ms(200);
					BUZZ=0;
					delay_ms(200);
				}	
				delay_ms(1500);
			#endif

			#ifndef _DEBUG_TEST
			if(C1<=PULSE_TRIGGER || C2<=PULSE_TRIGGER || C3<=PULSE_TRIGGER)		//are we missing some pulses?
			{
				#ifdef _BOUNCEMODE
					bounceTimeout[j].time=BOUNCE_TIMEOUT;	//start a new timeout
					bounceTimeout[j].miss=0;				//with zero miss
					for(ii=0;ii<BOUNCE_WIDTH;ii++)			//for every timeout
					{
						if(bounceTimeout[ii].time>0)		//if timeout is running
							bounceTimeout[ii].miss++;		//lets add a miss!
					}
					j++;									//ready for the next timeout
					if(j>=BOUNCE_WIDTH)j=0;					//reset in case of overflow
				#endif
				#ifndef _BOUNCEMODE
					startAlarm();
					delay_ms(100);
				#endif	
			}
			#endif
			
			countersReset();
		}

	}//end while
}//end main
