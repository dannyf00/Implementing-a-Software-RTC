//dsPIC33duino code
// - supported chips: dspic33fj32GP302/304, dspic33fj64gp202/204/802/804, dspic33fj128gp202/204/802/804
// - free running timer2 for ticks, pwm and input capture
// - details: https://github.com/dannyf00/Minimalist-16-bit-Arduino-Clone
// - only XC16 support is provided
//
// - version history
// - v0.1, 12/29/2022: initial porting from pic24duino
//
//
//               dsPIC33FJ
//              |=====================|
//    Vcc       |                     |
//     |        |                Vcap |>--[.1u]-+->GND
//     |        |                     |         |
//     +-[10K]-<| MCLR        DISVreg |>--------+
//              |                     |
//              |                     |
//     +------->| OSCI            Vdd |>--+------>Vcc
//  [Xtal]      |                     | [.1u]
//     +-------<| OSCO            Vss |>--+------>GND
//              |                     |
//              |                 RP0 |>---------->Uart2TX
//              |                     |
//              |                 RB5 |>---------->LED
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |                     |
//              |=====================|
//
//

#include "dspic33duino.h"					//we use dspic33duino

//hardware configuration
#define LED			PB5					//led pin
#define LED_DLY		(F_PHB / 2)			//half a second
//end hardware configuration

#define TMR_TICK	100					//srtc timer tick rate -> so that F_CPU / TMR_TICK fits a 16-bit output compare register

//software RTC
//global defines
#define sRTC_TKS	1000000ul			//srtc tick rate per second

//global variables
typedef struct {
	time_t time;						//second counter since 1/1/1970
	uint32_t tick;						//tick counter
	int16_t cal;						//rtc calibration, in ppm. +=faster, -=slower
	//uint8_t halfsec;					//0=first half sec, 1=2nd half sec -> not implemented
} sRTC_TypeDef;

sRTC_TypeDef sRTC={0, 0, 0};			//software RTC	

//initialize software counter
void sRTCInit(int8_t cal) {				//calibration from -128ppm to +128ppm, if sRTC_RATE = 1M
	//sRTC.tick_rate=sRTC_RATE;
	sRTC.time=0;						//reset counter
	sRTC.cal=cal;						//initialize calibration
}

//increment -> called every second
void sRTCISR(void) {
	sRTC.tick += sRTC_TKS+sRTC.cal;		//increment tick, with error correction
	while (sRTC.tick>sRTC_TKS) {
		sRTC.tick-=sRTC_TKS;			//update tick
		sRTC.time+=1;					//increment sec
	}
}

//read rtc second counter
time_t sRTC2time(void) {
	return sRTC.time;
}

//read rtc tick
uint32_t sRTCTick(void) {
	return sRTC.tick;
}
	
//turn mktime (=time_t) to RTC
void time2sRTC(time_t t) {
	sRTC.time=t;
}
//end software RTC

//oc1isr
void oc1isr(void) {
	static int oc1cnt=TMR_TICK;
	
	if (oc1cnt--==0) {
		//oc1isr has been called TMR_TICK -> 1 sec has arrived
		oc1cnt=TMR_TICK;				//reset oc1
		sRTCISR();						//update the counter
	}
}
		
//flip led
void led_flp(void) {
	pinFlip(LED);						//flip led
}

//user defined set up code
void setup(void) {
    //SystemCoreClockFRC();				//FRC as main clock
    SystemCoreClockPOSC();				//set posc as main clock
    
    pinMode(LED, OUTPUT);				//led as output pin

    //initialize the uart
    //uart1Init(UART_BR9600);			//initialize uart1
    uart2Init(UART_BR38K4);				//initialize uart2

	//initialize software RTC
	sRTCInit(+10);						//initialize sRTC, with calibration of +10ppm
	time2sRTC(1234567890ul);			//initialize sRTC second counter to Friday, February 13, 2009 11:31:30 PM
	
	//generate a 1pps call mechanism
	//using output compare 1, at a rate specified by TMR_TICK (=100x)
	//oc1isr called 100x per sec. so it needs to call sRTCISR() once 100 calls
	//to yield a 1 call / sec rate for sRTCISR()
	oc1Init(F_CPU/TMR_TICK);			//output compare set to trigger 100x per sec (to yield a 16-bit compare register)
	oc1AttachISR(oc1isr);				//install oc1isr every 10ms (=100x calls per sec)	
	
    //enable interrupts
    ei();

}


//user defined main loop
void loop(void) {
    static uint32_t tick0=0;
    uint32_t tmp0;
    time_t tmp_time;

    //if enough time has elapsed
    if (ticks() - tick0 > LED_DLY) {			//if enough time has passed
        tick0 += LED_DLY;						//advance to the next match point
        pinFlip(LED);							//digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks

        //measure timing
        tmp0=ticks();
        //put some tasks here
        tmp0=ticks() - tmp0;

        //display something
        //u2Print("F_CPU =                    ", F_CPU);
        u2Print("ticks =                    ", ticks());
        //u2Print("tmp0  =                    ", tmp0);
        u2Print("sRTCTk=                    ", sRTCTick());
        u2Print("sRTCtm=                    ", tmp_time=sRTC2time());
        uart2Puts(ctime(&tmp_time));
        u2Println();
    }
}
