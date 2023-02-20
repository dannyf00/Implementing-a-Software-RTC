#include "avrduino.h"					//we use avrduino

//AVRduino code
// - supported chips: ATmega328
// - free running timer0 for ticks, 8-bit mode, 256:1 prescaler
// - details:
//    - uart transmission supported
// - version history
// - v0.1, 11/06/2022: initial implementation: gpio, systick, uart, timer0/1/2, output compares, pwm output
// - v0.2, 11/07/2022: adc implemented, including oversampling and reading temperature sensor
// - v0.2a,02/20/2023: implemented software RTC
//
//
//               ATmega328
//              |=====================|
//    Vcc       |                     |
//     |        |                     |
//     |        |                     |         |
//     +-[10K]-<| MCLR                |
//              |                     |
//     +------->| OSCI            Vdd |>--+------>Vcc
//  [Xtal]      |                     | [.1u]
//     +-------<| OSCO            Vss |>--+------>GND
//              |                     |
//              |                 PB0 |>--------->UartTX
//              |                     |
//              |                 PB1 |>--------->UartRX
//              |                     |
//              |                 PB5 |>--------->LED (ardino pin13)
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

//hardware configuration
#define LED			RB5				//led pin on PB5/pin 13
#define LED2		RC2				//led2 pin
#define LED_DLY		(F_CPU / 2)			//half a second
//end hardware configuration

//global defines

//global variables

//flip led2
void led2_flp(void) {
	pinFlip(LED2);
}
//user defined set up code
void setup(void) {
    pinMode(LED, OUTPUT);				//led as output pin
    pinMode(LED2,OUTPUT);

    //initialize the uart
    uart0Init(UART_BR38K4);				//initialize uart1
    //uart2Init(UART_BR9600);			//initialize uart2

	//output compare
	//tmr2Init(); tmr2SetPS(TMR_PS256x);
	//tmr2OCAInit(); tmr2OCASetPR(100); tmr2OCAAttachISR(led2_flp);
	//tmr2OCBInit(); tmr2OCBSetPR(101); tmr2OCBAttachISR(led2_flp);

	//pwm
	tmr0Init(); analogWrite(RD6, 100); analogWrite(RD5, 200);
	tmr1Init(); analogWrite(RB1, 10); analogWrite(RB2, -10);
	tmr2Init(); analogWrite(RB3, 10); analogWrite(RD3, -10);

	//initialize the software RTC
	sRTCInit(); time2sRTC(1234567890ul);

	//adc
	adcInit();
}


//user defined main loop
void loop(void) {
    static uint32_t tick0=0;
    uint32_t tmp0;
    time_t tmp_time;

    sRTCISR();									//update software RTC

    //if enough time has elapsed
    digitalWrite(LED2, sRTCHalfsec());			//flip led2
    if (ticks() - tick0 > LED_DLY) {			//if enough time has passed
        tick0 += LED_DLY;						//advance to the next match point
        pinFlip(LED);							//digitalWrite(LED, !digitalRead(LED));	//flip led, 105 ticks

		//measure timing
		//put some tasks here
        tmp0=ticks();
        //u0Print("F_CPU=                    ", F_CPU);
        //adcRead(ADC_VREF);
        sRTCISR();
		tmp0=ticks() - tmp0;

		//display something
        u0Print("F_CPU =                    ", F_CPU);
        u0Print("ticks =                    ", ticks());
        //u0Print("tmp0  =                    ", tmp0);
        //u0Print("UBRR0 =                    ", UBRR0);
        //u0Print("u0bps =                    ", u0bps());
        u0Print("rtcSec=                    ", sRTC2time(&tmp_time)); uart0Puts(ctime(&tmp_time));
		u0Println();
    }
}
