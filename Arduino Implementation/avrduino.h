#ifndef _AVRDUINO_H
#define _AVRDUINO_H

//AVRduino code
// - supported chips: ATmega328
// - free running timer0 for ticks, 8-bit mode, 256:1 prescaler
// - details:
//    - uart transmission supported
// - version history
// - v0.1, 11/06/2022: initial implementation: gpio, systick, uart, timer0/1/2, output compares, pwm output
// - v0.2, 11/07/2022: adc implemented, including oversampling and reading temperature sensor
// - v0.2a,02/20/2023: implemented software RTC, int0/1, and pcint0/1/2
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

#include <stdint.h>								//we use strcpy()
#include <string.h>								//we use strcpy()
#include <time.h>								//we use time functions for software RTC
#if defined(__GNUC__)
	#include <stdint.h>							//uint8_t ... types
	#include <avr/io.h>							//we use avr
	#include <avr/interrupt.h>
	#include <avr/boot.h>						//we read fuse bits
#elif defined(__ICCAVR__)
//#if defined(__EWAVR__)						//alternatively
	#include <ioavr.h>							//we use iar avr
	#include <stdint.h>							//have to use dlib (General Options->Library Configuration->Normal DLIB
	#include <intrinsics.h>						//we use _nop_(). IAR AVR only
#else
	#warning "only GCC AVR and IAR AVR compilers supported"	//need to put something here if not covered by gcc/iar avr
#endif

//hardware configuration
//oscillator configuration
#define F_XTAL				16000000ul			//crystal frequency, user-specified
#define F_SOSC				32768				//SOSC = 32768Hz, user-specified

//#define UART0_TXISR							//uncomment if uart tx via polling
#define uRAM_SIZE			40					//size of uRAM
#define F_SPI				1000000ul			//bitrate for spi
#define F_I2C				100000ul			//bit rate for hardware i2c
#define F_sI2C				100000ul			//bit rate for software i2c
#define ADC_DLY()			NOP32()				//need 20us per sample

//define pins
#define U0TXPIN()			IO_OUT(DDRB, 1<<0)	//U0TX on DDRB0
#define U0RXPIN()			IO_IN(DDRB, 1<<1)	//U0RX on DDRB1
#define sI2CSCLPIN			PB1
#define sI2CSDAPIN			PB2
#define PWM0A_PIN()			IO_OUT(DDRD, 1<<6)	//oc0a on pd6
#define PWM0B_PIN()			IO_OUT(DDRD, 1<<5)	//oc0a on pd5
//end user specification

#define F_CPU				(SystemCoreClock)	//user defined F_CPU
#define F_PHB				F_CPU				//cpu runs at F_SYS/4 by default -> Fxtal = 8Mhz. *4 for PLL. RCDIV set to 0 (1:1 postscaler)
#define F_FRC				F_RC8M				//FRC frequency = 8Mhz, fixed (divide by 1)
#define F_LPOSC				F_SOSC				//low frequency oscillator, 31Khz
#define F_RC128K			128000ul			//low freuqency internal RC oscillator, 128Khz
#define F_RC8M				8000000ul			//calibrated internal RC oscillator, 8Mhz
extern uint32_t SystemCoreClock;				//pheriphral core clock, before dividing by 2

#define PWM_PR				0x00ff				//pwm period for tmr1 (10-bit pwm, mode 7) - don't change

//port manipulation macros for PIC.
#define IO_SET(port, bits)              port |= (bits)			//set bits on port
#define IO_CLR(port, bits)              port &=~(bits)			//clear bits on port
#define IO_FLP(port, bits)              port ^= (bits)			//flip bits on port
#define IO_GET(port, bits)              ((port) & (bits))		//return bits on port
#define IO_OUT(ddr, bits)               ddr |= (bits)			//set bits as output
#define IO_IN(ddr, bits)                ddr &=~(bits)			//set bits as input

//tiny scheduler macro
#define TS_RUN_WHILE(cs)	if (cs)						//tiny scheduler macro
#define TSwhile(cs)			TS_RUN_WHILE(cs)			//shorter marco


//#define NOP()				Nop()                           //asm("nop")					//nop()
#define NOP2()				{NOP(); NOP();}
#define NOP4()				{NOP2(); NOP2();}
#define NOP8()				{NOP4(); NOP4();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP16()				{NOP8(); NOP8();}
#define NOP24()				{NOP16(); NOP8();}
#define NOP32()				{NOP16(); NOP16();}
#define NOP40()				{NOP32(); NOP8();}
#define NOP64()				{NOP32(); NOP32();}

#define sleep()				asm("sleep")						//put the mcu into sleep

#ifndef ei
#define ei()				sei()							//asm volatile ("ei")				//__builtin_enable_interrupts()	//do {INTEnableInterrupts();	INTEnableSystemMultiVectoredInt();} while (0)	//__builtin_enable_interrupts()
#endif

#ifndef di
#define di()				cli()							////asm volatile ("di")				//__builtin_enable_interrupts()	//INTDisableInterrupts()			//__builtin_disable_interrupts()	//
#endif

//simple multiples
#define x1(val)				(val)								//multiply val by 1
#define x2(val)				(((val) << 1))						//multiply val by 2
#define x3(val)				(x2(val) + (val))					//multiply val by 3
#define x4(val)				(((val) << 2))						//multiply val by 4
#define x5(val)				(x4(val) + (val))					//multiply val by 5
#define x6(val)				(x4(val) + x2(val))					//multiply val by 6
#define x7(val)				(x6(val) + (val))					//multiply val by 7
#define x8(val)				((val) << 3)						//multiply val by 8
#define x9(val)				(x8(val) + (val))					//multiply val by 9

//multiples of 10s
#define x10(val)			(x8(val) + x2(val))					//multiply val by 10
#define x100(val)			(x10(x10(val)))						//multiply val by 100
#define x1000(val)			(x100(x10(val)))					//multiply val by 1000
#define x1k(val)			x1000(val)							//multiply val by 1000
#define x10k(val)			(x100(x100(val)))					//multiply val by 10000

#define x20(val)			(x2(x10(val)))
#define x30(val)			(x3(x10(val)))
#define x40(val)			(x4(x10(val)))
#define x50(val)			(x5(x10(val)))
#define x60(val)			(x6(x10(val)))
#define x70(val)			(x7(x10(val)))
#define x80(val)			(x8(x10(val)))
#define x90(val)			(x9(x10(val)))

//multiples of 100s
#define x200(val)			(x2(x100(val)))
#define x300(val)			(x3(x100(val)))
#define x400(val)			(x4(x100(val)))
#define x500(val)			(x5(x100(val)))
#define x600(val)			(x6(x100(val)))
#define x700(val)			(x7(x100(val)))
#define x800(val)			(x8(x100(val)))
#define x900(val)			(x9(x100(val)))

//custom definitions
#define x34(val)			(x30(val) + x4(val))				//multiply val by 34
#define x97(val)			(x90(val) + x7(val))				//multiply val by 97x

//global defines

//gpio definitions

//pin enum - matches GPIO_PinDef[]
//may need to modify the headerfile for a few pins, like PA1, PA2, ...
typedef enum {
#if defined(PORTA)
	RA0, RA1, RA2, RA3, RA4, RA5, RA6, RA7, //RA8, RA9, RA10, RA11, RA12, RA13, RA14, RA15,
#endif
#if defined(PORTB)
	RB0, RB1, RB2, RB3, RB4, RB5, RB6, RB7, //RB8, RB9, RB10, RB11, RB12, RB13, RB14, RB15,
#endif
#if defined(PORTC)
	RC0, RC1, RC2, RC3, RC4, RC5, RC6, RC7, //RC8, RC9, RC10, RC11, RC12, RC13, RC14, RC15,
#endif		//_PORTC
#if defined(PORTD)
	RD0, RD1, RD2, RD3, RD4, RD5, RD6, RD7, //RD8, RD9, RD10, RD11, RD12, RD13, RD14, RD15,
#endif		//_PORTD
#if defined(PORTE)
	RE0, RE1, RE2, RE3, RE4, RE5, RE6, RE7, //RE8, RE9, RE10, RE11, RE12, RE13, RE14, RE15,
#endif		//_PORTE
#if defined(PORTF)
	RF0, RF1, RF2, RF3, RF4, RF5, RF6, RF7, //RF8, RF9, RF10, RF11, RF12, RF13, RF14, RF15,
#endif		//_PORTF
#if defined(PORTG)
	RG0, RG1, RG2, RG3, RG4, RG5, RG6, RG7, //RG8, RG9, RG10, RG11, RG12, RG13, RG14, RG15,
#endif		//_PORTG
#if defined(PORTH)
	RH0, RH1, RH2, RH3, RH4, RH5, RH6, RH7, //RH8, RH9, RH10, RH11, RH12, RH13, RH14, RH15,
#endif		//_PORTH
	PTEMP, PDAC, PFVR,			//analog temp sensor, CVref output, and internal reference. for analogRead()
	PMAX
} PIN_TypeDef;

#define INPUT				0
#define OUTPUT				1			//(!INPUT)
#define INPUT_PULLUP		2			//pull_up

#define LOW					0
#define HIGH				(!LOW)

#define PI 					3.1415926535897932384626433832795
#define HALF_PI 			(PI / 2)							//1.5707963267948966192313216916398
#define TWO_PI 				(PI + PI)							//6.283185307179586476925286766559
#define DEG_TO_RAD 			(TWO_PI / 360)						//0.017453292519943295769236907684886
#define RAD_TO_DEG 			(360 / TWO_PI)						//57.295779513082320876798154814105
#define EULER 				2.718281828459045235360287471352	//Euler's number

#define SERIAL  			0x0
#define DISPLAY 			0x1

#define LSBFIRST 			0
#define MSBFIRST 			1									//(!LSBFIRST)							//1

#define CHANGE 				1
#define FALLING 			2
#define RISING 				3

#ifndef min
#define min(a,b) 			((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) 			((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) 				((x)>0?(x):-(x))
#endif
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMillisecond() 	( F_PHB / 1000L )
#define clockCyclesPerMicrosecond() 	( F_PHB / 1000000L )
#define clockCyclesToMicroseconds(a) 	( (a) / clockCyclesPerMicrosecond() )
#define microsecondsToClockCycles(a) 	( (a) * clockCyclesPerMicrosecond() )

#define lowByte(w) 			((uint8_t) ((w) & 0xff))
#define highByte(w) 		((uint8_t) ((w) >> 8))

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) 	((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bit(n)				(1ul<<(n))

#define false				0
#define true				(!false)

//characters
#define isAlphaNumeric(c)	isalnum(c)
#define isAlpha(c)			isalpha(c)
#define isAscii(c)			isascii(c)
#define isWhitespace(c)		isblank(c)
#define isControl(c)		iscntrl(c)
#define isDigit(c)			isdigit(c)
#define isGraph(c)			isgraph(c)
#define isLowerCase(c)		islower(c)
#define isPrintable(c)		isprint(c)
#define isPunct(c)			ispunct(c)
#define isSpace(c)			isspace(c)
#define isUpperCase(c)		isupper(c)
#define isHexadecimalDigit(c)	isxdigit(c)

//external setup/loop - defined by user
extern void setup(void);
extern void loop(void);

//random number
#define randomSeed(seed)	srand(seed)
#define random(max)			random2(0, max)
#define random2(min, max)	((min) + (int32_t) ((max) - (min)) * rand() / 32768)

//GPIO
//flip the pin
#define pinFlip(pin)		digitalWrite(pin, !digitalRead(pin))
void pinMode(PIN_TypeDef pin, uint8_t mode);
void digitalWrite(PIN_TypeDef pin, uint8_t mode);
char digitalRead(PIN_TypeDef pin);

//time base
uint32_t systicks(void);							//use tmr0 as systick
#define ticks()				systicks()				//for compatability
#define coreticks()			ticks()					//for compatability
#define millis()			(ticks() / cyclesPerMillisecond())
#define micros()			(ticks() / cyclesPerMicrosecond())
void delayTks(uint32_t tks);						//delay a given number of ticks
//void delayUs(uint32_t us);						//delay a given number of micro seconds
//void delayMs(uint32_t ms);						//delay a given number of millie seconds
#define delayUs(us)				delayTks((us) * cyclesPerMicrosecond())
#define delayMs(ms)				delayTks((ms) * cyclesPerMillisecond())
#define delay(ms)				delayMs(ms)
#define delayMilliseconds(ms)	delayMs(ms)
#define delayMicroseconds(us)	delayUs(us)
#define cyclesPerMillisecond()	(clockCyclesPerMillisecond())
#define cyclesPerMicrosecond()	(clockCyclesPerMicrosecond())

//advanced IO
//void tone(void);									//tone frequency specified by F_TONE in STM8Sduino.h
//void noTone(void);
//shiftin/out: bitOrder = MSBFIRST or LSBFIRST
uint8_t shiftIn(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder);
void shiftOut(PIN_TypeDef dataPin, PIN_TypeDef clockPin, uint8_t bitOrder, uint8_t val);
uint32_t pulseIn(PIN_TypeDef pin, uint8_t state);		//wait for a pulse and return timing

//pwm output
void analogWrite(uint8_t pin, uint16_t dc);

//analog read on ADC1
//read DRL first for right aligned results
//uint16_t analogRead(uint8_t pin);

//analog reference - default to AVdd-AVss
//Vref sources: 0->Vref = AVdd-AVss, 1->Vref+-AVss, 2->AVdd-Vref-, 3->Vref+ - Vref-
//void analogReference(uint8_t Vref);

//interrupts
//install external interrupt handler
//mode 1: falling edge, 0: rising edge
//void attachInterrupt(uint8_t intx, void (*isrptr) (void), uint8_t mode);
//void detachInterrupt(uint8_t intx);

//change notification interrupts
//install user CN interrupt handler
//void attachCNInterrupt(void (*isrptr) (void));
//void detachCNInterrupt(void);
//void activateCNInterrupt(uint8_t cnx, uint8_t pue);
//void deactivateCNInterrupt(uint8_t cnx);

//global variables


uint32_t dsID(void);				//return 24-bit dsvice id
uint8_t rcCalib(void);				//return rc oscillabor calibration byte
uint8_t tsOfst(void);				//return TS offset
uint8_t tsGain(void); 				//return TS offset
//reset the mcu
void mcuInit(void);
//111 = Fast RC Oscillator with Postscaler (FRCDIV)
//110 = Reserved
//101 = Low-Power RC Oscillator (LPRC)
//100 = Secondary Oscillator (SOSC)
//011 = Primary Oscillator with PLL module (XTPLL, HSPLL, ECPLL)
//010 = Primary Oscillator (XT, HS, EC)
//001 = Fast RC Oscillator with Postscaler and PLL module (FRCPLL)
//000 = Fast RC Oscillator (FRC)
//Needs to be executed during mcu initialization or after oscillator reconfiguration
//updates SystemCoreClock.
uint32_t SystemCoreClockUpdate(void);

//switch oscillator
//return SystemCoreClock in frequency

//empty interrupt handler
void empty_handler(void);


//#define Mhz					000000ul	//suffix for Mhz
#define F_UART				(F_PHB)		//peripheral clock
#define UART_BR300			300ul		//baudrate=300
#define UART_BR600			600ul		//baudrate=600
#define UART_BR1200			1200ul		//baudrate=1200
#define UART_BR2400			2400ul		//baudrate=2400
#define UART_BR4800			4800ul		//baudrate=4800
#define UART_BR9600			9600ul		//baudrate=9600
#define UART_BR14400		14400ul		//baudrate=19200
#define UART_BR19200		19200ul		//baudrate=19200
#define UART_BR38400		38400ul		//baudrate=38400
#define UART_BR43000		43000ul		//baudrate=38400
#define UART_BR57600		57600ul		//baudrate=57600
#define UART_BR76800		76800ul		//baudrate=57600
#define UART_BR115200		115200ul	//baudrate=115200
#define UART_BR128000		128000ul	//baudrate=115200
#define UART_BR230400		230400ul	//baudrate=115200
#define UART_BR250000		250000ul	//baudrate=250000
#define UART_BR256000		256000ul	//baudrate=115200
#define UART_BR460800		460800ul	//baudrate=115200
#define UART_BR512000		512000ul	//baudrate=115200
#define UART_BR576000		576000ul	//baudrate=115200
#define UART_BR921600		921600ul	//baudrate=115200
#define UART_BR1000000		1000000ul	//baudrate=115200
#define UART_BR1250000		1250000ul	//baudrate=115200
#define UART_BR1500000		1500000ul	//baudrate=115200
#define UART_BR2000000		2000000ul	//baudrate=115200
//short notations
#define UART_BR0K3			UART_BR300
#define UART_BR0K6			UART_BR600
#define UART_BR1K2			UART_BR1200
#define UART_BR2K4			UART_BR2400
#define UART_BR4K8			UART_BR4800
#define UART_BR9K6			UART_BR9600
#define UART_BR14K4			UART_BR14400
#define UART_BR19K2			UART_BR19200
#define UART_BR38K4			UART_BR38400
#define UART_BR43K			UART_BR43000
#define UART_BR57K6			UART_BR57600
#define UART_BR76K8			UART_BR76800
#define UART_BR115K2		UART_BR115200
#define UART_BR128K			UART_BR128000
#define UART_BR230K4		UART_BR230400
#define UART_BR250K			UART_BR250000
#define UART_BR256K			UART_BR256000
#define UART_BR460K8		UART_BR460800
#define UART_BR512K			UART_BR512000
#define UART_BR576K			UART_BR576000
#define UART_BR1M			UART_BR1000000
#define UART_BR1M25			UART_BR1250000
#define UART_BR1M5			UART_BR1500000
#define UART_BR2M			UART_BR2000000

//initiate the hardware usart/uart1
void uart0Init(uint32_t bps);//initialize uart
#define uart0Putch(ch)		do {uart0Wait(); UDR0 = (ch);} while (0)	//void uart1Putch(char ch);				//output a char on uart
void uart0Puts(char *str);				//output a string on uart
//void uart1Putline(char *ln);			//output a string + linefeed on uart
#define uart0Putline(str)	do {uart0Puts(str); uart0Puts("\r\n");} while (0)
#define uart0Getch()		(UDR0)		//uint8_t uart1Getch(void);				//read a char from usart
#define uart0Available()	(UCSR0A & (1<<RXC0))	//U1STAbits.URXDA	//uint16_t uart1Available(void);			//test if data rx is available
#if defined(UART0_TXISR)
#define uart0Busy()			(_U0TX_BUSY)
#else
#define uart0Busy()			((UCSR0A & (1<<UDRE0))==0)	//IEC0bits.U1TXIE)	//(!U1STAbits.TRMT)	//uint16_t uart1Busy(void);				//test if uart tx is busy
#endif
#define uart0Wait()			while (uart0Busy())
void u0Print(const char *str, int32_t dat);	//output a number on uart
#define u0Println()			uart0Puts((char *)"\r\n")
//for compatability
#define uart0Put(ch)		uartPutch(ch)
#define uart0Get()			uartGetch()
#define u0bps()				(F_UART / ((UCSR0A & (1<<U2X0))?8:16) / (UBRR0 + 1))

//initiate the hardware usart1 - not all chips have uart1
void uart1Init(uint32_t bps);//initialize uart
#define uart1Putch(ch)		do {uart1Wait(); UDR1 = (ch);} while (0)	//void uart1Putch(char ch);				//output a char on uart
void uart1Puts(char *str);				//output a string on uart
//void uart1Putline(char *ln);			//output a string + linefeed on uart
#define uart1Putline(str)	do {uart1Puts(str); uart1Puts("\r\n");} while (0)
#define uart1Getch()		(UDR1)		//uint8_t uart1Getch(void);				//read a char from usart
#define uart1Available()	(UCSR1A & (1<<RXC1))	//U1STAbits.URXDA	//uint16_t uart1Available(void);			//test if data rx is available
#if defined(UART1_TXISR)
#define uart1Busy()			(_U1TX_BUSY)
#else
#define uart1Busy()			((UCSR1A & (1<<UDRE1))==0)	//IEC0bits.U1TXIE)	//(!U1STAbits.TRMT)	//uint16_t uart1Busy(void);				//test if uart tx is busy
#endif
#define uart1Wait()			while (uart1Busy())
void u1Print(const char *str, int32_t dat);	//output a number on uart
#define u1Println()			uart1Puts((char *)"\r\n")
//for compatability
#define uart1Put(ch)		uart1Putch(ch)
#define uart1Get()			uart1Getch()
#define u1bps()				(F_UART / ((UCSR1A & (1<<U2X0))?8:16) / (UBRR1 + 1))

//initiate the hardware usart2 - not all chips have uart2
void uart2Init(uint32_t bps);//initialize uart
#define uart2Putch(ch)		do {uart2Wait(); UDR2 = (ch);} while (0)	//void uart1Putch(char ch);				//output a char on uart
void uart2Puts(char *str);				//output a string on uart
//void uart2Putline(char *ln);			//output a string + linefeed on uart
#define uart2Putline(str)	do {uart2Puts(str); uart2Puts("\r\n");} while (0)
#define uart2Getch()		(UDR2)		//uint8_t uart1Getch(void);				//read a char from usart
#define uart2Available()	(UCSR2A & (1<<RXC2))	//U1STAbits.URXDA	//uint16_t uart1Available(void);			//test if data rx is available
#if defined(UART2_TXISR)
#define uart2Busy()			(_U2TX_BUSY)
#else
#define uart2Busy()			((UCSR2A & (1<<UDRE2))==0)	//IEC0bits.U1TXIE)	//(!U1STAbits.TRMT)	//uint16_t uart1Busy(void);				//test if uart tx is busy
#endif
#define uart2Wait()			while (uart2Busy())
void u2Print(const char *str, int32_t dat);	//output a number on uart
#define u2Println()			uart2Puts((char *)"\r\n")
//for compatability
#define uart2Put(ch)		uart2Putch(ch)
#define uart2Get()			uart2Getch()
#define u2bps()				(F_UART / ((UCSR2A & (1<<U2X0))?8:16) / (UBRR2 + 1))

//end Serial


//tmr prescaler for tmr0/1
#define TMR_PS1x			0x01
#define TMR_PS8x			0x02
#define TMR_PS64x			0x03
#define TMR_PS256x			0x04
#define TMR_PS1024x			0x05
#define TMR_PSMASK			0x07
//tmr prescaler for tmr2
#define TMR2_PS1x			0x01
#define TMR2_PS8x			0x02
#define TMR2_PS32x			0x03
#define TMR2_PS64x			0x04
#define TMR2_PS128x			0x05
#define TMR2_PS256x			0x06
#define TMR2_PS1024x		0x07
#define TMR2_PSMASK			0x07

//tmrs
void tmr0Init(void);							//256:1 prescaler -> ~100hz
void tmr0AttachISR(void (*isr_ptr)(void));	//install tmr handler
#define tmr0SetPS(ps)		TCCR0B = (TCCR0B &~TMR_PSMASK) | ((ps) & TMR_PSMASK)	//set the prescaler
#define tmr0GetPS()			(TCCR0B & TMR_PSMASK)
#define tmr0Get()			TMR0
//output compares
extern volatile uint8_t _tmr0oca_pr, _tmr0ocb_pr;
void tmr0OCAInit(void);		//reset the output compare ch a
void tmr0OCAAttachISR(void (*isr_ptr)(void));	//install user handler
#define tmr0OCASetPR(pr)	_tmr0oca_pr = (pr)
#define tmr0OCAGetPR()		_tmr0oca_pr
void tmr0OCBInit(void);		//reset the output compare ch b
void tmr0OCBAttachISR(void (*isr_ptr)(void));	//install user handler#define tmr0OCBSetPR(pr)	OCF0B = (pr)
#define tmr0OCBSetPR(pr)	_tmr0ocb_pr = (pr)
#define tmr0OCBGetPR()		_tmr0ocb_pr

void tmr1Init(void);						//reset tmr, //prescaler = 64:1 -> 100hz
void tmr1AttachISR(void (*isr_ptr)(void));	//install tmr handler
#define tmr1SetPS(ps)		TCCR1B = (TCCR1B &~TMR_PSMASK) | ((ps) & TMR_PSMASK)	//set the prescaler
#define tmr1GetPS()			(TCCR1B & TMR_PSMASK)
#define tmr1Get()			TMR1
//output compares
extern volatile uint16_t _tmr1oca_pr, _tmr1ocb_pr;
void tmr1OCAInit(void);		//reset the output compare ch a
void tmr1OCAAttachISR(void (*isr_ptr)(void));	//install user handler
#define tmr1OCASetPR(pr)	_tmr1oca_pr = (pr)
#define tmr1OCAGetPR()		_tmr1oca_pr
void tmr1OCBInit(void);		//reset the output compare ch b
void tmr1OCBAttachISR(void (*isr_ptr)(void));	//install user handler#define tmr0OCBSetPR(pr)	OCF0B = (pr)
#define tmr1OCBSetPR(pr)	_tmr1ocb_pr = (pr)
#define tmr1OCBGetPR()		_tmr1ocb_pr

void tmr2Init(void);		//reset tmr, //mode 0, prescaler = 256:1 -> 100hz
void tmr2AttachISR(void (*isr_ptr)(void));	//install tmr handler
#define tmr2SetPS(ps)		TCCR2B = (TCCR2B &~TMR_PSMASK) | ((ps) & TMR_PSMASK)	//set the prescaler
#define tmr2GetPS()			(TCCR2B & TMR_PSMASK)
#define tmr2Get()			TMR2
//output compares
extern volatile uint8_t _tmr2oca_pr, _tmr2ocb_pr;
void tmr2OCAInit(void);		//reset the output compare ch a
void tmr2OCAAttachISR(void (*isr_ptr)(void));	//install user handler
#define tmr2OCASetPR(pr)	_tmr2oca_pr = (pr)
#define tmr2OCAGetPR()		_tmr2oca_pr
void tmr2OCBInit(void);		//reset the output compare ch b
void tmr2OCBAttachISR(void (*isr_ptr)(void));	//install user handler#define tmr0OCBSetPR(pr)	OCF0B = (pr)
#define tmr2OCBSetPR(pr)	_tmr2ocb_pr = (pr)
#define tmr2OCBGetPR()		_tmr2ocb_pr
//end tmr

//pwm
//use analogWrite()
//end pwm

//adc
//adc channels
//for mega48/88/168/328, gcc-avr and iar-avr
//input channel - for atmega48/88/168/328
#define ADC_0			0x00			//adc channels
#define ADC_1			0x01			//adc channels
#define ADC_2			0x02			//adc channels
#define ADC_3			0x03			//adc channels
#define ADC_4			0x04			//adc channels
#define ADC_5			0x05			//adc channels
#define ADC_6			0x06			//adc channels
#define ADC_7			0x07			//adc channels
#define ADC_TEMP		0x08			//adc channels - temperature sensor
//0X09 - 0X0D RESERVED
#define ADC_VREF		0x0e			//1.1v internal bandgap
#define ADC_GND			0x0f			//gnd

//adc reference voltage
void adcInit(void);						//reset the adc
int16_t adcRead(uint8_t ch);			//read an adc channel
int16_t adcReadN(uint8_t ch, uint16_t N);			//read an adc channel, with oversampling
int16_t analogRead(PIN_TypeDef pin);	//read the adc pin
//set adc reference: 0=AVref pin, 1=AVcc pin, 3=internal 1.1v Vref
#define analogReference(ch)	ADMUX = (ADMUX &~(3<<REFS0)) | (((ch) & 0x03)<<REFS0)
int32_t adcReadTempCx100(void);			//read temperature, 8x oversampling, in 0.01C
#define adcReadTempFx100()	(adcReadTempCx100() * 9 / 5 + 3200)	//read temperature, 8x oversampling, in 0.01F

//lifted from avr/boot.h
//original def is mising SIGRD (5)
#ifndef SIGRD
//datasheet indicating this bit is read only in SPMCSR
//ATmel examples wants to set this bit however -> this is weird
#define SIGRD		5
#endif	//sigrd
//end ADC

//external interrupt int0 / int1
void int0Init(void);								//external interrupt on int0
void int0AttachISR(void (*isr_ptr)(void));			//install user isr
void int1Init(void);								//external interrupt on int0
void int1AttachISR(void (*isr_ptr)(void));			//install user isr
//external interrupt

//pin change interrupt pcint0/1/2
void pcint0Init(void);								//external interrupt on int0
void pcint0AttachISR(void (*isr_ptr)(void));			//install user isr
void pcint1Init(void);								//external interrupt on int0
void pcint1AttachISR(void (*isr_ptr)(void));			//install user isr
void pcint2Init(void);								//external interrupt on int0
void pcint2AttachISR(void (*isr_ptr)(void));			//install user isr
//end pin change interrupt

//spi-master mode
#define SPI_WAIT()						while (!SSPSTATbits.BF) continue		//wait for transmission to finish
#define spiRead()						spiWrite(0x00)				//get a byte
void spiInit(uint32_t bps);				//spi - master mode only
char spiWrite(char dat);				//spi - send data

#define SPI2_WAIT()						while (!SSP2STATbits.BF) continue		//wait for transmission to finish
#define spi2Read()						spi2Write(0x00)				//get a byte
void spi2Init(uint32_t bps);			//spi - master mode only
char spi2Write(char dat);				//spi - send data

//end spi

//i2c
//i2c commands
#define I2C_ACK				0							//0-> ack, to be consistent with i2c protocol
#define I2C_NOACK			1							//1-> no ack
#define I2C_CMD_READ		0x01						//i2c command for read
#define I2C_CMD_WRITE		0x00						//i2c command for write
#define I2C_ACK_ERROR		0x09						//max number of loops before ack times out

//initialize i2c
void i2cInit(void);
//-----------------START Condition-----------------------
void i2cStart(void);
//-----------------Repeated START Condition-----------------------
void i2cRestart(void);
//------------------STOP Condition--------------------------
void i2cStop(void);
//-------------------I2C Write---------------------------
uint8_t i2cWrite(uint8_t dat);
//-----------------------i2c read------------------------------
uint8_t i2cRead(uint8_t ack);
//end i2c

//software i2c
void sI2CInit(void);									//reset the software i2c
void sI2CStart(void);									//send a start condition
void sI2CStop(void);									//send a stop condition
uint8_t sI2CWrite(uint8_t dat);							//send a i2c byte
uint8_t sI2CRead(uint8_t ack);							//read i2c data
//write from a buffer
uint8_t sI2CWrites(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
//read to a buffer
uint8_t sI2CReads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
//end software i2c

//software rtc
//hardware configuration
//end hardware configuration

//software RTC
//global defines
#define sRTC_1sec		1000			//1000ms = 1 secon

//global variables
typedef struct {
	time_t time;						//second counter since 1/1/1970
	uint32_t millis;					//millis counter from the last call
	int16_t cal;						//rtc calibration, in ppm. +=faster, -=slower
	uint8_t halfsec;					//0=first half sec, 1=2nd half sec
} sRTC_TypeDef;

void sRTCInit(void);					//initialize software counter //calibration from -128ppm to +128ppm, if sRTC_RATE = 1M
void sRTCSetCal(int16_t cal);			//set sRTC calibration
void sRTCISR(void);						//increment -> called sRTC_CALLRATE times per second
#define sRTCUpdate()	sRTCISR()		//update the software RTC
uint32_t sRTCTick(void);				//read rtc tick
time_t sRTC2time(time_t *t);			//read sRTC second counter
time_t time2sRTC(time_t t);				//set sRTC second counter
uint8_t sRTCHalfsec(void);				//return 0: first half of a sec, 1: 2nd half of a sec
#define sRTCGetSec()	sRTC2time(NULL)	//for compatability
//end software RTC

#endif		//avrduino
