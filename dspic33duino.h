#ifndef _dsPIC33DUINO_H
#define _dsPIC33DUINO_H

//dsPIC33duino code
// - supported chips: dspic33fj32GP302/304, dspic33fj64gp202/204/802/804, dspic33fj128gp202/204/802/804
// -   note: should work with other chips in the same family, like PIC24H
// - free running timer2 for ticks, pwm and input capture
// - details: https://github.com/dannyf00/Minimalist-16-bit-Arduino-Clone
// - only XC16 support is provided
//
// - version history
// - v0.1, 12/29/2022: initial porting from pic24duino (FRC@7.3728Mhz!)
// - v0.2, 01/08/2023: added support for PLL
// - v0.2a,01/14/2023: added support for DAC1
// - v0.2c,01/29/2023: fixed pll bugs
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
//     +------->| OSCI            Vdd |>--+------->Vcc
//  [Xtal]      |                     | [.1u]
//     +-------<| OSCO            Vss |>--+------->GND
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

#include <xc.h>								//we use xc16
#include <stdint.h>							//should uncomment below//define my own uint types
#include <string.h>							//we use strcpy()
#include <time.h>							//we use time

typedef unsigned char 		uint8_t;
typedef signed char 		int8_t;
typedef unsigned short 		uint16_t;
typedef signed short 		int16_t;
typedef unsigned long 		uint32_t;
typedef signed long 		int32_t;

//hardware configuration
//#define USE_MAIN							//use self-defined main() in user code
//#define USE_CORETICK						//for compatability with pic32duino. ignored
//#define SYSTICK_TMR1						//systick running on tmr1 if defined (default). otherwise on tmr2

//oscillator configuration
#define F_XTAL				10000000ul		//crystal frequency, user-specified
#define F_SOSC				32768			//SOSC = 32768Hz, user-specified
#define F_sI2C				100000ul		//frquency for software i2c
#define uRAM_SIZE			40				//uart transmission buffer size
//#define UART1_TXISR						//uncomment if polling transmission is used for uart1. Otherwise, uart1 transmission via interrupt
//#define UART2_TXISR							//uncomment if polling transmission is used for uart1. Otherwise, uart2 transmission via interrupt
//configuration for circular buffer
/*******NOTE*************/
/******cBUFF_SIZE********/
/***must be power of 2***/
/************************/
#define cBUFF_SIZE 			128				//must be power of 2, 0..256 -> for longer buffers, change data type for writeIndex/readIndex
typedef char cbuffer_t;						//storage unit for the buffer
//end configuration for circular buffer


#define TxIP_DEFAULT		5				//default priority for timer interrupts 0..7. 0->interrupt disabled, 7->highest priority
#define UxIP_DEFAULT		3				//default priority for uart interrupts 0..7. 0->interrupt disabled, 7->highest priority
#define TxIS_DEFAULT		5				//default subpriority for timer interrupts
#define OCxIP_DEFAULT		5				//default priority for timer interrupts 0..7. 0->interrupt disabled, 7->highest priority
#define CRCIP_DEFAULT		5				//default priority for crc interrupt
#define SPIIP_DEFAULT		1				//default interrupt priority
#define SPIIS_DEFAULT		1
#define RTCIP_DEFAULT		1
//end user specification

//uart1 pin configuration
#define U1TX2RP()			PPS_U1TX_TO_RP(0)			//map u1tx pin to an rp pin
//#define U1RX2RP()			PPS_U1RX_TO_RP(1)			//map u1rx pin to an rp pin

//uart2 pin configuration
#define U2TX2RP()			PPS_U2TX_TO_RP(0)			//u2tx pin
//#define U2RX2RP()			PPS_U2RX_TO_RP(1)			//u2rx pin

//pwm/oc pin configuration
#define PWM12RP()			PPS_OC1_TO_RP(6)			//oc1 pin:
#define PWM22RP()			PPS_OC2_TO_RP(6)			//oc2 pin:
#define PWM32RP()			PPS_OC3_TO_RP(6)			//oc3 pin:
#define PWM42RP()			PPS_OC4_TO_RP(6)			//oc4 pin:
//#define PWM52RP()			PPS_OC5_TO_RP(6)			//oc5 pin:
//#define PWM62RP()			PPS_OC6_TO_RP(6)			//oc6 pin:
//#define PWM72RP()			PPS_OC7_TO_RP(6)			//oc7 pin:
//#define PWM82RP()			PPS_OC8_TO_RP(6)			//oc8 pin:

//timer input pin configure: default input=F_PHB
//#define T1CK2RP()										//t1ck pin: not remappable
//#define T2CK2RP()			PPS_T2CK_TO_RP(6)			//t2ck pin:
//#define T3CK2RP()			PPS_T3CK_TO_RP(6)			//t3ck pin:
//#define T4CK2RP()			PPS_T4CK_TO_RP(6)			//t4ck pin:
//#define T5CK2RP()			PPS_T5CK_TO_RP(6)			//t5ck pin:

//input capture pin configuration
#define IC12RP()			PPS_IC1_TO_RP(4)			//ic1 pin:
#define IC22RP()			PPS_IC2_TO_RP(7)			//ic2 pin:
#define IC32RP()			PPS_IC3_TO_RP(4)			//ic3 pin:
#define IC42RP()			PPS_IC4_TO_RP(4)			//ic4 pin:
#define IC52RP()			PPS_IC5_TO_RP(4)			//ic5 pin:
#define IC62RP()			PPS_IC6_TO_RP(4)			//ic6 pin:
#define IC72RP()			PPS_IC7_TO_RP(4)			//ic7 pin:
#define IC82RP()			PPS_IC8_TO_RP(4)			//ic8 pin:

//spi1 pin configuration
#define SCK1RP()			PPS_SCK1OUT_TO_RP(0)		//spi1 serial clock output
#define SDO1RP()			PPS_SDO1_TO_RP(1)			//spi1 serial data output
//#define SDI1RP()			PPS_SDI1_TO_RP(3)			//spi1 serial data input
#define SCK2RP()			PPS_SCK2OUT_TO_RP(1)		//spi1 serial clock output
#define SDO2RP()			PPS_SDO2_TO_RP(2)			//spi1 serial data output
#define SDI2RP()			PPS_SDI2_TO_RP(3)			//spi1 serial data input

//extint pin configuration
//#define INT02RP()			PPS_INT0_TO_RP(7)			//int0 pin: fixed to rp7
#define INT12RP()			PPS_INT1_TO_RP(5)			//int1 pin:
#define INT22RP()			PPS_INT2_TO_RP(5)			//int2 pin:
//#define INT32RP()			PPS_INT3_TO_RP(5)			//int3 pin:
//#define INT42RP()			PPS_INT4_TO_RP(5)			//int4 pin:

//i2c
//end i2c

//software i2c
#define sI2CSCLPIN			PB2
#define sI2CSDAPIN			PB1
//end software i2c

//end pin configuration

#define PWM_PR				0xffff			//pwm period - don't change

//port manipulation macros for PIC.
#define IO_SET(port, bits)			  port |= (bits)			//set bits on port
#define IO_CLR(port, bits)			  port &=~(bits)			//clear bits on port
#define IO_FLP(port, bits)			  port ^= (bits)			//flip bits on port
#define IO_GET(port, bits)			  ((port) & (bits))		//return bits on port
#define IO_OUT(ddr, bits)			   ddr &=~(bits)			//set bits as output
#define IO_IN(ddr, bits)				ddr |= (bits)			//set bits as input

//tiny scheduler macro
#define TS_RUN_WHILE(cs)	if (cs)						//tiny scheduler macro

//if gpio_typedef is used
#define GIO_SET(port, bits)			  port->LAT |= (bits)			//set bits on port
#define GIO_CLR(port, bits)			  port->LAT &=~(bits)			//clear bits on port
#define GIO_FLP(port, bits)			  port->LAT ^= (bits)			//flip bits on port
#define GIO_GET(port, bits)			  ((port->PORT) & (bits))		//return bits on port
#define GIO_OUT(port, bits)			  port->TRIS &=~(bits)			//set bits as output
#define GIO_IN(port, bits)			   port->TRIS |= (bits)			//set bits as input

//fast port operations
#define FIO_SET(port, bits)			  port->LATSET = (bits)			//set bits on port
#define FIO_CLR(port, bits)			  port->LATCLR = (bits)			//clear bits on port
#define FIO_FLP(port, bits)			  port->LATINV = (bits)			//flip bits on port
#define FIO_GET(port, bits)			  ((port->PORT) & (bits))		//return bits on port
#define FIO_OUT(port, bits)			  port->TRISCLR = (bits)			//set bits as output
#define FIO_IN(port, bits)			   port->TRISSET = (bits)			//set bits as input

#define NOP()				Nop()						   //asm("nop")					//nop()
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
#define ei()				__builtin_disi(0x0000)				//asm volatile ("ei")				//__builtin_enable_interrupts()	//do {INTEnableInterrupts();	INTEnableSystemMultiVectoredInt();} while (0)	//__builtin_enable_interrupts()
#endif

#ifndef di
#define di()				__builtin_disi(0x3fff)			//asm volatile ("di")				//__builtin_enable_interrupts()	//INTDisableInterrupts()			//__builtin_disable_interrupts()	//
#endif

#define F_PHB				(SystemCoreClock / 2)			//cpu runs at F_SYS/2 by default -> Fxtal = 8Mhz. *4 for PLL. RCDIV set to 0 (1:1 postscaler)
#define F_CPU				(CLKDIVbits.DOZEN?(F_PHB >> CLKDIVbits.DOZE):(F_PHB))			//peripheral block runs at F_PHB - default = F_CPU / 1
#define F_FRC				(7372800ul)						//FRC frequency = 7.3728Mhz, fixed. corrected for the particular mcu by measuring uart at 9600bps
#define F_LPRC				32768							//LPRC = 32.768Khz, fixed
extern uint32_t SystemCoreClock;							//pheriphral core clock, before dividing by 2

//iolock/unlock sequence
//unlock IOLOCK
#define IO_UNLOCK()	{asm volatile ( "MOV #OSCCON, w1 \n" \
					"MOV #0x46, w2 \n" \
					"MOV #0x57, w3 \n" \
					"MOV.b w2, [w1] \n" \
					"MOV.b w3, [w1] \n" \
					"BCLR OSCCON,#6"); \
					}

//lock IOLOCK
#define IO_LOCK()	{asm volatile ( "MOV #OSCCON, w1 \n" \
					"MOV #0x46, w2 \n" \
					"MOV #0x57, w3 \n" \
					"MOV.b w2, [w1] \n" \
					"MOV.b w3, [w1] \n" \
					"BSET OSCCON, #6" ); \
					}

//pps macros
//more on macro
//macros for remappable pins
//for peripheral pin select (PPS)
#if defined(_INT1R)
/// Maps INT1 to a remappable pin;
/// see PPS_xxx_TO_RP(pin) for more informatino.
#define PPS_INT1_TO_RP(pin) _INT1R = pin
#else
#define PPS_INT1_TO_RP(pin)
#endif


#if defined(_INT2R)
#define PPS_INT2_TO_RP(pin) _INT2R = pin
#else
#define PPS_INT2_TO_RP(pin)
#endif

#if defined(_T2CKR)
#define PPS_T2CK_TO_RP(pin) _T2CKR = pin
#else
#define PPS_T2CK_TO_RP(pin)
#endif

#if defined(_T3CKR)
#define PPS_T3CK_TO_RP(pin) _T3CKR = pin
#else
#define PPS_T3CK_TO_RP(pin)
#endif

#if defined(_T4CKR)
#define PPS_T4CK_TO_RP(pin) _T4CKR = pin
#else
#define PPS_T4CK_TO_RP(pin)
#endif

#if defined(_T5CKR)
#define PPS_T5CK_TO_RP(pin) _T5CKR = pin
#else
#define PPS_T5CK_TO_RP(pin)
#endif

#if defined(_IC1R)
#define PPS_IC1_TO_RP(pin) _IC1R = pin
#else
#define PPS_IC1_TO_RP(pin)
#endif

#if defined(_IC2R)
#define PPS_IC2_TO_RP(pin) _IC2R = pin
#else
#define PPS_IC2_TO_RP(pin)
#endif

#if defined(_IC3R)
#define PPS_IC3_TO_RP(pin) _IC3R = pin
#else
#define PPS_IC3_TO_RP(pin)
#endif

#if defined(_IC4R)
#define PPS_IC4_TO_RP(pin) _IC4R = pin
#else
#define PPS_IC4_TO_RP(pin)
#endif

#if defined(_IC5R)
#define PPS_IC5_TO_RP(pin) _IC5R = pin
#else
#define PPS_IC5_TO_RP(pin)
#endif

#if defined(_IC6R)
#define PPS_IC6_TO_RP(pin) _IC6R = pin
#else
#define PPS_IC6_TO_RP(pin)
#endif

#if defined(_IC7R)
#define PPS_IC7_TO_RP(pin) _IC7R = pin
#else
#define PPS_IC7_TO_RP(pin)
#endif

#if defined(_IC8R)
#define PPS_IC8_TO_RP(pin) _IC8R = pin
#else
#define PPS_IC8_TO_RP(pin)
#endif

#if defined(_OCFAR)
#define PPS_OCFA_TO_RP(pin) _OCFAR = pin
#else
#define PPS_OCFA_TO_RP(pin)
#endif

#if defined(_OCFBR)
#define PPS_OCFB_TO_RP(pin) _OCFBR = pin
#else
#define PPS_OCFB_TO_RP(pin)
#endif

#if defined(_U1RXR)
#define PPS_U1RX_TO_RP(pin) _U1RXR = pin
#else
#define PPS_U1RX_TO_RP(pin)
#endif

#if defined(_U1CTSR)
#define PPS_U1CTS_TO_RP(pin) _U1CTSR = pin
#else
#define PPS_U1CTS_TO_RP(pin)
#endif

#if defined(_U2RXR)
#define PPS_U2RX_TO_RP(pin) _U2RXR = pin
#else
#define PPS_U2RX_TO_RP(pin)
#endif

#if defined(_U2CTSR)
#define PPS_U2CTS_TO_RP(pin) _U2CTSR = pin
#else
#define PPS_U2CTS_TO_RP(pin)
#endif

#if defined(_SDI1R)
#define PPS_SDI1_TO_RP(pin) _SDI1R = pin
#else
#define PPS_SDI1_TO_RP(pin)
#endif

#if defined(_SCK1R)
#define PPS_SCK1IN_TO_RP(pin) _SCK1R = pin
#else
#define PPS_SCK1IN_TO_RP(pin)
#endif

#if defined(_SS1R)
#define PPS_SS1IN_TO_RP(pin) _SS1R = pin
#else
#define PPS_SS1IN_TO_RP(pin)
#endif

#if defined(_SDI2R)
#define PPS_SDI2_TO_RP(pin) _SDI2R = pin
#else
#define PPS_SDI2_TO_RP(pin)
#endif

#if defined(_SCK2R)
#define PPS_SCK2IN_TO_RP(pin) _SCK2R = pin
#else
#define PPS_SCK2IN_TO_RP(pin)
#endif

#if defined(_SS2R)
#define PPS_SS2IN_TO_RP(pin) _SS2R = pin
#else
#define PPS_SS2IN_TO_RP(pin)
#endif

#if defined(_C1RXR)
#define PPS_C1RXR_TO_RP(pin) _C1RXR = pin
#else
#define PPS_C1RXR_TO_RP(pin)
#endif

#if defined(_C2RXR)
#define PPS_C2RXR_TO_RP(pin) _C2RXR = pin
#else
#define PPS_C2RXR_TO_RP(pin)
#endif


//end RP input mapping
//Your device may not have all of these peripherals!

//start RP output mapping


#if defined(_RP0R)
/// Maps C1OUT to a remappable pin;
/// see PPS_yyy_TO_RP(pin) for more informatino.
#define PPS_C1OUT_TO_RP(pin) _RP##pin##R = 1
#else
#define PPS_C1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C2OUT_TO_RP(pin) _RP##pin##R = 2
#else
#define PPS_C2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U1TX_TO_RP(pin) _RP##pin##R = 3
#else
#define PPS_U1TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U1RTS_TO_RP(pin) _RP##pin##R = 4
#else
#define PPS_U1RTS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U2TX_TO_RP(pin) _RP##pin##R = 5
#else
#define PPS_U2TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_U2RTS_TO_RP(pin) _RP##pin##R = 6
#else
#define PPS_U2RTS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SDO1_TO_RP(pin) _RP##pin##R = 7
#else
#define PPS_SDO1_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SCK1OUT_TO_RP(pin) _RP##pin##R = 8
#else
#define PPS_SCK1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SS1OUT_TO_RP(pin) _RP##pin##R = 9
#else
#define PPS_SS1OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SDO2_TO_RP(pin) _RP##pin##R = 10
#else
#define PPS_SDO2_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SCK2OUT_TO_RP(pin) _RP##pin##R = 11
#else
#define PPS_SCK2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_SS2OUT_TO_RP(pin) _RP##pin##R = 12
#else
#define PPS_SS2OUT_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C1TX_TO_RP(pin) _RP##pin##R = 16
#else
#define PPS_C1TX_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC1_TO_RP(pin) _RP##pin##R = 18
#else
#define PPS_OC1_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC2_TO_RP(pin) _RP##pin##R = 19
#else
#define PPS_OC2_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC3_TO_RP(pin) _RP##pin##R = 20
#else
#define PPS_OC3_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC4_TO_RP(pin) _RP##pin##R = 21
#else
#define PPS_OC4_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_OC5_TO_RP(pin) _RP##pin##R = 22
#else
#define PPS_OC5_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_CTPLS_TO_RP(pin) _RP##pin##R = 29
#else
#define PPS_CTPLS_TO_RP(pin)
#endif

#if defined(_RP0R)
#define PPS_C3OUT_TO_RP(pin) _RP##pin##R = 30
#else
#define PPS_C3OUT_TO_RP(pin)
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

//to surpress warning messages about PSV
#define _ISR_PSV __attribute__((__interrupt__, __auto_psv__))
#define _ISR_NOPSV __attribute__((__interrupt__, __no_auto_psv__))

//gpio
//port structure
typedef struct {
	volatile uint16_t TRIS;			 //data direction register -> 0ffset 0x0000, little endian
	volatile uint16_t PORT;			 //input data register
	volatile uint16_t LAT;			  //output data register
	volatile uint16_t ODC;			  //open drain configuration register. set to activate open drain
} GPIO_TypeDef;						 //port definition registers

#define GPIOA				((GPIO_TypeDef *) &TRISA)
#define GPIOB				((GPIO_TypeDef *) &TRISB)
#if defined(TRISC)
#define GPIOC				((GPIO_TypeDef *) &TRISC)
#endif
#if defined(TRISD)
#define GPIOD				((GPIO_TypeDef *) &TRISD)
#endif
#if defined(TRISE)
#define GPIOE				((GPIO_TypeDef *) &TRISE)
#endif
#if defined(TRISF)
#define GPIOF				((GPIO_TypeDef *) &TRISF)
#endif
#if defined(TRISG)
#define GPIOG				((GPIO_TypeDef *) &TRISG)
#endif


//global defines
//gpio definitions

//pin enum - matches GPIO_PinDef[]
typedef enum {
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
#if defined(GPIOC)
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
#endif		//GPIOC
#if defined(GPIOD)
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
#endif		//GPIOD
#if defined(GPIOE)
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
#endif		//GPIOE
#if defined(GPIOF)
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
#endif		//GPIOF
#if defined(GPIOG)
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15,
#endif		//GPIOG
	PMAX
} PIN_TypeDef;

//map pin number to GPIOx
typedef struct {
	GPIO_TypeDef *gpio;					//gpio for a pin
	uint16_t mask;						//pin mask - 16-bit port
} PIN2GPIO;
//end gpio

//devid
//devid
typedef struct {
	uint16_t ID;						//device id	- 0..5: DevID, 6..13: FamilyID
	uint16_t REV;						//family id - 0..3: DOT, 6..8: MajRev
} DevID_TypeDef;

#define DEVID							((DevID_TypeDef *) 0xff0000ul)	//for PIC24FxxxGA1/GB1 devices

//user id
typedef struct {
	uint8_t UID0;						//user id
	uint8_t UID1;						//user id
	uint8_t UID2;						//user id
	uint8_t UID3;						//user id
} UID_TypeDef;

#define UID								((DevID_TypeDef *) 0xf80010ul)	//for dspic33 fuid location
//end devid

#define INPUT				0			//input pin - floating
#define OUTPUT				1			//push-pull output pins
#define INPUT_PULLUP		2			//input with pull-up
#define INPUT_PULLDOWN		3			//input with pull-down
#define OUTPUT_OD			4			//output with open drain

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
#define abs(x) 				((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)	 		((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) 		((deg)*DEG_TO_RAD)
#define degrees(rad) 		((rad)*RAD_TO_DEG)
#define sq(x) 				((x)*(x))

#define interrupts() 		ei()
#define noInterrupts() 		di()

#define clockCyclesPerMillisecond() 	( F_CPU / 1000L )
#define clockCyclesPerMicrosecond() 	( F_CPU / 1000000L )
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
int digitalRead(PIN_TypeDef pin);

//time base
uint32_t systicks(void);								//timer ticks from timer2
#define ticks()				systicks()
#define coreticks()			ticks()
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
//dc = 0x00..0x0fff for pwm2/3/4/5, 0x00..0xffff for pwm1
//RP4=PWM1, RP12=PWM2, RP13=PWM3, RP14=PWM4, RP15=PWM5
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
uint32_t SystemCoreClockSwitch(uint8_t nosc);

//pll - for dspic33
//set up pll multipliers
//Fout=Fin / N1 * M / N2
//N1: 0 (2:1), 1 (3:1), ..., 31 (33:1)
//M : 0 (2x ), 1 (3x ), ..., 511 (513x)
//N2: 0 (2:1), 1 (4:1), 2 (6:1), 3 (8:1). DS says 2 is reserved
void pllSet(uint32_t N1, uint32_t M, uint32_t N2);
//read pll multiplier
//Fout=Fin / N1 * M / N2
uint32_t pllGet(uint32_t Fin);
//may need adjustments to satisfy PLL operating conditions
//Fin: 0.8 - 8Mhz
//Fpll:100 - 200Mhz
//Fout: < 80Mhz
//not all multipliers below are valid
//need to check for specific applications
#define pllx25()	pllSet(2, 8*1 -2, 3)		//pllSet(0, 8*2-2,  1)
#define pll0x5()	pllSet(2, 16*1-2, 3)		//pllSet(0, 8*2-2,  1)
#define pll1x()		pllSet(2, 32*1-2, 3)		//pllSet(0, 8*2-2,  1)
#define pll1x5()	pllSet(2, 48  -2, 3)		//pllSet(0, 8*2-2,  1)
#define pll2x()		pllSet(0, 16*2-2, 3)		//pllSet(0, 8*2-2,  1)
#define pll2x5()	pllSet(0, 24  -2, 3)		//pllSet(0, 8*2-2,  1)
#define pll3x()		pllSet(0, 4*3-2,  0)		//pllSet(0, 16*3-2, 3)		//pllSet(0, 4*3-2,  0)
#define pll3x5()	pllSet(0, 14 -2,  0)		//4*3.5-2
#define pll4x()		pllSet(0, 4*4-2,  0)		//pllSet(0, 16*4-2, 3)		//pllSet(0, 4*4-2,  0)
#define pll4x5()	pllSet(0, 18 -2,  0)		//4*4.5-2
#define pll5x()		pllSet(0, 4*5-2,  0)
#define pll5x5()	pllSet(0, 22 -2,  0)		//4*5.5-2
#define pll6x()		pllSet(0, 4*6-2,  0)
#define pll6x5()	pllSet(0, 26 -2,  0)		//4*6.5-2
#define pll7x()		pllSet(0, 4*7-2,  0)
#define pll7x5()	pllSet(0, 30 -2,  0)		//4*7.5-2
#define pll8x()		pllSet(0, 4*8-2,  0)
#define pll8x5()	pllSet(0, 34 -2,  0)		//4*8.5-2
#define pll9x()		pllSet(0, 4*9-2,  0)
#define pll9x5()	pllSet(0, 38 -2,  0)		//4*9.5-2
#define pll10x()	pllSet(0, 4*10-2, 0)
#define pll11x()	pllSet(0, 4*11-2, 0)
#define pll12x()	pllSet(0, 4*12-2, 0)
#define pll14x()	pllSet(0, 4*14-2, 0)
#define pll15x()	pllSet(0, 4*15-2, 0)
#define pll16x()	pllSet(0, 4*16-2, 0)
#define pll18x()	pllSet(0, 4*18-2, 0)
#define pll20x()	pllSet(0, 4*20-2, 0)
#define pll24x()	pllSet(0, 4*24-2, 0)
#define pll25x()	pllSet(0, 4*25-2, 0)
#define pll30x()	pllSet(0, 4*30-2, 0)
#define pll32x()	pllSet(0, 4*32-2, 0)
#define pll35x()	pllSet(0, 4*35-2, 0)
#define pll36x()	pllSet(0, 4*36-2, 0)
#define pll40x()	pllSet(0, 4*40-2, 0)
#define pll45x()	pllSet(0, 4*45-2, 0)
#define pll48x()	pllSet(0, 4*48-2, 0)
#define pll50x()	pllSet(0, 4*50-2, 0)
#define pll55x()	pllSet(0, 4*55-2, 0)
#define pll64x()	pllSet(0, 4*64-2, 0)
#define pll65x()	pllSet(0, 4*65-2, 0)
#define pll96x()	pllSet(0, 4*96-2, 0)	//max multiplier is 533 > 4*96
#define pllWait()	while (OSCCONbits.LOCK == 0) continue			//1->pll in lock, 0->pll not locked
#define SystemCoreClockFRC()			SystemCoreClockSwitch(0b000)
#define SystemCoreClockFRCPLLx25()		do {pllx25();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL0x5()		do {pll0x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL1x()		do {pll1x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL1x5()		do {pll1x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL2x()		do {pll2x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL2x5()		do {pll2x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL3x()		do {pll3x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL3x5()		do {pll3x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL4x()		do {pll4x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL4x5()		do {pll4x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL5x()		do {pll5x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL5x5()		do {pll5x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL6x()		do {pll6x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL6x5()		do {pll6x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL7x()		do {pll7x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL7x5()		do {pll7x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL8x()		do {pll8x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL8x5()		do {pll8x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL9x()		do {pll9x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL9x5()		do {pll9x5();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL10x()		do {pll10x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL11x()		do {pll11x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL12x()		do {pll12x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL14x()		do {pll14x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL15x()		do {pll15x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockFRCPLL16x()		do {pll16x();SystemCoreClockSwitch(0b001); pllWait();} while (0)
//#define SystemCoreClockFRCPLL18x()		do {pll18x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
//#define SystemCoreClockFRCPLL20x()		do {pll20x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
//#define SystemCoreClockFRCPLL24x()		do {pll24x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
//#define SystemCoreClockFRCPLL32x()		do {pll32x(); SystemCoreClockSwitch(0b001); pllWait();} while (0)
#define SystemCoreClockPOSC()			SystemCoreClockSwitch(0b010)
#define SystemCoreClockPOSCPLLx25()		do {pllx25();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL0x5()		do {pll0x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL1x()		do {pll1x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL1x5()		do {pll1x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL2x()		do {pll2x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL2x5()		do {pll2x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL3x()		do {pll3x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL3x5()		do {pll3x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL4x()		do {pll4x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL4x5()		do {pll4x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL5x()		do {pll5x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL5x5()		do {pll5x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL6x()		do {pll6x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL6x5()		do {pll6x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL7x()		do {pll7x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL7x5()		do {pll7x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL8x()		do {pll8x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL8x5()		do {pll8x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL9x()		do {pll9x(); SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL9x5()		do {pll9x5();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL10x()		do {pll10x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL11x()		do {pll11x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL12x()		do {pll12x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL14x()		do {pll14x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL15x()		do {pll15x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL16x()		do {pll16x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL18x()		do {pll18x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL20x()		do {pll20x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL24x()		do {pll24x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL25x()		do {pll25x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockPOSCPLL32x()		do {pll32x();SystemCoreClockSwitch(0b011); pllWait();} while (0)
#define SystemCoreClockSOSC()			SystemCoreClockSwitch(0b100)
#define SystemCoreClockLPRC()			SystemCoreClockSwitch(0b101)
#define SystemCoreClockFRCDIV()			SystemCoreClockSwitch(0b111)

//empty interrupt handler
void empty_handler(void);


//#define Mhz					000000ul	//suffix for Mhz
#define F_UART				(F_PHB)	//peripheral clock
#define UART_BR300			300ul		//buadrate=300
#define UART_BR600			600ul		//buadrate=600
#define UART_BR1200			1200ul		//buadrate=1200
#define UART_BR2400			2400ul		//buadrate=2400
#define UART_BR4800			4800ul		//buadrate=4800
#define UART_BR9600			9600ul		//buadrate=9600
#define UART_BR14400		14400ul		//buadrate=19200
#define UART_BR19200		19200ul		//buadrate=19200
#define UART_BR38400		38400ul		//buadrate=38400
#define UART_BR43000		43000ul		//buadrate=38400
#define UART_BR57600		57600ul		//buadrate=57600
#define UART_BR76800		76800ul		//buadrate=57600
#define UART_BR115200		115200ul	//buadrate=115200
#define UART_BR128000		128000ul	//buadrate=115200
#define UART_BR230400		230400ul	//buadrate=115200
#define UART_BR256000		256000ul	//buadrate=115200
#define UART_BR460800		460800ul	//buadrate=115200
#define UART_BR512000		512000ul	//buadrate=115200
#define UART_BR576000		576000ul	//buadrate=115200
#define UART_BR921600		921600ul	//buadrate=115200
#define UART_BR1000000		1000000ul	//buadrate=115200
#define UART_BR1250000		1250000ul	//buadrate=115200
#define UART_BR1500000		1500000ul	//buadrate=115200
#define UART_BR2000000		2000000ul	//buadrate=115200
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
#define UART_BR256K			UART_BR256000
#define UART_BR460K8		UART_BR460800
#define UART_BR512K			UART_BR512000
#define UART_BR576K			UART_BR576000
#define UART_BR1M			UART_BR1000000
#define UART_BR1M25			UART_BR1250000
#define UART_BR1M5			UART_BR1500000
#define UART_BR2M			UART_BR2000000

//initiate the hardware usart1
void uart1Init(unsigned long baud_rate);//initialize uart
//void uart1Putch(char ch);				//output a char on uart
#define uart1Putch(ch)		do {uart1Wait(); U1TXREG=(ch);} while (0)	//void uart1Putch(char ch);					//send a char
void uart1Puts(char *str);				//output a string on uart
//void uart1Putline(char *ln);			//output a string + linefeed on uart
#define uart1Putline(str)	do {uart1Puts(str); uart1Puts("\r\n");} while (0)
#define uart1Getch()		U1RXREG		//uint8_t uart1Getch(void);				//read a char from usart
#define uart1Available()	(U1STAbits.URXDA)	//uint16_t uart1Available(void);			//test if data rx is available
#if defined(UART1_TXISR)
#define uart1Busy()			(IEC0bits.U1TXIE)	//(!U1STAbits.TRMT)	//
#else
#define uart1Busy()			(U1STAbits.UTXBF)
#endif
#define uart1Wait()			while (uart1Busy()) continue
void u1Print(char *str, int32_t dat);	//output a number on uart
#define u1Println()			uart1Puts("\r\n")
//for compatability
#define uart1Put(ch)		uart1Putch(ch)
#define uart1Get()			uart1Getch()
#define uart1bps()			((U1MODEbits.BRGH==1)?(F_UART / 4 / (U1BRG+1)):(F_UART / 16 / (U1BRG+1)))
#define u1bps()				uart1bps()

//initiate the hardware usart2
void uart2Init(unsigned long baud_rate);//initialize uart
//void uart2Putch(char ch);				//output a char on uart
#define uart2Putch(ch)		do {uart2Wait(); U2TXREG=(ch);} while (0)
#define uart2Putc(ch)		uart2Putch(ch)
void uart2Puts(char *str);				//output a string on uart
//void uart2Putline(char *ln);			//output a string + linefeed on uart
#define uart2Putline(str)	do {uart2Puts(str); uart2Puts("\r\n");} while (0)
#define uart2Getch()		U2RXREG		//uint8_t uart2Getch(void);				//read a char from usart
#define uart2Available()	(U2STAbits.URXDA)	//uint16_t uart2Available(void);			//test if data rx is available
#if defined(UART2_TXISR)
#define uart2Busy()			(IEC1bits.U2TXIE)
#else
#define uart2Busy()			(U2STAbits.UTXBF)	//uint16_t uart2Busy(void);				//test if uart tx is busy
#endif
#define uart2Wait()			while (uart2Busy()) continue
void u2Print(char *str, int32_t dat);	//output a number on uart
#define u2Println()			uart2Puts("\r\n")
//for compatability
#define uart2Put(ch)		uart2Putch(ch)
#define uart2Get()			uart2Getch()
#define uart2bps()			((U2MODEbits.BRGH==1)?(F_UART / 4 / (U2BRG+1)):(F_UART / 16 / (U2BRG+1)))
#define u2bps()				uart2bps()
//end Serial


//tmr1-5 prescaler
#define TMR_PS1x			0x00
#define TMR_PS4x			0x01
#define TMR_PS8x			0x02
#define TMR_PS256x			0x03
#define TMR_PSMASK			TMR_PS256x

//#define TMR_IPDEFAULT		2
//#define TMR_ISDEFAULT		0

//initialize the timer1 (16bit)
void tmr1Init(uint8_t ps, uint16_t period);
//activate the isr handler
void tmr1AttachISR(void (*isrptr)(void));

//initialize the timer2 (16bit)
void tmr2Init(uint8_t ps, uint16_t period);
//activate the isr handler
void tmr2AttachISR(void (*isrptr)(void));

//initialize the timer3 (16bit)
void tmr3Init(uint8_t ps, uint16_t period);
//activate the isr handler
void tmr3AttachISR(void (*isrptr)(void));

//initialize the timer4 (16bit)
void tmr4Init(uint8_t ps, uint16_t period);
//activate the isr handler
void tmr4AttachISR(void (*isrptr)(void));

//initialize the timer5 (16bit)
void tmr5Init(uint8_t ps, uint16_t period);
//activate the isr handler
void tmr5AttachISR(void (*isrptr)(void));

//32-bit timers
//initialize the timer23 (32bit)
void tmr23Init(uint8_t ps, uint32_t period);
//read 32-bit tmr23
uint32_t tmr23Get(void);
//activate the isr handler
#define tmr23AttachISR(isr)		tmr3AttachISR(isr)

//initialize the timer45 (32bit)
void tmr45Init(uint8_t ps, uint32_t period);
//read 32-bit tmr45
uint32_t tmr45Get(void);
//activate the isr handler
#define tmr45AttachISR(isr)		tmr5AttachISR(isr)

//pwm / oc
//initialize pwm1
void pwm1Init(void);
#define pwm1SetDC(dc)			do {OC1RS = (dc);} while (0)
#define pwm1GetDC()				(OC1RS)

//initialize pwm2
void pwm2Init(void);
#define pwm2SetDC(dc)			do {OC2RS = (dc);} while (0)
#define pwm2GetDC()				(OC2RS)

//initialize pwm3
void pwm3Init(void);
#define pwm3SetDC(dc)			do {OC3RS = (dc);} while (0)
#define pwm3GetDC()				(OC3RS)

//initialize pwm4
void pwm4Init(void);
#define pwm4SetDC(dc)			do {OC4RS = (dc);} while (0)
#define pwm4GetDC()				(OC4RS)

//initialize pwm5
void pwm5Init(void);
#define pwm5SetDC(dc)			do {OC5RS = (dc);} while (0)
#define pwm5GetDC()				(OC5RS)

//initialize pwm6
void pwm6Init(void);
#define pwm6SetDC(dc)			do {OC6RS = (dc);} while (0)
#define pwm6GetDC()				(OC6RS)

//initialize pwm7
void pwm7Init(void);
#define pwm7SetDC(dc)			do {OC7RS = (dc);} while (0)
#define pwm7GetDC()				(OC7RS)

//initialize pwm8
void pwm8Init(void);
#define pwm8SetDC(dc)			do {OC8RS = (dc);} while (0)
#define pwm8GetDC()				(OC8RS)

//adc
//adc channels
#define ADC_AN0					(0)		//adc an0
#define ADC_AN1					(1)		//adc an1
#define ADC_AN2					(2)		//adc an2
#define ADC_AN3					(3)		//adc an3
#define ADC_AN4					(4)		//adc an4
#define ADC_AN5					(5)		//adc an5
#define ADC_AN6					(6)		//adc an6		-unimplemented on 28pin devices
#define ADC_AN7					(7)		//adc an7		-unimplemented on 28pin devices
#define ADC_AN8					(8)		//adc an8		-unimplemented on 28pin devices
#define ADC_AN9					(9)		//adc an9
#define ADC_AN10				(10)	//adc an10
#define ADC_AN11				(11)	//adc an11
#define ADC_AN12				(12)	//adc an12
//#define ADC_VCORE				(13)	//adc vcore
//#define ADC_VBG2				(14)	//adc bandgap/2
//#define ADC_VBG					(15)	//adc bandgap
//#define ADC_CTMU				(0xffff)	//for ctmu
#define ADC_ARAND				ADC_AN1	//the channel for random number generators
#define ADC_DLY					100		//cycle delay for adc
//rest the adc
//automatic sampling (ASAM=1), manual conversion
//12-bit adc, ch0 sample b only
void adcInit(void);

//read the adc
uint16_t analogRead(uint16_t ch);
//end ADC

//arand
//random number generators
#define arand1()			(analogRead(ADC_ARAND) & 0x01)	//generate 1-bit random number
uint8_t arand4(void);							//generate 4-bit random numbers
uint8_t arand8(void);							//generate 8-bit random numbers
uint16_t arand16(void);							//generate 16-bit random numbers
uint32_t arand32(void);							//generate 32-bit random numbers
//end arand

//output compare - TMR2 is the base
#define OC_TIMEBASE()			TMR2			//TMR2 as the timebase
void oc1Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc1AttachISR(void (*isrptr)(void));		//install user isr
void oc2Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc2AttachISR(void (*isrptr)(void));		//install user isr
void oc3Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc3AttachISR(void (*isrptr)(void));		//install user isr
void oc4Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc4AttachISR(void (*isrptr)(void));		//install user isr
void oc5Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc5AttachISR(void (*isrptr)(void));		//install user isr
void oc6Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc6AttachISR(void (*isrptr)(void));		//install user isr
void oc7Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc7AttachISR(void (*isrptr)(void));		//install user isr
void oc8Init(uint16_t pr);						//initialize oc1, by prescaler + period
void oc8AttachISR(void (*isrptr)(void));		//install user isr
//end output compare

//input capture
//16-bit mode, rising edge, single capture, Timer2 as timebase
#define IC_TIMEBASE()			TMR2			//TMR2 as the timebase
//interrupt disabled
void ic1Init(void);
void ic1AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic1Get()		IC1BUF					//read buffer value

void ic2Init(void);
void ic2AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic2Get()		IC2BUF					//read buffer value

void ic3Init(void);
void ic3AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic3Get()		IC3BUF					//read buffer value

void ic4Init(void);
void ic4AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic4Get()		IC4BUF					//read buffer value

void ic5Init(void);
void ic5AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic5Get()		IC5BUF					//read buffer value

void ic6Init(void);
void ic6AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic6Get()		IC6BUF					//read buffer value

void ic7Init(void);
void ic7AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic7Get()		IC7BUF					//read buffer value

void ic8Init(void);
void ic8AttachISR(void (*isrptr)(void));		//activate user ptr
#define ic8Get()		IC8BUF					//read buffer value

//end input capture

//extint
void int0Init(void);							//initialize the module
void int0AttachISR(void (*isrptr) (void));		//attach user isr

void int1Init(void);							//initialize the module
void int1AttachISR(void (*isrptr) (void));		//attach user isr

void int2Init(void);							//initialize the module
void int2AttachISR(void (*isrptr) (void));		//attach user isr

void int3Init(void);							//initialize the module
void int3AttachISR(void (*isrptr) (void));		//attach user isr

void int4Init(void);							//initialize the module
void int4AttachISR(void (*isrptr) (void));		//attach user isr
//end extint

//spi
void spi1Init(uint16_t br);						//reset the spi
#define spi1Busy()			(SPI1STATbits.SPITBF)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi1Available()		(!SPI1STATbits.SPIRBE)	//receive buffer not empty -> there is data
#define spi1Write(dat)		SPI1BUF=(dat)		//send data via spi
#define spi1Read()			(SPI1BUF)			//read from the buffer

void spi2Init(uint16_t br);						//reset the spi
#define spi2Busy()			(SPI2STATbits.SPITBF)	//transmit buffer full, must wait before writing to SPIxBUF
#define spi2Available()		(!SPI2STATbits.SPIRBE)	//receive buffer not empty -> there is data
#define spi1Write(dat)		SPI1BUF=(dat)		//send data via spi
#define spi2Read()			(SPI2BUF)			//read from the buffer

//end spi

//i2c
//i2c commands
#define I2C_ACK				0							//0-> ack, to be consistent with i2c protocol
#define I2C_NOACK			1							//1-> no ack
#define I2C_CMD_READ		0x01						//i2c command for read
#define I2C_CMD_WRITE		0x00						//i2c command for write
#define I2C_ACK_ERROR		0x09						//max number of loops before ack times out

//#define F_I2C1			100000ul	//I2C frequency
void i2c1Init(uint32_t bps);			//initialize the i2c
void i2c1Start(void);					//send a start condition
void i2c1Stop(void);					//send a stop condition
void i2c1Restart(void);					//send a restart condition
//#define i2c1Wait()		do {while (I2C1CON & 0x1f); while (I2C1STATbits.TRSTAT);} while (0)		//wait for i2c
uint8_t i2c1Write(uint8_t dat);			//send i2c
uint8_t i2c1Read(uint8_t ack);			//read i2c
//write from a buffer
uint8_t i2c1Writes(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
//read to a buffer
uint8_t i2c1Reads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToRead);

//i2c2
//#define F_I2C2			100000ul		//I2C frequency
void i2c2Init(uint32_t bps);			//initialize the i2c
void i2c2Start(void);					//send a start condition
void i2c2Stop(void);					//send a stop condition
void i2c2Restart(void);					//send a restart condition
//#define i2c1Wait()		do {while (I2C2CON & 0x1f); while (I2C2STATbits.TRSTAT);} while (0)		//wait for i2c
uint8_t i2c2Write(uint8_t dat);			//send i2c
uint8_t i2c2Read(uint8_t ack);			//read i2c
//write from a buffer
uint8_t i2c2Writes(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
//read to a buffer
uint8_t i2c2Reads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
//end i2c

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

//rtcc
//allows write to rtc registers
//#define RTCC_WREN()			do {NVMKEY=0x55; NVMKEY=0xaa; RCFGCALbits.RTCWREN = 1;} while (RCFGCALbits.RTCWREN == 0)
#define RTCC_WREN()	{	asm volatile("push w7"); \
						asm volatile("push w8"); \
						asm volatile("disi #5"); \
						asm volatile("mov #0x55, w7"); \
						asm volatile("mov w7, _NVMKEY"); \
						asm volatile("mov #0xAA, w8"); \
						asm volatile("mov w8, _NVMKEY"); \
						asm volatile("bset _RCFGCAL, #13"); \
						asm volatile("pop w8"); \
						asm volatile("pop w7"); \
					}
//do not allow any write to rtc registers - assumes the nvmkey sequence has been sent
#define RTCC_WRDIS()	do {RCFGCALbits.RTCWREN = 0;} while (RCFGCALbits.RTCWREN == 1)

//for RTCC PTR values to be used with RTCCRead() and RTCCWrite()
#define RTCPTR_MINSEC	0
#define RTCPTR_WKDHRS	1
#define RTCPTR_MONDAY	2
#define RTCPTR_YEAR		3

//bcd to dec conversion
#define BCD2DEC(bcd)			((((bcd) & 0xf0)>>4)*10+((bcd) & 0x0f))	//convert bcd to int
#define DEC2BCD(dec)			((((dec) / 10)<<4) +((dec) % 10))		//convert int to bcd

void RTCInit(void);							//initialize the rtcc
uint16_t RTCRead(uint16_t mask);				//read from rtcc, based on the value of mask (0..3)
#define RTCGetSec()				(RTCRead(RTCPTR_MINSEC) & 0xff)
#define RTCGetMin()				(RTCRead(RTCPTR_MINSEC) >> 8)
#define RTCGetHour()			(RTCRead(RTCPTR_WKDHRS) & 0xff)
#define RTCGetWDay()			(RTCRead(RTCPTR_WKDHRS) >> 8)
#define RTCGetDay()				(RTCRead(RTCPTR_MONDAY) & 0xff)
#define RTCGetMonth()			(RTCRead(RTCPTR_MONDAY) >> 8)
#define RTCGetYear()			(RTCRead(RTCPTR_YEAR  ) & 0xff)
void RTCWrite(uint16_t mask, uint16_t byte_t);	//write to rtcc,  based on the value of mask (0..3)
#define RTCSetSec(dat)			RTCWrite(RTCPTR_MINSEC, (RTCRead(RTCPTR_MINSEC) & ~0x00ff) | ((dat) & 0x00ff))
#define RTCSetMin(dat)			RTCWrite(RTCPTR_MINSEC, (RTCRead(RTCPTR_MINSEC) & ~0xff00) | ((dat) & 0xff00))
#define RTCSetHour(dat)			RTCWrite(RTCPTR_WKDHRS, (RTCRead(RTCPTR_WKDHRS) & ~0x00ff) | ((dat) & 0x00ff))
#define RTCSetWDay(dat)			RTCWrite(RTCPTR_WKDHRS, (RTCRead(RTCPTR_WKDHRS) & ~0xff00) | ((dat) & 0xff00))
#define RTCSetDay(dat)			RTCWrite(RTCPTR_MONDAY, (RTCRead(RTCPTR_MONDAY) & ~0x00ff) | ((dat) & 0x00ff))
#define RTCSetMonth(dat)		RTCWrite(RTCPTR_MONDAY, (RTCRead(RTCPTR_MONDAY) & ~0xff00) | ((dat) & 0xff00))
#define RTCSetYear(dat)			RTCWrite(RTCPTR_YEAR,   (RTCRead(RTCPTR_YEAR  ) & ~0x00ff) | ((dat) & 0x00ff))
void RTCSetCal(uint8_t cal); 	//write to rtcc calibration register
uint8_t RTCGetCal(void);		//read from rtcc calibration register
void RTCAttachISR(void (*isrptr)(void));		//attach rtc isr
time_t RTC2time(void);						//read rtcc time into time_t
void time2RTC(time_t t);						//turn mktime (=time_t) to RTCC
//end rtcc

//cnint
void cnInit(uint32_t pins);						//initialize change notification
void cnAttachISR(void (*isrptr) (void));		//attach user isr
//end cnint

//crc
void CRCInit(uint8_t len, uint32_t poly);							//initialize the crc
uint16_t CRC16(char *msg, uint16_t length, uint16_t init_val);		//return crc check on a message. length must be multiples of 2
#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)
#define CRCBusy()				(CRCCON1bits.CRCGO == 1)			////if CRC is busy
#define CRCAvailable()		(CRCCON1bits.CRCGO == 0)			//crc complete, data available
#else
#define CRCBusy()				(CRCCONbits.CRCGO == 1)			////if CRC is busy
#define CRCAvailable()		(CRCCONbits.CRCGO == 0)			//crc complete, data available
#endif
#define CRCGet()				(CRCWDAT)							//read CRC back
#define CRC8Init(xor, init)		CRCInit(0x07, xor, init)			//initialize 16-bit CRC
#define CRC16Init(xor, init)	CRCInit(0x0f, xor, init)			//initialize 16-bit CRC
#define CRC32Init(xor, init)	CRCInit(0x1f, x0r, init)			//initialize 32-bit CRC
//end crc

//comparator voltage reference
//comparator voltage reference
void CVrefInit(void);
#define CVrefSet(range)			CVRCONbits.CVR=(range)			//set output range
#define CVrefGet()				(CVRCONbits.CVR)				//return output range
#define CVrefOE()				CVRCONbits.CVROE=1				//0->disable output, 1->enable output
#define CVrefOD()				CVRCONbits.CVROE=0				//0->disable output, 1->enable output

//comparator
#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)

void CM1Init(void);				//initialize comparator
#define CM1OUT()				(CM1CONbits.COUT)				//output status
#define CM1INA()				CM1CONbits.CREF=0				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM1CVrefp()				CM1CONbits.CREF=1				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM1INB()				CM1CONbits.CCH =0				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM1INC()				CM1CONbits.CCH =1				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM1IND()				CM1CONbits.CCH =2				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM1CVrefn()				CM1CONbits.CCH =3				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM1OE()					CM1CONbits.COE =1				//1->enable output, 0->disable output
#define CM1OD()					CM1CONbits.COE =0				//1->enable output, 0->disable output

void CM2Init(void);				//initialize comparator
#define CM2OUT()				(CM2CONbits.COUT)				//output status
#define CM2INA()				CM2CONbits.CREF=0				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM2CVrefp()				CM2CONbits.CREF=1				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM2INB()				CM2CONbits.CCH =0				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM2INC()				CM2CONbits.CCH =1				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM2IND()				CM2CONbits.CCH =2				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM2CVrefn()				CM2CONbits.CCH =3				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM2OE()					CM2CONbits.COE =1				//1->enable output, 0->disable output
#define CM2OD()					CM2CONbits.COE =0				//1->enable output, 0->disable output

void CM3Init(void);				//initialize comparator
#define CM3OUT()				(CM3CONbits.COUT)				//output status
#define CM3INA()				CM3CONbits.CREF=0				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM3CVrefp()				CM3CONbits.CREF=1				//0->non-inventing input to CxINA, 1->non-inventing input to CVref+
#define CM3INB()				CM3CONbits.CCH =0				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM3INC()				CM3CONbits.CCH =1				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM3IND()				CM3CONbits.CCH =2				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM3CVrefn()				CM3CONbits.CCH =3				//0->inventing input to CxINB, 1->to CxINC, 2->to CxIND, 3->to CVref-
#define CM3OE()					CM3CONbits.COE =1				//1->enable output, 0->disable output
#define CM3OD()					CM3CONbits.COE =0				//1->enable output, 0->disable output

#else		//ga002 chips

void CM1Init(void);				//initialize comparator
#define CM1OUT()				(CMCONbits.C1OUT)				//output status
#define CM1NVinp()				CMCONbits.C1NEG=1				//1->C1NEG to Vin+, 0->C1NEG to Vin-
#define CM1NVinn()				CMCONbits.C1NEG=0				//1->C1NEG to Vin+, 0->C1NEG to Vin-
#define CM1PVinp()				CMCONbits.C1POS=1				//1->C1POS to Vin+, 0->C1NEG to CVref
#define CM1PCVref()				CMCONbits.C1POS=0				//1->C1POS to Vin+, 0->C1NEG to CVref
#define CM1OE()					CMCONbits.C1OUTEN =1			//1->enable output, 0->disable output
#define CM1OD()					CMCONbits.C1OUTEN =0			//1->enable output, 0->disable output

void CM2Init(void);				//initialize comparator
#define CM2OUT()				(CMCONbits.C2OUT)				//output status
#define CM2NVinp()				CMCONbits.C2NEG=1				//1->C1NEG to Vin+, 0->C1NEG to Vin-
#define CM2NVinn()				CMCONbits.C2NEG=0				//1->C1NEG to Vin+, 0->C1NEG to Vin-
#define CM2PVinp()				CMCONbits.C2POS=1				//1->C1POS to Vin+, 0->C1NEG to CVref
#define CM2PCVref()				CMCONbits.C2POS=0				//1->C1POS to Vin+, 0->C1NEG to CVref
#define CM2OE()					CMCONbits.C2OUTEN =1			//1->enable output, 0->disable output
#define CM2OD()					CMCONbits.C2OUTEN =0			//1->enable output, 0->disable output

#endif		//comparator

//circular buffer
//circular buffer to store char
typedef struct {
	cbuffer_t buffer[cBUFF_SIZE];		//buffer
	uint8_t writeIndex;					//index for the current write-> advance first and then write
	uint8_t readIndex;					//index for the current read -> advance first and then read
} CB_TypeDef;

void cbInit(CB_TypeDef *cbuffer);		//reset the cbuffer
char cbEmpty(CB_TypeDef *cbuffer);		//returns true if cbuffer is empty
char cbFull(CB_TypeDef *cbuffer);		//returns true if cbuffer is full;
cbuffer_t cbWrite(CB_TypeDef *cbuffer, cbuffer_t dat);	//write to the buffer - needs to test if the buffer is full first
cbuffer_t cbRead(CB_TypeDef *cbuffer);	//read from the buffer - needs to test if the buffer is empty
void str2cb(CB_TypeDef *cb, char *str);	//write string to CB
void cb2str(char *str, CB_TypeDef *cb);	//write CB to string
//end circular buffer


//dac1
//initialize dac1
//dac1xM output disabled
//input data = unsigned format
//len=0: disable left channel, 1: enable left channel
//ren=0: disable right channel,1: enable right channel
void dac1Init(char len, char ren);
//output data on left/right channel
#define dac1Lout(dat)		DAC1LDAT = (dat)				//output on DAC1LP/N pins
#define dac1Rout(dat)		DAC1RDAT = (dat)				//output on DAC1RP/N pins
//end dac1

//crc
//programmable crc
//initialize crc
void crcInit(uint16_t poly, uint8_t plen);
#define crcWrite(dat)	do {while (CRCCONbits.CRCFUL) continue; CRCDAT = (dat); CRCCONbits.CRCGO = 1;} while (0)
#define crcRead()		(CRCWDAT)//end crc

#endif		//pic24duino
