#include "dspic33duino.h"						//dspic33duino

//mcus supported
//dspic33fj128GP804, dspic33fj128GP802, dspic33fj128GP204, dspic33fj128GP202
//dspic33fj64GP804,  dspic33fj64GP802,  dspic33fj64GP204,  dspic33fj64GP202
//dspic33fj32GP304,  dspic33fj32GP302


//************XC********************
//config words settings
#if 	defined(__dsPIC33FJ128GP804__) | defined (__dsPIC33FJ128GP802__) | defined(__dsPIC33FJ128GP204__) | defined (__dsPIC33FJ128GP202__) | \
		defined(__dsPIC33FJ64GP804__)  | defined (__dsPIC33FJ64GP802__)  | defined(__dsPIC33FJ64GP204__)  | defined (__dsPIC33FJ64GP202__)

//dspic33fj64gp202/204/802/804, dspic33fj128gp202/204/802/804
#pragma config FNOSC = FRC			//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV16, LPRCDIVN
#pragma config POSCMD = XT			//EC, XT, HS, NONE
#pragma config BWRP = WRPROTECT_OFF
#pragma config BSS =  NO_BOOT_CODE
#pragma config RBS =  NO_BOOT_RAM
#pragma config SWRP = WRPROTECT_OFF
#pragma config SSS =  NO_SEC_CODE
#pragma config RSS =  NO_SEC_RAM
#pragma config GWRP = OFF
#pragma config IESO = OFF
#pragma config OSCIOFNC=OFF			//ON: OSC2 as digital IO, OFF: OSC2 has digital output
#pragma config IOL1WAY =OFF			//off: allow multiple reconfiguration, on: allow only one re-configuration
#pragma config FCKSM =  CSECMD		//CSECME, CSECMD, CSDCMD
#pragma config WDTPOST =PS32768
#pragma config WDTPRE = PR128		//PR32, PR128
#pragma config WINDIS = OFF			//off: watchdog timer in non-window mode
#pragma config FWDTEN = OFF
#pragma config FPWRT =  PWR128
#pragma config ALTI2C = OFF
#pragma config ICS =    PGD1		//PGD1, PGD2, PGD3
#pragma config JTAGEN = OFF


/*configuration bits
Boot Segment Write Protect
BWRP = WRPROTECT_ON	Boot segment is write-protected
BWRP = WRPROTECT_OFF	Boot Segment may be written

Boot Segment Program Flash Code Protection
BSS = LARGE_FLASH_HIGH	High Security, Large-sized Boot Flash
BSS = HIGH_LARGE_BOOT_CODE	High Security, Large-sized Boot Flash
BSS = MEDIUM_FLASH_HIGH	High Security, Medium-sized Boot Flash
BSS = HIGH_MEDIUM_BOOT_CODE	High Security, Medium-sized Boot Flash
BSS = SMALL_FLASH_HIGH	High Security, Small-sized Boot Flash
BSS = HIGH_SMALL_BOOT_CODE	High Security, Small-sized Boot Flash
BSS = LARGE_FLASH_STD	Standard Security, Large-sized Boot Flash
BSS = STRD_LARGE_BOOT_CODE	Standard Security, Large-sized Boot Flash
BSS = MEDIUM_FLASH_STD	Standard Security, Medium-sized Boot Flash
BSS = STRD_MEDIUM_BOOT_CODE	Standard Security, Medium-sized Boot Flash
BSS = SMALL_FLASH_STD	Standard Security, Small-sized Boot Flash
BSS = STRD_SMALL_BOOT_CODE	Standard Security, Small-sized Boot Flash
BSS = NO_FLASH	No Boot program Flash segment
BSS = NO_BOOT_CODE	No Boot program Flash segment

Boot Segment RAM Protection
RBS = LARGE_RAM	Large-sized Boot RAM
RBS = LARGE_BOOT_RAM	Large-sized Boot RAM
RBS = MEDIUM_RAM	Medium-sized Boot RAM
RBS = MEDIUM_BOOT_RAM	Medium-sized Boot RAM
RBS = SMALL_RAM	Small-sized Boot RAM
RBS = SMALL_BOOT_RAM	Small-sized Boot RAM
RBS = NO_RAM	No Boot RAM
RBS = NO_BOOT_RAM	No Boot RAM

Secure Segment Program Write Protect
SWRP = WRPROTECT_ON	Secure segment is write-protected
SWRP = WRPROTECT_OFF	Secure segment may be written

Secure Segment Program Flash Code Protection
SSS = LARGE_FLASH_HIGH	High Security, Large-sized Secure Flash
SSS = HIGH_LARGE_SEC_CODE	High Security, Large-sized Secure Flash
SSS = MEDIUM_FLASH_HIGH	High Security, Medium-sized Secure Flash
SSS = HIGH_MEDIUM_SEC_CODE	High Security, Medium-sized Secure Flash
SSS = SMALL_FLASH_HIGH	High Security, Small-sized Secure Flash
SSS = HIGH_SMALL_SEC_CODE	High Security, Small-sized Secure Flash
SSS = LARGE_FLASH_STD	Standard Security, Large-sized Secure Flash
SSS = STRD_LARGE_SEC_CODE	Standard Security, Large-sized Secure Flash
SSS = MEDIUM_FLASH_STD	Standard Security, Medium-sized Secure Flash
SSS = STRD_MEDIUM_SEC_CODE	Standard Security, Medium-sized Secure Flash
SSS = SMALL_FLASH_STD	Standard Security, Small-sized Secure Flash
SSS = STRD_SMALL_SEC_CODE	Standard Security, Small-sized Secure Flash
SSS = NO_FLASH	No Secure Segment
SSS = NO_SEC_CODE	No Secure Segment

Secure Segment Data RAM Protection
RSS = LARGE_RAM	Large-sized Secure RAM
RSS = LARGE_SEC_RAM	Large-sized Secure RAM
RSS = MEDIUM_RAM	Medium-sized Secure RAM
RSS = MEDIUM_SEC_RAM	Medium-sized Secure RAM
RSS = SMALL_RAM	Small-sized Secure RAM
RSS = SMALL_SEC_RAM	Small-sized Secure RAM
RSS = NO_RAM	No Secure RAM
RSS = NO_SEC_RAM	No Secure RAM

General Code Segment Write Protect
GWRP = ON	User program memory is write-protected
GWRP = OFF	User program memory is not write-protected

General Segment Code Protection
GSS = HIGH	High Security Code Protection is Enabled
GSS = ON	Standard Security Code Protection is Enabled
GSS = OFF	User program memory is not code-protected

Oscillator Mode
FNOSC = FRC	Internal Fast RC (FRC)
FNOSC = FRCPLL	Internal Fast RC (FRC) w/ PLL
FNOSC = PRI	Primary Oscillator (XT, HS, EC)
FNOSC = PRIPLL	Primary Oscillator (XT, HS, EC) w/ PLL
FNOSC = SOSC	Secondary Oscillator (SOSC)
FNOSC = LPRC	Low Power RC Oscillator (LPRC)
FNOSC = FRCDIV16	Internal Fast RC (FRC) divide by 16
FNOSC = LPRCDIVN	Internal Fast RC (FRC) with divide by N

Internal External Switch Over Mode
IESO = OFF	Start-up device with user-selected oscillator source
IESO = ON	Start-up device with FRC, then automatically switch to user-selected oscillator source when ready

Primary Oscillator Source
POSCMD = EC	EC Oscillator Mode
POSCMD = XT	XT Oscillator Mode
POSCMD = HS	HS Oscillator Mode
POSCMD = NONE	Primary Oscillator Disabled

OSC2 Pin Function
OSCIOFNC = ON	OSC2 pin has digital I/O function
OSCIOFNC = OFF	OSC2 pin has clock out function

Peripheral Pin Select Configuration
IOL1WAY = OFF	Allow Multiple Re-configurations
IOL1WAY = ON	Allow Only One Re-configuration

Clock Switching and Monitor
FCKSM = CSECME	Both Clock Switching and Fail-Safe Clock Monitor are enabled
FCKSM = CSECMD	Clock switching is enabled, Fail-Safe Clock Monitor is disabled
FCKSM = CSDCMD	Both Clock Switching and Fail-Safe Clock Monitor are disabled

Watchdog Timer Postscaler
WDTPOST = PS1	1 : 1
WDTPOST = PS2	1 : 2
WDTPOST = PS4	1 : 4
WDTPOST = PS8	1 : 8
WDTPOST = PS16	1 : 16
WDTPOST = PS32	1 : 32
WDTPOST = PS64	1 : 64
WDTPOST = PS128	1 : 128
WDTPOST = PS256	1 : 256
WDTPOST = PS512	1 : 512
WDTPOST = PS1024	1 : 1,024
WDTPOST = PS2048	1 : 2,048
WDTPOST = PS4096	1 : 4,096
WDTPOST = PS8192	1 : 8,192
WDTPOST = PS16384	1 : 16,384
WDTPOST = PS32768	1 : 32,768

WDT Prescaler
WDTPRE = PR32	1 : 32
WDTPRE = PR128	1 : 128

Watchdog Timer Window
WINDIS = ON	Watchdog Timer in Window mode
WINDIS = OFF	Watchdog Timer in Non-Window mode

Watchdog Timer Enable
FWDTEN = OFF	Watchdog timer enabled/disabled by user software
FWDTEN = ON	Watchdog timer always enabled

POR Timer Value
FPWRT = PWR1	Disabled
FPWRT = PWR2	2ms
FPWRT = PWR4	4ms
FPWRT = PWR8	8ms
FPWRT = PWR16	16ms
FPWRT = PWR32	32ms
FPWRT = PWR64	64ms
FPWRT = PWR128	128ms

Alternate I2C pins
ALTI2C = ON	I2C mapped to ASDA1/ASCL1 pins
ALTI2C = OFF	I2C mapped to SDA1/SCL1 pins

Comm Channel Select
ICS = PGD3	Communicate on PGC3/EMUC3 and PGD3/EMUD3
ICS = PGD2	Communicate on PGC2/EMUC2 and PGD2/EMUD2
ICS = PGD1	Communicate on PGC1/EMUC1 and PGD1/EMUD1

JTAG Port Enable
JTAGEN = OFF	JTAG is Disabled
JTAGEN = ON	JTAG is Enabled
*/

#elif 	defined(__dsPIC33FJ32GP304__)  | defined (__dsPIC33FJ32GP302__)

//dspic33fj64gp202/204/802/804, dspic33fj128gp202/204/802/804
#pragma config FNOSC = FRC			//FRC, FRCPLL, PRI, PRIPLL, SOSC, LPRC, FRCDIV16, LPRCDIVN
#pragma config POSCMD = HS			//EC, XT, HS, NONE
#pragma config BWRP = WRPROTECT_OFF
#pragma config BSS = NO_BOOT_CODE
#pragma config GWRP = OFF
#pragma config IESO = OFF
#pragma config OSCIOFNC = ON		//ON: OSC2 as digital IO, OFF: OSC2 has digital output
#pragma config IOL1WAY = OFF		//off: allow multiple reconfiguration, on: allow only one re-configuration
#pragma config FCKSM = CSECMD		//CSECME, CSECMD, CSDCMD
#pragma config WDTPOST = PS32768
#pragma config WDTPRE = PR128		//PR32, PR128
#pragma config WINDIS = OFF			//off: watchdog timer in non-window mode
#pragma config FWDTEN = OFF
#pragma config FPWRT = PWR128
#pragma config ALTI2C = OFF
#pragma config ICS = PGD1			//PGD1, PGD2, PGD3
#pragma config JTAGEN = OFF


/*configuration bits
Boot Segment Write Protect
BWRP = WRPROTECT_ON	Boot segment is write-protected
BWRP = WRPROTECT_OFF	Boot Segment may be written

Boot Segment Program Flash Code Protection
BSS = LARGE_FLASH_HIGH	High Security, Large-sized Boot Flash
BSS = HIGH_LARGE_BOOT_CODE	High Security, Large-sized Boot Flash
BSS = MEDIUM_FLASH_HIGH	High Security, Medium-sized Boot Flash
BSS = HIGH_MEDIUM_BOOT_CODE	High Security, Medium-sized Boot Flash
BSS = SMALL_FLASH_HIGH	High Security, Small-sized Boot Flash
BSS = HIGH_SMALL_BOOT_CODE	High Security, Small-sized Boot Flash
BSS = LARGE_FLASH_STD	Standard Security, Large-sized Boot Flash
BSS = STRD_LARGE_BOOT_CODE	Standard Security, Large-sized Boot Flash
BSS = MEDIUM_FLASH_STD	Standard Security, Medium-sized Boot Flash
BSS = STRD_MEDIUM_BOOT_CODE	Standard Security, Medium-sized Boot Flash
BSS = SMALL_FLASH_STD	Standard Security, Small-sized Boot Flash
BSS = STRD_SMALL_BOOT_CODE	Standard Security, Small-sized Boot Flash
BSS = NO_FLASH	No Boot program Flash segment
BSS = NO_BOOT_CODE	No Boot program Flash segment

General Code Segment Write Protect
GWRP = ON	User program memory is write-protected
GWRP = OFF	User program memory is not write-protected

General Segment Code Protection
GSS = HIGH	High Security Code Protection is Enabled
GSS = ON	Standard Security Code Protection is Enabled
GSS = OFF	User program memory is not code-protected

Oscillator Mode
FNOSC = FRC	Internal Fast RC (FRC)
FNOSC = FRCPLL	Internal Fast RC (FRC) w/ PLL
FNOSC = PRI	Primary Oscillator (XT, HS, EC)
FNOSC = PRIPLL	Primary Oscillator (XT, HS, EC) w/ PLL
FNOSC = SOSC	Secondary Oscillator (SOSC)
FNOSC = LPRC	Low Power RC Oscillator (LPRC)
FNOSC = FRCDIV16	Internal Fast RC (FRC) divide by 16
FNOSC = LPRCDIVN	Internal Fast RC (FRC) with divide by N

Internal External Switch Over Mode
IESO = OFF	Start-up device with user-selected oscillator source
IESO = ON	Start-up device with FRC, then automatically switch to user-selected oscillator source when ready

Primary Oscillator Source
POSCMD = EC	EC Oscillator Mode
POSCMD = XT	XT Oscillator Mode
POSCMD = HS	HS Oscillator Mode
POSCMD = NONE	Primary Oscillator Disabled

OSC2 Pin Function
OSCIOFNC = ON	OSC2 pin has digital I/O function
OSCIOFNC = OFF	OSC2 pin has clock out function

Peripheral Pin Select Configuration
IOL1WAY = OFF	Allow Multiple Re-configurations
IOL1WAY = ON	Allow Only One Re-configuration

Clock Switching and Monitor
FCKSM = CSECME	Both Clock Switching and Fail-Safe Clock Monitor are enabled
FCKSM = CSECMD	Clock switching is enabled, Fail-Safe Clock Monitor is disabled
FCKSM = CSDCMD	Both Clock Switching and Fail-Safe Clock Monitor are disabled

Watchdog Timer Postscaler
WDTPOST = PS1	1 : 1
WDTPOST = PS2	1 : 2
WDTPOST = PS4	1 : 4
WDTPOST = PS8	1 : 8
WDTPOST = PS16	1 : 16
WDTPOST = PS32	1 : 32
WDTPOST = PS64	1 : 64
WDTPOST = PS128	1 : 128
WDTPOST = PS256	1 : 256
WDTPOST = PS512	1 : 512
WDTPOST = PS1024	1 : 1,024
WDTPOST = PS2048	1 : 2,048
WDTPOST = PS4096	1 : 4,096
WDTPOST = PS8192	1 : 8,192
WDTPOST = PS16384	1 : 16,384
WDTPOST = PS32768	1 : 32,768

WDT Prescaler
WDTPRE = PR32	1 : 32
WDTPRE = PR128	1 : 128

Watchdog Timer Window
WINDIS = ON	Watchdog Timer in Window mode
WINDIS = OFF	Watchdog Timer in Non-Window mode

Watchdog Timer Enable
FWDTEN = OFF	Watchdog timer enabled/disabled by user software
FWDTEN = ON	Watchdog timer always enabled

POR Timer Value
FPWRT = PWR1	Disabled
FPWRT = PWR2	2ms
FPWRT = PWR4	4ms
FPWRT = PWR8	8ms
FPWRT = PWR16	16ms
FPWRT = PWR32	32ms
FPWRT = PWR64	64ms
FPWRT = PWR128	128ms

Alternate I2C pins
ALTI2C = ON	I2C mapped to ASDA1/ASCL1 pins
ALTI2C = OFF	I2C mapped to SDA1/SCL1 pins

Comm Channel Select
ICS = PGD3	Communicate on PGC3/EMUC3 and PGD3/EMUD3
ICS = PGD2	Communicate on PGC2/EMUC2 and PGD2/EMUD2
ICS = PGD1	Communicate on PGC1/EMUC1 and PGD1/EMUD1

JTAG Port Enable
JTAGEN = OFF	JTAG is Disabled
JTAGEN = ON	JTAG is Enabled
*/


#else
#error "dspic33duino.c: specified dspic33 chip not supported!"
#endif

//global defines

//global variables
//declare pins
//ALL PINS ARE MAPPED, WHETHER THEY EXIST OR NOT
//SO MAKE SURE THAT THE PINS YOU PICKED ACTUALLY EXIST FOR YOUR PACKAGE
//Pin  0.. 7 -> GPIOA
//Pin  8..15 -> GPIOB
const PIN2GPIO GPIO_PinDef[]= {
	{GPIOA, 1<<0},						//PICduino Pin  0 = RP0/PB0/CHIP PIN4
	{GPIOA, 1<<1},						//PICduino Pin  1 = RP1/PB1/CHIP PIN5
	{GPIOA, 1<<2},						//PICduino Pin  2 = RP2/PB2/CHIP PIN6
	{GPIOA, 1<<3},						//PICduino Pin  3 = RP3/PB3/CHIP PIN7
	{GPIOA, 1<<4},						//PICduino Pin  4 = RP4/PB4/CHIP PIN11
	{GPIOA, 1<<5},						//PICduino Pin  5 = RP5/PB5/CHIP PIN14
	{GPIOA, 1<<6},						//PICduino Pin  6 = RP6/PB6/CHIP PIN15
	{GPIOA, 1<<7},						//PICduino Pin  7 = RP7/PB7/CHIP PIN16
	{GPIOA, 1<<8},						//PICduino Pin  8 = RP8/PB8/CHIP PIN17
	{GPIOA, 1<<9},						//PICduino Pin  9 = RP9/PB9/CHIP PIN18
	{GPIOA, 1<<10},						//PICduino Pin 10 = RP10/PB10/CHIP PIN21
	{GPIOA, 1<<11},						//PICduino Pin 11 = RP11/PB11/CHIP PIN22
	{GPIOA, 1<<12},						//PICduino Pin 12 = RP12/PB12/CHIP PIN23
	{GPIOA, 1<<13},						//PICduino Pin 13 = RP13/PB13/CHIP PIN24
	{GPIOA, 1<<14},						//PICduino Pin 14 = RP14/PB14/CHIP PIN25
	{GPIOA, 1<<15},						//PICduino Pin 15 = RP15/PB15/CHIP PIN26

	{GPIOB, 1<<0},						//PICduino Pin 16 = RP0/PB0/CHIP PIN4
	{GPIOB, 1<<1},						//PICduino Pin 17 = RP1/PB1/CHIP PIN5
	{GPIOB, 1<<2},						//PICduino Pin 18 = RP2/PB2/CHIP PIN6
	{GPIOB, 1<<3},						//PICduino Pin 19 = RP3/PB3/CHIP PIN7
	{GPIOB, 1<<4},						//PICduino Pin 20 = RP4/PB4/CHIP PIN11
	{GPIOB, 1<<5},						//PICduino Pin 21 = RP5/PB5/CHIP PIN14
	{GPIOB, 1<<6},						//PICduino Pin 22 = RP6/PB6/CHIP PIN15
	{GPIOB, 1<<7},						//PICduino Pin 23 = RP7/PB7/CHIP PIN16
	{GPIOB, 1<<8},						//PICduino Pin 24 = RP8/PB8/CHIP PIN17
	{GPIOB, 1<<9},						//PICduino Pin 25 = RP9/PB9/CHIP PIN18
	{GPIOB, 1<<10},						//PICduino Pin 26 = RP10/PB10/CHIP PIN21
	{GPIOB, 1<<11},						//PICduino Pin 27 = RP11/PB11/CHIP PIN22
	{GPIOB, 1<<12},						//PICduino Pin 28 = RP12/PB12/CHIP PIN23
	{GPIOB, 1<<13},						//PICduino Pin 29 = RP13/PB13/CHIP PIN24
	{GPIOB, 1<<14},						//PICduino Pin 30 = RP14/PB14/CHIP PIN25
	{GPIOB, 1<<15},						//PICduino Pin 31 = RP15/PB15/CHIP PIN26

#if defined(GPIOC)
	{GPIOC, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOC, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOC, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOC, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOC, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOC, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOC, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOC, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOC, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOC, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOC, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOC, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOC, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOC, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOC, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOC, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOC

#if defined(GPIOD)
	{GPIOD, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOD, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOD, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOD, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOD, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOD, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOD, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOD, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOD, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOD, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOD, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOD, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOD, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOD, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOD, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOD, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOD

#if defined(GPIOE)
	{GPIOE, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOE, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOE, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOE, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOE, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOE, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOE, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOE, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOE, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOE, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOE, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOE, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOE, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOE, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOE, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOE, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOE

#if defined(GPIOF)
	{GPIOF, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOF, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOF, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOF, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOF, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOF, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOF, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOF, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOF, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOF, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOF, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOF, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOF, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOF, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOF, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOF, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOF

#if defined(GPIOG)
	{GPIOG, 1<<0},						//PICduino Pin 32 = RP0/PB0/CHIP PIN4
	{GPIOG, 1<<1},						//PICduino Pin 33 = RP1/PB1/CHIP PIN5
	{GPIOG, 1<<2},						//PICduino Pin 34 = RP2/PB2/CHIP PIN6
	{GPIOG, 1<<3},						//PICduino Pin 35 = RP3/PB3/CHIP PIN7
	{GPIOG, 1<<4},						//PICduino Pin 36 = RP4/PB4/CHIP PIN11
	{GPIOG, 1<<5},						//PICduino Pin 37 = RP5/PB5/CHIP PIN14
	{GPIOG, 1<<6},						//PICduino Pin 38 = RP6/PB6/CHIP PIN15
	{GPIOG, 1<<7},						//PICduino Pin 39 = RP7/PB7/CHIP PIN16
	{GPIOG, 1<<8},						//PICduino Pin 40 = RP8/PB8/CHIP PIN17
	{GPIOG, 1<<9},						//PICduino Pin 41 = RP9/PB9/CHIP PIN18
	{GPIOG, 1<<10},						//PICduino Pin 42 = RP10/PB10/CHIP PIN21
	{GPIOG, 1<<11},						//PICduino Pin 43 = RP11/PB11/CHIP PIN22
	{GPIOG, 1<<12},						//PICduino Pin 44 = RP12/PB12/CHIP PIN23
	{GPIOG, 1<<13},						//PICduino Pin 45 = RP13/PB13/CHIP PIN24
	{GPIOG, 1<<14},						//PICduino Pin 46 = RP14/PB14/CHIP PIN25
	{GPIOG, 1<<15},						//PICduino Pin 47 = RP15/PB15/CHIP PIN26
#endif	//GPIOG

};

//set up pll multipliers
//Fout=Fin / N1 * M / N2
//N1: 0 (2:1), 1 (3:1), ..., 31 (33:1)
//M : 0 (2x ), 1 (3x ), ..., 511 (513x)
//N2: 0 (2:1), 1 (4:1), 2 (6:1), 3 (8:1). DS says 2 is reserved/not use
void pllSet(uint32_t N1, uint32_t M, uint32_t N2) {
	CLKDIVbits.PLLPRE = N1 & 0x1f; 
	CLKDIVbits.PLLPOST= N2 & 0x03;
	PLLFBD = M & 0x1ff;
   	//while (OSCCONbits.LOCK == 0) continue;			//1->pll in lock, 0->pll not locked
}

//read pll multiplier
//Fout=Fin / N1 * M / N2
uint32_t pllGet(uint32_t Fin) {
	uint32_t M, N1, N2;
	
	N1 = CLKDIVbits.PLLPRE + 2;						//valid values: 0(/2)..7(/128)
	M  = PLLFBD + 2;								//valid values: 0(2x)..511(513x)
	N2 = 2 * (CLKDIVbits.PLLPOST + 1);				//valid values: 0(/2), 1(/4), 3(/8). 2(/6) is reserved - don't use
	return Fin / N1 * M / N2;
}

	
//set up core timer
//global variables
uint32_t SystemCoreClock=F_FRC;			//system core clock, before devided by 2. Updated by SystemCoreClockUpdate()

//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t systick_ovf = 0;
//static uint16_t timer1_fract = 0;
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
uint32_t SystemCoreClockUpdate(void) {
	uint32_t tmp=F_FRC;

	//test for current oscillator configuration
	switch (OSCCONbits.COSC) {
	case 0b111:						//FRCDIVN
		tmp = (F_FRC >> CLKDIVbits.FRCDIV); break;
	case 0b110:						//FRCDIV16
		tmp = F_FRC / 16;				   break;
	case 0b101:						//LPRC
		tmp = F_LPRC;						break;
	case 0b100:						//SOSC
		tmp = F_SOSC;						break;
	case 0b011:						//POSCxPLL
		tmp = pllGet(F_XTAL);				break;
	case 0b010:						//POSC
		tmp = F_XTAL;						break;
	case 0b001:						//FRCxPLL, driven by FRCDIV
		tmp = pllGet(F_FRC >> CLKDIVbits.FRCDIV); 	break;
	case 0b000:						//FRC
		tmp = F_FRC;						break;
	}
	return SystemCoreClock=tmp;
}

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
uint32_t SystemCoreClockSwitch(uint8_t nosc) {
	//uint32_t tmp=F_FRC;

	//switch oscillator
	nosc = nosc & 0b111;						//only lowest 3 bits effective
	di();										//disable the interrupt
	do {
		__builtin_write_OSCCONH(nosc);
		__builtin_write_OSCCONL(OSCCON | 0x01);		//set oswen bit -> start the switch
		while (OSCCONbits.OSWEN != 0) continue;		//0->clock switch complete
	} while (OSCCONbits.COSC != nosc);	//make sure that the clock switch is successful. otherwise, hang here
	ei();										//enable the interrupt
	return SystemCoreClockUpdate();				//update the core clock
}

//timer1 overflow isr
//void _ISR_PSV _T2Interrupt(void) {
//	IFS0bits.T2IF=0;							//clear tmr1 interrupt flag
//	systick_ovf+=0x10000ul;							//increment overflow count: 16-bit timer
//}

//reset the mcu
void mcuInit(void) {
	//switch clock - not all clock sources show (LPRC)
	//SystemCoreClockPOSC();					//switch to primary oscillator
	//SystemCoreClockPOSCPLL();					//switch to primary oscillator x PLL
	//SystemCoreClockFRC();						//switch to FRC
	//SystemCoreClockFRCPLL();					//switch to FRCxPLL
	
	//powering up on FRC (default)
	//set poster scaler for FRC (default 2:1), for FRCDIV
	CLKDIVbits.FRCDIV=0;						//rc divider (0..7->1:1..128:1): 0->1:1 (default), 1->2:1, 2->4:1, 3=8:1, ...

	//turn off all peripherals by setting the bits to 1
	PMD1=0xffff;
	PMD2=0xffff;
	PMD3=0xffff;

	//all pins digital
	AD1PCFGL = 0xffff;							//1->all pins digital

#if defined(SYSTICK_TMR1)						//Systick running on tmr1
	//initialize tmr1 for tick/pwm generation - do not stop this
	PMD1bits.T1MD = 0;							//0->enable the peripheral, 1->disable the peripheral
	T1CON = 0x0000;				 			//stop timer
	T1CONbits.TCKPS = 0;						//set the prescaler: 0->1:1, 1->8:1, 2->64:1, 3->256:1
	T1CONbits.TCS = 0;			 				//use internal instruction clock from F_PHB
	T1CONbits.TGATE = 0;						//no gating
	//T1CONbits.T32 = 0;							//0->16 bit mode, 1->32 bit mode
	TMR1 = 0;									//initialize the timer2 counter
	PR1 = PWM_PR;								//set pwm period
	IFS0bits.T1IF = 0;							//reset the flag
	IPC0bits.T1IP = TxIP_DEFAULT;
	//IPC0bits.T2IS = TxIS_DEFAULT;
	IEC0bits.T1IE = 1;							//0->disable tmr2 isr, 1->enable tmr2 isr
	T1CONbits.TON = 1;			 				//turn on the timer
#else											//systick running on tmr2
	//initialize tmr2 for tick/pwm generation - do not stop this
	PMD1bits.T2MD = 0;							//0->enable the peripheral, 1->disable the peripheral
	T2CON = 0x0000;				 			//stop timer
	T2CONbits.TCKPS = 0;						//set the prescaler: 0->1:1, 1->8:1, 2->64:1, 3->256:1
	T2CONbits.TCS = 0;			 				//use internal instruction clock from F_PHB
	T2CONbits.TGATE = 0;						//no gating
	T2CONbits.T32 = 0;							//0->16 bit mode, 1->32 bit mode
	TMR2 = 0;									//initialize the timer2 counter
	PR2 = PWM_PR;								//set pwm period
	IFS0bits.T2IF = 0;							//reset the flag
	IPC1bits.T2IP = TxIP_DEFAULT;
	//IPC1bits.T2IS = TxIS_DEFAULT;
	IEC0bits.T2IE = 1;							//0->disable tmr2 isr, 1->enable tmr2 isr
	T2CONbits.TON = 1;			 				//turn on the timer
#endif		//systick on tmr1 or tmr2

	//update SystemCoreClock
	SystemCoreClockUpdate();					//update system core clock

	//enable global interrupts
	ei();										//testing

}

//empty interrupt handler
void empty_handler(void) {
	//do nothing here
}

//C main loop
#ifndef USE_MAIN
int main(void) {

	mcuInit();						//reset the mcu
	setup();						//run the setup code
	while (1) {
		loop();						//run the default loop
	}
}
#endif	//use_main

//Arduino Functions: GPIO
void pinPullup(PIN_TypeDef pin) {
   	switch (pin) {
   		case PA4: IO_SET(CNPU1, 1<<0 ); break;	//PA4=CN0
   		case PB4: IO_SET(CNPU1, 1<<1 ); break;	//PB4=CN1
   		case PA0: IO_SET(CNPU1, 1<<2 ); break;	//PA0=CN2
   		case PA1: IO_SET(CNPU1, 1<<3 ); break;	//PA1=CN3
   		case PB0: IO_SET(CNPU1, 1<<4 ); break;	//PB0=CN4
   		case PB1: IO_SET(CNPU1, 1<<5 ); break;	//PB1=CN5
   		case PB2: IO_SET(CNPU1, 1<<6 ); break;	//PB2=CN6
   		case PB3: IO_SET(CNPU1, 1<<7 ); break;	//PB3=CN7
#if defined(GPIOC)
   		case PC0: IO_SET(CNPU1, 1<<8 ); break;	//PC0=CN8
   		case PC1: IO_SET(CNPU1, 1<<9 ); break;	//PC1=CN9
   		case PC2: IO_SET(CNPU1, 1<<10); break;	//PC2=CN10
#endif
   		case PB15:IO_SET(CNPU1, 1<<11); break;	//PB15=CN11
   		case PB14:IO_SET(CNPU1, 1<<12); break;	//PB14=CN12
   		case PB13:IO_SET(CNPU1, 1<<13); break;	//PB13=CN13
   		case PB12:IO_SET(CNPU1, 1<<14); break;	//PB12=CN14
   		case PB11:IO_SET(CNPU1, 1<<15); break;	//PB11=CN15
   		case PB10:IO_SET(CNPU2, 1<<0 ); break;	//PB10=CN16
#if defined(GPIOC)
   		case PC7: IO_SET(CNPU2, 1<<1 ); break;	//PC7=CN17
   		case PC6: IO_SET(CNPU2, 1<<2 ); break;	//PC6=CN18
   		case PC9: IO_SET(CNPU2, 1<<3 ); break;	//PC9=CN19
   		case PC8: IO_SET(CNPU2, 1<<4 ); break;	//PC8=CN20
#endif
   		case PB9: IO_SET(CNPU2, 1<<5 ); break;	//PB9=CN21
   		case PB8: IO_SET(CNPU2, 1<<6 ); break;	//PB8=CN22
   		case PB7: IO_SET(CNPU2, 1<<7 ); break;	//PB7=CN23
   		case PB6: IO_SET(CNPU2, 1<<8 ); break;	//PB6=CN24
#if defined(GPIOC)
   		case PC4: IO_SET(CNPU2, 1<<0 ); break;	//PC4=CN25
   		case PC5: IO_SET(CNPU2, 1<<10); break;	//PC5=CN26
#endif
   		case PB5: IO_SET(CNPU2, 1<<11); break;	//PB5=CN27
#if defined(GPIOC)
   		case PC3: IO_SET(CNPU2, 1<<12); break;	//PC3=CN28
#endif
   		case PA3: IO_SET(CNPU2, 1<<13); break;	//PA3=CN29
   		case PA2: IO_SET(CNPU2, 1<<14); break;	//PA2=CN30
   		//case PB5: IO_SET(CNPU2, 1<<15); break;	//PC3=CN31
   		default: break;							//do nothing if the pin isn't a CN pin
   	};
}

//enable pulldown - not available on pic24
void pinPulldown(PIN_TypeDef pin) {
}

	
//set a pin mode to INPUT or OUTPUT
//no error checking on PIN
inline void pinMode(PIN_TypeDef pin, uint8_t mode) {
	//if (mode==OUTPUT) GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	//else GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	switch (mode) {
		case INPUT_PULLUP:
			pinPullup(pin);						//enable pull-up
			GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
		case INPUT_PULLDOWN: 							//pull down not available: implemented as floating input
			pinPulldown(pin);
			GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
		case INPUT:
			GIO_IN(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
		case OUTPUT_OD:
			pinPullup(pin);						//enable pullup
			IO_SET(GPIO_PinDef[pin].gpio->ODC, GPIO_PinDef[pin].mask);
			GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
		case OUTPUT:
			GIO_OUT(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask); break;
		default: break;
	} 
}

//set / clear a pin
inline void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	if (val==LOW) GIO_CLR(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
	else GIO_SET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask);
}

//read a pin
inline int digitalRead(PIN_TypeDef pin) {
	return (GIO_GET(GPIO_PinDef[pin].gpio, GPIO_PinDef[pin].mask))?HIGH:LOW;
}
//end GPIO

//ticks()
//Arduino Functions: Time
//return timer ticks
uint32_t systicks(void) {
	volatile uint32_t m;					//stores overflow count
	volatile uint16_t f;					//return the fractions / TMR1 value

	//use double reads
	do {
		m = systick_ovf;
#if defined(SYSTICK_TMR1)
		f = TMR1;
#else
		f = TMR2;
#endif	//systick_tmr1
	} while (m != systick_ovf);
	//now m and f are consistent
	return (m | f);
}

//delay millisseconds
//void delay(uint32_t ms) {
//	uint32_t start_time = ticks();
//	ms *= cyclesPerMillisecond();
//	while (ticks() - start_time < ms) continue;
//}

//delay micros seconds
//void delayMicroseconds(uint32_t us) {
//	uint32_t start_time = ticks();
//	us *= cyclesPerMicrosecond();
//	while (ticks() - start_time < us) continue;
//}

//delay ticks
void delayTks(uint32_t tks) {
	uint32_t start_time = ticks();
	while (ticks() - start_time < tks) continue;
}
//end Time

//static char uRAM[uRAM_SIZE];			//transmission buffer
//uart1
#if defined(UART1_TXISR)
//uart1 isr
volatile char *_U1TX_ptr;
//volatile unsigned char _U1TX_BUSY=0;	//0=u1 transmission done, 1=u1 transmission in process

//ux transmit isr
//'\0' is now transmitted
//void _ISR _U1TXInterrupt(void) {
void _ISR_PSV _U1TXInterrupt(void) {
	//char tmp;
	//TX only, RX not implemented
	IFS0bits.U1TXIF = 0;				//UxTXIF = 0;							//clear the flag
	if (*_U1TX_ptr) {					//0 indicates the end of the string
		//_UxTX_ptr;					//increment to the next char
		//tmp = *_UxTX_ptr++; UxTXREG = tmp;
		U1TXREG = *_U1TX_ptr++;			//load up a char to be transmitted
		//NOP();
		//IFS0bits.U1TXIF = 0;			//UxTXIF = 0;						//clear the flag
	} else {
		//UxSTA.UTXEN = 0;				//turn off the transmission
		IEC0bits.U1TXIE = 0;			//UxTXIE = 0;						//disable the interrupt
		//_U1TX_BUSY = 0;				//transmission done
	}
	//IFS0bits.U1TXIF = 0;				//clear the flag
}
#endif		//UART1_TXISR

//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart1Init(unsigned long baud_rate) {
	//enable the pins
#if defined(U1RX2RP)
	U1RX2RP();					//PPS_U1RX_TO_RP(U1RX_RP);
#endif

#if defined(U1TX2RP)
	U1TX2RP();					//PPS_U1TX_TO_RP();
#endif


	//disable md bits
	PMD1bits.U1MD = 0;				//power up the module

	//U2MODEbits register
	//bit 15 UARTEN: UARTx Enable bit(1)
	//1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
	//0 = UARTx is disabled; all UARTx pins are controlled by PORT latches; UARTx power consumption is	minimal
	U1MODEbits.UARTEN=1;	//enable the uart module
	//bit 14 Unimplemented: Read as `0'
	//bit 13 USIDL: Stop in Idle Mode bit
	//1 = Discontinue module operation when device enters Idle mode
	//0 = Continue module operation in Idle mode
	//bit 12 IREN: IrDAr Encoder and Decoder Enable bit(2)
	//1 = IrDA encoder and decoder enabled
	//0 = IrDA encoder and decoder disabled
	U1MODEbits.IREN = 0;		//disable irda
	U1MODEbits.RTSMD = 0;		//1=in simplex mode. 0 = no flow control
	//bit 11 RTSMD: Mode Selection for UxRTS Pin bit
	//1 = UxRTS pin in Simplex mode
	//0 = UxRTS pin in Flow Control mode
	//bit 10 Unimplemented: Read as `0'
	//bit 9-8 UEN1:UEN0: UARTx Enable bits(3)
	//11 = UxTX, UxRX and BCLKx pins are enabled and used; UxCTS pin controlled by PORT latches
	//10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
	//01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by PORT latches
	//00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLKx pins controlled by PORT latches
	U1MODEbits.UEN1=0, U2MODEbits.UEN0=0;
	//bit 7 WAKE: Wake-up on Start Bit Detect During Sleep Mode Enable bit
	//1 = UARTx will continue to sample the UxRX pin; interrupt generated on falling edge, bit cleared in hardware on following rising edge
	//0 = No wake-up enabled
	U1MODEbits.WAKE = 0;
	//bit 6 LPBACK: UARTx Loopback Mode Select bit
	//1 = Enable Loopback mode
	//0 = Loopback mode is disabled
	U1MODEbits.LPBACK = 0;
	//bit 5 ABAUD: Auto-Baud Enable bit
	//1 = Enable baud rate measurement on the next character - requires reception of a Sync field (55h); cleared in hardware upon completion
	//0 = Baud rate measurement disabled or completed
	U1MODEbits.ABAUD = 0;
	//bit 4 RXINV: Receive Polarity Inversion bit
	//1 = UxRX Idle state is `0'
	//0 = UxRX Idle state is `1'
	U1MODEbits.URXINV = 0;
	//bit 3 BRGH: High Baud Rate Enable bit
	//1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
	//0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
	U1MODEbits.BRGH = (F_UART / 4 / baud_rate < 0xffff)?1:0;	//U1MODEbits.BRGH = 1;
	//bit 2-1 PDSEL1:PDSEL0: Parity and Data Selection bits
	//11 = 9-bit data, no parity
	//10 = 8-bit data, odd parity
	//01 = 8-bit data, even parity
	//00 = 8-bit data, no parity
	U1MODEbits.PDSEL1=0, U1MODEbits.PDSEL0=0;
	//bit 0 STSEL: Stop Bit Selection bit
	//1 = Two Stop bits
	//0 = One Stop bit
	U1MODEbits.STSEL=0;
	//Note 1: If UARTEN = 1, the peripheral inputs and outputs must be configured to an available RPn pin. See
	//Section 10.4 "Peripheral Pin Select" for more information.
	//2: This feature is only available for the 16x BRG mode (BRGH = 0).
	//3: Bit availability depends on pin availability.

#if 1	//BAUDCON - with rounding up
	if (U1MODEbits.BRGH==1) U1BRG = (F_UART + 2 * baud_rate) / 4 / baud_rate - 1;				//set lower byte of brg
	else U1BRG = (F_UART + 8 * baud_rate) / 16 / baud_rate - 1;
#else	//BAUDCON - without rounding up
	if (U1MODEbits.BRGH==1) U1BRG = (F_UART + 0*2 * baud_rate) / 4 / baud_rate - 1;				//set lower byte of brg
	else U1BRG = (F_UART + 0*8 * baud_rate) / 16 / baud_rate - 1;
#endif	

	//disable interrupts

	//set up status register
//#if defined(UxTX2RP)				//tx is used
	IFS0bits.U1TXIF = 0;						//clera the flag
	IEC0bits.U1TXIE = 0;						//disable the interrupt
	IPC3bits.U1TXIP = UxIP_DEFAULT;				//set priority

	//bit 15,13 UTXISEL1:UTXISEL0: Transmission Interrupt Mode Selection bits
	//11 = Reserved; do not use
	//10 = Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
	//01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit operations are completed
	//00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies there is at least one character open in the transmit buffer)
	U1STAbits.UTXISEL1=0, U1STAbits.UTXISEL0=0;
//#endif
	//bit 14 UTXINV: IrDAr Encoder Transmit Polarity Inversion bit
	//If IREN = 0:
	//1 = UxTX Idle `0'
	//0 = UxTX Idle `1'
	//If IREN = 1:
	//1 = UxTX Idle `1'
	//0 = UxTX Idle `0'
	U1STAbits.UTXINV=0;
	//bit 12 Unimplemented: Read as `0'
	//bit 11 UTXBRK: Transmit Break bit
	//1 = Send Sync Break on next transmission - Start bit, followed by twelve `0' bits, followed by Stop bit;	cleared by hardware upon completion
	//0 = Sync Break transmission disabled or completed
	U1STAbits.UTXBRK=0;
	//bit 10 UTXEN: Transmit Enable bit(1)
	//1 = Transmit enabled, UxTX pin controlled by UARTx
	//0 = Transmit disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled by the PORT register.
#if defined(U1TX2RP)
	U1STAbits.UTXEN=1;
#else
	U1STAbits.UTXEN=0;
#endif
#if defined(U1RX2RP)
	//U1STAbits.URXEN=1;
#else
	//U1STAbits.URXEN=0;
#endif
	//bit 9 UTXBF: Transmit Buffer Full Status bit (read-only)
	//1 = Transmit buffer is full
	//0 = Transmit buffer is not full, at least one more character can be written
	//bit 8 TRMT: Transmit Shift Register Empty bit (read-only)
	//1 = Transmit Shift Register is empty and transmit buffer is empty (the last transmission has completed)
	//0 = Transmit Shift Register is not empty, a transmission is in progress or queued
//#if defined(UxRX2RP)
	IFS0bits.U1RXIF = 0;						//clear the flag
	IEC0bits.U1RXIE = 0;						//disable the interrupt

	//bit 7-6 URXISEL1:URXISEL0: Receive Interrupt Mode Selection bits
	//11 = Interrupt is set on RSR transfer, making the receive buffer full (i.e., has 4 data characters)
	//10 = Interrupt is set on RSR transfer, making the receive buffer 3/4 full (i.e., has 3 data characters)
	//0x = Interrupt is set when any character is received and transferred from the RSR to the receive buffer. Receive buffer has one or more characters.
	U1STAbits.URXISEL1 = 0, U1STAbits.URXISEL0 = 0;
//#endif
	//bit 5 ADDEN: Address Character Detect bit (bit 8 of received data = 1)
	//1 = Address Detect mode enabled. If 9-bit mode is not selected, this does not take effect.
	//0 = Address Detect mode disabled
	//bit 4 RIDLE: Receiver Idle bit (read-only)
	//1 = Receiver is Idle
	//0 = Receiver is active
	//bit 3 PERR: Parity Error Status bit (read-only)
	//1 = Parity error has been detected for the current character (character at the top of the receive FIFO)
	//0 = Parity error has not been detected
	//bit 2 FERR: Framing Error Status bit (read-only)
	//1 = Framing error has been detected for the current character (character at the top of the receive FIFO)
	//0 = Framing error has not been detected
	//bit 1 OERR: Receive Buffer Overrun Error Status bit (clear/read-only)
	//1 = Receive buffer has overflowed
	//0 = Receive buffer has not overflowed (clearing a previously set OERR bit (1 ? 0 transition) will reset the receiver buffer and the RSR to the empty state)
	//bit 0 URXDA: Receive Buffer Data Available bit (read-only)
	//1 = Receive buffer has data; at least one more character can be read
	//0 = Receive buffer is empty
	//REGISTER 17-2: U1STAbits: UARTx STATUS AND CONTROL REGISTER (CONTINUED)
	//Note 1: If UARTEN = 1, the peripheral inputs and outputs must be configured to an available RPn pin. See
	//Section 10.4 "Peripheral Pin Select" for more information.
}

//put a string
void uart1Puts(char *str) {
#ifndef UART1_TXISR
	//polling transmission
	while(*str) {
		uart1Putch(*str++);	//send the ch and advance the pointer
	}
#else
	//interrupt transmission
	if (*str) {								//if the string isn't empty to begin with
		uart1Wait();						//for compatability with polling transmission
		//U1TXREG = 0;						//clear the buffer
		//_U1TX_BUSY  = 1;					//transmission in progress
		_U1TX_ptr=str;						//point to the string to be transmitted
		//U1STAbits.UTXEN = 1;				//enable transmission - always enabled
		//tmp = *_UxTX_ptr++;	UxTXREG = tmp;						//*_UxTX_ptr++;				//load up the 1st char. advance to the next char
		IFS0bits.U1TXIF = 0;				//UxTXIF = 0;							//clear the flag
		IEC0bits.U1TXIE = 1;				//UxTXIE = 1;							//enable the interrupt
		U1TXREG = *_U1TX_ptr++;
	}
#endif	//UART1_TXISR
}
/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

//put a line termined with ln
//void uart1Putline(char *ln) {
//	//USARTWriteString(ln);
//	uart1Puts(ln);
//	//USARTWriteString("\r\n");
//	uart1Puts((char *)"\r\n");
//}

//get the received char
//uint8_t uart1Getch(void) {
//	return U1RXREG;		//return it
//}

//test if data rx is available
//uint16_t uart1Available(void) {
//	return U1STAbits.URXDA;
//}

//test if uart tx is busy
//uint16_t uart1Busy(void) {
//	return U1STAbits.UTXBF;
//}

//print to uart1
void u1Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	uint8_t i=20;
	char negFlag=0;						//0->positive, 1->negative

	uart1Wait();						//wait for uart to free up
	
	strcpy(uRAM, str);					//copy to uarm
	if (dat < 0) {
		negFlag=1;						//uRAM[i-13]='-';
		dat = -dat;
	}
	
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=negFlag?'-':' ';
	
	uart1Puts(uRAM);	//send a message on uart1
}

//uart2
#if defined(UART2_TXISR)
//uart1 isr
volatile char *_U2TX_ptr;
//volatile unsigned char _U1TX_BUSY=0;	//0=u1 transmission done, 1=u1 transmission in process

//ux transmit isr
//'\0' is now transmitted
//void _ISR _U1TXInterrupt(void) {
void _ISR_PSV _U2TXInterrupt(void) {
	//char tmp;
	//TX only, RX not implemented
	IFS1bits.U2TXIF = 0;				//UxTXIF = 0;							//clear the flag
	if (*_U2TX_ptr) {					//0 indicates the end of the string
		//_UxTX_ptr;					//increment to the next char
		//tmp = *_UxTX_ptr++; UxTXREG = tmp;
		U2TXREG = *_U2TX_ptr++;			//load up a char to be transmitted
		//NOP();
		//IFS1bits.U2TXIF = 0;			//UxTXIF = 0;						//clear the flag
	} else {
		//UxSTA.UTXEN = 0;				//turn off the transmission
		IEC1bits.U2TXIE = 0;			//UxTXIE = 0;						//disable the interrupt
		//_U1TX_BUSY = 0;				//transmission done
	}
	//IFS1bits.U2TXIF = 0;				//UxTXIF = 0;							//clear the flag
}
#endif		//UART1_TXISR

//initialize usart: high baudrate (brgh=1), 16-bit baudrate (brg16=1)
//baudrate=Fxtal/(4*(spbrg+1))
//spbrg=(Fxtal/4/baudrate)-1
//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart2Init(unsigned long baud_rate) {
	//enable the pins
#if defined(U2RX2RP)
	U2RX2RP();					//PPS_U1RX_TO_RP(U1RX_RP);
#endif

#if defined(U2TX2RP)
	U2TX2RP();					//PPS_U1TX_TO_RP();
#endif


	//disable md bits
	PMD1bits.U2MD = 0;				//power up the module

	//U2MODEbits register
	//bit 15 UARTEN: UARTx Enable bit(1)
	//1 = UARTx is enabled; all UARTx pins are controlled by UARTx as defined by UEN<1:0>
	//0 = UARTx is disabled; all UARTx pins are controlled by PORT latches; UARTx power consumption is	minimal
	U2MODEbits.UARTEN=1;	//enable the uart module
	//bit 14 Unimplemented: Read as `0'
	//bit 13 USIDL: Stop in Idle Mode bit
	//1 = Discontinue module operation when device enters Idle mode
	//0 = Continue module operation in Idle mode
	//bit 12 IREN: IrDAr Encoder and Decoder Enable bit(2)
	//1 = IrDA encoder and decoder enabled
	//0 = IrDA encoder and decoder disabled
	U2MODEbits.IREN = 0;		//disable irda
	U2MODEbits.RTSMD = 0;		//1=in simplex mode. 0 = no flow control
	//bit 11 RTSMD: Mode Selection for UxRTS Pin bit
	//1 = UxRTS pin in Simplex mode
	//0 = UxRTS pin in Flow Control mode
	//bit 10 Unimplemented: Read as `0'
	//bit 9-8 UEN1:UEN0: UARTx Enable bits(3)
	//11 = UxTX, UxRX and BCLKx pins are enabled and used; UxCTS pin controlled by PORT latches
	//10 = UxTX, UxRX, UxCTS and UxRTS pins are enabled and used
	//01 = UxTX, UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by PORT latches
	//00 = UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLKx pins controlled by PORT latches
	U2MODEbits.UEN1=0, U2MODEbits.UEN0=0;
	//bit 7 WAKE: Wake-up on Start Bit Detect During Sleep Mode Enable bit
	//1 = UARTx will continue to sample the UxRX pin; interrupt generated on falling edge, bit cleared in hardware on following rising edge
	//0 = No wake-up enabled
	U2MODEbits.WAKE = 0;
	//bit 6 LPBACK: UARTx Loopback Mode Select bit
	//1 = Enable Loopback mode
	//0 = Loopback mode is disabled
	U2MODEbits.LPBACK = 0;
	//bit 5 ABAUD: Auto-Baud Enable bit
	//1 = Enable baud rate measurement on the next character - requires reception of a Sync field (55h); cleared in hardware upon completion
	//0 = Baud rate measurement disabled or completed
	U2MODEbits.ABAUD = 0;
	//bit 4 RXINV: Receive Polarity Inversion bit
	//1 = UxRX Idle state is `0'
	//0 = UxRX Idle state is `1'
	U2MODEbits.URXINV = 0;
	//bit 3 BRGH: High Baud Rate Enable bit
	//1 = BRG generates 4 clocks per bit period (4x baud clock, High-Speed mode)
	//0 = BRG generates 16 clocks per bit period (16x baud clock, Standard mode)
	U2MODEbits.BRGH = (F_UART / 4 / baud_rate < 0xffff)?1:0;		//U2MODEbits.BRGH = 1;
	//bit 2-1 PDSEL1:PDSEL0: Parity and Data Selection bits
	//11 = 9-bit data, no parity
	//10 = 8-bit data, odd parity
	//01 = 8-bit data, even parity
	//00 = 8-bit data, no parity
	U2MODEbits.PDSEL1=0, U2MODEbits.PDSEL0=0;
	//bit 0 STSEL: Stop Bit Selection bit
	//1 = Two Stop bits
	//0 = One Stop bit
	U2MODEbits.STSEL=0;
	//Note 1: If UARTEN = 1, the peripheral inputs and outputs must be configured to an available RPn pin. See
	//Section 10.4 "Peripheral Pin Select" for more information.
	//2: This feature is only available for the 16x BRG mode (BRGH = 0).
	//3: Bit availability depends on pin availability.


#if 1	//BAUDCON - with rounding up
	if (U2MODEbits.BRGH==1) U2BRG = (F_UART + 2 * baud_rate) / 4 / baud_rate - 1;				//set lower byte of brg
	else U2BRG = (F_UART + 8 * baud_rate) / 16 / baud_rate - 1;
#else	//BAUDCON - without rounding up
	if (U2MODEbits.BRGH==1) U2BRG = (F_UART + 0*2 * baud_rate) / 4 / baud_rate - 1;				//set lower byte of brg
	else U2BRG = (F_UART + 0*8 * baud_rate) / 16 / baud_rate - 1;
#endif	//1

	//disable interrupts
//#if defined(UxTX2RP)
	IFS1bits.U2TXIF = 0;						//clera the flag
	IEC1bits.U2TXIE = 0;						//disable the interrupt
	IPC7bits.U2TXIP = UxIP_DEFAULT;				//set interrupt priority
	
	//set up status register
	//bit 15,13 UTXISEL1:UTXISEL0: Transmission Interrupt Mode Selection bits
	//11 = Reserved; do not use
	//10 = Interrupt when a character is transferred to the Transmit Shift Register (TSR) and as a result, the transmit buffer becomes empty
	//01 = Interrupt when the last character is shifted out of the Transmit Shift Register; all transmit operations are completed
	//00 = Interrupt when a character is transferred to the Transmit Shift Register (this implies there is at least one character open in the transmit buffer)
	U2STAbits.UTXISEL1=0, U2STAbits.UTXISEL0=0;
//#endif
	//bit 14 UTXINV: IrDAr Encoder Transmit Polarity Inversion bit
	//If IREN = 0:
	//1 = UxTX Idle `0'
	//0 = UxTX Idle `1'
	//If IREN = 1:
	//1 = UxTX Idle `1'
	//0 = UxTX Idle `0'
	U2STAbits.UTXINV=0;
	//bit 12 Unimplemented: Read as `0'
	//bit 11 UTXBRK: Transmit Break bit
	//1 = Send Sync Break on next transmission - Start bit, followed by twelve `0' bits, followed by Stop bit;	cleared by hardware upon completion
	//0 = Sync Break transmission disabled or completed
	U2STAbits.UTXBRK=0;
	//bit 10 UTXEN: Transmit Enable bit(1)
	//1 = Transmit enabled, UxTX pin controlled by UARTx
	//0 = Transmit disabled, any pending transmission is aborted and buffer is reset. UxTX pin controlled by the PORT register.
#if defined(U2TX2RP)
	U2STAbits.UTXEN=1;
#else
	U2STAbits.UTXEN=0;
#endif
#if defined(U2RX2RP)
	//U2STAbits.URXEN=1;
#else
	//U2STAbits.URXEN=0;
#endif
	//bit 9 UTXBF: Transmit Buffer Full Status bit (read-only)
	//1 = Transmit buffer is full
	//0 = Transmit buffer is not full, at least one more character can be written
	//bit 8 TRMT: Transmit Shift Register Empty bit (read-only)
	//1 = Transmit Shift Register is empty and transmit buffer is empty (the last transmission has completed)
	//0 = Transmit Shift Register is not empty, a transmission is in progress or queued
//#if defined(UxRX2RP)
	IFS1bits.U2RXIF = 0;						//clear the flag
	IEC1bits.U2RXIE = 0;						//disable the interrupt

	//bit 7-6 URXISEL1:URXISEL0: Receive Interrupt Mode Selection bits
	//11 = Interrupt is set on RSR transfer, making the receive buffer full (i.e., has 4 data characters)
	//10 = Interrupt is set on RSR transfer, making the receive buffer 3/4 full (i.e., has 3 data characters)
	//0x = Interrupt is set when any character is received and transferred from the RSR to the receive buffer. Receive buffer has one or more characters.
	U2STAbits.URXISEL1 = 0, U2STAbits.URXISEL0 = 0;
//#endif
	//bit 5 ADDEN: Address Character Detect bit (bit 8 of received data = 1)
	//1 = Address Detect mode enabled. If 9-bit mode is not selected, this does not take effect.
	//0 = Address Detect mode disabled
	//bit 4 RIDLE: Receiver Idle bit (read-only)
	//1 = Receiver is Idle
	//0 = Receiver is active
	//bit 3 PERR: Parity Error Status bit (read-only)
	//1 = Parity error has been detected for the current character (character at the top of the receive FIFO)
	//0 = Parity error has not been detected
	//bit 2 FERR: Framing Error Status bit (read-only)
	//1 = Framing error has been detected for the current character (character at the top of the receive FIFO)
	//0 = Framing error has not been detected
	//bit 1 OERR: Receive Buffer Overrun Error Status bit (clear/read-only)
	//1 = Receive buffer has overflowed
	//0 = Receive buffer has not overflowed (clearing a previously set OERR bit (1 ? 0 transition) will reset the receiver buffer and the RSR to the empty state)
	//bit 0 URXDA: Receive Buffer Data Available bit (read-only)
	//1 = Receive buffer has data; at least one more character can be read
	//0 = Receive buffer is empty
	//REGISTER 17-2: U1STAbits: UARTx STATUS AND CONTROL REGISTER (CONTINUED)
	//Note 1: If UARTEN = 1, the peripheral inputs and outputs must be configured to an available RPn pin. See
	//Section 10.4 "Peripheral Pin Select" for more information.

}

void uart2Puts(char *str) {
#ifndef UART2_TXISR
	//polling implementation
	while(*str) {
		uart2Putch(*str++);	//send the ch and advance the pointer
	}
#else
	//interrupt transmission
	if (*str) {								//if the string isn't empty to begin with
		uart2Wait();						//for compatability with polling transmission
		//U1TXREG = 0;						//clear the buffer
		//_U1TX_BUSY  = 1;					//transmission in progress
		_U2TX_ptr=str;						//point to the string to be transmitted
		//U1STAbits.UTXEN = 1;				//enable transmission - always enabled
		//tmp = *_UxTX_ptr++;	UxTXREG = tmp;						//*_UxTX_ptr++;				//load up the 1st char. advance to the next char
		IFS1bits.U2TXIF = 0;				//UxTXIF = 0;							//clear the flag
		IEC1bits.U2TXIE = 1;				//UxTXIE = 1;							//enable the interrupt
		U2TXREG = *_U2TX_ptr++;
	}
#endif	//UART2_TXISR
}

/*

Writes a line of text to USART and goes to new line
The new line is Windows style CR/LF pair.

This will work on Hyper Terminal Only NOT on Linux

*/

//void uart2Putline(char *ln) {
//	//USARTWriteString(ln);
//	uart2Puts(ln);
//	//USARTWriteString("\r\n");
//	uart2Puts((char *)"\r\n");
//}

//uint8_t uart2Getch(void) {
//	//while(!RCIF); RCIF=0;		//Wait for a byte
//   //USART_WAIT(U1STAbits.TRMT);		//wait for the prior transmission to end
//
//	return U2RXREG;		//return it
//}

//test if data rx is available
//uint16_t uart2Available(void) {
//	return U2STAbits.URXDA;
//}

//test if uart tx is busy
//uint16_t uart2Busy(void) {
//	return U2STAbits.UTXBF;
//}

//print to uart2
void u2Print(char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];						//transmission buffer, 40-1 char max
	uint8_t i=20;
	char negFlag=0;

	uart2Wait();						//wait for complete
	
	strcpy(uRAM, str);					//copy to uarm
	if (dat < 0) {
		negFlag=1;						//uRAM[i-13]='-';
		dat = -dat;
	}
	
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[i--]='0'+(dat % 10); dat /= 10;
	uRAM[i--]=(negFlag)?'-':' ';
	
	uart2Puts(uRAM);	//send a message on uart1
}

//end Serial

//tmr1
//global variables
static void (* _tmr1_isrptr)(void)=empty_handler;				//tmr1_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _T1Interrupt(void) {
	IFS0bits.T1IF=0;							//clear tmr1 interrupt flag
#if defined(SYSTICK_TMR1)
	systick_ovf+=0x10000ul;							//increment overflow count: 16-bit timer
#else	//systick on tmr2
	//do nothing
#endif
	_tmr1_isrptr();								//execute user tmr1 isr
}

//initialize the timer1 (16bit)
void tmr1Init(uint8_t ps, uint16_t period) {
	_tmr1_isrptr=empty_handler;					//point to default handler

	PMD1bits.T1MD = 0;							//enable power to tmr
	T1CONbits.TON = 0;							//turn off rtc1
	T1CONbits.TCS = 0;							//use internal clock = Fosc
	//T1CONbits.T32 = 0;						//0->16 bit timer, 1->32bit timer
	T1CONbits.TCKPS=ps & TMR_PSMASK;			//set prescaler to 1:1
	T1CONbits.TGATE = 0;						//rtc1 gate disabled
	TMR1 = 0;									//reset the timer/counter
	PR1=period-0;								//minimum rtc resolution is 1ms
	IFS0bits.T1IF = 0;							//reset the flag
	IEC0bits.T1IE = 0;							//rtc1 interrupt off
	T1CONbits.TON = 1;							//turn on rtc1

	//configure timer input pin (TxCK)
#if defined(T1CK2RP)
	T1CK2RP();									//configure the input pin
	T1CONbits.TCS = 1;							//1->use TxCK as input, 0->se internal clock = Fosc
#endif
}

//activate the isr handler
void tmr1AttachISR(void (*isrptr)(void)) {
	_tmr1_isrptr=isrptr;						//activate the isr handler
	IPC0bits.T1IP = TxIP_DEFAULT;
	//IPC0bits.T1IS = TxIS_DEFAULT;
	IFS0bits.T1IF = 0;							//reset the flag
	IEC0bits.T1IE = 1;							//rtc1 interrupt on
}

//tmr2
//global variables
static void (* _tmr2_isrptr)(void)=empty_handler;				//tmr1_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _T2Interrupt(void) {
	IFS0bits.T2IF=0;							//clear tmr1 interrupt flag
#if defined(SYSTICK_TMR1)
	//do nothing
#else	//systick on tmr2
	systick_ovf+=0x10000ul;							//increment overflow count: 16-bit timer
#endif
	_tmr2_isrptr();								//execute user tmr2 isr
}

//initialize the timer2 (16bit)
void tmr2Init(uint8_t ps, uint16_t period) {
	_tmr2_isrptr=empty_handler;					//point to default handler

	PMD1bits.T2MD = 0;							//enable power to tmr
	T2CONbits.TON = 0;							//turn off rtc1
	T2CONbits.TCS = 0;							//use internal clock = Fosc
	T2CONbits.T32 = 0;							//0->16 bit timer, 1->32bit timer
	T2CONbits.TCKPS=ps & TMR_PSMASK;			//set prescaler to 1:1
	T2CONbits.TGATE = 0;						//rtc1 gate disabled
	TMR2 = 0;									//reset the timer/counter
	PR2=period-0;								//minimum rtc resolution is 1ms
	IFS0bits.T2IF = 0;							//reset the flag
	IEC0bits.T2IE = 0;							//rtc1 interrupt off
	T2CONbits.TON = 1;							//turn on rtc1

	//configure timer input pin (TxCK)
#if defined(T2CK2RP)
	T2CK2RP();									//configure the input pin
	T2CONbits.TCS = 1;							//1->use TxCK as input, 0->se internal clock = Fosc
#endif
}

//activate the isr handler
void tmr2AttachISR(void (*isrptr)(void)) {
	_tmr2_isrptr=isrptr;						//activate the isr handler
	IPC1bits.T2IP = TxIP_DEFAULT;
	//IPC2bits.T2IS = TxIS_DEFAULT;
	IFS0bits.T2IF = 0;							//reset the flag
	IEC0bits.T2IE = 1;							//rtc1 interrupt on
}

//tmr3
//global variables
static void (* _tmr3_isrptr)(void)=empty_handler;				//tmr1_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _T3Interrupt(void) {
	IFS0bits.T3IF=0;							//clear tmr1 interrupt flag
	_tmr3_isrptr();								//execute user tmr1 isr
}

//initialize the timer3 (16bit)
void tmr3Init(uint8_t ps, uint16_t period) {
	_tmr3_isrptr=empty_handler;					//point to default handler

	PMD1bits.T3MD = 0;							//enable power to tmr
	T3CONbits.TON = 0;							//turn off rtc1
	T3CONbits.TCS = 0;							//use internal clock = Fosc
	//T3CONbits.T32 = 0;						//0->16 bit timer, 1->32bit timer
	T3CONbits.TCKPS=ps & TMR_PSMASK;			//set prescaler to 1:1
	T3CONbits.TGATE = 0;						//rtc1 gate disabled
	TMR3 = 0;									//reset the timer/counter
	PR3=period-0;								//minimum rtc resolution is 1ms
	IFS0bits.T3IF = 0;							//reset the flag
	IEC0bits.T3IE = 0;							//rtc1 interrupt off
	T3CONbits.TON = 1;							//turn on rtc1

	//configure timer input pin (TxCK)
#if defined(T3CK2RP)
	T3CK2RP();									//configure the input pin
	T3CONbits.TCS = 1;							//1->use TxCK as input, 0->se internal clock = Fosc
#endif
}

//activate the isr handler
void tmr3AttachISR(void (*isrptr)(void)) {
	_tmr3_isrptr=isrptr;						//activate the isr handler
	IPC2bits.T3IP = TxIP_DEFAULT;
	//IPC2bits.T3IS = TxIS_DEFAULT;
	IFS0bits.T3IF = 0;							//reset the flag
	IEC0bits.T3IE = 1;							//rtc1 interrupt on
}

//tmr4
//global variables
static void (* _tmr4_isrptr)(void)=empty_handler;				//tmr1_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _T4Interrupt(void) {
	IFS1bits.T4IF=0;							//clear tmr1 interrupt flag
	_tmr4_isrptr();								//execute user tmr1 isr
}

//initialize the timer4 (16bit)
void tmr4Init(uint8_t ps, uint16_t period) {
	_tmr4_isrptr=empty_handler;					//point to default handler

	PMD1bits.T4MD = 0;							//enable power to tmr
	T4CONbits.TON = 0;							//turn off rtc1
	T4CONbits.TCS = 0;							//use internal clock = Fosc
	T4CONbits.T32 = 0;							//0->16 bit timer, 1->32bit timer
	T4CONbits.TCKPS=ps & TMR_PSMASK;			//set prescaler to 1:1
	T4CONbits.TGATE = 0;						//rtc1 gate disabled
	TMR4 = 0;									//reset the timer/counter
	PR4=period-0;								//minimum rtc resolution is 1ms
	IFS1bits.T4IF = 0;							//reset the flag
	IEC1bits.T4IE = 0;							//rtc1 interrupt off
	T4CONbits.TON = 1;							//turn on rtc1


	//configure timer input pin (TxCK)
#if defined(T4CK2RP)
	T4CK2RP();									//configure the input pin
	T4CONbits.TCS = 1;							//1->use TxCK as input, 0->se internal clock = Fosc
#endif
}

//activate the isr handler
void tmr4AttachISR(void (*isrptr)(void)) {
	_tmr4_isrptr=isrptr;						//activate the isr handler
	IPC6bits.T4IP = TxIP_DEFAULT;
	//IPC6bits.T4IS = TxIS_DEFAULT;
	IFS1bits.T4IF = 0;							//reset the flag
	IEC1bits.T4IE = 1;							//rtc1 interrupt on
}

//tmr5
//global variables
static void (* _tmr5_isrptr)(void)=empty_handler;				//tmr1_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _T5Interrupt(void) {
	IFS1bits.T5IF=0;							//clear tmr1 interrupt flag
	_tmr5_isrptr();								//execute user tmr1 isr
}

//initialize the timer5 (16bit)
void tmr5Init(uint8_t ps, uint16_t period) {
	_tmr5_isrptr=empty_handler;					//point to default handler

	PMD1bits.T5MD = 0;							//enable power to tmr
	T5CONbits.TON = 0;							//turn off rtc1
	T5CONbits.TCS = 0;							//use internal clock = Fosc
	//T5CONbits.T32 = 0;						//0->16 bit timer, 1->32bit timer
	T5CONbits.TCKPS=ps & TMR_PSMASK;			//set prescaler to 1:1
	T5CONbits.TGATE = 0;						//rtc1 gate disabled
	TMR5 = 0;									//reset the timer/counter
	PR5=period-0;								//minimum rtc resolution is 1ms
	IFS1bits.T5IF = 0;							//reset the flag
	IEC1bits.T5IE = 0;							//rtc1 interrupt off
	T5CONbits.TON = 1;							//turn on rtc1

	//configure timer input pin (TxCK)
#if defined(T5CK2RP)
	T5CK2RP();									//configure the input pin
	T5CONbits.TCS = 1;							//1->use TxCK as input, 0->se internal clock = Fosc
#endif
}

//activate the isr handler
void tmr5AttachISR(void (*isrptr)(void)) {
	_tmr5_isrptr=isrptr;						//activate the isr handler
	IPC7bits.T5IP = TxIP_DEFAULT;
	//IPC7bits.T5IS = TxIS_DEFAULT;
	IFS1bits.T5IF = 0;							//reset the flag
	IEC1bits.T5IE = 1;							//rtc1 interrupt on
}

//set up 32-bit timer
//using tmr2 (lsw) and tmr3 (msw)
void tmr23Init(uint8_t ps, uint32_t period) {
	tmr2Init(ps, period);
	tmr3Init(ps, period >>16);
	T2CONbits.T32=1;					//1->as 32-bit timer

	TMR2 = TMR3 = 0;					//reset the timer
}

//read 32-bit timer2 and 3
uint32_t tmr23Get(void) {
	uint16_t tmp2, tmp3;

	do {
		tmp3 = TMR3;					//read the mws
		tmp2 = TMR2;					//read the lsw
	} while (tmp3 ^ TMR2);

	return ((uint32_t) tmp3 << 16) | tmp2;
}

//#define tmr23AttachISR(isr)				tmr3AttachISR(isr)

//using tmr4 (lsw) and tmr5 (msw)
void tmr45Init(uint8_t ps, uint32_t period) {
	tmr4Init(ps, period);
	tmr5Init(ps, period>>16);
	T4CONbits.T32=1;					//1->as 32-bit timer

	TMR4 = TMR5 = 0;					//reset the timer
}

//read 32-bit timer4 and 5
uint32_t tmr45Get(void) {
	uint16_t tmp4, tmp5;

	do {
		tmp5 = TMR5;					//read the mws
		tmp4 = TMR4;					//read the lsw
	} while (tmp5 ^ TMR5);

	return ((uint32_t) tmp5 << 16) | tmp5;
}

//#define tmr45AttachISR(isr)				tmr5AttachISR(isr)
//end Timer

//define pwm functions
//time base is Timer2, defined in mcu_init()

//pwm1/oc1
//reset pwm
void pwm1Init(void) {
	//power up the pwm module
	PMD2bits.OC1MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

#if defined(PWM12RP)
	//assign the output pins
	PWM12RP();
#endif

	//tris pin presumed to have been set to output


	//for GA002 chips
	//OCxCON1 register settings
	//reset the registers
	OC1CON = 0x0000;
	//OC1CON2 = 0x0000;
	OC1CONbits.OCM = 0x06;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled
	OC1CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	OC1R = OC1RS = 0;						//reset the duty cycle registers
	//OC1CON1bits.ON= 1;					//1->turn on oc, 0->turn off oc

}

//pwm2/oc2
//reset pwm
void pwm2Init(void) {
	//power up the pwm module
	PMD2bits.OC2MD = 0;						//0->turn on the peripheral, 1->turn off the peripheral

#if defined(PWM22RP)
	//assign the output pins
	PWM22RP();
#endif

	//tris pin presumed to have been set to output


	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC2CON = 0x0000;
	//OC2CON2 = 0x0000;
	OC2CONbits.OCM = 0x06;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled
	OC2CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	OC2R = OC2RS = 0;						//reset the duty cycle registers
	//OC2CON1bits.ON= 1;					//1->turn on oc, 0->turn off oc

}


//pwm3/oc3
//reset pwm
void pwm3Init(void) {
	//power up the pwm module
	PMD2bits.OC3MD = 0;						//0->turn on the peripheral, 1->turn off the peripheral

#if defined(PWM32RP)
	//assign the output pins
	PWM32RP();
#endif

	//tris pin presumed to have been set to output


	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC3CON = 0x0000;
	//OC3CON2 = 0x0000;
	OC3CONbits.OCM = 0x06;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled
	OC3CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	OC3R = OC3RS = 0;						//reset the duty cycle registers
	//OC3CON1bits.ON= 1;					//1->turn on oc, 0->turn off oc

}


//pwm4/oc4
//reset pwm
void pwm4Init(void) {
	//power up the pwm module
	PMD2bits.OC4MD = 0;						//0->turn on the peripheral, 1->turn off the peripheral

#if defined(PWM42RP)
	//assign the output pins
	PWM42RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC4CON = 0x0000;
	//OC4CON2 = 0x0000;
	OC4CONbits.OCM = 0x06;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled
	OC4CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	OC4R = OC4RS = 0;						//reset the duty cycle registers
	//OC4CON1bits.ON= 1;					//1->turn on oc, 0->turn off oc

}


//end pwm/oc

//adc module
//rest the adc
//automatic sampling (ASAM=1), manual conversion
//12-bit adc, ch0 only
void adcInit(void) {
	PMD1bits.AD1MD = 0;					//enable power to adc

	//reset the adc control registers
	AD1CON1 = 0;							//AVdd-AVss
	AD1CON1bits.AD12B=1;					//1->12bit adc, 0->10bit adc
	AD1CON2 = 0;							//convert ch0 only
	AD1CON3 = 0;
	AD1CON4 = 0;							//allocate 1 word for each channel
	AD1CHS123=0;							//default value
	AD1CHS0 = 0;							//negative input is Vref-(=AVss)

	AD1CON1bits.SSRC = 7;					//0->samp ends sampling and starts conversion. 7=auto mode
	AD1CON2bits.VCFG = 0;					//0..7. adc reference: 0->AVdd-AVss,1=Vref+-AVss, 2=AVdd-Vref-, 3=Vref+ - Vref-
	AD1CON2bits.ALTS = 0;					//0->always use mux A inputs. 1->alternate between mux a and mux b inputs
	AD1CON3bits.SAMC = 31;					//0..31. 0->0 Tad (not recommended), 1->1 Tad, ... 31->32 Tad
	AD1CON3bits.ADCS = 2;					//0..63. 0->Tcy, 1-> 1Tcy, 63->64 Tcy
	AD1CSSL = 0;							//scanning disabled
	//AD1PCFGLbits.PCFG15=1;					//1->band gap enabled, 0->band gap disabled
	AD1CON1bits.ADON = 1;					//1->enable adc
}

//read the adc
//single ended only.
uint16_t analogRead(uint16_t adc_ch) {
	uint16_t tmp;

	tmp=AD1PCFGL;							//save current port configuration setting
	//set up adc port configuration bits
	//for the positive channel (mux a only)
	AD1CON1bits.DONE=0;
	adc_ch = adc_ch & 0x0f;					//adc_ch limited to 16 channels
#if 1
	AD1PCFGL &=~(1<<adc_ch);				//configure the port to be analog
	AD1CHS0 = adc_ch;						//select the channel
#else
	switch (adc_ch) {
	case ADC_AN0:
		AD1PCFGL &=~(1<< 0); AD1CHS0=0x0000; break;		//AD1PCFG: 1->digital GPIO, 0->analog input
	case ADC_AN1:
		AD1PCFGL &=~(1<< 1); AD1CHS0=0x0001; break;
	case ADC_AN2:
		AD1PCFGL &=~(1<< 2); AD1CHS0=0x0002; break;
	case ADC_AN3:
		AD1PCFGL &=~(1<< 3); AD1CHS0=0x0003; break;
	case ADC_AN4:
		AD1PCFGL &=~(1<< 4); AD1CHS0=0x0004; break;
	case ADC_AN5:
		AD1PCFGL &=~(1<< 5); AD1CHS0=0x0005; break;
	case ADC_AN6:
		AD1PCFGL &=~(1<< 6); AD1CHS0=0x0006; break;									//not implemented on 28pin devices. do not use
	case ADC_AN7:
		AD1PCFGL &=~(1<< 7); AD1CHS0=0x0007; break;									//not implemented on 28pin devices. do not use
	case ADC_AN8:
		AD1PCFGL &=~(1<< 8); AD1CHS0=0x0008; break;									//not implemented on 28pin devices. do not use
	case ADC_AN9:
		AD1PCFGL &=~(1<< 9); AD1CHS0=0x0008; break;
	case ADC_AN10:
		AD1PCFGL &=~(1<<10); AD1CHS0=0x000a; break;
	case ADC_AN11:
		AD1PCFGL &=~(1<<11); AD1CHS0=0x000b; break;
	case ADC_AN12:
		AD1PCFGL &=~(1<<12); AD1CHS0=0x000c; break;									//not impelemned on 28pin devices. do not use
	case ADC_VCORE:
		AD1PCFGL &=~(1<<13); AD1CHS0=0x000d; break;
	case ADC_VBG_2:
		AD1PCFGL &=~(1<<14); AD1CHS0=0x000e; break;
	case ADC_VBG:
		AD1PCFGL &=~(1<<15); AD1CHS0=0x000f; break;									//AD1PCFG = ~adc_ch;					//mux b: not used; mux a: negative is Vref-,
	//case ADC_CTMU:		AD1PCFG &=~(0<<0); AD1CHS = 0x001f; break;						//reserved for ctmu
	default:
		break;									//do nothing
	}
#endif
	AD1CON1bits.SAMP = 1;						//start the adc
	//insert delays if necessary
	delayTks(ADC_DLY);
	while (!AD1CON1bits.DONE) continue;		//wait for adc to finish
	AD1PCFGL = tmp;								//restore ad1pcfg setting
	return ADC1BUF0;							//return the adc results
}
//end ADC

//arand
//analog random
//generate 4-bit random numbers
uint8_t arand4(void) {
	uint8_t tmp=arand1();
	
	//tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	tmp = (tmp << 1) | arand1();
	return tmp;
}

//generate 8-bit random number
uint8_t arand8(void) {
	return (arand4() << 4) | arand4();
}

//return 16-bit random number
uint16_t arand16(void) {
	return ((uint16_t) arand8() << 8) | arand8();
}


//return 32-bit random number
uint32_t arand32(void) {
	return ((uint32_t) arand16() << 16) | arand16();
}
//end analog random
//end arand

//output compare
//oc1
uint16_t _oc1pr=0xffff;							//oc isr period
void (*_oc1_isrptr)(void)=empty_handler;		//tmr1_ptr pointing to empty_handler by default
//OC ISR
void _ISR_PSV _OC1Interrupt(void) {				//for PIC24
	//clear the flag
	IFS0bits.OC1IF = 0;							//clear the flag
	OC1R += _oc1pr;								//update to the next match point
	_oc1_isrptr();								//run user handler
}

void oc1Init(uint16_t pr) {
	_oc1_isrptr=empty_handler;
	_oc1pr = pr;								//reset the pr

	//power up the pwm module
	PMD2bits.OC1MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

	//assign the output pins
#if defined(PWM12RP)
	PWM12RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC1CON = 0x0000;
	//OC5CON2 = 0x0000;
	OC1CONbits.OCM = 0x03;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled, 0b011->compare toggles ocx
	OC1CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	//OC1R = OC1RS = TMR2 + _oc1pr;			//reset the duty cycle registers

	IFS0bits.OC1IF = 0;						//0->clear the flag;
	IEC0bits.OC1IE = 0;						//0->disable the interrupt, 1->enable the interrupt
	IPC0bits.OC1IP = OCxIP_DEFAULT;			//clear the interrupt priority

	//OC1CON1bits.ON= 1;						//1->turn on oc, 0->turn off oc

}

//activate user isr
void oc1AttachISR(void (*isrptr)(void)) {
	_oc1_isrptr=isrptr;						//activate the isr handler
	OC1R = TMR2 + _oc1pr;					//update to the next match point
	IFS0bits.OC1IF = 0;						//0->clear the flag;
	IEC0bits.OC1IE = 1;						//0->disable the interrupt, 1->enable the interrupt
}

//oc2
uint16_t _oc2pr=0xffff;							//oc isr period
void (*_oc2_isrptr)(void)=empty_handler;		//tmr1_ptr pointing to empty_handler by default
//OC ISR
void _ISR_PSV _OC2Interrupt(void) {				//for PIC24
	//clear the flag
	IFS0bits.OC2IF = 0;							//clear the flag
	OC2R += _oc2pr;								//update to the next match point
	_oc2_isrptr();								//run user handler
}

void oc2Init(uint16_t pr) {
	_oc2_isrptr=empty_handler;
	_oc2pr = pr;								//reset the pr

	//power up the pwm module
	PMD2bits.OC2MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

	//assign the output pins
#if defined(PWM22RP)
	PWM22RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC2CON = 0x0000;
	//OC5CON2 = 0x0000;
	OC2CONbits.OCM = 0x03;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled, 0b011->compare toggles ocx
	OC2CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	//OC2R = OC2RS = TMR2 + _oc2pr;			//reset the duty cycle registers

	IFS0bits.OC2IF = 0;						//0->clear the flag;
	IEC0bits.OC2IE = 0;						//0->disable the interrupt, 1->enable the interrupt
	IPC1bits.OC2IP = OCxIP_DEFAULT;			//clear the interrupt priority

	//OC2CON1bits.ON= 1;						//1->turn on oc, 0->turn off oc

}

//activate user isr
void oc2AttachISR(void (*isrptr)(void)) {
	_oc2_isrptr=isrptr;						//activate the isr handler
	OC2R = TMR2 + _oc2pr;					//update to the next match point
	IFS0bits.OC2IF = 0;						//0->clear the flag;
	IEC0bits.OC2IE = 1;						//0->disable the interrupt, 1->enable the interrupt
}

//oc3
uint16_t _oc3pr=0xffff;							//oc isr period
void (*_oc3_isrptr)(void)=empty_handler;		//tmr1_ptr pointing to empty_handler by default
//OC ISR
void _ISR_PSV _OC3Interrupt(void) {				//for PIC24
	//clear the flag
	IFS1bits.OC3IF = 0;							//clear the flag
	OC3R += _oc3pr;								//update to the next match point
	_oc3_isrptr();								//run user handler
}

void oc3Init(uint16_t pr) {
	_oc3_isrptr=empty_handler;
	_oc3pr = pr;								//reset the pr

	//power up the pwm module
	PMD2bits.OC3MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

	//assign the output pins
#if defined(PWM32RP)
	PWM32RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC3CON = 0x0000;
	//OC3CON2 = 0x0000;
	OC3CONbits.OCM = 0x03;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled, 0b011->compare toggles ocx
	OC3CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	//OC3R = OC3RS = TMR2 + _oc3pr;			//reset the duty cycle registers

	IFS1bits.OC3IF = 0;						//0->clear the flag;
	IEC1bits.OC3IE = 0;						//0->disable the interrupt, 1->enable the interrupt
	IPC6bits.OC3IP = OCxIP_DEFAULT;			//clear the interrupt priority

	//OC3CON1bits.ON= 1;						//1->turn on oc, 0->turn off oc

}

//activate user isr
void oc3AttachISR(void (*isrptr)(void)) {
	_oc3_isrptr=isrptr;						//activate the isr handler
	OC3R = TMR2 + _oc3pr;					//update to the next match point
	IFS1bits.OC3IF = 0;						//0->clear the flag;
	IEC1bits.OC3IE = 1;						//0->disable the interrupt, 1->enable the interrupt
}

//oc4
uint16_t _oc4pr=0xffff;							//oc isr period
void (*_oc4_isrptr)(void)=empty_handler;		//tmr1_ptr pointing to empty_handler by default
//OC ISR
void _ISR_PSV _OC4Interrupt(void) {				//for PIC24
	//clear the flag
	IFS1bits.OC4IF = 0;							//clear the flag
	OC4R += _oc4pr;								//update to the next match point
	_oc4_isrptr();								//run user handler
}

void oc4Init(uint16_t pr) {
	_oc4_isrptr=empty_handler;
	_oc4pr = pr;								//reset the pr

	//power up the pwm module
	PMD2bits.OC4MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

	//assign the output pins
#if defined(PWM42RP)
	PWM42RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC4CON = 0x0000;
	//OC4CON2 = 0x0000;
	OC4CONbits.OCM = 0x03;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled, 0b011->compare toggles ocx
	OC4CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	//OC4R = OC4RS = TMR2 + _oc4pr;			//reset the duty cycle registers

	IFS1bits.OC4IF = 0;						//0->clear the flag;
	IEC1bits.OC4IE = 0;						//0->disable the interrupt, 1->enable the interrupt
	IPC6bits.OC4IP = OCxIP_DEFAULT;			//clear the interrupt priority

	//OC1CON1bits.ON= 1;						//1->turn on oc, 0->turn off oc

}

//activate user isr
void oc4AttachISR(void (*isrptr)(void)) {
	_oc4_isrptr=isrptr;						//activate the isr handler
	OC4R = TMR2 + _oc4pr;					//update to the next match point
	IFS1bits.OC4IF = 0;						//0->clear the flag;
	IEC1bits.OC4IE = 1;						//0->disable the interrupt, 1->enable the interrupt
}

//oc5
#if 0	//not avalable on target chip
uint16_t _oc5pr=0xffff;							//oc isr period
void (*_oc5_isrptr)(void)=empty_handler;		//tmr1_ptr pointing to empty_handler by default
//OC ISR
void _ISR_PSV _OC5Interrupt(void) {				//for PIC24
	//clear the flag
	IFS1bits.OC5IF = 0;							//clear the flag
	OC5R += _oc5pr;								//update to the next match point
	_oc5_isrptr();								//run user handler
}

void oc5Init(uint16_t pr) {
	_oc5_isrptr=empty_handler;
	_oc5pr = pr;								//reset the pr

	//power up the pwm module
	PMD2bits.OC5MD = 0;							//0->turn on the peripheral, 1->turn off the peripheral

	//assign the output pins
#if defined(PWM52RP)
	PWM52RP();
#endif

	//tris pin presumed to have been set to output

	//for ga002 devices
	//OCxCON1 register settings
	//reset the registers
	OC5CON = 0x0000;
	//OC5CON2 = 0x0000;
	OC5CONbits.OCM = 0x03;					//0b110 -> pwm on OCx, fault pin disabled, 0b111->pwm on OCx, fault pin enabled, 0b011->compare toggles ocx
	OC5CONbits.OCTSEL = 0;					//0->timebase = timer2, 1->timebase = timer3
	//OC5R = OC5RS = TMR2 + _oc5pr;			//reset the duty cycle registers

	IFS1bits.OC5IF = 0;						//0->clear the flag;
	IEC1bits.OC5IE = 0;						//0->disable the interrupt, 1->enable the interrupt
	IPC6bits.OC5IP = OCxIP_DEFAULT;			//clear the interrupt priority

	//OC1CON1bits.ON= 1;						//1->turn on oc, 0->turn off oc

}

//activate user isr
void oc5AttachISR(void (*isrptr)(void)) {
	_oc5_isrptr=isrptr;						//activate the isr handler
	OC5R = TMR2 + _oc5pr;					//update to the next match point
	IFS1bits.OC5IF = 0;						//0->clear the flag;
	IEC1bits.OC5IE = 1;						//0->disable the interrupt, 1->enable the interrupt
}
#endif	//0
//end output compare

//input capture
static void (*_ic1_isrptr)(void)=empty_handler;	//function pointer pointing to empty_handler by default
//volatile uint16_t IC1DAT=0;						//buffer

//input capture ISR
void _ISR_PSV _IC1Interrupt(void) {				//for PIC24
	//clear the flag
	//IC1DAT = IC1BUF;							//read the captured value
	IFS0bits.IC1IF = 0;							//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic1_isrptr();								//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic1Init(void) {
	_ic1_isrptr = empty_handler;				//reset user handler

	PMD2bits.IC1MD = 0;							//0->enable power to input capture

	//for ga002 devices
	IC1CON = 0;							//reset to default value
	//IC1CON2 = 0;						//reset to default value
	IC1CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC1CONbits.ICBNE) IC1BUF;	//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC1CONbits.ON = 1;				//1->enable the module, 0->disable the module

	//input capture running now
#if defined(IC12RP)
	IC12RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic1AttachISR(void (*isrptr)(void)) {
	_ic1_isrptr = isrptr;				//install user ptr
	//IC1BUF;								//read the buffer to clear the flag
	IFS0bits.IC1IF   = 0;				//0->clear the flag
	IEC0bits.IC1IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//ic2
static void (*_ic2_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC2DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC2Interrupt(void) {		//for PIC24
	//clear the flag
	//IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC2IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic2_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic2Init(void) {
	_ic2_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC2MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC2CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC2CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC2CONbits.ICBNE) IC2BUF;	//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC22RP)
	IC22RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic2AttachISR(void (*isrptr)(void)) {
	_ic2_isrptr = isrptr;				//install user ptr
	//IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC2IF   = 0;				//0->clear the flag
	IEC0bits.IC2IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

#if 0	//ic3/4/5 do not exist on target chip
//ic3
static void (*_ic3_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC2DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC3Interrupt(void) {		//for PIC24
	//clear the flag
	//IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC3IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic3_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic3Init(void) {
	_ic3_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC3MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC3CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC3CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC3CONbits.ICBNE) IC3BUF;	//read the buffer to clear the flag
	IFS0bits.IC3IF   = 0;				//0->clear the flag
	IEC0bits.IC3IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC32RP)
	IC32RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic3AttachISR(void (*isrptr)(void)) {
	_ic3_isrptr = isrptr;				//install user ptr
	//IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC3IF   = 0;				//0->clear the flag
	IEC0bits.IC3IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//ic4
static void (*_ic4_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC2DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC4Interrupt(void) {		//for PIC24
	//clear the flag
	//IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC4IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic4_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic4Init(void) {
	_ic4_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC4MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC4CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC4CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC4CONbits.ICBNE) IC4BUF;	//read the buffer to clear the flag
	IFS0bits.IC4IF   = 0;				//0->clear the flag
	IEC0bits.IC4IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC42RP)
	IC42RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic4AttachISR(void (*isrptr)(void)) {
	_ic4_isrptr = isrptr;				//install user ptr
	//IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC4IF   = 0;				//0->clear the flag
	IEC0bits.IC4IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//ic5
static void (*_ic5_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC2DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC5Interrupt(void) {		//for PIC24
	//clear the flag
	//IC2DAT = IC2BUF;					//read the captured value
	IFS0bits.IC5IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic5_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic5Init(void) {
	_ic5_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC5MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC5CON = 0;							//reset to default value
	//IC2CON2 = 0;						//reset to default value
	IC5CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC5CONbits.ICBNE) IC5BUF;	//read the buffer to clear the flag
	IFS0bits.IC5IF   = 0;				//0->clear the flag
	IEC0bits.IC5IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC2CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC52RP)
	IC52RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic5AttachISR(void (*isrptr)(void)) {
	_ic5_isrptr = isrptr;				//install user ptr
	//IC2BUF;								//read the buffer to clear the flag
	IFS0bits.IC5IF   = 0;				//0->clear the flag
	IEC0bits.IC5IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}
#endif	//0

static void (*_ic7_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC7DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC7Interrupt(void) {		//for PIC24
	//clear the flag
	//IC7DAT = IC7BUF;					//read the captured value
	IFS1bits.IC7IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic7_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic7Init(void) {
	_ic7_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC7MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC7CON = 0;							//reset to default value
	//IC7CON2 = 0;						//reset to default value
	IC7CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC7CONbits.ICBNE) IC7BUF;	//read the buffer to clear the flag
	IFS1bits.IC7IF   = 0;				//0->clear the flag
	IEC1bits.IC7IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC7CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC72RP)
	IC72RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic7AttachISR(void (*isrptr)(void)) {
	_ic7_isrptr = isrptr;				//install user ptr
	//IC7BUF;								//read the buffer to clear the flag
	IFS1bits.IC7IF   = 0;				//0->clear the flag
	IEC1bits.IC7IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

static void (*_ic8_isrptr)(void)=empty_handler;				//function pointer pointing to empty_handler by default
//volatile uint16_t IC8DAT=0;				//buffer

//input capture ISR
void _ISR_PSV _IC8Interrupt(void) {		//for PIC24
	//clear the flag
	//IC8DAT = IC8BUF;					//read the captured value
	IFS1bits.IC8IF = 0;					//clear the flag after the buffer has been read (the interrupt flag is persistent)
	_ic8_isrptr();						//run user handler
}

//reset input capture 1
//16-bit mode, rising edge, single capture, Timer2 as timebase
//interrupt disabled
void ic8Init(void) {
	_ic8_isrptr = empty_handler;		//reset user handler

	PMD2bits.IC8MD = 0;					//0->enable power to input capture

	//for ga002 devices
	IC8CON = 0;							//reset to default value
	//IC8CON2 = 0;						//reset to default value
	IC8CON  = 	(0<<15) |				//1->enable the module, 0->disable the module
				(0<<13) |				//0->operates in idle, 1->don't operate in idle
				(1<<9) |				//1-.capture rising edge first (only used for ICM110)
				(0<<8) |				//1->32-bit mode, 0->16-bit mode
				(1<<7) |				//1->timer2 as timebase, 0->timer3 as timebase
				(0<<5) |				//0->interrupt on every capture event, 1->on every second capture event, 2->on every 3rd event, 3->on every 4th event
				(0<<4) |				//0->ICx no overflow, 1->ICx overflow
				(0<<3) |				//0->buffer is empty, 1->buffer is not empty
				(3<<0) |				//0->ICx disabled, 1->every edge, 2->every falling edge, 3->every rising edge, ...
				0x00;

	while (IC8CONbits.ICBNE) IC8BUF;	//read the buffer to clear the flag
	IFS1bits.IC8IF   = 0;				//0->clear the flag
	IEC1bits.IC8IE   = 0;				//1->enable the interrupt, 0->disable the interrupt
	//ICxIP   = 1;						//optional
	//enable the input capture
	//IC8CONbits.ON = 1;				//1->enable the module, 0->disable the module

#if defined(IC82RP)
	IC82RP();							//assign pin to IC
#endif
}

//activate user ptr
void ic8AttachISR(void (*isrptr)(void)) {
	_ic8_isrptr = isrptr;				//install user ptr
	//IC8BUF;								//read the buffer to clear the flag
	IFS1bits.IC8IF   = 0;				//0->clear the flag
	IEC1bits.IC8IE   = 1;				//1->enable the interrupt, 0->disable the interrupt
}

//end input capture

//extint
//extint0
void (* _int0_isrptr) (void)=empty_handler;

void _ISR_PSV _INT0Interrupt(void) {
	IFS0bits.INT0IF = 0;				//clera the flag
	_int0_isrptr();						//run the isr
}

void int0Init(void) {
	//INT02RP();						//map int0_pin - int0 cannot be remapped
	_int0_isrptr = empty_handler;		//initialize int isr ptr
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT0EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int0AttachISR(void (*isrptr) (void)) {
	_int0_isrptr = isrptr;
	IFS0bits.INT0IF = 0;				//clear int0 flag
	IEC0bits.INT0IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint1
void (* _int1_isrptr) (void)=empty_handler;

void _ISR_PSV _INT1Interrupt(void) {
	IFS1bits.INT1IF = 0;				//clera the flag
	_int1_isrptr();						//run the isr
}

void int1Init(void) {
	INT12RP();							//map int1_pin
	_int1_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT1EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int1AttachISR(void (*isrptr) (void)) {
	_int1_isrptr = isrptr;
	IFS1bits.INT1IF = 0;				//clear int0 flag
	IEC1bits.INT1IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}

//extint2
void (* _int2_isrptr) (void)=empty_handler;

void _ISR_PSV _INT2Interrupt(void) {
	IFS1bits.INT2IF = 0;				//clera the flag
	_int2_isrptr();						//run the isr
}

void int2Init(void) {
	INT22RP();							//map int2_pin
	_int2_isrptr = empty_handler;		//initialize int isr ptr
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 0;				//1->enable int0 interrupt, 0->disable the interrupt
	//INTCONbits.INT2EP = 1;			//1=triggered on the falling edge. 0 = rising edge
}

void int2AttachISR(void (*isrptr) (void)) {
	_int2_isrptr = isrptr;
	IFS1bits.INT2IF = 0;				//clear int0 flag
	IEC1bits.INT2IE = 1;				//1->enable int0 interrupt, 0->disable the interrupt
}


//end extint

//spi
//rest spi1
void spi1Init(uint16_t br) {
	PMD1bits.SPI1MD = 0;				//0->enable the module

	//map the pins
#if defined(SCK1RP)
	SCK1RP();
#endif
#if defined(SDO1RP)
	SDO1RP();
#endif
#if defined(SDI1RP)
	SDI1RP();
#endif

	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	SPI1CON1 = SPI1CON2 = 0; 			//reset the spi module
	SPI1CON1bits.MSTEN = 1;				//1->master mode, 0->slave mode
	SPI1CON1bits.PPRE = br;				//set the baudrate generator (primary prescaler)
	SPI1CON1bits.SPRE = 0;				//set the secondary prescaler
	SPI1STATbits.SPIROV=0;				//clear rov flag
	//SPI1BUF;							//perform a read to clear the flag
	//SPI1CON2bits.SPIBEN= 1;				//1->enable enhanced buffer mode, 0->disable enhanced buffer mode
	//need to deal with secondary as well
	SPI1BUF;							//read the buffer to reset the flag
	IFS0bits.SPI1IF = 0;				//0->reset the flag
	IEC0bits.SPI1IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	IPC2bits.SPI1IP = SPIIP_DEFAULT;	//default interrupt priority

	SPI1STATbits.SPIEN = 1;				//1->enable the module, 0->disable the module
}

//rest spi2
void spi2Init(uint16_t br) {
	PMD1bits.SPI2MD = 0;				//0->enable the module

	//map the pins
#if defined(SCK2RP)
	SCK2RP();
#endif
#if defined(SDO2RP)
	SDO2RP();
#endif
#if defined(SDI2RP)
	SDI2RP();
#endif


	//initialize the spi module
	//master mode, PBCLK as clock, 8-bit data, enhanced buffer mode
	SPI2CON1 = SPI2CON2 = 0; 			//reset the spi module
	SPI2CON1bits.MSTEN = 1;				//1->master mode, 0->slave mode
	SPI2CON1bits.PPRE = br;				//set the baudrate generator
	SPI2CON1bits.SPRE = 0;				//set the secondary prescaler
	SPI1STATbits.SPIROV=0;				//clear rov flag
	//SPI1BUF;							//perform a read to clear the flag
	//SPI1CON2bits.SPIBEN= 1;				//1->enable enhanced buffer mode, 0->disable enhanced buffer mode
	//need to deal with 2nd ary as well
	SPI2BUF;							//read the buffer to reset the flag
	IFS2bits.SPI2IF = 0;				//0->reset the flag
	IEC2bits.SPI2IE = 0;				//0->disable the interrupt, 1->enable the interrupt
	IPC8bits.SPI2IP = SPIIP_DEFAULT;	//default interrupt priority

	SPI2STATbits.SPIEN = 1;				//1->enable the module, 0->disable the module
}

//send data via spi
//void spi2Write(uint8_t dat) {
//	while (spi2Busy()) continue;		//tx buffer is full
//	SPI2BUF = dat;						//load the data
//}
//end spi

//i2c
//i2c1
//wait for i2c
#define i2c1Wait()		do {while (I2C1CON & 0x1f); while (I2C1STATbits.TRSTAT);} while (0)		//wait for i2c1

//#define F_I2C1			100000ul	//I2C frequency
//initialize the i2c
void i2c1Init(uint32_t bps) {
	PMD1bits.I2C1MD = 0;				//0->enable the module, 1->disable the module
	I2C1CON = 0;						//reset i2c
	I2C1BRG = F_PHB / 2 / bps - 1 - (F_PHB / 2 / 10000000ul);	//TPGOB = 130ns, minimum of 4
	if (I2C1BRG < 0x04) I2C1BRG = 0x04;	//values 0..3 prohibited
	I2C1CONbits.I2CEN = 1;					//1->turn on the i2c, 0->turn off the i2c
}

//send a start condition
void i2c1Start(void) {
	i2c1Wait();
	I2C1CONbits.SEN = 1;
	while (I2C1CONbits.SEN == 1) continue;
}

//send a stop condition
void i2c1Stop(void) {
	i2c1Wait();
	I2C1CONbits.PEN = 1;
	while (I2C1CONbits.PEN == 1) continue;
}

//send a restart condition
void i2c1Restart(void) {
	i2c1Wait();
	I2C1CONbits.RSEN = 1;
	while (I2C1CONbits.RSEN == 1) continue;
}

//send i2c
unsigned char i2c1Write(unsigned char dat) {
	I2C1TRN = dat; 					 			// Send slave address with Read/Write bit cleared
	//while (I2C1STATbits.TRSTAT == 1) continue;	// Wait until transmit buffer is empty
	while (I2C1STATbits.TBF == 1) continue;		// Wait until transmit buffer is empty
	//i2c1Wait();								//wait for i2c bus to idle
	while (I2C1STATbits.ACKSTAT == 1) continue;	//wait for the ack signal
	return I2C1STATbits.ACKSTAT; 				// Wait until ACK is received
}

//read i2c
unsigned char i2c1Read(unsigned char ack) {
	I2C1CONbits.RCEN = 1;			   		// Receive enable
	while (I2C1CONbits.RCEN == 1) continue;		// Wait until RCEN is cleared (automatic)
	I2C1STATbits.I2COV = 0;						//clear the flag
	I2C1CONbits.ACKDT = ack;					//send ack bits
	I2C1CONbits.ACKEN = 1;
	while (I2C1CONbits.ACKEN == 1) continue;
	return I2C1RCV;								// Retrieve value from I2C1RCV
}
//write from a buffer
uint8_t i2c1Writes(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToWrite) {
	i2c1Start();							//send start condition
	i2c1Write(DeviceAddr | I2C_CMD_WRITE);	//send device addr, for write operations
	i2c1Write(RegAddr);					//send register addr
	while (NumByteToWrite) {
		i2c1Write(*pBuffer);				//send data
		pBuffer++;							//increment buffer pointer
		NumByteToWrite--;					//decrement counter
	}
	i2c1Stop();							//send stop condition
	return 0;								//indicating success
}

//read to a buffer
uint8_t i2c1Reads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToRead) {
	i2c1Start();							//send start condition
	i2c1Write(DeviceAddr | I2C_CMD_WRITE);	//send device addr, for write operations
	i2c1Write(RegAddr);					//send register addr
	i2c1Start();							//send restart
	i2c1Write(DeviceAddr | I2C_CMD_READ);	//send device addr, for read operations
	while (NumByteToRead) {
		if (NumByteToRead==1) {
			*pBuffer=i2c1Read(I2C_NOACK);
			i2c1Stop();							//send the stop condition
			break;
		}
		*pBuffer=i2c1Read(I2C_ACK);		//read the data
		pBuffer++;							//increment buffer
		NumByteToRead--;					//decrement count
	}
	return 0;
}

//end i2c1



//software i2c
#define sI2C_HIGH(pin)				do {						pinMode(pin, INPUT);  sI2C_DLY();} while (0)	//let pin float to high
#define sI2C_LOW(pin)				do {digitalWrite(pin, LOW); pinMode(pin, OUTPUT); sI2C_DLY();} while (0)	//pull pin low
#define sI2C_GET(pin)				digitalRead(pin)			//read a pin
#define sI2C_DLY()					delayTks(F_CPU / F_sI2C / 2)								//software I2C delay for half of the period to achieve F_sI2C

//initialize i2c
void sI2CInit(void) {
	//pins idle high / as input
	sI2C_HIGH(sI2CSDAPIN); sI2C_HIGH(sI2CSCLPIN);				//clear the bus (idles high)
}


//-----------------START Condition-----------------------
void sI2CStart(void) {
	sI2C_HIGH(sI2CSDAPIN);						//let sda high
	sI2C_HIGH(sI2CSCLPIN);						//let scl high
	sI2C_LOW(sI2CSDAPIN);						//pull i2c_sda low
	sI2C_LOW(sI2CSCLPIN);						//pull i2c_scl low
}

//------------------STOP Condition--------------------------
void sI2CStop(void) {
	sI2C_LOW(sI2CSCLPIN);						//let scl float high
	sI2C_LOW(sI2CSDAPIN);
	sI2C_HIGH(sI2CSCLPIN);						//let scl float high
	sI2C_HIGH(sI2CSDAPIN);						//let sda  high
}

//------------------restart condition---------------------

//-------------------I2C Write---------------------------
uint8_t sI2CWrite(uint8_t dat) {
	unsigned char i;

	sI2C_HIGH(sI2CSDAPIN);						//let sda float
	i=0x80;
	do {
		sI2C_LOW(sI2CSCLPIN);					//clear i2c_scl
	  	if(dat & i)
			{sI2C_HIGH(sI2CSDAPIN);}			//set i2c_sda
		else
			{sI2C_LOW(sI2CSDAPIN);}				//clear i2c_sda
		sI2C_HIGH(sI2CSCLPIN);					//set i2c_scl
	  	i = i >> 1;								//shift out the highest bit
	} while (i);
	sI2C_LOW(sI2CSCLPIN);						//clear i2c_scl

	sI2C_HIGH(sI2CSDAPIN);						//float sda, let the slave control it
	sI2C_HIGH(sI2CSCLPIN);
	i=0;
	while (sI2C_GET(sI2CSDAPIN)&&(i<I2C_ACK_ERROR))
		i++;									//wait for ack from the slave (ack = sda pulled to low by the slave
	sI2C_LOW(sI2CSCLPIN);
	if (i<I2C_ACK_ERROR) return I2C_ACK;		//no trouble
	else return I2C_NOACK;						//trouble! ack timed out
}

//-----------------------i2c read------------------------------
//to be consistent with i2c protocol, use negative logic
//ack = 0 -> send ack
//ack = 1 -> no ack
uint8_t sI2CRead(uint8_t ack) {
	unsigned char i, data_t=0;

	sI2C_HIGH(sI2CSDAPIN);						//let sda float
	i=0x80;
	do {
		sI2C_LOW(sI2CSCLPIN);					//clear i2c_scl
		data_t <<=1;							//left shift the data
		i = i >> 1;
		//i2c_delay(0); i2c_delay(0);i2c_delay(0);i2c_delay(0);
		sI2C_HIGH(sI2CSCLPIN);					//let scl float to high
		if (sI2C_GET(sI2CSDAPIN)) data_t |= 0x01;	//set the last bit high
		else data_t |= 0x00;
		//i2c_delay(0);
	} while (i);
	sI2C_LOW(sI2CSCLPIN);						//pull scl low
	if (ack==I2C_ACK)
		{sI2C_LOW(sI2CSDAPIN);}					//send ack
	else
		{sI2C_HIGH(sI2CSDAPIN);}				//send no-ack
	sI2C_HIGH(sI2CSCLPIN);						//send ack/no-ack
	sI2C_LOW(sI2CSCLPIN);
	return data_t;
}

//write from a buffer
uint8_t sI2CWrites(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToWrite) {
	sI2CStart();							//send start condition
	sI2CWrite(DeviceAddr | I2C_CMD_WRITE);	//send device addr, for write operations
	sI2CWrite(RegAddr);					//send register addr
	while (NumByteToWrite) {
		sI2CWrite(*pBuffer);				//send data
		pBuffer++;							//increment buffer pointer
		NumByteToWrite--;					//decrement counter
	}
	sI2CStop();							//send stop condition
	return 0;								//indicating success
}

//read to a buffer
uint8_t sI2CReads(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t *pBuffer, uint16_t NumByteToRead) {
	sI2CStart();							//send start condition
	sI2CWrite(DeviceAddr | I2C_CMD_WRITE);	//send device addr, for write operations
	sI2CWrite(RegAddr);					//send register addr
	sI2CStart();							//send restart
	sI2CWrite(DeviceAddr | I2C_CMD_READ);	//send device addr, for read operations
	while (NumByteToRead) {
		if (NumByteToRead==1) {
			*pBuffer=sI2CRead(I2C_NOACK);
			sI2CStop();							//send the stop condition
			break;
		}
		*pBuffer=sI2CRead(I2C_ACK);		//read the data
		pBuffer++;							//increment buffer
		NumByteToRead--;					//decrement count
	}
	return 0;
}

//end software i2c

//rtcc

//global variables
static void (* _rtcc_isrptr)(void)=empty_handler;				//rtcc_ptr pointing to empty_handler by default

//interrupt service routine
void _ISR_PSV _RTCCInterrupt(void) {
	IFS3bits.RTCIF=0;							//clear interrupt flag
	_rtcc_isrptr();								//execute user tmr1 isr
}

//initialize the rtcc
//clock source choosen in config bits: RTCOSC (SOSC and LPRC)
void RTCInit(void) {
	PMD3bits.RTCCMD = 0;						//0->enable power to rtcc
	IFS3bits.RTCIF = 0;							//0->clear the flag
	IEC3bits.RTCIE = 0;							//0->disable the interrupt, 1->enable the interrupt
	//enable sosc
	di();
	__builtin_write_OSCCONL(OSCCON | (1<<1));	//1->enable sosc, 0->disable sosc
	ei();
	RTCC_WREN();								//allows write to rtc registers
	RCFGCALbits.RTCEN=1;						//start the rtc
	RTCC_WRDIS();
}

//read from rtcc
//mask: 0b00->min.sec, 0b01->weekday.hour, 0b10->month.day, 0b11->..year
uint16_t RTCRead(uint16_t mask) {
	//uint16_t tmp;
	//RTCC_WREN();								//allows write to rtc registers
	//RCFGCAL &= RTCC_MASK;						//mask off all bits not related to pointers
	//RCFGCAL |= mask;							//point to min/sec
	RCFGCALbits.RTCPTR=mask;
	//tmp = RTCVAL;								//return the lower byte
	//RTCC_WRDIS();
	return RTCVAL;
}

//write to rtcc
void RTCWrite(uint16_t mask, uint16_t byte_t) {
	RTCC_WREN();								//allows write to rtc registers
	//RCFGCAL &= RTCC_MASK;						//mask off all bits not related to pointers
	//RCFGCAL |= mask;							//point to min/sec
	RCFGCALbits.RTCPTR=mask;
	RTCVAL = byte_t;							//return the lower byte
	RTCC_WRDIS();
}

//write to rtcc calibration register
void RTCSetCal(uint8_t cal) {
	RTCC_WREN();								//allows write to rtc registers
	RCFGCALbits.CAL=cal;
	RTCC_WRDIS();
}

//read from rtcc calibration register
uint8_t RTCGetCal(void) {
	return RCFGCALbits.CAL;
}

//attach rtc isr
void RTCAttachISR(void (*isrptr)(void)) {
	_rtcc_isrptr=isrptr;						//activate the isr handler
	IPC15bits.RTCIP = RTCIP_DEFAULT;
	//IPC0bits.T1IS = TxIS_DEFAULT;
	IFS3bits.RTCIF = 0;							//reset the flag
	IEC3bits.RTCIE = 1;							//rtc1 interrupt on
}

//convert rtcc to time_t
time_t RTC2time(void) {
	struct tm tm;
	uint16_t year, mondat, wkhour, minsec;
	uint8_t tmp;
	
	//read the rtcc
	RCFGCALbits.RTCPTR=3;			//start from reading year, decreaments after each read
	year  =RTCVAL;					//year since 1/1/2000
	mondat=RTCVAL;					//month (0..11) and date (1..31)
	wkhour=RTCVAL;					//weekday (0..6) and hour (0..23)
	minsec=RTCVAL;					//minute (0..59) and second (0..59)
	
	tmp = minsec;	tm.tm_sec = BCD2DEC(tmp);
	tmp = minsec>>8;tm.tm_min = BCD2DEC(tmp);
	tmp = wkhour;	tm.tm_hour= BCD2DEC(tmp);
	tmp = mondat; 	tm.tm_mday= BCD2DEC(tmp);
	tmp = mondat>>8;tm.tm_mon = BCD2DEC(tmp)-1;			//month = 0..11
	tmp = year;     tm.tm_year= BCD2DEC(tmp)+2000-1900;	//dspic33 RTCC starts at 1/1/2000, posix year relative to 1/1/1970
	return mktime (&tm);	
}

//turn time (=time_t) to RTCC
void time2RTC(time_t t) {
	struct tm *tmptr;
	uint16_t year, mondat, wkhour, minsec;
	//uint8_t tmp;
	
	tmptr=gmtime(&t);
	minsec = (DEC2BCD(tmptr->tm_min) << 8) | DEC2BCD(tmptr->tm_sec);
	wkhour =                                 DEC2BCD(tmptr->tm_hour);
	mondat = (DEC2BCD(tmptr->tm_mon+1)<<8) | DEC2BCD(tmptr->tm_mday);
	year   =                                 DEC2BCD(tmptr->tm_year+1900-2000);	//posix starts at 1/1/1900 and dspic33 RTCC starts at 1/1/2000
	//set rtcc
	RTCC_WREN();							//enable write to rtcc
	//write to the rtcc
	RCFGCALbits.RTCPTR=3;					//start from reading year, decreaments after each read
	RTCVAL = year;							//year since 1/1/2000
	RTCVAL = mondat;						//month (0..11) and date(1..31)
	RTCVAL = wkhour;						//weekday (0..6) and hour (0..23)
	RTCVAL = minsec;						//minute (0..59) and second (0..59)
	RTCC_WRDIS();							//disable write to rtcc
}

//end rtcc

//cnint
void (* _cn_isrptr) (void)=empty_handler;

void _ISR_PSV _CNInterrupt(void) {
	IFS1bits.CNIF = 0; 							//clear the flag
	_cn_isrptr();								//run the isr
}

//initialize change notification
void cnInit(uint32_t pins) {
	_cn_isrptr=empty_handler;					//reset the isr

	//enable power - always enabled

	IFS1bits.CNIF = 0;							//clear the flag
	IEC1bits.CNIE = 1;							//1->enable the interrupt, 0->disable the interrupt

	//set up cnie
	CNEN1 = pins;
	CNPU1 = pins;
	pins = pins >> 16;
	CNEN2 = pins;
	CNPU1 = pins;

	IEC1bits.CNIE = 1;							//1->enable the interrupt, 0->disable the interrupt
}

//attach user isr
void cnAttachISR(void (*isrptr) (void)) {
	_cn_isrptr = isrptr;						//point the isrptr
	IFS1bits.CNIF= 0;							//0->clear the flag
	IEC1bits.CNIE= 1;							//0->disable the interrupt
}

//end cnint

//crc, 16-bit and 32-bit
//initialize crc engine: little endian, no interrupt
//weight in xor, and initial value in init
void CRCInit(uint8_t len, uint32_t poly) {
#if defined(__PIC24GA10x__) | defined(__PIC24GB00x__)
	/*#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
			defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__) | \
			defined(__PIC24FJ64GB002__) | defined(__PIC24FJ64GB004__) | \
			defined(__PIC24FJ48GB002__) | defined(__PIC24FJ48GB004__) | \
			defined(__PIC24FJ32GB002__) | defined(__PIC24FJ32GB004__)*/
	PMD3bits.CRCMD = 0;							//0->enable the model, 1->disable the model

	//configure the crc engine
	CRCCON1 = CRCCON2 = 0;						//reset the configuration word
	CRCCON1bits.LENDIAN=0;						//0->data shifting LSB first (little endian), 1->data shifting MSB first (big endian)
	CRCCON1bits.CRCISEL=0;						//0->CRCIF set on shift completion; 1->CRCIF set on buffer empty
	//CRCCON1bits.CRCGO  =1;					//1->start crc shift, 0->crc shifter turned off

	CRCCON2bits.PLEN = len-1;	//0..31: length of CRC. 0x07->8bit, 0x0f->16 bit, 0x1f->32 bit
	CRCCON2bits.DWIDTH = CRCCON2bits.PLEN;		//set data width to plen

	//CRCXORH = CRCXORL = 0;					//crc xor registers
	//CRCXORH = xor >> 16; CRCXORL = xor;		//configure the xor words
	CRCXOR = poly;

	//CRCWDATH= init >> 16; CRCWDATL = init;
	//CRCWDAT = init;

	IFS4bits.CRCIF = 0;							//0->clear the flag, 1->set the flag
	IPC16bits.CRCIP= CRCIP_DEFAULT;				//set the interrupt priority
	//IPC16bits.CRCIS= CRCIS_DEFAULT;			//set the interrupt subpriority
	IEC4bits.CRCIE = 0;							//0->disable the interrupt, 1->enable the interrupt

	CRCCON1bits.CRCEN = 1;						//1->enable the model, 0->disable the model
#else	//ga00x chips
#if defined(__PIC24GA00x__)
	PMD3bits.CRCPMD = 0;							//0->enable the model, 1->disable the model
#else	//ga11x
	PMD3bits.CRCMD = 0;							//0->enable the model, 1->disable the model
#endif
	//configure the crc engine
	CRCCON = 0;									//reset the configuration word
	//CRCCONbits.LENDIAN=0;						//0->data shifting LSB first (little endian), 1->data shifting MSB first (big endian)
	//CRCCONbits.CRCISEL=0;						//0->CRCIF set on shift completion; 1->CRCIF set on buffer empty
	//CRCCON1bits.CRCGO  =1;					//1->start crc shift, 0->crc shifter turned off

	CRCCONbits.PLEN = len-1;	//0..31: length of CRC. 0x07->8bit, 0x0f->16 bit, 0x1f->32 bit
	//CRCCONbits.DWIDTH = CRCCON2bits.PLEN;		//set data width to plen

	//CRCXORH = CRCXORL = 0;					//crc xor registers
	//CRCXORH = xor >> 16; CRCXORL = xor;		//configure the xor words
	CRCXOR = poly;

	//CRCWDATH= init >> 16; CRCWDATL = init;
	//CRCWDAT = init;

	IFS4bits.CRCIF = 0;							//0->clear the flag, 1->set the flag
	IPC16bits.CRCIP= CRCIP_DEFAULT;				//set the interrupt priority
	//IPC16bits.CRCIS= CRCIS_DEFAULT;			//set the interrupt subpriority
	IEC4bits.CRCIE = 0;							//0->disable the interrupt, 1->enable the interrupt

	//CRCCONbits.CRCEN = 1;						//1->enable the model, 0->disable the model
#endif	//ga102
}

//calculate crc
//length must be multiples of 2
uint16_t CRC16(char *msg, uint16_t length, uint16_t init_val) {
	uint16_t tmp;
#if defined(__PIC24GA10x__)
	/*#if 	defined(__PIC24FJ64GA102__) | defined (__PIC24FJ64GA104__) | \
			defined(__PIC24FJ32GA102__) | defined (__PIC24FJ32GA104__)*/
	CRCWDAT = init_val;							//load the initial word
	CRCCON1bits.CRCGO = 1;						//CRCGo(); //while (CRCCON1bits.CRCMPT==0) continue;	//wait for crc to complete

	while (length) {
		//tmp = *(uint16_t *)msg++;				//load the data
		//tmp = ((uint16_t) (*msg++) << 8) | (*msg++);
		tmp = *msg++;
		tmp = tmp << 8;			//form the data
		tmp|= *msg++;
		while (CRCCON1bits.CRCFUL==1) continue;	//wait for buffer to have at least 1 space
		CRCDAT = tmp;
		length-=2;
	}

	while (CRCCON1bits.CRCFUL==1) continue;		//wait until CRC is not full
	CRCDAT = 0;									//shift out the last two bytes
	while (CRCCON1bits.CRCMPT==0) continue;
	NOP4();										//wait for the shift to complete. 1 NOP() is enough
	//CRCCON1bits.CRCGO=0;						//why?
	//while (CRCCON1bits.CRCGO == 1) continue;

#else
	CRCWDAT = init_val;							//load the initial word
	CRCCONbits.CRCGO = 1;						//CRCGo(); //while (CRCCON1bits.CRCMPT==0) continue;	//wait for crc to complete

	while (length) {
		//tmp = *(uint16_t *)msg++;				//load the data
		//tmp = ((uint16_t) (*msg++) << 8) | (*msg++);
		tmp = *msg++;
		tmp = tmp << 8;			//form the data
		tmp|= *msg++;
		while (CRCCONbits.CRCFUL==1) continue;	//wait for buffer to have at least 1 space
		CRCDAT = tmp;
		length-=2;
	}

	while (CRCCONbits.CRCFUL==1) continue;		//wait until CRC is not full
	CRCDAT = 0;									//shift out the last two bytes
	while (CRCCONbits.CRCMPT==0) continue;
	NOP4();										//wait for the shift to complete. 1 NOP() is enough
	//CRCCON1bits.CRCGO=0;						//why?
	//while (CRCCON1bits.CRCGO == 1) continue;

#endif

	return CRCWDAT;
}

//comparator voltage reference
void CVrefInit(void) {
	//no PMD bits?

	CVRCON = 0;									//reset value
	CVRCONbits.CVRR = 1;						//1->0..0.625 in 24 steps. 0->0.25..0.719 in 32 steps
	//output disabled
	//
	CVRCONbits.CVREN = 1;						//enable the module
}
//end comparator voltage reference

//comparator
#if defined(__PIC24GA10x__) | defined(__PIC24GB00x__) | defined(__PIC24GA11x__) | defined(__PIC24GB11x__)
/*#if 	defined(__PIC24FJ64GA102__) | defined(__PIC24FJ64GA104__) | \
		defined(__PIC24FJ32GA102__) | defined(__PIC24FJ32GA104__) | \
	 	defined(__PIC24FJ64GB002__) | defined(__PIC24FJ64GB004__) | \
		defined(__PIC24FJ48GB002__) | defined(__PIC24FJ48GB004__) | \
		defined(__PIC24FJ32GB002__) | defined(__PIC24FJ32GB004__)*/
//initialize comparator
void CM1Init(void) {
	PMD3bits.CMPMD = 0;							//1->disable the module, 0->enable the model

	CM1CON = 0;									//reset value
	CM1CONbits.CEN = 1;							//1->enable the module, 0->disable the module
}

void CM2Init(void) {
	PMD3bits.CMPMD = 0;							//1->disable the module, 0->enable the model

	CM2CON = 0;									//reset value
	CM2CONbits.CEN = 1;							//1->enable the module, 0->disable the module
}

void CM3Init(void) {
	PMD3bits.CMPMD = 0;							//1->disable the module, 0->enable the model

	CM3CON = 0;									//reset value
	CM3CONbits.CEN = 1;							//1->enable the module, 0->disable the module
}
#else		//ga002 devices
//initialize comparator
void CM1Init(void) {
	PMD3bits.CMPMD = 0;							//1->disable the module, 0->enable the model

	CMCON = CMCON & 0b0010101010101100;			//reset value
	CMCONbits.C1EN = 1;							//1->enable the module, 0->disable the module
}

void CM2Init(void) {
	PMD3bits.CMPMD = 0;							//1->disable the module, 0->enable the model

	CMCON = CMCON &~0b0010101010101100;			//reset value
	CMCONbits.C2EN = 1;							//1->enable the module, 0->disable the module
}
#endif

//end comparator

//circular buffer
//global defines
#define cBUFF_SIZE_MASK 	(cBUFF_SIZE-1U)		//mask

//global variables
//reset the cbuffer
void cbInit(CB_TypeDef *cbuffer) {
	cbuffer->writeIndex = cbuffer->readIndex = -1;
}

//returns true if cbuffer is empty
char cbEmpty(CB_TypeDef *cbuffer) {
	return (cbuffer->writeIndex & cBUFF_SIZE_MASK) == (cbuffer->readIndex & cBUFF_SIZE_MASK);
}

//returns true if cbuffer is full;
char cbFull(CB_TypeDef *cbuffer) {
	return ((cbuffer->writeIndex + 1) & cBUFF_SIZE_MASK) == (cbuffer->readIndex & cBUFF_SIZE_MASK);
}

//write to the buffer - needs to test if the buffer is full before writing to it
cbuffer_t cbWrite(CB_TypeDef *cbuffer, cbuffer_t dat) {
	cbuffer->buffer[(++cbuffer->writeIndex) & cBUFF_SIZE_MASK] = dat;
	return dat;
}

//read from the buffer - needs to test if the buffer is empty before reading from it
cbuffer_t cbRead(CB_TypeDef *cbuffer) {
	return cbuffer->buffer[(++cbuffer->readIndex) & cBUFF_SIZE_MASK];
}

//write string to CB
void str2cb(CB_TypeDef *cb, char *str) {
	while (!cbFull(cb) && (*str)) cbWrite(cb, *str++);
}

//write CB to string
void cb2str(char *str, CB_TypeDef *cb) {
	while (!cbEmpty(cb)) *str++=cbRead(cb);
}

//end circular buffer


#if 	defined(__dsPIC33FJ128GP804__) | defined (__dsPIC33FJ128GP802__) | defined(__dsPIC33FJ64GP804__)  | defined (__dsPIC33FJ64GP802__)
//dac1
//initialize dac1
//dac1xM output disabled
//input data = unsigned format
//len=0: disable left channel, 1: enable left channel
//ren=0: disable right channel,1: enable right channel
void dac1Init(char len, char ren) {
	PMD3bits.DAC1MD = 0;					//0->enable peripheral, 1->disable peripheral
	
	//configure dac1
	DAC1CON = 	(0<<15) |					//1->enable dac, 0->disable dac
				(0<<13) |					//0->enable dac in idle, 1->disable dac in idle
				(0<<12) |					//0->disable amp in sleep / idle, 1->enable amp in sleep / idle
				(0<< 8) |					//0->unsigned data, 1->signed data
				(6<< 0) |					//DAC clock divider. 0..127 (6=default)
				0x00;
	DAC1STAT=	(0<<15) |					//1->enable left channel output, 0->disable left channel output
				(0<<13) |					//1->enable left channel mid point output, 0->disable it
				(0<<10) |					//1->interrupt if left channel fifo is empty, 0->interrupt if left channel fifo is not full
				(0<< 7) |					//1->enable right channel output, 0->disable right channel output
				(0<< 5) |					//1->enable right channel mid point output, 0->disable it
				(0<< 2) |					//1->interrupt if right channel fifo is empty, 0->interrupt if right channel fifo is not full
				0x00;
	DAC1DFLT= 	0xffff/2;					//default output
	DAC1STATbits.LOEN = len?1:0;
	DAC1STATbits.ROEN = ren?1:0;
	DAC1CONbits.DACEN = 1;					//1->enable dac, 0->disable dac
}
//end dac1
#endif

//programmable crc
//initialize crc
void crcInit(uint16_t poly, uint8_t plen) {
	PMD3bits.CRCMD = 0;						//0->enable peripheral, 1->disable peripheral
	
	CRCCON = 0;								//default value
	
	CRCCONbits.PLEN = plen;					//poly length
	CRCXOR = poly;							//update the poly register
	//CRCCONbits.CRCGO= 1;					//start crc
}

//end programmable crc
