#include "avrduino.h"						//my own implementation of ardiuno on avr

//mcus supported
//ATmega328p, ATtiny2313

//global defines

//global variables
//empty interrupt handler
void empty_handler(void) {
	//do nothing here
}

//global interrupt
//set up core timer
//global variables
uint32_t SystemCoreClock=F_FRC;			//system core clock, before devided by 2. Updated by SystemCoreClockUpdate()

//for time base off TIMER1 @ 1:1 prescaler
//volatile uint32_t timer1_millis = 0;
volatile uint32_t systick_ovf = 0;
//Needs to be executed during mcu initialization or after oscillator reconfiguration
//updates SystemCoreClock.
uint32_t SystemCoreClockUpdate(void) {
	uint32_t tmp=F_RC8M;
	char lfuse, hfuse, efuse, lbits;	//low and high fuse, extended fuse and lock bits

	//read the fuses - only the low fuse is used
	cli();
	lfuse = boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
	hfuse = boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
	efuse = boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
	lbits = boot_lock_fuse_bits_get(GET_LOCK_BITS);
	sei();

	//calculate SystemCoreClock
	switch (lfuse & 0x0f) {
		case 0b0000:					//external clock
			tmp = F_XTAL;  break;
		//case 0b0001:					//reserved - do nothing
		case 0b0010:					//calibrated internal RC oscillator
			tmp = F_RC8M;  break;
		case 0b0011:					//internal 128Khz oscillator
			tmp = F_RC128K;break;
		case 0b0100:					//external low power oscillator (32768Khz)
		case 0b0101:
			tmp = F_LPOSC; break;
		case 0b0110:					//external full swing oscillator, 0.4-20Mhz
		case 0b0111:					//0.4-20Mhz
			tmp = F_XTAL;  break;
		case 0b1000:					//low power crystal oscillator, 0.4-0.9Mhz
		case 0b1001:					//0.4-0.9Mhz
		case 0b1010:					//0.9-3.0Mhz
		case 0b1011:					//0.9-3.0Mhz
		case 0b1100:					//3.0-8.0Mhz
		case 0b1101:					//3.0-8.0Mhz
		case 0b1110:					//8.0-16.Mhz
		case 0b1111:					//8.0-16.Mhz
			tmp = F_XTAL; break;
		}

#if defined(CLKPR)
	tmp = tmp >> (CLKPR & 0x0f);
#else
	tmp = (lfuse & 0x80)?tmp:(tmp / 8);	//if MSB=0 (programmed), output frequency divided by 8
#endif
	return SystemCoreClock=tmp;			//return the systemcoreclock
}

//switch oscillator
//return SystemCoreClock in frequency
//Needs to be executed during mcu initialization or after oscillator reconfiguration
//updates SystemCoreClock.
uint32_t SystemCoreClockSwitch(uint8_t nosc) {
	//uint32_t tmp=F_FRC;

	//switch oscillator - not yet implemented
	di();									//disable the interrupt
	//put code here
	ei();									//enable the interrupt
	return SystemCoreClockUpdate();			//update the core clock
}

//read device parameter row
uint8_t DS_ID1, DS_ID2, DS_ID3;				//device signature id 1, 2, 3
uint8_t RCCalib;							//RC osc calibration byte. = OSCCAL
uint8_t TS_OFFSET, TS_GAIN;					//TS offset and TS_gain


//read device signature parameters
//for ATmega328
void dsParamRead(void) {
	DS_ID1 = boot_signature_byte_get(0x0000);	//DS_ID1 at offset 0x0000
	DS_ID2 = boot_signature_byte_get(0x0002);	//DS_ID2 at offset 0x0002 -> in some examples this is at offset 0x0001
	DS_ID3 = boot_signature_byte_get(0x0004);	//DS_ID3 at offset 0x0004 -> in some examples this is at offset 0x0002
	RCCalib= boot_signature_byte_get(0x0001);	//RC oscillator calibration byte at offset 0x0001
	TS_OFFSET= boot_signature_byte_get(0x0002);	//TS_OFFSET at offset 0x0002
	TS_GAIN= boot_signature_byte_get(0x0003);	//TS_GAIN at offset 0x0003
}

//return 24-bit dsvice id
uint32_t dsID(void) {
	return ((uint32_t) DS_ID3 << 16) | (DS_ID2 << 8) | DS_ID1;
}

//return rc oscillabor calibration byte
uint8_t rcCalib(void) {
	return RCCalib;
}

//return TS offset
uint8_t tsOfst(void) {
	return TS_OFFSET;
}

//return TS_GAIN
uint8_t tsGain(void) {
	return TS_GAIN;
}
//reset the mcu
void mcuInit(void) {
	di(); dsParamRead();					//read device parameters

	systick_ovf = 0;						//reset systick
	//SystemCoreClock=F_FRC;				//defaults to FRC

	//configure the device
	//initialize tmr0 for systick generation - do not stop this
	tmr0Init();								//prescaler = 256:1, mode 0
	TIFR0 |= 	(1<<TOV0);					//clear the flag
	TIMSK0|=	(1<<TOIE0);					//tmr overflow interrupt: enabled

	//update SystemCoreClock
	SystemCoreClockUpdate();				//update system core clock

	//enable global interrupts
	ei();									//testing
}

//C main loop
int main(void) {

	mcuInit();								//reset the mcu
	setup();								//run the setup code
	while (1) {
		loop();								//run the default loop
	}
}

//Arduino Functions: GPIO
//set a pin mode to INPUT_PULLUP, INPUT or OUTPUT
//no error checking on PIN
//set pin mode
void pinMode(PIN_TypeDef pin, uint8_t mode) {
	char mask;

	switch (pin) {
		//porta
#if defined(PORTA)
		case RA0 ... RA7: mask = 1<< (pin - RA0); if(mode==OUTPUT) IO_OUT(DDRA, mask); else IO_IN(DDRA, mask); if (mode==INPUT_PULLUP) IO_SET(PINA, mask); break;
#endif
		//portB
#if defined(PORTB)
		case RB0 ... RB7: mask = 1<< (pin - RB0); if(mode==OUTPUT) IO_OUT(DDRB, mask); else IO_IN(DDRB, mask); if (mode==INPUT_PULLUP) IO_SET(PINB, mask); break;
#endif
		//portC
#if defined(PORTC)
		case RC0 ... RC7: mask = 1<< (pin - RC0); if(mode==OUTPUT) IO_OUT(DDRC, mask); else IO_IN(DDRC, mask); if (mode==INPUT_PULLUP) IO_SET(PINC, mask); break;
#endif
		//portD
#if defined(PORTD)
		case RD0 ... RD7: mask = 1<< (pin - RD0); if(mode==OUTPUT) IO_OUT(DDRD, mask); else IO_IN(DDRD, mask); if (mode==INPUT_PULLUP) IO_SET(PIND, mask); break;
#endif
		//porte
#if defined(PORTE)
		case RE0 ... RE7: mask = 1<< (pin - RE0); if(mode==OUTPUT) IO_OUT(DDRE, mask); else IO_IN(DDRE, mask); if (mode==INPUT_PULLUP) IO_SET(PINE, mask); break;
#endif
		//portf
#if defined(PORTF)
		case RF0 ... RF7: mask = 1<< (pin - RF0); if(mode==OUTPUT) IO_OUT(DDRF, mask); else IO_IN(DDRF, mask); if (mode==INPUT_PULLUP) IO_SET(PINF, mask); break;
#endif
		//portg
#if defined(PORTG)
		case RG0 ... RG7: mask = 1<< (pin - RG0); if(mode==OUTPUT) IO_OUT(DDRG, mask); else IO_IN(DDRG, mask); if (mode==INPUT_PULLUP) IO_SET(PING, mask); break;
#endif
		//portH
#if defined(PORTH)
		case RH0 ... RH7: mask = 1<< (pin - RH0); if(mode==OUTPUT) IO_OUT(DDRH, mask); else IO_IN(DDRH, mask); if (mode==INPUT_PULLUP) IO_SET(PINH, mask); break;
#endif
		default: break; //do nothing
	}
}

//set pin high/low
void digitalWrite(PIN_TypeDef pin, uint8_t val) {
	char mask;

	switch (pin) {
		//porta
#if defined(PORTA)
		case RA0 ... RA7: mask = 1<< (pin - RA0); if (val==LOW) IO_CLR(PORTA, mask); else IO_SET(PORTA, mask); break;
#endif
		//portB
#if defined(PORTB)
		case RB0 ... RB7: mask = 1<< (pin - RB0); if (val==LOW) IO_CLR(PORTB, mask); else IO_SET(PORTB, mask); break;
#endif
		//portC
#if defined(PORTC)
		case RC0 ... RC7: mask = 1<< (pin - RC0); if (val==LOW) IO_CLR(PORTC, mask); else IO_SET(PORTC, mask); break;
#endif
		//portD
#if defined(PORTD)
		case RD0 ... RD7: mask = 1<< (pin - RD0); if (val==LOW) IO_CLR(PORTD, mask); else IO_SET(PORTD, mask); break;
#endif
		//porte
#if defined(PORTE)
		case RE0 ... RE7: mask = 1<< (pin - RE0); if (val==LOW) IO_CLR(PORTE, mask); else IO_SET(PORTE, mask); break;
#endif
		//portf
#if defined(PORTF)
		case RF0 ... RF7: mask = 1<< (pin - RF0); if (val==LOW) IO_CLR(PORTF, mask); else IO_SET(PORTF, mask); break;
#endif
		//portg
#if defined(PORTG)
		case RG0 ... RG7: mask = 1<< (pin - RG0); if (val==LOW) IO_CLR(PORTG, mask); else IO_SET(PORTG, mask); break;
#endif
		//portH
#if defined(PORTH)
		case RH0 ... RH7: mask = 1<< (pin - RH0); if (val==LOW) IO_CLR(PORTH, mask); else IO_SET(PORTH, mask); break;
#endif
		default: break; //do nothing
	}

}

//read a pin state
char digitalRead(PIN_TypeDef pin) {
	char mask,val=0;

	switch (pin) {
		//porta
#if defined(PORTA)
		case RA0 ... RA7: mask = 1<< (pin - RA0); val = PINA & mask; break;
#endif
		//portB
#if defined(PORTB)
		case RB0 ... RB7: mask = 1<< (pin - RB0); val = PINB & mask;  break;
#endif
		//portC
#if defined(PORTC)
		case RC0 ... RC7: mask = 1<< (pin - RC0); val = PINC & mask; break;
#endif
		//portD
#if defined(PORTD)
		case RD0 ... RD7: mask = 1<< (pin - RD0); val = PIND & mask; break;
#endif
		//porte
#if defined(PORTE)
		case RE0 ... RE7: mask = 1<< (pin - RE0); val = PINE & mask; break;
#endif
		//portf
#if defined(PORTF)
		case RF0 ... RF7: mask = 1<< (pin - RF0); val = PINF & mask; break;
#endif
		//portG
#if defined(PORTG)
		case RG0 ... RG7: mask = 1<< (pin - RG0); val = PING & mask; break;
#endif
		//portH
#if defined(PORTH)
		case RH0 ... RH7: mask = 1<< (pin - RH0); val = PINH & mask; break;
#endif
		default: break; //do nothing
	}
	return val;
}
//end RIO

//ticks()
//Arduino Functions: Time
//return timer ticks
uint32_t systicks(void) {
	volatile uint32_t m;					//stores overflow count
	volatile uint16_t f;					//return the fractions / TMR1 value

	//use double reads
	do {
		m = systick_ovf;
		f = TCNT0 << 8;						//TMR0 is 8bit + 256:1 prescaler;
	} while (m != systick_ovf);
	//now m and f are consistent
	return (m | f);
}

//delay milliseconds
//void delayMs(uint32_t ms) {
//	uint32_t start_time = ticks();
//	ms *= cyclesPerMillisecond();
//	while (ticks() - start_time < ms) continue;
//}

//delay micros seconds
//void delayUs(uint32_t us) {
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

//uart availability
//uart0: atmega328
//uart1: atmega328

#if defined(UART0_TXISR)						//to save space
static volatile uint8_t	_U0TX_BUSY = 0;			//1->UxTX is busy
static char *_U0TX_ptr;							//point to string to be transmitted

//UxTX isr
//usi ovf interrupt
ISR(USART_TX_vect) {
	//UCSR0A |= (1<<UDRE0);						//UCSR0A |= (1<<TXC0);						//clear the flag by writing 1 to it
	if (*_U0TX_ptr) {							//last non-zero char has been sent
		UDR0 = *_U0TX_ptr++;					//load the data into transmitter
	} else {
		_U0TX_BUSY = 0;							//no long busy
		UCSR0B &=~(1<<UDRIE0);					//UCSR0B &=~(1<<TXCIE0);					//disable the interrupt
	}
}
#endif	//uart0_txisr

//tx/rx pins to be assumed in gpio mode
//data bits: 	8
//parity: 		none
//stop bits: 	1
//Xon/Xoff:		none
void uart0Init(uint32_t bps) {
#if defined(UART0_TXISR)
	_U0TX_BUSY = 0;								//0->uart0 not busy
#endif

	/* enable receiver and transmitter */
	UCSR0A=		(1<<U2X0);						//double speed. 0->for synchronous transmission
	UCSR0B=		(0<<RXCIE0) |					//0->disable RX complete interrupt
				(0<<TXCIE0) |					//0->disable TX complete interrupt
				(0<<UDRIE0) |					//0->disable usart data register empty interrupt
#if defined(U0RXPIN)
				(1<<RXEN0) |					//1->enable receiver
#else
				(0<<RXEN0) |					//1->enable receiver
#endif
#if defined(U0TXPIN)
				(1<<TXEN0) |					//1->enable transmitter
#else
				(0<<TXEN0) |					//1->enable transmitter
#endif
				(0<<UCSZ02) |					//char size 0b011->8bit
				(0<<RXB80) |					//receive data bit 8 / 9th data bit received
				(0<<TXB80);						//transmitter data bit 8 / 9th data bit to be sent

	/* set frame format: 8 data, 1 stop bit */
	UCSR0C=		(0<<UMSEL01) | (0<<UMSEL00) |	//00-> asynchronous usart
												//01-> synchronous usart
												//10-> reserved
												//11-> master spi
				(0<<UPM01) | (0<<UPM00) |		//parity check 00-> disabled
												//01-> reserved
												//10-> enabled, even parity
												//11-> enabled, odd parity
				(0<<USBS0) |					//stop bit select. 0->1 bit, 1->2bit
				(1<<UCSZ01) | (1<<UCSZ00) |		//char size.
												//000-> 5-bit
												//001-> 6-bit
												//010-> 7-bit
												//011-> 8-bit
												//100-> reserved
												//101-> reserved
												//110-> reserved
												//111-> 9-bit
				(1<<UCPOL0);					//clock polarity. 0-> rising xck edge. 1-> falling xck edge
	/* baud rate generator */
	//UBRR0H=(unsigned char) (baud >> 8);
	//UBRR0L=(unsigned char) baud;
	UBRR0=F_UART / ((UCSR0A & (1<<U2X0))?8:16) / bps - 1;			//generate baud rate register

	//check to see if if rx/tx pins are defined
#if defined(U0TXPIN)							//TX as output
	//U0TXPIN();								//IO_OUT(UxDDR, UxTX);
#endif

#if defined(U0RXPIN)							//RX as input
	//U0RXPIN();								//IO_IN(UxDDR, UxRX);
#endif
}

//put a string
void uart0Puts(char *str) {
#if defined(UART0_TXISR)
	if (*str) {
		uart0Wait();							//wait for it to become available
		_U0TX_BUSY = 1;							//1->UxTX is busy
		_U0TX_ptr = str;						//point to string to be transmitted
		UDR0 = *_U0TX_ptr++;					//load up the string and advance to the next char
		UCSR0A |= (1<<UDRE0);					//UCSR0A |= (1<<TXC0);					//clear the flag by writing 1 to it
		UCSR0B |= (1<<UDRIE0);					//UCSR0B |= (1<<TXCIE0);				//enable interrupt
	}
#else
	//polling transmission
	while(*str) {
		uart0Putch(*str++);	//send the ch and advance the pointer
	}
#endif
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
//    return U1RXREG;		//return it
//}

//test if data rx is available
//uint16_t uart1Available(void) {
//    return U1STAbits.URXDA;
//}

//test if uart tx is busy
//uint16_t uart1Busy(void) {
//    return U1STAbits.UTXBF;
//}

//print to uart1
void u0Print(const char *str, int32_t dat) {
	static char uRAM[uRAM_SIZE];		//transmission buffer, 40-1 char max
	uint8_t idx=20;

	uart0Wait();						//wait for the current transmission to finish
	//while (uart0Busy()) continue;

	//form the string
	strcpy(uRAM, str);					//copy to uarm
	if (dat < 0) {
		uRAM[6]='-';
		dat = -dat;
	}
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uRAM[idx--]=',';	//'0'+(dat % 10); dat /= 10;
	uRAM[idx--]='0'+(dat % 10); dat /= 10;
	uart0Puts(uRAM);	//send a message on uart1
}

//tmr0..6
static void (* _tmr0_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
static void (* _tmr1_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
static void (* _tmr2_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
//static void (* _tmr3_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
//static void (* _tmr4_isrptr)(void)=empty_handler;				//tmr4_ptr pointing to empty_handler by default
//static void (* _tmr5_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
//static void (* _tmr6_isrptr)(void)=empty_handler;				//tmr6_ptr pointing to empty_handler by default

//tmr0
//tmr0 isr
ISR(TIMER0_OVF_vect) {
	//tmr0 isr / systick
	//TIFR0 |= 	(1<<TOV0);					//clear the flag
	systick_ovf += 0x10000ul;				//8-bit tmr0+256:1 prescaler -> 16bit
	_tmr0_isrptr();							//execute user isr
}

//reset the tmr
//prescaler at 256:1
//mode 0: top = 0xff
void tmr0Init(void) {
	_tmr0_isrptr=empty_handler;		//reset isr ptr
	TCCR0B =	TCCR0B & (~0x07);			//turn off tmr0
	TCCR0A =	(0<<COM0A1) | (0<<COM0A0) |	//output compare a pins normal operation
				(0<<COM0B1) | (0<<COM0B0) |	//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |	//output compare c pins normal operation
				(0<<WGM01) | (0<<WGM00)	|	//wgm02..0 = 0b000 -> normal, overflow at 0xff->0x00
				0x00;
	//OCR1A = period;						//minimum time interval is 1ms
	TCNT0 = 0;								//reset the timer / counter
	TIFR0 |= 	(1<<TOV0);					//clear the flag
	TIMSK0|=	(1<<TOIE0);					//tmr overflow interrupt: enabled
	TCCR0B =	(0<<FOC0A) |				//forced output on ch a disabled
				(0<<FOC0B) |				//forced output on ch b disabled
				//(0<<FOC1C)				//forced output on ch c disabled
				(0<<WGM02) |				//wgm02..0 = 0b000 -> normal, overflow at 0xff->0x00
				(TMR_PS256x & TMR_PSMASK)	|	//prescaler = 256:1, per the header file
				0x00;
}

void tmr0AttachISR(void (*isr_ptr)(void)) {
	TIFR0 |= 	(1<<TOV0);					//clear the flag
	TIMSK0|=	(1<<TOIE0);					//tmr overflow interrupt: enabled
	_tmr0_isrptr=isr_ptr;					//reassign tmr0 isr ptr
}

static void (* _tmr0oca_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
static void (* _tmr0ocb_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
volatile uint8_t _tmr0oca_pr=0xff;
volatile uint8_t _tmr0ocb_pr=0xff;

//tmr0 output compare cha isr
ISR(TIMER0_COMPA_vect) {
	//TIFR0 |= (1<<OCF0A);					//clear the flag
	OCR0A += _tmr0oca_pr;					//increment to the next match point
	_tmr0oca_isrptr();						//execute user isr
}

//output compare cha
void tmr0OCAInit(void) {
	_tmr0oca_isrptr=empty_handler;			//reset isr ptr
	_tmr0oca_pr = 0xff;						//initialize the match point
	TIFR0 |= (1<<OCF0A);					//clear the flag
	TIMSK0&=~(1<<OCIE0A);					//disable the interrupt
}

void tmr0OCAAttachISR(void (*isr_ptr)(void)) {
	TIFR0 |= (1<<OCF0A);					//clear the flag
	TIMSK0|= (1<<OCIE0A);					//tmr overflow interrupt: enabled
	_tmr0oca_isrptr=isr_ptr;				//reassign tmr0 isr ptr
	OCR0A  = TCNT0 + _tmr0oca_pr;			//set to the next match point
}

//tmr0 output compare chb isr
ISR(TIMER0_COMPB_vect) {
	//TIFR0 |= (1<<OCF0B);					//clear the flag
	OCR0B += _tmr0ocb_pr;					//increment to the next match point
	_tmr0ocb_isrptr();						//execute user isr
}

//output compare chb
void tmr0OCBInit(void) {
	_tmr0ocb_isrptr=empty_handler;			//reset isr ptr
	_tmr0ocb_pr = 0xff;						//reset the period
	TIFR0 |= (1<<OCF0B);					//clear the flag
	TIMSK0&=~(1<<OCIE0B);					//disable the interrupt
}

void tmr0OCBAttachISR(void (*isr_ptr)(void)) {
	TIFR0 |= (1<<OCF0B);					//clear the flag
	TIMSK0|= (1<<OCIE0B);				//tmr overflow interrupt: enabled
	_tmr0ocb_isrptr=isr_ptr;					//reassign tmr0 isr ptr
	OCR0B  = TCNT0 + _tmr0ocb_pr;			//set to the next match point
}
//end tmr0

//tmr1
//tmr1 isr
ISR(TIMER1_OVF_vect) {
	//TIFR1 |= ;	//clear the flag - done automatically
	_tmr1_isrptr();							//execute the handler
}

//reset the tmr
//mode 0,
//prescaler = 64:1
void tmr1Init(void) {
	_tmr1_isrptr=/*_tmr1_*/empty_handler;			//reset isr ptr

	//set up the timer
	TCCR1B =	TCCR1B & (~TMR_PSMASK);			//turn off tmr1
	TCCR1A =	(0<<COM1A1) | (0<<COM1A0) |	//output compare a pins normal operation
				(0<<COM1B1) | (0<<COM1B0) |	//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |	//output compare c pins normal operation
				(0<<WGM11) | (0<<WGM10)		//wgm13..0 = 0b0100 -> ctc, top at ocr1a
				;
	TCCR1B =	(TCCR1B & ~((1<<WGM13) | (1<<WGM12))) |	//clear wgm13..2
				(0<<WGM13) | (0<<WGM12);	//wgm13.0=0b0100
	TCCR1C =	(0<<FOC1A) |				//forced output on ch a disabled
				(0<<FOC1B) |				//forced output on ch b disabled
				//(0<<FOC1C) |					//forced output on ch c disabled
				0x00;
	//OCRxA = pr - 1;
	TCNT1 = 0;								//reset the timer / counter
	TIFR1 |= (1<<TOV1);						//clear the flag by writing '1' to it
	TIMSK1=	//(0<<TICIE1) |					//input capture isr: disabled
				//(0<<OCIE1C) |				//output compare isr for ch a: disabled
				(0<<OCIE1B) |				//output compare isr for ch b: disabled
				(0<<OCIE1A) |				//output compare isr for ch c: disabled
				(0<<TOIE1) |				//tmr overflow interrupt: disabled
				0x00;
	TCCR1B |=	(TMR_PS64x & TMR_PSMASK) |	//prescaler, per the header file
				0x00;
	//now timer1 is running
}

//overflow isr
void tmr1AttachISR(void (*isr_ptr)(void)) {
	_tmr1_isrptr=isr_ptr;					//reassign tmr1 isr ptr
	TIFR1 |= (1<<TOV1);						//clear the flag by writing '1' to it
	TIMSK1|= (1<<TOIE1);					//tmr overflow interrupt: enabled
}

static void (* _tmr1oca_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
static void (* _tmr1ocb_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
volatile uint16_t _tmr1oca_pr=0xffff;
volatile uint16_t _tmr1ocb_pr=0xffff;

//tmr1 output compare cha isr
ISR(TIMER1_COMPA_vect) {
	//TIFR1 |= (1<<OCF1A);					//clear the flag
	OCR1A += _tmr1oca_pr;					//increment to the next match point
	_tmr1oca_isrptr();						//execute user isr
}

//output compare cha
void tmr1OCAInit(void) {
	_tmr1oca_isrptr=empty_handler;			//reset isr ptr
	_tmr1oca_pr = 0xffff;					//reset the period
	TIFR1 |= (1<<OCF1A);					//clear the flag
	TIMSK1&=~(1<<OCIE1A);					//disable the interrupt
}

void tmr1OCAAttachISR(void (*isr_ptr)(void)) {
	TIFR1 |= (1<<OCF1A);					//clear the flag
	TIMSK1|= (1<<OCIE1A);					//tmr overflow interrupt: enabled
	_tmr1oca_isrptr=isr_ptr;				//reassign tmr0 isr ptr
	OCR1A  = TCNT1 + _tmr1oca_pr;			//set to the next match point
}

//tmr1 output compare chb isr
ISR(TIMER1_COMPB_vect) {
	//TIFR1 |= (1<<OCF1B);					//clear the flag
	OCR1B += _tmr1ocb_pr;					//increment to the next match point
	_tmr1ocb_isrptr();						//execute user isr
}

//output compare chb
void tmr1OCBInit(void) {
	_tmr1ocb_isrptr=empty_handler;			//reset isr ptr
	_tmr1ocb_pr = 0xffff;					//reset the period
	TIFR1 |= (1<<OCF1B);					//clear the flag
	TIMSK1&=~(1<<OCIE1B);					//disable the interrupt
}

void tmr1OCBAttachISR(void (*isr_ptr)(void)) {
	TIFR1 |= (1<<OCF1B);					//clear the flag
	TIMSK1|= (1<<OCIE1B);				//tmr overflow interrupt: enabled
	_tmr1ocb_isrptr=isr_ptr;					//reassign tmr0 isr ptr
	OCR1B  = TCNT1 + _tmr1ocb_pr;			//set to the next match point
}

//end tmr1

//tmr2
//tmr2 isr - overflow isr
ISR(TIMER2_OVF_vect) {
	//TIFR2 |= (1<<TOV2) | (0<<OCF2A) | (0<<OCF2B);	//clear the flag
	_tmr2_isrptr();						//execute the handler
}

//reset the tmr
//mode 0, prescaler = 256:1
void tmr2Init(void) {
	_tmr2_isrptr = empty_handler;

	//initialize the timer
	TCCR2B =	TCCR2B & (~TMR_PSMASK);			//turn off tmr1
	TCCR2A |=	(0<<COM2A1) | (0<<COM2A0) |			//output compare a pins normal operation
				(0<<COM2B1) | (0<<COM2B0) |			//output compare b pins normal operation
				//(0<<COM1C1) | (0<<COM1C0) |		//output compare c pins normal operation
				(0<<WGM21) | (0<<WGM20)				//wgm2..0 = 0b010 -> normal mode
				;
	TCCR2B = 	(TCCR2B & ~(1<<WGM21)) |
				(0<<WGM21);
	TCNT2 = 0;										//reset the timer / counter
	//OCR2A = pr - 1;
	TIFR2 |= (1<<TOV2) | (0<<OCF2A) |  (0<<OCF2B);						//clear the flag by writing '1' to it
	TIMSK2 =		//(0<<TICIE1) |					//input capture isr: disabled
				//(0<<OCIE1C) |						//output compare isr for ch a: disabled
				(0<<OCIE2B) |						//output compare isr for ch b: disabled
				(0<<OCIE2A) |						//output compare isr for ch c: disabled
				(0<<TOIE2) |						//tmr overflow interrupt: disabled
				0x00;
	TCCR2B |=	(TMR2_PS256x & TMR_PSMASK) |	//prescaler, per the header file
				0x00;
	//now timer1 is running
}

void tmr2AttachISR(void (*isr_ptr)(void)) {
	_tmr2_isrptr=isr_ptr;					//reassign tmr1 isr ptr
	TIFR2 |= (1<<TOV2) | (0<<OCF2A) | (0<<OCF2B);						//clear the flag by writing '1' to it
	TIMSK2|= (1<<TOIE2) | (0<<OCIE2B) | (0<<OCIE2A);					//tmr overflow interrupt: enabled
}


static void (* _tmr2oca_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
static void (* _tmr2ocb_isrptr)(void)=empty_handler;				//tmr2_ptr pointing to empty_handler by default
volatile uint8_t _tmr2oca_pr=0xff;
volatile uint8_t _tmr2ocb_pr=0xff;

//tmr2 output compare cha isr
ISR(TIMER2_COMPA_vect) {
	//TIFR2 |= (1<<OCF2A);					//clear the flag
	OCR2A += _tmr2oca_pr;					//increment to the next match point
	_tmr2oca_isrptr();						//execute user isr
}

//output compare cha
void tmr2OCAInit(void) {
	_tmr2oca_isrptr=empty_handler;			//reset isr ptr
	_tmr2oca_pr = 0xff;
	TIFR2 |= (1<<OCF2A);					//clear the flag
	TIMSK2&=~(1<<OCIE2A);					//disable the interrupt
}

void tmr2OCAAttachISR(void (*isr_ptr)(void)) {
	TIFR2 |= (1<<OCF2A);					//clear the flag
	TIMSK2|= (1<<OCIE2A);				//tmr overflow interrupt: enabled
	_tmr2oca_isrptr=isr_ptr;					//reassign tmr0 isr ptr
	OCR2A  = TCNT2 + _tmr2oca_pr;			//set to the next match point
}

//tmr1 output compare chb isr
ISR(TIMER2_COMPB_vect) {
	//TIFR1 |= (1<<OCF1B);					//clear the flag
	OCR2B += _tmr2ocb_pr;					//increment to the next match point
	_tmr2ocb_isrptr();						//execute user isr
}

//output compare chb
void tmr2OCBInit(void) {
	_tmr2ocb_isrptr=empty_handler;			//reset isr ptr
	_tmr2ocb_pr = 0xff;						//reset the period
	TIFR2 |= (1<<OCF2B);					//clear the flag
	TIMSK2&=~(1<<OCIE2B);					//disable the interrupt
}

void tmr2OCBAttachISR(void (*isr_ptr)(void)) {
	TIFR2 |= (1<<OCF2B);					//clear the flag
	TIMSK2|= (1<<OCIE2B);				//tmr overflow interrupt: enabled
	_tmr2ocb_isrptr=isr_ptr;					//reassign tmr0 isr ptr
	OCR2B  = TCNT2 + _tmr2ocb_pr;			//set to the next match point
}
//end tmr2

//pwm
//clear on match, edge aligned
void analogWrite(uint8_t pin, uint16_t dc) {
	switch (pin) {
		//oc0a -> pd6, oc0b -> pd5, mode 3 (fast pwm, top at 0xff = 8bit pwm)
		case RD6: TCCR0A |= (1<<COM0A1) | (1<<WGM01) | (1<<WGM00); TCCR0A &=~(1<<COM0A0); OCR0A = dc; IO_OUT(DDRD, 1<<6); break;
		case RD5: TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00); TCCR0A &=~(1<<COM0B0); OCR0B = dc; IO_OUT(DDRD, 1<<5); break;
		//oc1a -> pb1, oc1b -> pb2, mode 7 (fast pwm, top at 0x3ff = 10bit pwm)
		case RB1: TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10); TCCR1A &=~(1<<COM1A0); TCCR1B |= (1<<WGM12); OCR1A = dc & PWM_PR; IO_OUT(DDRB, 1<<1); break;
		case RB2: TCCR1A |= (1<<COM1B1) | (1<<WGM11) | (1<<WGM10); TCCR1A &=~(1<<COM1B0); TCCR1B |= (1<<WGM12); OCR1B = dc & PWM_PR; IO_OUT(DDRB, 1<<2); break;
		//oc2a -> pb3, oc2b -> pd3, mode 3 (fast pwm, top at 0xff = 8bit pwm)
		case RB3: TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); TCCR2A &=~(1<<COM2A0); OCR2A = dc; IO_OUT(DDRB, 1<<3); break;
		case RD3: TCCR2A |= (1<<COM2B1) | (1<<WGM21) | (1<<WGM20); TCCR2A &=~(1<<COM2B0); OCR2B = dc; IO_OUT(DDRD, 1<<3); break;
		default: break;	//do nothing for invalid pins
	}
}
//end pwm

//adc
//for mega48/88/168/328
//use internal 1.1v Vref
void adcInit(void) {							//reset the adc

	//adc reference voltage
	//0: use AVref pin, 1: use AVcc pin
	//2: reserved, 3: use internal Vref
    ADMUX =		(0<<REFS0) |					//adc ref voltage: 0, 1, 3
                (0<<ADLAR) |					//right adjusted
                0x00;
    ADCSRA =	(1<<ADEN) |						//enable the adc
                (0<<ADSC) |						//don't start ads, yet
                (0<<ADATE) |					//auto trigger not enabled
                (0<<ADIF) |						//clear ad interrupt flag
                (0<<ADIE) |						//adc interrupt disabled
                (1<<ADPS2) |					//ad prescaler, adps2..0=100->1:16, 010->1:4x, 111->1:128x
                (1<<ADPS1) |
                (0<<ADPS0) |
                0x00;
    ADCSRB =	(0<<ACME) |						//disable high speed adc
                (0<<ADTS2) |					//adc auto source, 000->free running
                (0<<ADTS1) |
                (0<<ADTS0) |
                0x00;
    //can be commented out
    //DIDR0 = 	0;								//set to disable digital input buffer
}

//read analog input
int16_t adcRead(uint8_t ch) {					//read the adc
    ch = ch & 0x0f;								//retain only the lowest 4 bits
    //can be commented out
    switch (ch) {
    	case ADC_0: IO_IN(DDRC, (1<<0)); break;
    	case ADC_1: IO_IN(DDRC, (1<<1)); break;
    	case ADC_2: IO_IN(DDRC, (1<<2)); break;
    	case ADC_3: IO_IN(DDRC, (1<<3)); break;
    	case ADC_4: IO_IN(DDRC, (1<<4)); break;
    	case ADC_5: IO_IN(DDRC, (1<<5)); break;
    	//case ADC_6: IO_IN(DDRC, (1<<6)); break;
    	//case ADC_7: IO_IN(DDRC, (1<<7)); break;
    	default: break; 	//do nothing
    }
    ADMUX = (ADMUX & ~0x0f) | (ch & 0x0f);		//set the channel, mux3..0
    ADCSRA |= (1<<ADSC);						//start the adc
    while (ADCSRA & (1<<ADSC)) continue;		//wait for the adc to finish

    //return (ADCH << 8) | (ADCL);					//return the results
    return ADC;
}

//read analog multiple times
//N is power of 2
int16_t adcReadN(uint8_t ch, uint16_t N) {
	uint16_t tmp;
	int16_t sum=0;

	for (tmp=0; tmp<N; tmp++)
		sum+=adcRead(ch);
	return sum / N;
}

int16_t analogRead(PIN_TypeDef pin) {
	uint8_t ch;

	switch (pin) {
		case RC0 ... RC5: ch = ADC_0 + (pin - RC0); return adcRead(ch); break;
		default: break;	//do nothing, pin is invalid
	}
	return 0;
}

//read temperature, in x100C mode
//newer datasheet for atmega328 has the following:
//C = (ADC reading - (273+100 - TS_OFFSET)) x 128 / TS_GAIN + 25
//TS_OFFSET and TS_GAIN can be read as signature row paramters (via Z-pointer)
//
//alternatively, per datasheet, typical values are
// -40C->adc = 0x010d
// +25C->adc = 0x0160
//+120C->adc = 0x01e0
//8x oversampling, with 1.1v internal reference
int32_t adcReadTempCx100(void) {
	uint8_t sADMUX=ADMUX;					//save bit 7.6 = REF1 and REF0
	int32_t tmp;

	analogReference(3);						//3->adc reference = 1.1v internal reference
	tmp = adcReadN(ADC_TEMP, 8);
	ADMUX = sADMUX;							//restore the old ref value

	//using datasheet typical value
	return -4000 + (int32_t) (12500 + 4000) * (tmp - 0x010d) / (0x01e0 - 0x010d);
	//using TS_GAIN and TS_OFFSET
	return (tmp - (273+100 - tsOfst())) * 12800 / tsGain() + 2500;
}
//end adc

//external interrupt int0/int1
#define INT_LOW			0
#define INT_CHANGE		1
#define INT_FALLING		2
#define INT_RISING		3

//tmr0..6
static void (* _int0_isrptr)(void)=empty_handler;				//int0_ptr pointing to empty_handler by default
static void (* _int1_isrptr)(void)=empty_handler;				//int1_ptr pointing to empty_handler by default

//int0 isr
ISR(INT0_vect) {
	//clear the flag - done automatically
	_int0_isrptr();							//execute user isr
}

//external interrupt on int0
void int0Init(void) {
	_int0_isrptr=empty_handler;
	EIFR |= (1 << 0);							//write 1 to clear the flag
	EIMSK&=~(1 << 0);						//0->disable the interrupt, 1->enable the interrupt
	EICRA = (EICRA & ~(3 << 0)) | (INT_RISING << 0);
}

//install user isr
void int0AttachISR(void (*isr_ptr)(void)) {
	_int0_isrptr=empty_handler;
	EIFR |= (1 << 0);							//write 1 to clear the flag
	EIMSK|= (1 << 0);						//0->disable the interrupt, 1->enable the interrupt
}

//int1 isr
ISR(INT1_vect) {
	//clear the flag - done automatically
	_int1_isrptr();							//execute user isr
}

//external interrupt on int1
void int1Init(void) {
	_int0_isrptr=empty_handler;
	EIFR |= (1 << 1);							//write 1 to clear the flag
	EIMSK&=~(1 << 1);						//0->disable the interrupt, 1->enable the interrupt
	EICRA = (EICRA & ~(3 << 2)) | (INT_RISING << 2);
}

//install user isr
void int1AttachISR(void (*isr_ptr)(void)) {
	_int1_isrptr=empty_handler;
	EIFR |= (1 << 1);							//write 1 to clear the flag
	EIMSK|= (1 << 1);						//0->disable the interrupt, 1->enable the interrupt
}

//pin change interrupt pcint0/1/2
//tmr0..6
static void (* _pcint0_isrptr)(void)=empty_handler;				//int0_ptr pointing to empty_handler by default
static void (* _pcint1_isrptr)(void)=empty_handler;				//int1_ptr pointing to empty_handler by default
static void (* _pcint2_isrptr)(void)=empty_handler;				//int1_ptr pointing to empty_handler by default

//pcint0 isr
ISR(PCINT0_vect) {
	//clear the flag - done automatically
	_pcint0_isrptr();							//execute user isr
}

//external interrupt on int0
void pcint0Init(void) {
	_pcint0_isrptr=empty_handler;
	PCIFR |= (1 << 0);							//write 1 to clear the flag
	PCICR &=~(1 << 0);						//0->disable the interrupt, 1->enable the interrupt
}

//install user isr
void pcint0AttachISR(void (*isr_ptr)(void)) {
	_pcint0_isrptr=empty_handler;
	PCIFR |= (1 << 0);							//write 1 to clear the flag
	PCICR |= (1 << 0);						//0->disable the interrupt, 1->enable the interrupt
}

//pcint1 isr
ISR(PCINT1_vect) {
	//clear the flag - done automatically
	_pcint1_isrptr();							//execute user isr
}

//external interrupt on int0
void pcint1Init(void) {
	_pcint1_isrptr=empty_handler;
	PCIFR |= (1 << 1);							//write 1 to clear the flag
	PCICR &=~(1 << 1);						//0->disable the interrupt, 1->enable the interrupt
}

//install user isr
void pcint1AttachISR(void (*isr_ptr)(void)) {
	_pcint1_isrptr=empty_handler;
	PCIFR |= (1 << 1);							//write 1 to clear the flag
	PCICR |= (1 << 1);						//0->disable the interrupt, 1->enable the interrupt
}

//pcint2 isr
ISR(PCINT2_vect) {
	//clear the flag - done automatically
	_pcint2_isrptr();							//execute user isr
}

//external interrupt on int0
void pcint2Init(void) {
	_pcint2_isrptr=empty_handler;
	PCIFR |= (1 << 2);							//write 1 to clear the flag
	PCICR &=~(1 << 2);						//0->disable the interrupt, 1->enable the interrupt
}

//install user isr
void pcint2AttachISR(void (*isr_ptr)(void)) {
	_pcint2_isrptr=empty_handler;
	PCIFR |= (1 << 2);							//write 1 to clear the flag
	PCICR |= (1 << 2);						//0->disable the interrupt, 1->enable the interrupt
}

//end pin change interrupt

#if 0
//spi
//spi1 - master mode only
void spiInit(uint32_t bps) {
	SPI_PINs();							//initialize spi pins (SCK, SDO, SDI)
	SSPSTAT = SSPCON1 = 0;					//reset the registers (SSPCON2/3 used for I2C)
	SSPSTATbits.SMP = 0;					//0->data sampled in the middle; 1->data sampled in the end
	SSPSTATbits.CKE = 1;					//data sampled on the rising edge of sck, when ckp=0
	//SSPCON1bits.CKE=1;					//data sampled on the rising edge of sck, when ckp=0;
	SSPADD = F_PHB / bps;
	SSPCON1bits.SSPM3=1, SSPCON1bits.SSPM2=0;
	SSPCON1bits.SSPM1=1, SSPCON1bits.SSPM0=0;				//set the baud rate to F_PHB / SSPADD
	//input data sampled at the end of the output data. SMP=0 works as well
	SSPCON1bits.CKP=0;						//sck idles low
	SSPCON1bits.SSPEN=1;					//enable spi
}

//spi - send data
char spiWrite(char dat) {
	SSPBUF=dat;								//load sspbuf and start sending the byte
	SPI_WAIT();								//wait for the send to complete
	//data=SSPBUF;							//dummy read, to clear sspbuf flag
	return SSPBUF;
}

#if defined(PIC18F2XK22) | defined(PIC18F4XK22)
//spi2 - master mode only
void spi2Init(uint32_t bps) {
	SPI2_PINs();							//initialize spi pins (SCK, SDO, SDI)
	SSP2STAT = SSPCON1 = 0;					//reset the registers (SSPCON2/3 used for I2C)
	SSP2STATbits.SMP = 0;					//0->data sampled in the middle; 1->data sampled in the end
	SSP2STATbits.CKE = 1;					//data sampled on the rising edge of sck, when ckp=0
	//SSPCON1bits.CKE=1;					//data sampled on the rising edge of sck, when ckp=0;
	SSP2ADD = F_PHB / bps;
	SSP2CON1bits.SSPM3=1, SSPCON1bits.SSPM2=0;
	SSP2CON1bits.SSPM1=1, SSPCON1bits.SSPM0=0;				//set the baud rate to F_PHB / SSPADD
	//input data sampled at the end of the output data. SMP=0 works as well
	SSP2CON1bits.CKP=0;						//sck idles low
	SSP2CON1bits.SSPEN=1;					//enable spi
}

//spi - send data
char spi2Write(char dat) {
	SSP2BUF=dat;								//load sspbuf and start sending the byte
	SPI2_WAIT();								//wait for the send to complete
	//data=SSPBUF;							//dummy read, to clear sspbuf flag
	return SSP2BUF;
}

#endif	//4xk22
//end spi

//i2c

#define I2C_WAIT()					do {while (!SSPIF);	SSPIF=0;} while (0)	//wait for the  interrupt to clear

//initialize i2c
void i2cInit(void) {
	I2C_PINs();						//enable i2c pins: SCK,
//	I2C_OUT(I2C_SDA | I2C_SCL);		//turn i2c_sda/i2c_scl as output
	SSPCON1bits.SSPEN = 1;						//enable the mssp port (scl/sda pins)
	SSPCON1bits.SSPM3=1, SSPCON1bits.SSPM2=0, SSPCON1bits.SSPM1=0, SSPCON1bits.SSPM0=0; 		//0b1000=i2c master mode, baud rate = Fosc/(4*(SSPADD+1))
	SSPCON2=0x00;					//clear sspcon2
	SSPSTATbits.SMP = 0;						//enable slew rate control
//	CKE = 0;						//spi edge select
//	SSPADD = 9;						//to yield a 100kbps speed at 4Mhz
	SSPADD = F_CPU / F_I2C - 1 + 1;	//i2c baud rate, minimum of 2

}


//-----------------START Condition-----------------------
void i2cStart(void) {
	SSPCON2bits.SEN=1;							//enable start condition
	//I2C_WAIT();
	while (SSPCON2bits.SEN) continue;
}

//-----------------Repeated START Condition-----------------------
void i2cRestart(void) {
	SSPCON2bits.RSEN=1;							//enable start condition
	//I2C_WAIT();
	while (SSPCON2bits.RSEN) continue;
}

//------------------STOP Condition--------------------------
void i2cStop(void) {
	SSPCON2bits.PEN = 1;						//enable stop condition
	//I2C_WAIT();					//wait for the interrupt to clear
	while (SSPCON2bits.PEN) continue;
}

//-------------------I2C Write---------------------------
uint8_t i2cWrite(uint8_t dat) {
	SSPIF=0;						//clear the flag
	SSPBUF=dat;						//load the data
	I2C_WAIT();						//wait for interrupt to clear
	//while (BF) continue;
	return SSPCON2bits.ACKSTAT;					//return ack bit
}

//-----------------------i2c read------------------------------
uint8_t i2cRead(uint8_t ack) {
	while (SSPSTATbits.BF) continue;			//wait for an existing transmission to end
	SSPCON2bits.RCEN=1;							//start to receive
	while (SSPCON2bits.RCEN) continue;
	if (ack==I2C_ACK) SSPCON2bits.ACKDT=I2C_ACK;		//send ack
	else SSPCON2bits.ACKDT=I2C_NOACK;			//send no-ack
	SSPCON2bits.ACKEN=1;						//send the ack bit
	while (SSPCON2bits.ACKEN);					//wait for ack to complete
	return SSPBUF;						//return
}

//i2c2 - on K22 chips
#if defined(PIC18F2XK22) | defined(PIC18F4XK22)
#define I2C2_WAIT()					do {while (!SSP2IF);	SSP2IF=0;} while (0)	//wait for the  interrupt to clear

//initialize i2c
void i2c2Init(void) {
	I2C2_PINs();						//enable i2c pins: SCK,
//	I2C_OUT(I2C_SDA | I2C_SCL);			//turn i2c_sda/i2c_scl as output
	SSP2CON1bits.SSPEN = 1;				//enable the mssp port (scl/sda pins)
	SSP2CON1bits.SSPM3=1, SSP2CON1bits.SSPM2=0, SSP2CON1bits.SSPM1=0, SSP2CON1bits.SSPM0=0; 		//0b1000=i2c master mode, baud rate = Fosc/(4*(SSPADD+1))
	SSP2CON2=0x00;						//clear sspcon2
	SSP2STATbits.SMP = 0;				//enable slew rate control
//	CKE = 0;							//spi edge select
//	SSP2ADD = 9;						//to yield a 100kbps speed at 4Mhz
	SSP2ADD = F_CPU / F_I2C - 1 + 1;	//i2c baud rate, minimum of 2

}


//-----------------START Condition-----------------------
void i2c2Start(void) {
	SEN2=1;								//enable start condition
	//I2C_WAIT();
	while (SEN2) continue;
}

//-----------------Repeated START Condition-----------------------
void i2c2Restart(void) {
	RSEN2=1;							//enable start condition
	//I2C_WAIT();
	while (RSEN2) continue;
}

//------------------STOP Condition--------------------------
void i2c2Stop(void) {
	PEN2 = 1;							//enable stop condition
	//I2C_WAIT();						//wait for the interrupt to clear
	while (PEN2) continue;
}

//-------------------I2C Write---------------------------
uint8_t i2c2Write(uint8_t dat) {
	SSP2IF=0;							//clear the flag
	SSP2BUF=dat;						//load the data
	I2C2_WAIT();						//wait for interrupt to clear
	//while (BF) continue;
	return ACKSTAT2;					//return ack bit
}

//-----------------------i2c read------------------------------
uint8_t i2c2Read(uint8_t ack) {
	while (SSP2STATbits.BF) continue;			//wait for an existing transmission to end
	RCEN2=1;							//start to receive
	while (RCEN2) continue;
	if (ack==I2C_ACK) ACKDT2=I2C_ACK;	//send ack
	else ACKDT2=I2C_NOACK;				//send no-ack
	ACKEN2=1;							//send the ack bit
	while (ACKEN2);						//wait for ack to complete
	return SSP2BUF;						//return
}
#endif	//K22
#endif	//xxxx
//software i2c
//software i2c
#define sI2C_HIGH(pin)				do {                        pinMode(pin, INPUT);  sI2C_DLY();} while (0)	//let pin float to high
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


//software RTC
//hardware configuration
//end hardware configuration

//software RTC
//global defines

//global variables

volatile sRTC_TypeDef sRTC={-1, 0, 0, 0};		//software RTC

//initialize software counter
void sRTCInit(void) {					//calibration from -128ppm to +128ppm, if sRTC_RATE = 1M
	//sRTC.tick_rate=sRTC_RATE;
	sRTC.time=-1;						//reset counter: -1=uninitiated timer
	sRTC.millis=0;						//reset the last millis
	sRTC.cal=0;							//initialize calibration
	sRTC.halfsec=0;						//0->1st half sec, 1->2nd half sec
}

//initialize the calibration
void sRTCSetCal(int16_t cal) {
	sRTC.cal=cal;
}

//increment sRTC time - in the main loop or via an interrupt
void sRTCISR(void) {
	uint32_t ms=millis();							//time stamp
	if (ms - sRTC.millis >= sRTC_1sec) {			//1sec has passed
		sRTC.millis += sRTC_1sec;					//increment the time stamp
		sRTC.time+=1;								//increment tile
		//calibration not implemented
	}
	sRTC.halfsec=((ms - sRTC.millis) < sRTC_1sec/2)?0:1;	//half sec indicator
}

//read sRTC second counter
time_t sRTC2time(time_t *t) {
	if (t==NULL) return sRTC.time;
	return *t=sRTC.time;
}

//set sRTC second counter - used for initialization
//returns -1 if uninitialized
time_t time2sRTC(time_t t) {
	return sRTC.time=t;
}

//read rtc tick
uint32_t sRTCTick(void) {
	return sRTC.millis;
}

//return half sec
//0: first half of a sec
//1: 2nd half of a sec
uint8_t sRTCHalfsec(void) {
	return sRTC.halfsec;
}

//end software rtc
