// Hardware: ATtiny841 5V 8MHz(internal Oscillator)

#define F_CPU 8000000UL
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>

////////////////////////////////////////////////////////////////////////////////
// SPI
////////////////////////////////////////////////////////////////////////////////
#define USE_SPI
#ifdef USE_SPI
#define CSN 2
#define SCK 3
#define SDO 1
void spi_init(void) {
    //  remap SPI to PA0-3
    REMAP |= (1 << SPIMAP);
    // set SCLK(PA3)/CSN(PA2)/MOSI(PA1) as outputs
    DDRA |= (1<<SCK)|(1<<CSN)|(1<<SDO);
    // enable SPI/set as master/enable posedge sampling
    // leave SPI rate as 2 MHz (8MHz/4)
    // leave SPI rate as 0.5 MHz (8MHz/16)
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)/*|(1<<SPR0)*/;
}
void spi_deinit(void) {
   SPCR = 0;
   REMAP = (1 << U0MAP);
}
void spi_write(uint8_t addr, uint8_t data) {
   volatile uint8_t retdat;
   spi_init();
   PORTA &=~(1<<CSN);
   SPDR = addr&0x3F;
   while(!(SPSR & (1<<SPIF)));
   SPDR = data;
   while(!(SPSR & (1<<SPIF)));
   PORTA |=(1<<CSN);
   retdat = SPDR;
   spi_deinit();
   _delay_ms((retdat&1)|1);
}
void spi_read(uint8_t addr, uint8_t *data, uint8_t num) {
   uint8_t i;
   spi_init();
   PORTA &=~(1<<CSN);
   SPDR = (1<<7)|(1<<6)|(addr&0x3F);
   while(!(SPSR & (1<<SPIF)));
   for (i=0; i<num; i++) {
      SPDR = 0;
      while(!(SPSR & (1<<SPIF)));
      *(data+i) = SPDR;
   }
   PORTA |=(1<<CSN);
   spi_deinit();
   _delay_ms(1);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// UART0
////////////////////////////////////////////////////////////////////////////////
#define UART0_BAUD 19200UL
#define UART0_UBRR_VAL ((F_CPU+UART0_BAUD*8)/(UART0_BAUD*16)-1)
#define UART0_BAUD_REAL (F_CPU/(16*(UART0_UBRR_VAL+1)))
#define UART0_BAUD_ERROR ((UART0_BAUD_REAL*1000)/UART0_BAUD)

#if ((UART0_BAUD_ERROR<990) || (UART0_BAUD_ERROR>1010))
#error UART0_BAUD_ERROR > 1%
#endif

void uart0_init(void) {
    UBRR0 = UART0_UBRR_VAL;
    // enable TX
    UCSR0B |= (1 << TXEN0);
    // enable RX
    UCSR0B |= (1 << RXEN0);
    // enable RX interrupt
    UCSR0B |= (1 << RXCIE0);
    // remap UART0 to PB2/PA7
    REMAP |= (1 << U0MAP);
    // 8bit no parity
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
int uart0_putc(uint8_t c) {
    while(!(UCSR0A & (1<<UDRE0))) { }
    UDR0 = c;
    return 0;
}
void uart0_puts (char *s) {
    while (*s) {
        uart0_putc(*s);
        s++;
    }
}
char c[4];
ISR (USART0_RX_vect) {
   while ( !(UCSR0A & (1<<RXC0)) );
   // must read UDR0 otherwise stuck with interrupts
   sprintf(c, "\r%c", UDR0);
   uart0_puts(c);
}

////////////////////////////////////////////////////////////////////////////////
// TIMER
////////////////////////////////////////////////////////////////////////////////
#ifdef USE_TIMER
uint32_t onehz = 0;
ISR (TIMER1_COMPA_vect) {
   onehz++;
}
void setup_timer_interrupt() {
   // disable interrupts
   cli();
   //set timer1 interrupt at 1Hz
   TCCR1A = 0;// set entire TCCR1A register to 0
   TCCR1B = 0;// same for TCCR1B
   TCNT1  = 0;//initialize counter value to 0
   // set compare match register for 1hz increments
   OCR1A = 7813;// = (8*10^6) / (1*1024) - 1 (must be <65536)
   // turn on CTC mode
   TCCR1B |= (1 << WGM12);
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS12) | (1 << CS10);
   // enable timer compare interrupt
   TIMSK1 |= (1 << OCIE1A);
   // enable interrupts
   sei();
}
#endif

int main(void) {
   char uartstr[64];
   enum state_t {S_TEMP_EN,S_CONVERT_EN,S_TEMP_RD, S_MAG_X, S_MAG_Y, S_MAG_Z};
   enum state_t state=S_TEMP_EN;

   // initialize uart
   uart0_init();
   // enable interrupts
   sei();

#ifdef USE_TIMER
   setup_timer_interrupt();
#endif
   volatile uint16_t ee_addr = 0x0;
   volatile uint16_t ee_data = 0x0;

   //eeprom_write_word((uint16_t*)ee_addr,0x3412);
   volatile uint16_t temp = 0x0;

   while (1) {
      _delay_ms(2000);
      uart0_puts("\r");
      ee_data = eeprom_read_word((uint16_t*)ee_addr);
      //eeprom_write_word((uint16_t*)ee_addr,0x1234);
      if ((state == S_TEMP_RD)||(state == S_MAG_X)||(state == S_MAG_Y)||(state == S_MAG_Z)) {
         switch (state) {
            case S_MAG_X: 
               sprintf(uartstr,"UART0 >> %04x %c T=%5d", ee_addr, ee_data, temp);
               break;
            case S_MAG_Y:
               sprintf(uartstr,"UART0 >> %04x %c X=%5d", ee_addr, ee_data, temp);
               break;
            case S_MAG_Z:
               sprintf(uartstr,"UART0 >> %04x %c Y=%5d", ee_addr, ee_data, temp);
               break;
            case S_TEMP_RD:
               sprintf(uartstr,"UART0 >> %04x %c Z=%5d", ee_addr, ee_data, temp);
               break;
            default:
               break;
         }
      } else {
         sprintf(uartstr,"UART0 >> %04x %c %04x", ee_addr, ee_data, temp);
      }
      ee_addr++;
      ee_addr&=0x1FF;
      uart0_puts(uartstr);
#ifdef USE_SPI
      switch (state) {
         case S_TEMP_EN:
            spi_read(0x20,(uint8_t*)&temp,2);
            if ((temp&0xFF)!=0x90) {
               spi_write(0x20,0x90);
            } else {
               state=S_CONVERT_EN;
            }
            break;
         case S_CONVERT_EN:
            spi_read(0x22,(uint8_t*)&temp,2);
            if ((temp&0x03)!=0x00) {
               spi_write(0x22,0x00);
            } else {
               state=S_TEMP_RD;
            }
            break;
         case S_TEMP_RD:
            spi_read(0x2E,(uint8_t*)&temp,2);
            state=S_MAG_X;
            break;
         case S_MAG_X:
            spi_read(0x28,(uint8_t*)&temp,2);
            state=S_MAG_Y;
            break;
         case S_MAG_Y:
            spi_read(0x2A,(uint8_t*)&temp,2);
            state=S_MAG_Z;
            break;
         case S_MAG_Z:
            spi_read(0x2C,(uint8_t*)&temp,2);
            state=S_TEMP_RD;
            break;
         default:
            break;
      }
#endif
   }
}
