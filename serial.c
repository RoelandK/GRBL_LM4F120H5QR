/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2011-2012 Sungeun K. Jeon

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/* This code was initially inspired by the wiring_serial module by David A. Mellis which
   used to be a part of the Arduino project. */
#if defined( PART_LM4F120H5QR )
  // ARM includes
  #include "inc/hw_memmap.h"
  #include "inc/hw_types.h"
  #include "driverlib/sysctl.h"
  #include "driverlib/uart.h"
  #include "driverlib/interrupt.h"
  #include "driverlib/pin_map.h"
  #include "driverlib/gpio.h"
  #include "inc/hw_ints.h"
#else
  // AVR includes
  #include <avr/interrupt.h>
#endif

#include "serial.h"
#include "config.h"
#include "motion_control.h"
#include "protocol.h"

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_buffer_head = 0;
volatile uint8_t rx_buffer_tail = 0;

volatile uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t tx_buffer_head = 0;
volatile uint8_t tx_buffer_tail = 0;

#ifdef ENABLE_XONXOFF
  volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable

// Returns the number of bytes in the RX buffer. This replaces a typical byte counter to prevent
// the interrupt and main programs from writing to the counter at the same time.
static uint8_t get_rx_buffer_count()
{
  if (rx_buffer_head == rx_buffer_tail) { return(0); }
  if (rx_buffer_head < rx_buffer_tail) { return(rx_buffer_tail-rx_buffer_head); }
  return (RX_BUFFER_SIZE - (rx_buffer_head-rx_buffer_tail));
}
#endif

inline uint8_t receive_buffer_empty() {
  return rx_buffer_head == rx_buffer_head;
}

inline uint8_t transmit_buffer_empty() {
  return tx_buffer_head == tx_buffer_tail;
}

#ifdef PART_LM4F120H5QR
//ARM code
void arm_uart_receive_data( void );
void arm_uart_send_data( void );

void arm_uart_interrupt_handler( void ) {
  //clear interrupt flag
  unsigned long ul = UARTIntStatus( UART0_BASE, true );
  UARTIntClear( UART0_BASE, ul );

  //Interrupted because the TX FIFO has space available?
  if ( ul & UART_INT_TX ) {
    UARTIntDisable( UART0_BASE, UART_INT_TX ); //disable uart interrupt during FIFO filling

    if ( transmit_buffer_empty() ) {
      //If the output buffer is empty, just turn off the transmit interrupt
      return;
    } else {
      //else fill FIFO with new data
      while ( UARTSpaceAvail( UART0_BASE ) && !transmit_buffer_empty() ) arm_uart_send_data();
      UARTIntEnable( UART0_BASE, UART_INT_TX );
      return;
    }
  }

  //Interrupted because received a character?
  if ( ul & ( UART_INT_RX | UART_INT_RT ) )
    while ( UARTCharsAvail( UART0_BASE) ) arm_uart_receive_data();
}
#endif

void serial_init()
{
#ifdef PART_LM4F120H5QR
  //code for ARM
  SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOA ); //enable pins which correspond to RxD and TxD signals
  SysCtlDelay( 26 ); // Delay 1usec for peripherial to start
  GPIOPinConfigure( GPIO_PA0_U0RX ); //configure pin to be RxD of UART0
  GPIOPinConfigure( GPIO_PA1_U0TX ); //configure pin to be TxD of UART0
  GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1 ); //configure pins 0 and 1 of PORTA to be RxD and TxD

  SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 ); // Enable the UART0 peripheral for use.
  SysCtlDelay( 26 ); // Delay 1usec for peripherial to start
  UARTConfigSetExpClk( UART0_BASE, SysCtlClockGet(), 115200, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE ); //115200 baud, 8-N-1

  UARTFIFOLevelSet( UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8 ); //Interrupt if TX FIFO is almost empty or any character is received.
  UARTIntDisable( UART0_BASE, 0xFFFFFFFF ); // Disable all interrupt sources for UART0 module
  UARTIntEnable( UART0_BASE, UART_INT_RX | UART_INT_RT ); //Enable only receive interrupts
  UARTIntRegister( UART0_BASE, arm_uart_interrupt_handler );
  IntPrioritySet( INT_UART0, 64 ); // lowest priority for UART interrupts
  IntEnable( INT_UART0 ); //Enable UART0 interrupts in the NVIC
  UARTEnable( UART0_BASE ); //Enable UART0 to work

#else
  //code for AVR
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx and tx
  UCSR0B |= 1<<RXEN0;
  UCSR0B |= 1<<TXEN0;

  // enable interrupt on complete reception of a byte
  UCSR0B |= 1<<RXCIE0;

  // defaults to 8-bit, no parity, 1 stop bit

#endif //for ARM
}

void serial_write(uint8_t data) {
  // Calculate next head
  uint8_t next_head = tx_buffer_head + 1;
  if (next_head == TX_BUFFER_SIZE) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == tx_buffer_tail) {
    if (sys.execute & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  tx_buffer[tx_buffer_head] = data;
  tx_buffer_head = next_head;

#ifdef PART_LM4F120H5QR
  // ARM code
  arm_uart_send_data(); //actually send data
  UARTIntEnable( UART0_BASE, UART_INT_TX );
#else
  // AVR code
  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
#endif
}

// Data Register Empty Interrupt handler
#if defined( PART_LM4F120H5QR )
void arm_uart_send_data( void )
#elif defined( __AVR_ATmega644P__ )
ISR(USART0_UDRE_vect)
#else
ISR(USART_UDRE_vect)
#endif
{
  if ( transmit_buffer_empty() ) {
    #ifdef PART_LM4F120H5QR
      UARTIntDisable( UART0_BASE, UART_INT_TX );
    #else
      // AVR code
      // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
      if (tail == tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
    #endif
    return;
  }

  // Temporary tx_buffer_tail (to optimize for volatile)
  uint8_t tail = tx_buffer_tail;

  #ifdef ENABLE_XONXOFF
    if (flow_ctrl == SEND_XOFF) {
      UDR0 = XOFF_CHAR;
      flow_ctrl = XOFF_SENT;
    } else if (flow_ctrl == SEND_XON) {
      UDR0 = XON_CHAR;
      flow_ctrl = XON_SENT;
    } else
  #endif
  {
    // Send a byte from the buffer
    #if defined( PART_LM4F120H5QR )
      // ARM code
      UARTCharPutNonBlocking( UART0_BASE, tx_buffer[ tail ] );
    #else
      // AVR code
      UDR0 = tx_buffer[tail];
    #endif

    // Update tail position
    tail++;
    if (tail == TX_BUFFER_SIZE) { tail = 0; }

    tx_buffer_tail = tail;
  }
}

uint8_t serial_read()
{
  if (rx_buffer_head == rx_buffer_tail) {
    return SERIAL_NO_DATA;
  } else {
    uint8_t data = rx_buffer[rx_buffer_tail];
    rx_buffer_tail++;
    if (rx_buffer_tail == RX_BUFFER_SIZE) { rx_buffer_tail = 0; }

    #ifdef ENABLE_XONXOFF
      if ((get_rx_buffer_count() < RX_BUFFER_LOW) && flow_ctrl == XOFF_SENT) {
        flow_ctrl = SEND_XON;
        UCSR0B |=  (1 << UDRIE0); // Force TX
      }
    #endif

    return data;
  }
}

// UART Receive Interrupt handler
#if defined( PART_LM4F120H5QR )
void arm_uart_receive_data( void )
#elif defined( __AVR_ATmega644P__ )
ISR(USART0_RX_vect)
#else
ISR(USART_RX_vect)
#endif
{

#if defined( PART_LM4F120H5QR )
  // ARM code
  uint8_t data = (uint8_t)( UARTCharGetNonBlocking( UART0_BASE ) & 0xFF ); //read a char and remove control bits (highest)
  serial_write( data ); //echo
#else
  // AVR code
  uint8_t data = UDR0;
#endif

  uint8_t next_head;

  // Pick off runtime command characters directly from the serial stream. These characters are
  // not passed into the buffer, but these set system state flag bits for runtime execution.
  switch (data) {
    case CMD_STATUS_REPORT: sys.execute |= EXEC_STATUS_REPORT; break; // Set as true
    case CMD_CYCLE_START:   sys.execute |= EXEC_CYCLE_START; break; // Set as true
    case CMD_FEED_HOLD:     sys.execute |= EXEC_FEED_HOLD; break; // Set as true
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    default: // Write character to buffer
      next_head = rx_buffer_head + 1;
      if (next_head == RX_BUFFER_SIZE) { next_head = 0; }

      // Write data to buffer unless it is full.
      if (next_head != rx_buffer_tail) {
        rx_buffer[rx_buffer_head] = data;
        rx_buffer_head = next_head;

        #ifdef ENABLE_XONXOFF
          if ((get_rx_buffer_count() >= RX_BUFFER_FULL) && flow_ctrl == XON_SENT) {
            flow_ctrl = SEND_XOFF;
            UCSR0B |=  (1 << UDRIE0); // Force TX
          }
        #endif

      }
  }
}

void serial_reset_read_buffer()
{
  rx_buffer_tail = rx_buffer_head;

  #ifdef ENABLE_XONXOFF
    flow_ctrl = XON_SENT;
  #endif
}
