#include "NU32.h"

// Device Configuration Registers
// These only have an effect for standalone programs but don't harm bootloaded programs.
// the settings here are the same as those used by the bootloader
#pragma config DEBUG = OFF          // Background Debugger disabled
#pragma config FWDTEN = OFF         // WD timer: OFF
#pragma config WDTPS = PS4096       // WD period: 4.096 sec
#pragma config POSCMOD = HS         // Primary Oscillator Mode: High Speed crystal
#pragma config FNOSC = PRIPLL       // Oscillator Selection: Primary oscillator w/ PLL
#pragma config FPLLMUL = MUL_20     // PLL Multiplier: Multiply by 20
#pragma config FPLLIDIV = DIV_2     // PLL Input Divider:  Divide by 2
#pragma config FPLLODIV = DIV_1     // PLL Output Divider: Divide by 1
#pragma config FPBDIV = DIV_1       // Peripheral Bus Clock: Divide by 1
#pragma config UPLLEN = ON          // USB clock uses PLL
#pragma config UPLLIDIV = DIV_2     // Divide 8 MHz input by 2, mult by 12 for 48 MHz
#pragma config FUSBIDIO = ON        // USBID controlled by USB peripheral when it is on
#pragma config FVBUSONIO = ON       // VBUSON controlled by USB peripheral when it is on
#pragma config FSOSCEN = OFF        // Disable second osc to get pins back
#pragma config BWP = ON             // Boot flash write protect: ON
#pragma config ICESEL = ICS_PGx2    // ICE pins configured on PGx2
#pragma config FCANIO = OFF         // Use alternate CAN pins
#pragma config FMIIEN = OFF         // Use RMII (not MII) for ethernet
#pragma config FSRSSEL = PRIORITY_6 // Shadow Register Set for interrupt priority 6

#define NU32_DESIRED_BAUD 115200    // Baudrate for RS232
// #define NU32_DESIRED_BAUD 230400    // Baudrate for RS232

// Perform startup routines:
//  Make NU32_LED1 and NU32_LED2 pins outputs (NU32_USER is by default an input)
//  Initialize the serial port - UART3 (no interrupt) 
//  Enable interrupts
void NU32_Startup() {
  // disable interrupts
  __builtin_disable_interrupts();

  // enable the cache 
  // This command sets the CP0 CONFIG register
  // the lower 4 bits can be either 0b0011 (0x3) or 0b0010 (0x2)
  // to indicate that kseg0 is cacheable (0x3) or uncacheable (0x2)
  // see Chapter 2 "CPU for Devices with M4K Core" of the PIC32 reference manual
  // most of the other bits have prescribed values
  // microchip does not provide a _CP0_SET_CONFIG macro, so we directly use
  // the compiler built-in command _mtc0
  // to disable cache, use 0xa4210582 
  __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583); 
  
  // set the prefectch cache wait state to 2, as per the
  // electrical characteristics data sheet
  CHECONbits.PFMWS = 0x2;   

  //enable prefetch for cacheable and noncacheable memory
  CHECONbits.PREFEN = 0x3; 

  // 0 data RAM access wait states
  BMXCONbits.BMXWSDRM = 0x0;
  
  // enable multi vector interrupts
  INTCONbits.MVEC = 0x1;

  // disable JTAG to get B10, B11, B12 and B13 back
  DDPCONbits.JTAGEN = 0;

  TRISFCLR = 0x0003;  // Make F0 and F1 outputs (LED1 and LED2)
  NU32_LED1 = 1;      // LED1 is off
  NU32_LED2 = 0;      // LED2 is on

  // turn on UART3 without an interrupt
  U3MODEbits.BRGH = 0; // set baud to NU32_DESIRED_BAUD
  U3BRG = ((NU32_SYS_FREQ / NU32_DESIRED_BAUD) / 16) - 1;

  // 8 bit, no parity bit, and 1 stop bit (8N1 setup)
  U3MODEbits.PDSEL = 0;
  U3MODEbits.STSEL = 0;

  // configure TX & RX pins as output & input pins
  U3STAbits.UTXEN = 1;
  U3STAbits.URXEN = 1;
  // configure hardware flow control using RTS and CTS
  U3MODEbits.UEN = 2;

  // enable the uart
  U3MODEbits.ON = 1;

  __builtin_enable_interrupts();
}

// Read from UART3
// block other functions until you get a '\r' or '\n'
// send the pointer to your char array and the number of elements in the array
void NU32_ReadUART3(char * message, int maxLength) {
  char data = 0;
  int complete = 0, num_bytes = 0;
  // loop until you get a '\r' or '\n'
  while (!complete) {
    if (U3STAbits.URXDA) { // if data is available
      data = U3RXREG;      // read the data
      if ((data == '\n') || (data == '\r')) {
        complete = 1;
      } else {
        message[num_bytes] = data;
        ++num_bytes;
        // roll over if the array is too small
        if (num_bytes >= maxLength) {
          num_bytes = 0;
        }
      }
    }
  }
  // end the string
  message[num_bytes] = '\0';
}

// Write a character array using UART3
void NU32_WriteUART3(const char * string) {
  while (*string != '\0') {
    while (U3STAbits.UTXBF) {
      ; // wait until tx buffer isn't full
    }
    U3TXREG = *string;
    ++string;
  }
}
