/*
  coolant_control.c - coolant control methods
  Part of Grbl

  Copyright (c) 2012 Sungeun K. Jeon

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

#ifdef PART_LM4F120H5QR
  #include "inc/hw_types.h"
  #include "inc/hw_memmap.h"
  #include "driverlib/sysctl.h"
  #include "driverlib/gpio.h"
#else
  #include <avr/io.h>
#endif

#include "coolant_control.h"
#include "settings.h"
#include "config.h"
#include "planner.h"

static uint8_t current_coolant_mode;

void coolant_init()
{
  current_coolant_mode = COOLANT_DISABLE;

  #if ENABLE_M7
  	#ifdef PART_LM4F120H5QR // code for ARM
      SysCtlPeripheralEnable( COOLANT_MIST_PERIPH ); //enable GPIO module
      SysCtlDelay(26); //wait 1us while init module
      GPIOPinTypeGPIOOutput( COOLANT_MIST_PORT, (1<<COOLANT_MIST_BIT) );
   	#else // code for AVR
	    COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
	  #endif
  #endif
  
 	#ifdef PART_LM4F120H5QR // code for ARM
    SysCtlPeripheralEnable( COOLANT_FLOOD_PERIPH ); //enable GPIO module
    SysCtlDelay(26); //wait 1us while init module
    GPIOPinTypeGPIOOutput( COOLANT_FLOOD_PORT, (1<<COOLANT_FLOOD_BIT) );
 	#else // code for AVR
	  COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  #endif
  coolant_stop();
}

void coolant_stop()
{
  #ifdef ENABLE_M7
  	#ifdef PART_LM4F120H5QR // code for ARM
    	GPIOPinWrite( COOLANT_MIST_PORT, COOLANT_MIST_BIT, 0 );
  	#else // code for AVR
	    COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
    #endif
  #endif
  
  #ifdef PART_LM4F120H5QR // code for ARM
  	GPIOPinWrite( COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, 0 );
  #else // code for AVR
	  COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  #endif
}


void coolant_run(uint8_t mode)
{
  if (mode != current_coolant_mode)
  { 
    plan_synchronize(); // Ensure coolant turns on when specified in program.
    if (mode == COOLANT_FLOOD_ENABLE) { 
      #ifdef PART_LM4F120H5QR // code for ARM
        GPIOPinWrite( COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, 0xFF );
      #else // code for AVR
      	COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
      #endif

    #ifdef ENABLE_M7  
    } else if (mode == COOLANT_MIST_ENABLE) {
     	#ifdef PART_LM4F120H5QR // code for ARM
     		GPIOPinWrite( COOLANT_MIST_PORT, COOLANT_MIST_BIT, 0xFF );
     	#else // code for AVR
        COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
      #endif
    #endif
    } else {
      coolant_stop();
    }
    current_coolant_mode = mode;
  }
}
