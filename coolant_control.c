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

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "coolant_control.h"
#include "settings.h"
#include "config.h"
#include "planner.h"

static uint8_t current_coolant_mode;

void coolant_init()
{
  current_coolant_mode = COOLANT_DISABLE;

  #if ENABLE_M7
    ///COOLANT_MIST_DDR |= (1 << COOLANT_MIST_BIT);
    SysCtlPeripheralEnable( COOLANT_MIST_SYSCTL_PERIPH );
    SysCtlDelay(26);
    GPIOPinTypeGPIOOutput( COOLANT_MIST_PORT, (1<<COOLANT_MIST_BIT) );
  #endif

  ///COOLANT_FLOOD_DDR |= (1 << COOLANT_FLOOD_BIT);
  SysCtlPeripheralEnable( COOLANT_FLOOD_SYSCTL_PERIPH );
  SysCtlDelay(26);
  GPIOPinTypeGPIOOutput( COOLANT_FLOOD_PORT, (1<<COOLANT_FLOOD_BIT) );
  coolant_stop();
}

void coolant_stop()
{
  #ifdef ENABLE_M7
    ///COOLANT_MIST_PORT &= ~(1 << COOLANT_MIST_BIT);
    GPIOPinWrite( COOLANT_MIST_PORT, COOLANT_MIST_BIT, 0 );
  #endif
  ///COOLANT_FLOOD_PORT &= ~(1 << COOLANT_FLOOD_BIT);
  GPIOPinWrite( COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, 0 );
}


void coolant_run(uint8_t mode)
{
  if (mode != current_coolant_mode)
  {
    plan_synchronize(); // Ensure coolant turns on when specified in program.
    if (mode == COOLANT_FLOOD_ENABLE) {
      ///COOLANT_FLOOD_PORT |= (1 << COOLANT_FLOOD_BIT);
      GPIOPinWrite( COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, 0xFF );
    #ifdef ENABLE_M7
      } else if (mode == COOLANT_MIST_ENABLE) {
          COOLANT_MIST_PORT |= (1 << COOLANT_MIST_BIT);
          GPIOPinWrite( COOLANT_MIST_PORT, COOLANT_MIST_BIT, 0xFF );
    #endif
    } else {
      coolant_stop();
    }
    current_coolant_mode = mode;
  }
}
