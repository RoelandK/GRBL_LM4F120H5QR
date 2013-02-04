/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud
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

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

#include "settings.h"
#include "spindle_control.h"
#include "planner.h"

static uint8_t current_direction;

void spindle_init()
{
  current_direction = 0;
  ///SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT);
  SysCtlPeripheralEnable( SPINDLE_ENABLE_PERIPH );
  SysCtlDelay(26); ///give time delay 1 microsecond for GPIO module to start
  GPIOPinTypeGPIOOutput( SPINDLE_ENABLE_PORT, (1<<SPINDLE_ENABLE_BIT) );

  ///SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT);
  SysCtlPeripheralEnable( SPINDLE_DIRECTION_PERIPH );
  SysCtlDelay(26); ///give time delay 1 microsecond for GPIO module to start
  GPIOPinTypeGPIOOutput( SPINDLE_DIRECTION_PORT, (1<<SPINDLE_DIRECTION_BIT) );

  spindle_stop();
}

void spindle_stop()
{
  ///SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
  GPIOPinWrite( SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, 0 );
}

void spindle_run(int8_t direction) //, uint16_t rpm)
{
  if (direction != current_direction) {
    plan_synchronize();
    if (direction) {
      if(direction > 0) {
        ///SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
        GPIOPinWrite( SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, 0 );
      } else {
        ///SPINDLE_DIRECTION_PORT |= 1<<SPINDLE_DIRECTION_BIT;
        GPIOPinWrite( SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, 0xFF );
      }
      ///SPINDLE_ENABLE_PORT |= 1<<SPINDLE_ENABLE_BIT;
      GPIOPinWrite( SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, 0xFF );
    } else {
      spindle_stop();
    }
    current_direction = direction;
  }
}
