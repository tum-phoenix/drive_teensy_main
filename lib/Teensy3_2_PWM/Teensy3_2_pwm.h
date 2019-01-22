#ifndef PWM_T32_h
#define PWM_T32_h

/*
  PWM_T32.h - Hardware Servo Timer Library
  http://arduiniana.org/libraries/PWM_T32/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to PWM_T32 by Mikal Hart
  ported to other chips by Paul Stoffregen

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>

#define SERVO_PIN_A 9
#define SERVO_PIN_B 10

class PWM_T32
{
  private:
    uint8_t pin;
    float angle;       // in degrees
    float min16;       // minimum pulse, 16uS units  (default is 34)
    float max16;       // maximum pulse, 16uS units, 0-4ms range (default is 150)
    static uint32_t attachedpins[]; // 1 bit per digital pin
  public:
    PWM_T32();
    uint8_t attach(uint8_t pinArg) { return attach(pinArg, 544, 2400); }
                             // pulse length for 0 degrees in microseconds, 544uS default
                             // pulse length for 180 degrees in microseconds, 2400uS default
    uint8_t attach(uint8_t pinArg, float min, float max);
                             // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
                             // Only works for 9 and 10.
    uint8_t detach();
    void write(float angleArg); // specify the angle in degrees, 0 to 180
    void writeMicros(float Arg);
    float read() { return angle; }
    uint8_t attached();
};

#endif
