#include "Arduino.h"
#include "Teensy3_2_pwm.h"

/*
  PWMServo.cpp - Hardware Servo Timer Library
  http://arduiniana.org/libraries/pwmservo/
  Author: Jim Studt, jim@federated.com
  Copyright (c) 2007 David A. Mellis.  All right reserved.
  renamed to PWMServo by Mikal Hart
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
#define PWM_FREQUENCY 68.665598535
#define PWM_RES_BITS 16
#define US_PRESCALER (PWM_FREQUENCY / 1000000 * ((1 << PWM_RES_BITS) - 1))
#define NO_ANGLE (0xff)

uint32_t PWMServo::attachedpins[(NUM_DIGITAL_PINS+31)/32]; // 1 bit per digital pin

PWMServo::PWMServo() : pin(255), angle(NO_ANGLE) {}

uint8_t PWMServo::attach(uint8_t pinArg, float min, float max)
{
	//Serial.printf("attach, pin=%d, min=%d, max=%d\n", pinArg, min, max);
	if (pinArg < 0 || pinArg >= NUM_DIGITAL_PINS) return 0;
	if (!digitalPinHasPWM(pinArg)) return 0;
	pin = pinArg;
	analogWriteFrequency(pin, PWM_FREQUENCY);
	min16 = min;
	max16 = max;
	angle = NO_ANGLE;
	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
	attachedpins[pin >> 5] |= (1 << (pin & 31));
	return 1;
}

uint8_t PWMServo::detach()
{
	//Serial.printf("attach, pin=%d, min=%d, max=%d\n", pinArg, min, max);
	if (pin < 0 || pin >= NUM_DIGITAL_PINS) return 0;
	digitalWrite(pin, LOW);
	pinMode(pin, INPUT);
	attachedpins[pin >> 5] &= ~(1 << (pin & 31));
	return 1;
}

void PWMServo::write(float angleArg)
{
	//Serial.printf("write, pin=%d, angle=%d\n", pin, angleArg);
	if (pin >= NUM_DIGITAL_PINS) return;
  constrain(angleArg, 0., 180.);
	angle = angleArg;
	double us = map((double)angleArg, 0., 180., min16, max16); // us
	uint32_t duty = (uint32_t)(us * US_PRESCALER);
#if TEENSYDUINO >= 137
	noInterrupts();
	uint32_t oldres = analogWriteResolution(PWM_RES_BITS);
	analogWrite(pin, duty);
	analogWriteResolution(oldres);
	interrupts();
#else
	analogWriteResolution(PWM_RES_BITS);
	analogWrite(pin, duty);
#endif
}

void PWMServo::writeMicros(float Arg)
{
	//Serial.printf("write, pin=%d, angle=%d\n", pin, angleArg);
	if (pin >= NUM_DIGITAL_PINS) return;
  constrain(Arg, 50., (1000000./PWM_FREQUENCY)-50.);  // minimum 50 micros between start and end of pwm cycle to give the controller safty buffer
	angle = map(Arg, 0., 180., min16, max16);
	double us = (double)Arg; // us
	uint32_t duty = (uint32_t)(us * US_PRESCALER);
#if TEENSYDUINO >= 137
	noInterrupts();
	uint32_t oldres = analogWriteResolution(PWM_RES_BITS);
	analogWrite(pin, duty);
	analogWriteResolution(oldres);
	interrupts();
#else
	analogWriteResolution(PWM_RES_BITS);
	analogWrite(pin, duty);
#endif
}

uint8_t PWMServo::attached()
{
	if (pin >= NUM_DIGITAL_PINS) return 0;
	return (attachedpins[pin >> 5] & (1 << (pin & 31))) ? 1 : 0;
}