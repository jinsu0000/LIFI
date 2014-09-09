#include "SLifiSender.h"
#include "SLifiGlobal.h"

// Provides ISR
#include <avr/interrupt.h>



void SLifiSender::send(unsigned long data, int nbits)
{
#if 0
	enableIROut(38);
	mark(SAMSUNG_HDR_MARK);  // 5000microSec MARK
	space(SAMSUNG_HDR_SPACE); // 5000microSec SPACE
	for (int i = 0; i < nbits; i++) {
		if (data & TOPBIT) { // 10000000000000000000000000000000 bit
			mark(SAMSUNG_BIT_MARK); // 560microSec MARK
			space(SAMSUNG_ONE_SPACE); // 1600microSec SPACE
		}
		else {
			mark(SAMSUNG_BIT_MARK); // 560microSec MARK
			space(SAMSUNG_ZERO_SPACE); // 560microSec SPACE
		}
		data <<= 1;
	}
	mark(SAMSUNG_BIT_MARK);
	space(0);
#else
	enableIROut(40);
	mark(HEADER_MARK);
	space(SONY_HDR_SPACE);
	data = data << (32 - nbits);
	for (int i = 0; i < nbits; i++) {
		if (data & TOPBIT) {
			mark(SONY_ONE_MARK);
			space(SONY_HDR_SPACE);
		}
		else {
			mark(SONY_ZERO_MARK);
			space(SONY_HDR_SPACE);
		}
		data <<= 1;
	}
#endif
}

void SLifiSender::mark(int time) {
	// Sends an IR mark for the specified number of microseconds.
	// The mark output is modulated at the PWM frequency.
	TIMER_ENABLE_PWM; // Enable pin 3 PWM output
	delayMicroseconds(time);
}

/* Leave pin off for time (given in microseconds) */
void SLifiSender::space(int time) {
	// Sends an IR space for the specified number of microseconds.
	// A space is no output, so the PWM output is disabled.
	TIMER_DISABLE_PWM; // Disable pin 3 PWM output
	delayMicroseconds(time);
}

void SLifiSender::enableIROut(int khz) {
	// Enables IR output.  The khz value controls the modulation frequency in kilohertz.
	// The IR output will be on pin 3 (OC2B).
	// This routine is designed for 36-40KHz; if you use it for other values, it's up to you
	// to make sure it gives reasonable results.  (Watch out for overflow / underflow / rounding.)
	// TIMER2 is used in phase-correct PWM mode, with OCR2A controlling the frequency and OCR2B
	// controlling the duty cycle.
	// There is no prescaling, so the output frequency is 16MHz / (2 * OCR2A)
	// To turn the output on and off, we leave the PWM running, but connect and disconnect the output pin.
	// A few hours staring at the ATmega documentation and this will all make sense.
	// See my Secrets of Arduino PWM at http://arcfn.com/2009/07/secrets-of-arduino-pwm.html for details.


	// Disable the Timer2 Interrupt (which is used for receiving IR)
	TIMER_DISABLE_INTR; //Timer2 Overflow Interrupt

	pinMode(TIMER_PWM_PIN, OUTPUT);
	digitalWrite(TIMER_PWM_PIN, LOW); // When not sending PWM, we want it low

	// COM2A = 00: disconnect OC2A
	// COM2B = 00: disconnect OC2B; to send signal set to 10: OC2B non-inverted
	// WGM2 = 101: phase-correct PWM with OCRA as top
	// CS2 = 000: no prescaling
	// The top value for the timer.  The modulation frequency will be SYSCLOCK / 2 / OCR2A.
	TIMER_CONFIG_KHZ(khz);
}