#include "SLifiReceiver.h"
#include "SLifiGlobal.h"

// Provides ISR
#include <avr/interrupt.h>

volatile irparams_t irparams;

// These versions of MATCH, MATCH_MARK, and MATCH_SPACE are only for debugging.
// To use them, set DEBUG in SLifiGlobal.h
// Normally macros are used for efficiency
#ifdef DEBUG
int MATCH(int measured, int desired) {
	Serial.print("Testing: ");
	Serial.print(TICKS_LOW(desired), DEC);
	Serial.print(" <= ");
	Serial.print(measured, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired), DEC);
	return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired);
}

int MATCH_MARK(int measured_ticks, int desired_us) {
	Serial.print("Testing mark ");
	Serial.print(measured_ticks * USECPERTICK, DEC);
	Serial.print(" vs ");
	Serial.print(desired_us, DEC);
	Serial.print(": ");
	Serial.print(TICKS_LOW(desired_us + MARK_EXCESS), DEC);
	Serial.print(" <= ");
	Serial.print(measured_ticks, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired_us + MARK_EXCESS), DEC);
	return measured_ticks >= TICKS_LOW(desired_us + MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us + MARK_EXCESS);
}

int MATCH_SPACE(int measured_ticks, int desired_us) {
	Serial.print("Testing space ");
	Serial.print(measured_ticks * USECPERTICK, DEC);
	Serial.print(" vs ");
	Serial.print(desired_us, DEC);
	Serial.print(": ");
	Serial.print(TICKS_LOW(desired_us - MARK_EXCESS), DEC);
	Serial.print(" <= ");
	Serial.print(measured_ticks, DEC);
	Serial.print(" <= ");
	Serial.println(TICKS_HIGH(desired_us - MARK_EXCESS), DEC);
	return measured_ticks >= TICKS_LOW(desired_us - MARK_EXCESS) && measured_ticks <= TICKS_HIGH(desired_us - MARK_EXCESS);
}
#else
int MATCH(int measured, int desired) { return measured >= TICKS_LOW(desired) && measured <= TICKS_HIGH(desired); }
int MATCH_MARK(int measured_ticks, int desired_us) { return MATCH(measured_ticks, (desired_us + MARK_EXCESS)); }
int MATCH_SPACE(int measured_ticks, int desired_us) { return MATCH(measured_ticks, (desired_us - MARK_EXCESS)); }
// Debugging versions are in IRremote.cpp
#endif


SLifiReceiver::SLifiReceiver(int recvpin)
{
	irparams.recvpin = recvpin;
	irparams.blinkflag = 0;
}

// initialization
void SLifiReceiver::enableIRIn() {
	cli();
	// setup pulse clock timer interrupt
	//Prescale /8 (16M/8 = 0.5 microseconds per tick)
	// Therefore, the timer interval can range from 0.5 to 128 microseconds
	// depending on the reset value (255 to 0)
	TIMER_CONFIG_NORMAL();

	//Timer2 Overflow Interrupt Enable
	TIMER_ENABLE_INTR;

	TIMER_RESET;

	sei();  // enable interrupts

	// initialize state machine variables
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;

	// set pin modes
	pinMode(irparams.recvpin, INPUT);
}

// enable/disable blinking of pin 13 on IR processing
void SLifiReceiver::blink13(int blinkflag)
{
	irparams.blinkflag = blinkflag;
	if (blinkflag)
		pinMode(BLINKLED, OUTPUT);
}

// TIMER2 interrupt code to collect raw data.
// Widths of alternating SPACE, MARK are recorded in rawbuf.
// Recorded in ticks of 50 microseconds.
// rawlen counts the number of entries recorded so far.
// First entry is the SPACE between transmissions.
// As soon as a SPACE gets long, ready is set, state switches to IDLE, timing of SPACE continues.
// As soon as first MARK arrives, gap width is recorded, ready is cleared, and new logging starts
ISR(TIMER_INTR_NAME)
{
	TIMER_RESET;

	uint8_t irdata = (uint8_t)digitalRead(irparams.recvpin);

	irparams.timer++; // One more 50us tick
	if (irparams.rawlen >= RAWBUF) {
		// Buffer overflow
		irparams.rcvstate = STATE_STOP;
	}
	switch (irparams.rcvstate) {
	case STATE_IDLE: // In the middle of a gap
		if (irdata == MARK) {
			if (irparams.timer < GAP_TICKS) {
				// Not big enough to be a gap.
				irparams.timer = 0;
			}
			else {
				// gap just ended, record duration and start recording transmission
				irparams.rawlen = 0;
				irparams.rawbuf[irparams.rawlen++] = irparams.timer;
				irparams.timer = 0;
				irparams.rcvstate = STATE_MARK;
			}
		}
		break;
	case STATE_MARK: // timing MARK
		if (irdata == SPACE) {   // MARK ended, record time
			irparams.rawbuf[irparams.rawlen++] = irparams.timer;
			irparams.timer = 0;
			irparams.rcvstate = STATE_SPACE;
		}
		break;
	case STATE_SPACE: // timing SPACE
		if (irdata == MARK) { // SPACE just ended, record it
			irparams.rawbuf[irparams.rawlen++] = irparams.timer;
			irparams.timer = 0;
			irparams.rcvstate = STATE_MARK;
		}
		else { // SPACE
			if (irparams.timer > GAP_TICKS) {
				// big SPACE, indicates gap between codes
				// Mark current code as ready for processing
				// Switch to STOP
				// Don't reset timer; keep counting space width
				irparams.rcvstate = STATE_STOP;
			}
		}
		break;
	case STATE_STOP: // waiting, measuring gap
		if (irdata == MARK) { // reset gap timer
			irparams.timer = 0;
		}
		break;
	}

	if (irparams.blinkflag) {
		if (irdata == MARK) {
			BLINKLED_ON();  // turn pin 13 LED on
		}
		else {
			BLINKLED_OFF();  // turn pin 13 LED off
		}
	}
}

void SLifiReceiver::resume() {
	irparams.rcvstate = STATE_IDLE;
	irparams.rawlen = 0;
}



// Decodes the received IR message
// Returns 0 if no data ready, 1 if data ready.
// Results of decoding are stored in results
int SLifiReceiver::decode(decode_results *results) {
	results->rawbuf = irparams.rawbuf;
	results->rawlen = irparams.rawlen;
	if (irparams.rcvstate != STATE_STOP) {
		return ERR;
	}
#ifdef DEBUG
	Serial.println("Attempting SAMSUNG decode");
#endif
	if (decodeResult(results)) {
		return DECODED;
	}
	// decodeHash returns a hash on any input.
	// Thus, it needs to be last in the list.
	// If you add any decodes, add them before this.
	if (decodeHash(results)) {
		return DECODED;
	}
	// Throw away and start over
	resume();
	return ERR;
}

// SAMSUNGs have a repeat only 4 items long
long SLifiReceiver::decodeResult(decode_results *results) {
#if 0
	long data = 0;
	int offset = 1; // Skip first space
	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], SAMSUNG_HDR_MARK)) {
		return ERR;
	}
	offset++;
	// Check for repeat
	if (irparams.rawlen == 4 &&
		MATCH_SPACE(results->rawbuf[offset], SAMSUNG_RPT_SPACE) &&
		MATCH_MARK(results->rawbuf[offset + 1], SAMSUNG_BIT_MARK)) {
		results->bits = 0;
		results->value = REPEAT;
		//results->decode_type = SAMSUNG;
		return DECODED;
	}
	if (irparams.rawlen < 2 * SAMSUNG_BITS + 4) {
		return ERR;
	}
	// Initial space  
	if (!MATCH_SPACE(results->rawbuf[offset], SAMSUNG_HDR_SPACE)) {
		return ERR;
	}
	offset++;
	for (int i = 0; i < SAMSUNG_BITS; i++) {
		if (!MATCH_MARK(results->rawbuf[offset], SAMSUNG_BIT_MARK)) {
			return ERR;
		}
		offset++;
		if (MATCH_SPACE(results->rawbuf[offset], SAMSUNG_ONE_SPACE)) {
			data = (data << 1) | 1;
		}
		else if (MATCH_SPACE(results->rawbuf[offset], SAMSUNG_ZERO_SPACE)) {
			data <<= 1;
		}
		else {
			return ERR;
		}
		offset++;
	}
	// Success
	results->bits = SAMSUNG_BITS;
	results->value = data;
	//results->decode_type = SAMSUNG;
	return DECODED;
#else 
	long data = 0;
	if (irparams.rawlen < 2 * SONY_BITS + 2) {
		return ERR;
	}
	int offset = 0; // Dont skip first space, check its size

	// Some Sony's deliver repeats fast after first
	// unfortunately can't spot difference from of repeat from two fast clicks
	if (results->rawbuf[offset] < SONY_DOUBLE_SPACE_USECS) {
		// Serial.print("IR Gap found: ");
		results->bits = 0;
		results->value = REPEAT;
		//results->decode_type = SANYO;
		return DECODED;
	}
	offset++;

	// Initial mark
	if (!MATCH_MARK(results->rawbuf[offset], HEADER_MARK)) {
		return ERR;
	}
	offset++;

	while (offset + 1 < irparams.rawlen) {
		if (!MATCH_SPACE(results->rawbuf[offset], SONY_HDR_SPACE)) {
			break;
		}
		offset++;
		if (MATCH_MARK(results->rawbuf[offset], SONY_ONE_MARK)) {
			data = (data << 1) | 1;
		}
		else if (MATCH_MARK(results->rawbuf[offset], SONY_ZERO_MARK)) {
			data <<= 1;
		}
		else {
			return ERR;
		}
		offset++;
	}

	// Success
	results->bits = (offset - 1) / 2;
	if (results->bits < 12) {
		results->bits = 0;
		return ERR;
	}
	results->value = data;
	//results->decode_type = SONY;
	return DECODED;
#endif
}

/* -----------------------------------------------------------------------
* hashdecode - decode an arbitrary IR code.
* Instead of decoding using a standard encoding scheme
* (e.g. Sony, NEC, RC5), the code is hashed to a 32-bit value.
*
* The algorithm: look at the sequence of MARK signals, and see if each one
* is shorter (0), the same length (1), or longer (2) than the previous.
* Do the same with the SPACE signals.  Hszh the resulting sequence of 0's,
* 1's, and 2's to a 32-bit value.  This will give a unique value for each
* different code (probably), for most code systems.
*
* http://arcfn.com/2010/01/using-arbitrary-remotes-with-arduino.html
*/

// Compare two tick values, returning 0 if newval is shorter,
// 1 if newval is equal, and 2 if newval is longer
// Use a tolerance of 20%
int SLifiReceiver::compare(unsigned int oldval, unsigned int newval) {
	if (newval < oldval * .8) {
		return 0;
	}
	else if (oldval < newval * .8) {
		return 2;
	}
	else {
		return 1;
	}
}

// Use FNV hash algorithm: http://isthe.com/chongo/tech/comp/fnv/#FNV-param
#define FNV_PRIME_32 16777619
#define FNV_BASIS_32 2166136261

/* Converts the raw code values into a 32-bit hash code.
* Hopefully this code is unique for each button.
* This isn't a "real" decoding, just an arbitrary value.
*/
long SLifiReceiver::decodeHash(decode_results *results) {
	// Require at least 6 samples to prevent triggering on noise
	if (results->rawlen < 6) {
		return ERR;
	}
	long hash = FNV_BASIS_32;
	for (int i = 1; i + 2 < results->rawlen; i++) {
		int value = compare(results->rawbuf[i], results->rawbuf[i + 2]);
		// Add value into the hash
		hash = (hash * FNV_PRIME_32) ^ value;
	}
	results->value = hash;
	results->bits = 32;
	//results->decode_type = UNKNOWN;
	return DECODED;
}
