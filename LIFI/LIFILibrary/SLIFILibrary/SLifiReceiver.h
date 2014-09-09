#pragma once

#ifndef ILifi_h
#define ILifi_h


#define DEBUG
#define TEST

// Results returned from the decoder
class decode_results {
public:
	//int decode_type; // NEC, SONY, RC5, UNKNOWN
	//unsigned int panasonicAddress; // This is only used for decoding Panasonic data
	unsigned long value; // Decoded value
	int bits; // Number of bits in decoded value
	volatile unsigned int *rawbuf; // Raw intervals in .5 us ticks
	int rawlen; // Number of records in rawbuf.
};

#define REPEAT 0xffffffff

// main class for receiving IR
class SLifiReceiver
{
public:
	SLifiReceiver(int recvpin);
	void blink13(int blinkflag);
	int decode(decode_results *results);
	void enableIRIn();
	void resume();
private:
	// These are called by decode
	int getRClevel(decode_results *results, int *offset, int *used, int t1);
	long decodeResult(decode_results *results);
	long decodeHash(decode_results *results);
	int compare(unsigned int oldval, unsigned int newval);

}
;


#endif

