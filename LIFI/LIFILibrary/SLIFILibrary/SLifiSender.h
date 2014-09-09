#pragma once


// Only used for testing; can remove virtual for shorter code
#ifdef TEST
#define VIRTUAL virtual
#else
#define VIRTUAL
#endif

class SLifiSender
{
public:
	SLifiSender() {}
	void send(unsigned long data, int nbits);
	void enableIROut(int khz);
	void mark(int usec);
	void space(int usec);
}
;
