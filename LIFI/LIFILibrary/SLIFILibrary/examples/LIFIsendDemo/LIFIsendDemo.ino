#include "SLifiSender.h"
//#include "SLifiGlobal.h"

SLifiSender sender;

#define TAG_SAFE "SAFE"
#define TAG_DANGER "DNGR"

#define TYPE_ULONG 1<<0

unsigned long gData = 0;

void char2type(const char *src, void *dst, int type)
{
	int rec = 0;
	switch (type) {
	case TYPE_ULONG:
		rec = sizeof(unsigned long); // 4
		for (int i = 0; i <rec; i++) {
			*((unsigned long *)dst) |= (*(src + i) << 8*(rec-1-i));
		}
		break;
	default:
		// do something
		break;
	}
}

unsigned long tag2ulong(const char *src)
{
	unsigned long retval = 0;
	char2type(src, &retval, TYPE_ULONG);
	return retval;
}

void setup()
{
	Serial.begin(9600);
	gData = tag2ulong(TAG_DANGER);
}

void loop() 
{
	for (int i = 0; i < 3; i++)  {
		sender.send(gData, 12);	
	}

}
