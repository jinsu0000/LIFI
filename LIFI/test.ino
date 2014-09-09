
#include "Thread.h"
#include "ThreadController.h"

#include "SLifiGlobal.h"
#include "SLifiReceiver.h"
#include "SLifiSender.h"

#define TAG_SAFE "SAFE"
#define TAG_DANGER "DNGR"

#define ABS(x) ((x)>0? (x): -(x))

#define TYPE_ULONG 1<<0


/// RECEIVER
int RECV_PIN = 11;
SLifiReceiver receiver(RECV_PIN);
decode_results results;


/// SENDER ///
SLifiSender sender;
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


// THREAD (TEST)

ThreadController gThreadConroller = ThreadController();

Thread gTXThread = Thread();
Thread gRXThread = Thread();

// callback for gTXThread
void txCallback()
{
 for (int i = 0; i < 3; i++)  {
    sender.send(gData, 12); 
  }
}

void rxCallback()
{
  if (receiver.decode(&results)) {
    Serial.println(results.value, HEX);
    receiver.resume(); // Receive the next value
  }
}




void setup(){
  Serial.begin(9600);
  receiver.enableIRIn(); // Start the receiver
  gData = tag2ulong(TAG_DANGER);

   gTXThread.onRun(txCallback);
   gTXThread.setInterval(0);
  
   gRXThread.onRun(rxCallback);
   gRXThread.setInterval(0);

  gThreadConroller.add(&gTXThread);
  gThreadConroller.add(&gRXThread);
}

void loop(){

  gThreadConroller.run();

}
