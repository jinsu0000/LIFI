
#include "SLifiGlobal.h"
#include "SLifiReceiver.h"
#include "SLifiSender.h"


int RECV_PIN = 11;

SLifiReceiver receiver(RECV_PIN);

decode_results results;

void setup()
{
  Serial.begin(9600);
  receiver.enableIRIn(); // Start the receiver
}

void loop() {
  if (receiver.decode(&results)) {
    Serial.println(results.value, HEX);
    receiver.resume(); // Receive the next value
  }
  delay(100);
}
