#include <SPI.h>
#include "printf.h"
#include "RF24.h"

RF24 radio(9, 10); // using pin 9 for the CE pin, and pin 10 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};

bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

char payload[32] = {"testing"};

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(32);
  
  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0
  radio.stopListening();  // put radio in TX mode

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

} // setup

void loop() {
  unsigned long start_timer = micros();                    // start the timer
  bool report = radio.write(&payload, 32);      // transmit & save the report
  unsigned long end_timer = micros();  // end the timer
  
  if (report) {
   Serial.print(F("Transmission successful! "));          // payload was delivered
   Serial.print(F("Time to transmit = "));
   Serial.print(end_timer - start_timer);                 // print the timer result
   Serial.print(F(" us. Sent: "));
   Serial.println(payload);                               // print payload sent   
  } else {
    Serial.println(F("Transmission failed or timed out")); // payload was not delivered
  }

  // to make this example readable in the serial monitor
  delay(1000);  // slow transmissions down by 1 second
} // loop
