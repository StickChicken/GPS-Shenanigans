#include <SPI.h>
#include "printf.h"
#include "RF24.h"

RF24 radio(9, 10); // using pin 7 for the CE pin, and pin 8 for the CSN pin

uint8_t address[][6] = {"1Node", "2Node"};

char payload[]= "abcdefghijklmnopqrstuvwxyz-abcdefghijkl";

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
  radio.setPayloadSize(32); // float datatype occupies 4 bytes

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[0]);     // always uses pipe 0

  // additional setup specific to the node's role
  radio.stopListening();  // put radio in TX mode

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data

} // setup

void loop() {
  radioSend(payload);
  // to make this example readable in the serial monitor
  delay(1000);  // slow transmissions down by 1 second

} // loop

void radioSend(char msg[]) {
  char toWrite[32];
  if (strlen(msg) <= 32) {
    strncpy(toWrite, msg, 32);
    radio.write(&toWrite, 32);
  } else {
    for (int i = 0; i < strlen(msg); i += 32) {
      strncpy(toWrite, msg + i, 32);
      radio.write(&toWrite, strlen(toWrite));
    }
  }
}
