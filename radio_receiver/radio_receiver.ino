#include <SPI.h>
#include "printf.h"
#include "RF24.h"
  
// instantiate an object for the nRF24L01 transceiver
RF24 radio(1, 13); // using pin 1 for the CE pin, and pin 13 for the CSN pin
  
// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination
  
// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
  
void setup() {
  
  Serial.begin(115200);
  radio.setPALevel(RF24_PA_MIN);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof("test"));
  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1   
  radio.startListening();
  
  // For debugging info
  // printf_begin();             // needed only once for printing details
  radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
  
} // setup
  
void loop() {
    uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      radio.read(&payload, bytes);            // fetch payload from FIFO
      Serial.print(F("Received "));       
      Serial.print(bytes);                    // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);                     // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);                // print the payload's value
    }
  } 
} // loop
