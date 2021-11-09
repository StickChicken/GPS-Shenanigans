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
 bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
  
 void setup() {
  
   Serial.begin(115200);
   radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  
   // set the TX address of the RX node into the TX pipe
   radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0
   radio.stopListening();  // put radio in TX mode
   
  
   // For debugging info
   // printf_begin();             // needed only once for printing details
   radio.printDetails();       // (smaller) function that prints raw register values
   // radio.printPrettyDetails(); // (larger) function that prints human readable data
  
 } // setup

 void write(String msg) {
    radio.write(msg, sizeof(msg));
    delay(1000);
 }
 
 void loop() {
    write("test");
 } // loop
