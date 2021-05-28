
#define DEBUG_MODE

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <OneWire.h>
#include <DS18B20.h>
#include "C:/Personal/nrf24stuff/temp_sender/payload.h"
#include "C:/Personal/nrf24stuff/temp_sender/debug_stuff.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "Adafruit_SI1145.h"
//#include "SI114X.h"
//#include "si7021.h"
//#include <dhtnew.h>
#include "DHT.h"

#define TS_PIN 9
#define MAX_RADIO_RETRIES 10
#define NODE_ID 3
#define DHT_PIN 5

#define SLEEP_CYCLES_SUCCESS 150 //20 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t receiver_address[6] = "1Node";
uint8_t nodes[][6] = {"2Node", "3Node", "4Node", "5Node", "6Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

// Used to control whether this node is sending or receiving

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
//float payload = 1.234;
PayLoad payload;


void setup() {


  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }

   // initialize the transceiver on the SPI bus
   if (!radio.begin()) {
     Serial.println(F("radio hardware is not responding!!"));
     while (1) {} // hold in infinite loop
   }
  
  Serial.println("Setting up channel...");

  radio.setChannel(5);

  Serial.println("Setting up auto ack...");

  radio.setAutoAck(true);
  //radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
  // Set the PA Level low to try preventing power supply related problems
  // because these examples are likely run with nodes in close proximity to
  // each other.
  //radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  Serial.println("Setting payload...");
  // save on transmission time by setting the radio to only transmit the
  // number of bytes we need to transmit a float
  radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

  Serial.println("Opening reading pipes...");
  // Set the addresses for all pipes to TX nodes
  for (uint8_t i = 0; i < 5; ++i)
    radio.openReadingPipe(i, nodes[i]);
  // set the TX address of the RX node into the TX pipe

  Serial.println("Starting to listen...");
  // additional setup specific to the node's role
  radio.startListening(); // put radio in RX mode

  Serial.print(sizeof(payload));
  Serial.println(" payload");
  Serial.println("-----------------");

} // setup

void loop() {

//  if (!radio.begin()) {
//    Debugln(F("radio hardware is not responding!!"));
//    radioReady = 0;
//  }
//  else
//  {
    uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      radio.read(&payload, bytes);            // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);                    // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);                     // print the pipe number
      Serial.print(F(" from node "));
      Serial.print(payload.nodeID);           // print the payload's origin
      Serial.print(F(". PayloadID: "));
      Serial.println(payload.payloadID);      // print the payload's number
//    }
  }


} // loop
