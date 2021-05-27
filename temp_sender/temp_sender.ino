
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <OneWire.h>
#include <DS18B20.h>
#include "payload.h"
#include "debug_stuff.h"

#define ONE_WIRE_BUS 6
#define TS_PIN 9
#define MAX_RADIO_RETRIES 10
#define NODE_ID 0


//#define DEBUG_MODE
#define SLEEP_CYCLES_SUCCESS 150 //20 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t receiver_address[6] = "1Node";
uint8_t nodes[][6] = {"2Node", "3Node", "4Node", "5Node"};
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
  #ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  #endif

  pinMode(TS_PIN, OUTPUT);    // sets the digital pin 13 as output

  // print example's introductory prompt
  Debugln(F("RF24/examples/GettingStarted"));

} // setup

void loop() {

  digitalWrite(TS_PIN, HIGH); // sets the digital pin 13 on
  //for (int il = 0; il < 20;il++)
  //  delay(1000);
  delay(100);

  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Debugln(F("radio hardware is not responding!!"));
    radioReady = 0;
  }
  else
  {

    // role variable is hardcoded to RX behavior, inform the user of this
    //Debugln(F("*** PRESS 'T' to begin transmitting to the other node"));
    radio.setChannel(5);
    //radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(nodes[NODE_ID]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, receiver_address); // using pipe 1

    // additional setup specific to the node's role
    radio.stopListening();  // put radio in TX mode

    // from https://nrf24.github.io/RF24/examples_2MulticeiverDemo_2MulticeiverDemo_8ino-example.html
    radio.setRetries(((NODE_ID * 3) % 12) + 3, 15); // maximum value is 15 for both args
    // For debugging info
    //printf_begin();             // needed only once for printing details
    //radio.printDetails();       // (smaller) function that prints raw register values
    //radio.printPrettyDetails(); // (larger) function that prints human readable data

    if (!sensor.begin())
    {
      Debugln(F("sensor hardware is not responding!!"));
      sensorReady = 0;
    }
    else
    {

      // This device is a TX node
      sensor.requestTemperatures();
      while (!sensor.isConversionComplete());  // wait until sensor is ready
      payload.temp = sensor.getTempC();
      payload.voltage = analogRead(A1);
      payload.nodeID = NODE_ID;
      Debug("Temp: ");
      Debugln(payload.temp);
      
      for (int retries = 0; retries < MAX_RADIO_RETRIES; retries++)
      {
        Debug("Retries: ");
        Debugln(retries);
        payload.payloadID = retries;
        radio.powerUp();
        unsigned long start_timer = micros();                    // start the timer
        bool report = radio.write(&payload, sizeof(payload));      // transmit & save the report
        Debugln("Payload size: ");
        Debugln(sizeof(payload));
        unsigned long end_timer = micros();                      // end the timer

        if (report) {
          Debug(F("Transmission successful! "));          // payload was delivered
          Debug(F("Time to transmit = "));
          Debug(end_timer - start_timer);                 // print the timer result
          Debug(F(" us. Sent: "));
          Debugln(payload.temp);                               // print payload sent
          success = 1;
          break;
        } else {
          success = 0;
          Debugln(F("Transmission failed or timed out")); // payload was not delivered
          delay(1000);
          //radio.printPrettyDetails(); // (larger) function that prints human readable data
        }
      }
      delay(100);
    }
  }
  radio.powerDown();
  digitalWrite(TS_PIN, LOW); // sets the digital pin 13 on
  delay(100);

  Debugln("Starting sleep");
  #ifdef DEBUG_MODE
  int sleep_time = 0;
  #else
  int sleep_time = (success == 1)?SLEEP_CYCLES_SUCCESS:SLEEP_CYCLES; 
  #endif
  
  for (int il = 0; il < sleep_time; il++)
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_OFF, USART0_OFF, TWI_OFF);

  Debugln("Sleep completed");
  // to make this example readable in the serial monitor



} // loop
