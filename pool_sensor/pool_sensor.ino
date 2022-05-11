

//#define DEBUG_MODE

#define WDT_ENABLED

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <OneWire.h>
#include <DS18B20.h>
#include <payload.h>
#include <debug_stuff.h>
#include <Vcc.h>
#include <avr/wdt.h>

#define ONE_WIRE_BUS 2
#define POWER_PIN 10
#define MAX_RADIO_RETRIES 10
#define NODE_ID 0  //pool

#define TX_TIMEOUT 2000

#define SLEEP_CYCLES_SUCCESS 100  //20 minutes on success -
#define SLEEP_CYCLES 10           //80 seconds otherwise

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7);  // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
const uint8_t receiver_address[6] = "1Node";
const uint8_t nodes[][6] = { "2Node", "3Node", "4Node", "5Node" };
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

// Used to control whether this node is sending or receiving

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
//float payload = 1.234;
BasePayLoad payload;

void setup() {

#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif

#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
#endif

  pinMode(POWER_PIN, OUTPUT);  // sets the digital pin 13 as output

  // print example's introductory prompt
  Debugln(F("DS18B20 sensor"));

}  // setup

void loop() {

  Debug(F("Reading voltage "));
  payload.voltage = Vcc::measure();
  Debugln(payload.voltage);

  digitalWrite(POWER_PIN, HIGH);  // sets the digital pin 13 on
  //for (int il = 0; il < 20;il++)
  //  delay(1000);
  radio.powerUp();


  int sensorReady = 1;
  for (int si = 0; si < 10; si++) {
    if (!sensor.begin()) {
      Debugln(F("sensor hardware is not responding!!"));
      sensorReady = 0;
      delay(100);
    } else {

      // This device is a TX node
      sensor.requestTemperatures();
      while (!sensor.isConversionComplete())
        ;  // wait until sensor is ready
      payload.temp = sensor.getTempC();
      //payload.voltage = analogRead(A1);
      payload.nodeID = NODE_ID;
      Debug("Temp: ");
      Debugln(payload.temp);
      break;
    }
  }

  int radioReady = 1;
  int success = 0;
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Debugln(F("radio hardware is not responding!!"));
    radioReady = 0;
  } else {

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
    //radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
    radio.enableDynamicPayloads();

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(nodes[NODE_ID]);  // always uses pipe 0

    // additional setup specific to the node's role
    radio.stopListening();  // put radio in TX mode

    // from https://nrf24.github.io/RF24/examples_2MulticeiverDemo_2MulticeiverDemo_8ino-example.html
    radio.setRetries(((NODE_ID * 3) % 12) + 3, 15);  // maximum value is 15 for both args
                                                     // For debugging info
                                                     //printf_begin();             // needed only once for printing details
                                                     //radio.printDetails();       // (smaller) function that prints raw register values
                                                     //radio.printPrettyDetails(); // (larger) function that prints human readable data


    payload.payloadID = 0;
    delay(10);    
    bool report = false;
    long transTime = 0;
    for (int rRetries = 0; rRetries < 10;rRetries++)
    {
      unsigned long start_timer = micros();                        // start the timer
      radio.writeFast(&payload, sizeof(payload));  // transmit & save the report
      report = radio.txStandBy(TX_TIMEOUT);
      Debugln("Payload size: ");
      Debugln(sizeof(payload));
      unsigned long end_timer = micros();  // end the timer
      transTime = end_timer - start_timer;
      if (report) break;
      Debugln("Transmission failed - retrying");
      delay(100);
    }

    if (report) {
      Debug(F("Transmission successful! "));  // payload was delivered
      Debug(F("Time to transmit = "));
      Debug(transTime);  // print the timer result
      Debug(F(" us. Sent: "));
      Debugln(payload.temp);  // print payload sent
      success = 1;
    } else {
      success = 0;
      Debugln(F("Transmission failed or timed out"));  // payload was not delivered
      delay(10);
      //radio.printPrettyDetails(); // (larger) function that prints human readable data
    }
  }

  radio.powerDown();
  digitalWrite(POWER_PIN, LOW);  // sets the digital pin 13 on

  wdt_reset();

  // disable wdt while sleeping
  wdt_disable();

  Debugln("Starting sleep");
#ifdef DEBUG_MODE
  int sleep_time = 1;
#else
  int sleep_time = (success == 1) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
#endif

  for (int il = 0; il < sleep_time; il++)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  Debugln("Sleep completed");
#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif


}  // loop
