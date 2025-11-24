#include <Arduino.h>

#define DEBUG_MODE

#include <SPI.h>
#include "RF24.h"
// Low-Power
#include <LowPower.h>
#include <OneWire.h>
// DallasTemperature
#include <DallasTemperature.h>
#include <nrfdata.h>

#define ONE_WIRE_BUS 6
#define TS_PIN 9
#define MAX_RADIO_RETRIES 10
#define NODE_ID 2 // hot tub

#define TX_TIMEOUT 2000

#define SLEEP_CYCLES_SUCCESS 75 // 75 = 10 minutes on success -
#define SLEEP_CYCLES 10         // 80 seconds otherwise

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);
// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
const uint8_t receiver_address[6] = "1Node";
const uint8_t nodes[][6] = {"2Node", "3Node", "4Node", "5Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

// Used to control whether this node is sending or receiving

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
// float payload = 1.234;
BasePayLoad payload;

void setup()
{
#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial)
  {
    // some boards need to wait to ensure access to serial over USB
  }
#endif

  pinMode(TS_PIN, OUTPUT); // sets the digital pin 13 as output

  // print example's introductory prompt
  Debugln(F("RF24/examples/GettingStarted"));

} // setup

long readVcc()
{
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2);            // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC))
    ; // measuring

  uint8_t low = ADCL;  // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result;              // Vcc in millivolts
}

void loop()
{

  Debug(F("Reading voltage "));
  payload.voltage = readVcc();
  Debugln(payload.voltage);

  digitalWrite(TS_PIN, HIGH); // sets the digital pin 13 on
  // for (int il = 0; il < 20;il++)
  //   delay(1000);
  delay(100);

  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  // initialize the transceiver on the SPI bus
  if (!radio.begin())
  {
    Debugln(F("radio hardware is not responding!!"));
    radioReady = 0;
  }
  else
  {

    // role variable is hardcoded to RX behavior, inform the user of this
    // Debugln(F("*** PRESS 'T' to begin transmitting to the other node"));
    radio.setChannel(5);
    // radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
    //  Set the PA Level low to try preventing power supply related problems
    //  because these examples are likely run with nodes in close proximity to
    //  each other.
    radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    // radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
    radio.enableDynamicPayloads();

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(nodes[NODE_ID]); // always uses pipe 0

    // additional setup specific to the node's role
    radio.stopListening(); // put radio in TX mode

    // from https://nrf24.github.io/RF24/examples_2MulticeiverDemo_2MulticeiverDemo_8ino-example.html
    radio.setRetries(((NODE_ID * 3) % 12) + 3, 15); // maximum value is 15 for both args
    // For debugging info
    // printf_begin();             // needed only once for printing details
    // radio.printDetails();       // (smaller) function that prints raw register values
    // radio.printPrettyDetails(); // (larger) function that prints human readable data

    sensor.begin();

    // This device is a TX node
    sensor.requestTemperatures();
    while (!sensor.isConversionComplete())
      ; // wait until sensor is ready
    payload.temp = sensor.getTempCByIndex(0);
    // payload.voltage = analogRead(A1);
    payload.nodeID = NODE_ID;
    Debug("Temp: ");
    Debugln(payload.temp);

    payload.payloadID = 0;
    radio.powerUp();
    unsigned long start_timer = micros();                       // start the timer
    radio.writeBlocking(&payload, sizeof(payload), TX_TIMEOUT); // transmit & save the report
    bool report = radio.txStandBy(TX_TIMEOUT);
    Debugln("Payload size: ");
    Debugln(sizeof(payload));
    unsigned long end_timer = micros(); // end the timer

    if (report)
    {
      Debug(F("Transmission successful! ")); // payload was delivered
      Debug(F("Time to transmit = "));
      Debug(end_timer - start_timer); // print the timer result
      Debug(F(" us. Sent: "));
      Debugln(payload.temp); // print payload sent
      success = 1;
    }
    else
    {
      success = 0;
      Debugln(F("Transmission failed or timed out")); // payload was not delivered
      delay(1000);
      // radio.printPrettyDetails(); // (larger) function that prints human readable data
    }

    delay(100);
  }

  radio.powerDown();
  digitalWrite(TS_PIN, LOW); // sets the digital pin 13 on
  delay(100);

  Debugln("Starting sleep");
#ifdef DEBUG_MODE
  int sleep_time = 0;
#else
  int sleep_time = (success == 1) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
#endif

  for (int il = 0; il < sleep_time; il++)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);

  Debugln("Sleep completed");
  // to make this example readable in the serial monitor

} // loop
