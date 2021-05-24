
#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <OneWire.h>
#include <DS18B20.h>

#define ONE_WIRE_BUS 6
#define TS_PIN 9
#define MAX_RADIO_RETRIES 10
//#define USE_SERIAL

typedef struct {
  float temp;
  float voltage;
} PayLoad;

OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
// instantiate an object for the nRF24L01 transceiver
RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

// Let these addresses be used for the pair
uint8_t address[][6] = {"1Node", "2Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit
bool radioNumber = 0; // 0 uses address[0] to transmit, 1 uses address[1] to transmit

// Used to control whether this node is sending or receiving

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
//float payload = 1.234;
PayLoad payload;

void setup() {
  #ifdef USE_SERIAL
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  #endif

  pinMode(TS_PIN, OUTPUT);    // sets the digital pin 13 as output

  // print example's introductory prompt
  Debug(F("RF24/examples/GettingStarted"));

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
    Debug(F("radio hardware is not responding!!"));
    radioReady = 0;
  }
  else
  {

    radioNumber = 0; //input == 1;
    Serial.print(F("radioNumber = "));
    Debug((int)radioNumber);

    // role variable is hardcoded to RX behavior, inform the user of this
    //Debug(F("*** PRESS 'T' to begin transmitting to the other node"));
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
    radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

    // additional setup specific to the node's role
    radio.stopListening();  // put radio in TX mode


    // For debugging info
    //printf_begin();             // needed only once for printing details
    //radio.printDetails();       // (smaller) function that prints raw register values
    //radio.printPrettyDetails(); // (larger) function that prints human readable data

    if (!sensor.begin())
    {
      Debug(F("sensor hardware is not responding!!"));
      sensorReady = 0;
    }
    else
    {

      // This device is a TX node
      sensor.requestTemperatures();
      while (!sensor.isConversionComplete());  // wait until sensor is ready
      payload.temp = sensor.getTempC();
      payload.voltage = analogRead(A1);
      Serial.print("Temp: ");
      Debug(payload.temp);
      
      for (int retries = 0; retries < MAX_RADIO_RETRIES; retries++)
      {
        Serial.print("Retries: ");
        Debug(retries);

        radio.powerUp();
        unsigned long start_timer = micros();                    // start the timer
        bool report = radio.write(&payload, sizeof(payload));      // transmit & save the report
        Serial.print("Payload size: ");
        Debug(sizeof(payload));
        unsigned long end_timer = micros();                      // end the timer

        if (report) {
          Serial.print(F("Transmission successful! "));          // payload was delivered
          Serial.print(F("Time to transmit = "));
          Serial.print(end_timer - start_timer);                 // print the timer result
          Serial.print(F(" us. Sent: "));
          Debug(payload.temp);                               // print payload sent
          success = 1;
          break;
        } else {
          success = 0;
          Debug(F("Transmission failed or timed out")); // payload was not delivered
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

  Debug("Starting sleep");
  int sleep_time = (success == 1)?150:10; //20 minutes on success - 80 seconds otherwise
  for (int il = 0; il < sleep_time; il++)
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_OFF, USART0_OFF, TWI_OFF);

  Debug("Sleep completed");
  // to make this example readable in the serial monitor



} // loop

void Debug(String string)
{
  #ifdef USE_SERIAL
  Serial.println(string);
  #endif
}

void Debug(float val)
{
  #ifdef USE_SERIAL
  Serial.println(val);
  #endif
}
