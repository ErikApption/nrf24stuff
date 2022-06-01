//#define DEBUG_MODE
#include <BH1750.h>
#define WDT_ENABLED
//#include <SPI.h>
#include "printf.h"
#include "RF24.h"

//#include <LowPower.h>
#include <Vcc.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <avr/wdt.h>
#include <debug_stuff.h>
#include <payload.h>

#ifdef __LGT8F__
#include <PMU.h>
#else
#include <LowPower.h>
#endif

#ifdef DEBUG
  #define SLEEP SLEEP_1S
#else
  #define SLEEP SLEEP_8S
#endif

#define SENSOR_VCC_PIN 2
#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 2
#define NODE_ID 3

#define SLEEP_CYCLES_SUCCESS 37 //5 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

#define I2C_TIMEOUT 500


uint32_t delayMS;


// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7); // using pin 6 for the CE pin, and pin 7 for the CSN pin - weather station setup
//RF24 radio(9, 10); // using pin 6 for the CE pin, and pin 7 for the CSN pin - keyestudio nano shield

// Let these addresses be used for the pair
uint8_t receiver_address[6] = "1Node";
uint8_t nodes[][6] = {"2Node", "3Node", "4Node", "5Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

BH1750 lightSensor;

BH1750Payload payload;

void setup() {

#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif

#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("Debug mode enabled");
#endif

  pinMode(SENSOR_VCC_PIN, OUTPUT);
  pinMode(VCC_PIN, INPUT);
  Debugln("Ready...");

  delayMS = 100;

  Wire.setTimeout(1000);
  //Wire.setClock(10000);  
  Wire.begin();
  //Wire.setTimeout(I2C_TIMEOUT);


} // setup


#define READ_RETRIES 5
bool readLux()
{

  payload.lux = 0;
  digitalWrite(SENSOR_VCC_PIN, HIGH); // sets the digital pin 13 on
  delay(10);

  lightSensor.begin();
  delay(200);

  unsigned long start_timer = micros();
  for (int retries = 0; retries < READ_RETRIES; retries++)
  {
      payload.lux += lightSensor.readLightLevel();
  }
  payload.lux /= READ_RETRIES;

  digitalWrite(SENSOR_VCC_PIN, LOW); // sets the digital pin 13 on
  Debug("Light=");
  Debugln(payload.lux);
  unsigned long end_timer = micros();
  payload.readoutms = end_timer-start_timer;
  return true;
}

void loop() {

  Wire.setTimeout(1000);
  //Wire.setClock(10000);  

  wdt_reset();
#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif

  Debug("Reading voltage ");
  payload.voltage = Vcc::measure();
  //payload.voltage = readA0_Voltage_mV();
  Debugln(payload.voltage);

  Debug("Reading Lux ");  
  bool luxAvailable = readLux();

  Debugln("Sending Radio packet");
  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  //delay(100);
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Debugln("radio hardware is not responding!!");
    radioReady = 0;
  }
  else
  {
    Debugln("++++++++++++ Radio initialized");

    radio.setChannel(5);
    //radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
#ifdef DEBUG_MODE
    radio.setPALevel(RF24_PA_LOW);
#else
    radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
#endif

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit a float
    //radio.setPayloadSize(p); // float datatype occupies 4 bytes
    radio.enableDynamicPayloads();

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(nodes[NODE_ID]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    //radio.openReadingPipe(1, receiver_address); // using pipe 1

    // additional setup specific to the node's role
    radio.stopListening();  // put radio in TX mode

    // from https://nrf24.github.io/RF24/examples_2MulticeiverDemo_2MulticeiverDemo_8ino-example.html
    radio.setRetries(((NODE_ID * 3) % 12) + 3, 15); // maximum value is 15 for both args
    // For debugging info
    //printf_begin();             // needed only once for printing details
    //radio.printDetails();       // (smaller) function that prints raw register values
    //radio.printPrettyDetails(); // (larger) function that prints human readable data

    // This device is a TX node
    //    payload.temp = sensor.getTempC(); TOFIX
    //    payload.voltage = analogRead(A1);

    payload.nodeID = NODE_ID;
    payload.payloadID = 3;

    radio.powerUp(); //This will take up to 5ms for maximum compatibility
    delay(10);
    //radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
    unsigned long start_timer = micros();                    // start the timer

    radio.writeFast(&payload, sizeof(payload));
    bool fifoSuccess = radio.txStandBy(2000);// Using extended timeouts, returns 1 if success. Retries failed payloads for 1 seconds before returning 0.
    unsigned long end_timer = micros();                      // end the timer

    if (fifoSuccess) {
      Debug("Time to transmit = ");
      Debug(end_timer - start_timer);                 // print the timer result
      Debug(" us. Sent ");
      success = 1;
    } else {
      success = 0;
      Debug("Transmission failed or timed out - fifo:"); // payload was not delivered
      Debugln(fifoSuccess);

      //radio.printPrettyDetails(); // (larger) function that prints human readable data`
    }
  }
  delay(100);
  radio.powerDown();

  //delay(100);
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
  {

#ifdef __LGT8F__
    Debugln("entering LGT8F sleep");                               // print payload sent

    bitClear(ADCSRA, ADEN);
    PMU.sleep(PM_POWERDOWN, SLEEP);
    bitSet(ADCSRA, ADEN);      

#else
    Debugln("entering arduino sleep");                               // print payload sent
    LowPower.powerDown(SLEEP, ADC_OFF, BOD_ON);
#endif
    

  }
  Debugln("End sleep");  

#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif


} // loop
