//#define DEBUG_MODE

//#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include "C:/Personal/nrf24stuff/temp_sender/payload.h"
#include "C:/Personal/nrf24stuff/temp_sender/debug_stuff.h"
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_VEML6075.h"
#include <avr/wdt.h>
#include "tinySHT2x.h"

#define TEMP_VCC_PIN 9
#define UV_VCC_PIN 10
#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 2
#define NODE_ID 1
#define DHT_PIN 5

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

Adafruit_VEML6075 uv = Adafruit_VEML6075();

uint32_t delayMS;

tinySHT2x sht;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7); // using pin 7 for the CE pin, and pin 8 for the CSN pin

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

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0) ;
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void setup() {

#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("Debug mode enabled");
#endif

  //Wire.begin();
  //Wire.setWireTimeout(100, true);

  pinMode(TEMP_VCC_PIN, OUTPUT);    // sets the digital pin 13 as output
  pinMode(UV_VCC_PIN, OUTPUT);    // sets the digital pin 13 as output
  pinMode(VCC_PIN, INPUT);
  Debugln("Ready...");

  delayMS = 100;


} // setup

//float SI7021_temperature = NAN;
//float SI7021_humidity = NAN;
void readTemp(void)
{
  sht.begin();

  //#ifdef DEBUG_MODE
  //#else
  //  delayMS = 2000;
  //#endif
  payload.humidity = NAN;
  payload.temp = NAN;

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  int success = 0;
  int retries = 0;
  for (retries; retries < MAX_TEMP_RETRIES; retries++)
  {

    payload.temp = sht.getTemperature();
    payload.humidity = sht.getHumidity();
    Debug("Temp: ");
    Debugln(payload.temp);
    Debug("Hum: ");
    Debugln(payload.humidity);
    if (payload.temp != NAN)
      break;

    Debug("Retries: ");
    Debug(retries);
  }
}

void readUV_VEML6075()
{
  if (! uv.begin()) {
    Debugln("Didn't find VEML6075");
  }
  else
  {

    byte failureCode = 0;
    payload.uv_a = 0;
    payload.uv_b = 0;
    payload.uv_index = 0;

    Debugln("=Reading VEML6075=");

    // Set the integration constant
    uv.setIntegrationTime(VEML6075_100MS);
    // Set the high dynamic mode
    uv.setHighDynamic(true);
    // Set the mode
    uv.setForcedMode(false);

    // Set the calibration coefficients
    // TODO: https://learn.sparkfun.com/tutorials/qwiic-uv-sensor-veml6075-hookup-guide/all read raw values instead
    uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                       2.95, 1.74,  // UVB_C and UVB_D coefficients
                       0.001461, 0.002591); // UVA and UVB responses

    payload.uv_a = uv.readUVA();
    payload.uv_b = uv.readUVB();
    payload.uv_index = uv.readUVI();
    Debug("UVA: "); Debugln(payload.uv_a);
    Debug("UVB: "); Debugln(payload.uv_b);
    Debug("UV Index: "); Debugln(payload.uv_index);

  }
}


void loop() {

  wdt_enable(WDTO_8S);
  digitalWrite(TEMP_VCC_PIN, HIGH); // sets the digital pin on to power AM2301
  digitalWrite(UV_VCC_PIN, HIGH); // sets the digital pin on to power UV & Light sensor
  //for (int il = 0; il < 20;il++)
  //  delay(1000);

  Debug("Reading voltage ");
  payload.voltage = readVcc();
  //payload.voltage = readVccINA219();
  Debugln(payload.voltage);

  readUV_VEML6075();

  Debugln("Reading temp");
  readTemp();

  int p = sizeof(payload);
#ifdef DEBUG_MODE
  Debugln("");
  Debug("Size of payload is ");
  Debugln(p);
  if (p > 32) Debugln("!!Warning!! invalid payload size");
#endif

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
    radio.setPayloadSize(p); // float datatype occupies 4 bytes

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

    Debugln("Payload size: ");
    Debugln(p);

    payload.payloadID = 0;
    radio.powerUp();

    unsigned long start_timer = micros();                    // start the timer
    radio.writeBlocking(&payload, p, 2000);      // transmit & save the report
    bool report = radio.txStandBy(2000);
    unsigned long end_timer = micros();                      // end the timer

    if (report) {
      Debug("Time to transmit = ");
      Debug(end_timer - start_timer);                 // print the timer result
      Debug(" us. Sent: ");
      Debugln(payload.temp);                               // print payload sent
      success = 1;
    } else {
      success = 0;
      Debugln("Transmission failed or timed out"); // payload was not delivered
      //radio.printPrettyDetails(); // (larger) function that prints human readable data`
    }
  }
  delay(100);

  radio.powerDown();
  digitalWrite(TEMP_VCC_PIN, LOW); // sets the digital pin 13 on
  digitalWrite(UV_VCC_PIN, LOW); // sets the digital pin on to power UV & Light sensor

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
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  //    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
  //                  SPI_OFF, USART0_OFF, TWI_OFF);



} // loop
