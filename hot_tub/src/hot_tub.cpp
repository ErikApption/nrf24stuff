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

#define SLEEP_CYCLES_SUCCESS 75  // 75 = 10 minutes on success -
#define SLEEP_CYCLES 10          // 80 seconds otherwise
#define SENSOR_ERROR_TEMP -127.0 // Temperature value indicating sensor error
#define CONVERSION_TIMEOUT 1000  // Timeout for sensor conversion in milliseconds

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
  delay(1000);
  
  // Verify OneWire pin configuration
  Debug(F("OneWire bus pin: "));
  Debugln(ONE_WIRE_BUS);

  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  // initialize the transceiver on the SPI bus

  // First, try a raw OneWire scan to see if any devices respond
  Debugln(F("Scanning OneWire bus..."));
  oneWire.reset_search();
  uint8_t addr[8];
  int foundDevices = 0;
  
  while (oneWire.search(addr)) {
    foundDevices++;
    Debug(F("Found device #"));
    Debug(foundDevices);
    Debug(F(" with address: "));
    for (int i = 0; i < 8; i++) {
#ifdef DEBUG_MODE
      if (addr[i] < 16) Serial.print("0");
      Serial.print(addr[i], HEX);
      if (i < 7) Serial.print(" ");
#endif
    }
#ifdef DEBUG_MODE
    Serial.println();
#endif
    
    // Check CRC
    if (OneWire::crc8(addr, 7) != addr[7]) {
      Debugln(F("  CRC is not valid!"));
    } else {
      Debugln(F("  CRC is valid"));
    }
  }
  
  Debug(F("OneWire scan found "));
  Debug(foundDevices);
  Debugln(F(" device(s)"));
  
  // Now try DallasTemperature library
  sensor.begin();

  // Check if DS18B20 sensor is connected
  int deviceCount = sensor.getDeviceCount();
  Debug(F("DS18B20 device count: "));
  Debugln(deviceCount);

  if (deviceCount == 0)
  {
    Debugln(F("ERROR: No DS18B20 sensor found! Check wiring and pull-up resistor."));
    sensorReady = 0;
    payload.temp = SENSOR_ERROR_TEMP;
  }
  Debugln(F("Requesting temperatures..."));
  sensor.requestTemperatures();

  // Add timeout to prevent infinite loop
  unsigned long conversionStart = millis();
  bool conversionComplete = false;

  while (!sensor.isConversionComplete())
  {
    if (millis() - conversionStart > CONVERSION_TIMEOUT)
    {
      Debugln(F("ERROR: DS18B20 conversion timeout! Sensor may be damaged or disconnected."));
      sensorReady = 0;
      conversionComplete = false;
      break;
    }
    delay(10); // Small delay to prevent tight loop
  }

  if (sensorReady && sensor.isConversionComplete())
  {
    conversionComplete = true;
    Debugln(F("Conversion complete, reading temperature..."));
  }

  if (conversionComplete)
  {
    payload.temp = sensor.getTempCByIndex(0);

    // Validate temperature reading (DS18B20 returns -127 or 85 on error)
    if (payload.temp == SENSOR_ERROR_TEMP || payload.temp == 85.0 ||
        payload.temp < -55.0 || payload.temp > 125.0)
    {
      Debug(F("ERROR: Invalid temperature reading: "));
      Debugln(payload.temp);
      Debugln(F("This usually means sensor error or incorrect reading."));
      sensorReady = 0;
      payload.temp = SENSOR_ERROR_TEMP;
    }
  }
  else
  {
    payload.temp = SENSOR_ERROR_TEMP;
  }

  // payload.voltage = analogRead(A1);
  payload.nodeID = NODE_ID;
  Debug("Temp: ");
  Debugln(payload.temp);

  if (!sensorReady)
  {
    Debugln(F("WARNING: Proceeding with error temperature value"));
  }

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

  digitalWrite(TS_PIN, LOW); // sets the digital pin 13 on
  delay(100);

  Debugln("Starting sleep");
#ifdef DEBUG_MODE
  int sleep_time = 1;
  for (int il = 0; il < sleep_time; il++)
    delay(1000);
#else
  int sleep_time = (success == 1) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
  for (int il = 0; il < sleep_time; il++)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
#endif

  Debugln("Sleep completed");
  // to make this example readable in the serial monitor

} // loop
