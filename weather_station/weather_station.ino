//#define DEBUG_MODE

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
#include <Adafruit_SI1145.h>
#include <Adafruit_INA219.h>
#include <DHT.h>
#include <DHT_U.h>

#define TS_PIN 9
#define TS_POWER_PIN 4
#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 2
#define NODE_ID 1
#define DHT_PIN 5

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

#define DHTTYPE DHT21

//DHT dht(DHT_PIN, DHTTYPE);

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

Adafruit_SI1145 uv = Adafruit_SI1145();

Adafruit_INA219 ina219;

DHT_Unified dht(DHT_PIN, DHTTYPE);

uint32_t delayMS;

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

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Debugln("------------------------------------");
  Debug  ("Sensor:       "); Debugln(sensor.name);
  Debug  ("Driver Ver:   "); Debugln(sensor.version);
  Debug  ("Unique ID:    "); Debugln(sensor.sensor_id);
  Debug  ("Max Value:    "); Debug(sensor.max_value); Debugln(" lux");
  Debug  ("Min Value:    "); Debug(sensor.min_value); Debugln(" lux");
  Debug  ("Resolution:   "); Debug(sensor.resolution); Debugln(" lux");
  Debugln("------------------------------------");
  Debugln("");
  delay(500);
}

//void configureGY1145Sensor(void)
//{
//  mySI1145.init();
//  //mySI1145.enableHighSignalVisRange();
//  //mySI1145.enableHighSignalIrRange();
//
//  /* choices: PS_TYPE, ALS_TYPE, PSALS_TYPE, ALSUV_TYPE, PSALSUV_TYPE || FORCE, AUTO, PAUSE */
//  mySI1145.enableMeasurements(ALS_TYPE, FORCE);
//
//   /* choose gain value: 0, 1, 2, 3, 4, 5, 6, 7 */
//  mySI1145.setAlsVisAdcGain(0);
//
//  //mySI1145.enableHighResolutionVis();
//  Debugln("SI1145 - forced ALS");
//}

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

long readVccINA219()
{
  if (ina219.begin()) {

    float shuntvoltage = 0;
    float busvoltage = 0;
    float current_mA = 0;
    float loadvoltage = 0;
    float power_mW = 0;

    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);

#ifdef DEBUG_MODE
    Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
    Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
    Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
    Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
#endif
    return (long)(loadvoltage * 1000.0L);
  }
  return -1;
}

long readVccExternal()
{
  digitalWrite(TS_POWER_PIN, HIGH);
  delay(10);
  long val = analogRead (VCC_PIN);
  //Debug(F("Raw voltage="));
  //Debug(val);
  //8.13 = 247
  //8130
  digitalWrite(TS_POWER_PIN, LOW);
  long volt = val * 8130L / 247L;
  return volt;
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureLightSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  //tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */
  //  Debugln("------------------------------------");
  //  Debug  ("Gain:         "); Debugln("Auto");
  //  Debug  ("Timing:       "); Debugln("13 ms");
  //  Debugln("------------------------------------");
}

void setup() {


#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("Debug mode enabled");
#endif

  Debugln(F("Starting Weather Station"));

  Debugln(F("Configuring Transistor Pin"));
  pinMode(TS_PIN, OUTPUT);    // sets the digital pin 13 as output
  pinMode(TS_POWER_PIN, OUTPUT);
  pinMode(VCC_PIN, INPUT);
  Debugln(F("Ready..."));

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
#ifdef DEBUG_MODE
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
#endif
  // Print humidity sensor details.
#ifdef DEBUG_MODE
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.print  (F("Delay:       ")); Serial.print(sensor.min_delay); Serial.println(F("us"));
  Serial.println(F("------------------------------------"));
#endif
  //  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;


} // setup

//float SI7021_temperature = NAN;
//float SI7021_humidity = NAN;
void readTemp(void)
{
  dht.begin();  
  //      mySensor.read();
  //      Debug(mySensor.getHumidity(), 1);
  //      Debug("\t");
  //      Debugln(mySensor.getTemperature(), 1);

  // Print temperature sensor details.

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

    // Delay between measurements.
    //delay(delayMS);
    LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_OFF, USART0_OFF, TWI_OFF);
    
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);

    if (!isnan(event.temperature)) {
      payload.temp = event.temperature;
      Debug("Temperature: ");
      Debug(payload.temp);
      Debugln(" *C");
      success++;
    } else {
      Debugln("Failed to get temprature value.");
    }
    dht.humidity().getEvent(&event);
    if (!isnan(event.temperature)) {
      payload.humidity = event.relative_humidity;
      Debug("Humidity: ");
      Debug(payload.humidity);
      success++;
    } else {
      Debugln("Failed to get humidity value.");
    }
    Debug(" Retries: ");
    Debug(retries);
    if (success == 2) break;

  }

}

void readUVSensor(void) {
  if (! uv.begin()) {
    Debugln("Didn't find Si1145");
  }
  else
  {

    byte failureCode = 0;
    payload.amb_als = 0;
    payload.amb_ir = 0;

    Debugln("=Reading Si1145=");

    Debug("Vis: "); Debugln(uv.readVisible());
    Debug("IR: "); Debugln(uv.readIR());
    float UVindex = uv.readUV();
    payload.uv_index = UVindex;
    //UVindex /= 100.0;
    Debug("UV: ");  Debugln(UVindex);

    for (int i = 0; i < 50; i++) {
      uv.begin();
      //mySI1145.clearAllInterrupts();
      //mySI1145.startSingleMeasurement();
      payload.amb_als += uv.readVisible();
      payload.amb_ir += uv.readIR();
    }
    payload.amb_als /= 50;
    payload.amb_ir /= 50;
    Debug("Ambient Light: ");
    Debugln(payload.amb_als);
    Debug("Infrared Light: ");
    Debugln(payload.amb_ir);
    //  failureCode = mySI1145.getFailureMode();  // reads the response register
    //  if((failureCode&128)){   // if bit 7 is set in response register, there is a failure
    //    handleFailure(failureCode);
    //  }
    Debugln("---------");
  }
}

void readTLSSensor()
{
  Debugln(F("TSL Sensor"));
  if (tsl.begin())
  {
    Debugln(F("Found a TSL2591 sensor"));

    /* Configure the sensor */
    configureLightSensor();

    /* Get a new sensor event */
    sensors_event_t event;
    tsl.getEvent(&event);
    payload.luxMeasure = 0;
    /* Display the results (light is measured in lux) */
    if (event.light)
    {
      payload.luxMeasure = event.light;
      Debug(event.light); Debugln(" lux");
    }
  }
  else
  {
    Debugln(F("No TSL sensor found ... check your wiring?"));
  }
}

void loop() {


  digitalWrite(TS_PIN, HIGH); // sets the digital pin 13 on
  //for (int il = 0; il < 20;il++)
  //  delay(1000);

  Debug(F("Reading voltage "));
  payload.voltage = readVcc();
  //payload.voltage = readVccINA219();
  Debugln(payload.voltage);

  readTLSSensor();
  readUVSensor();

  Debugln(F("Reading temp"));
  readTemp();

  Debugln(F("Sending Radio packet"));
  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  //delay(100);
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Debugln(F("radio hardware is not responding!!"));
    radioReady = 0;
  }
  else
  {
    Debugln(F("++++++++++++ Radio initialized"));

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
    radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes

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
    Debugln(sizeof(payload));

    payload.payloadID = 0;
    radio.powerUp();

    unsigned long start_timer = micros();                    // start the timer
    radio.writeBlocking(&payload, sizeof(payload), 2000);      // transmit & save the report
    bool report = radio.txStandBy(2000);
    unsigned long end_timer = micros();                      // end the timer

    if (report) {
      Debug(F("Transmission successful! "));          // payload was delivered
      Debug(F("Time to transmit = "));
      Debug(end_timer - start_timer);                 // print the timer result
      Debug(F(" us. Sent: "));
      Debugln(payload.temp);                               // print payload sent
      success = 1;
    } else {
      success = 0;
      Debugln(F("Transmission failed or timed out")); // payload was not delivered
      //radio.printPrettyDetails(); // (larger) function that prints human readable data`
    }
  }
  delay(100);

  radio.powerDown();
  digitalWrite(TS_PIN, LOW); // sets the digital pin 13 on
  //delay(100);

  Debugln("Starting sleep");
#ifdef DEBUG_MODE
  int sleep_time = 1;
#else
  int sleep_time = (success == 1) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
#endif

  for (int il = 0; il < sleep_time; il++)
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_OFF, USART0_OFF, TWI_OFF);

  Debugln("Sleep completed");
  // to make this example readable in the serial monitor



} // loop
