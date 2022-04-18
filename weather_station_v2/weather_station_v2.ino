

#define DEBUG_MODE
//#define LED_DEBUG
//#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <Vcc.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include "Adafruit_VEML6075.h"
#include <avr/wdt.h>
#include "SHT2x.h"
#include <debug_stuff.h>
#include <payload.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LED1_PIN 2
#define LED2_PIN 3
#define LED3_PIN 4
#define LED4_PIN 5

#define NRF_VCC_PIN 9
#define TEMP_VCC_PIN 8
#define UV_VCC_PIN 10
#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 2
#define NODE_ID 1
#define DHT_PIN 5
#include <SI1145_WE.h>

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

#define I2C_TIMEOUT 500

//Adafruit_VEML6075 uv = Adafruit_VEML6075();
SI1145_WE mySI1145 = SI1145_WE(&Wire);

uint32_t delayMS;

SHT2x sht;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7); // using pin 6 for the CE pin, and pin 7 for the CSN pin

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
BasePayLoad payload;
SI1145PayLoad si1145data;

void setup() {

  wdt_enable(WDTO_8S);

#ifdef LED_DEBUG
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  lcd_debug(1,1,1,1);
  delay(500);
#endif

#ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  Serial.println("Debug mode enabled");
#endif

  lcd_debug(1,1,1,0);

  pinMode(NRF_VCC_PIN, OUTPUT);
  pinMode(TEMP_VCC_PIN, OUTPUT);    // sets the digital pin 13 as output
  pinMode(UV_VCC_PIN, OUTPUT);    // sets the digital pin 13 as output
  pinMode(VCC_PIN, INPUT);
  Debugln("Ready...");
  lcd_debug(1,1,0,0);

  delayMS = 100;

  Wire.setTimeout(1000);
  Wire.setClock(10000);  
  Wire.begin();
  //Wire.setTimeout(I2C_TIMEOUT);
  lcd_debug(1,0,0,0);

} // setup

void lcd_debug(bool led1,bool led2,bool led3,bool led4)
{
#ifdef LED_DEBUG  
  digitalWrite(LED1_PIN, led1? HIGH:LOW);
  digitalWrite(LED2_PIN, led2? HIGH:LOW);
  digitalWrite(LED3_PIN, led3? HIGH:LOW);
  digitalWrite(LED4_PIN, led4? HIGH:LOW);
#endif  
}

double prevTemp = 0;

//float SI7021_temperature = NAN;
//float SI7021_humidity = NAN;
void readTemp(void)
{
  sht.begin(&Wire);
  
  payload.humidity = NAN;
  payload.temp = NAN;
  Debugln("Reading temperature");

  if ( sht.isConnected()  )
  {

    int success = 0;
    int retries = 0;
    for (retries; retries < MAX_TEMP_RETRIES; retries++)
    {

      sht.read();
      payload.temp = sht.getTemperature();
      payload.humidity = sht.getHumidity();
      Debug("Temp: ");
      Debugln(payload.temp);
      Debug("Hum: ");
      Debugln(payload.humidity);
      int isValid = abs(payload.temp - prevTemp) <= 20;
      prevTemp = payload.temp;
      if (payload.temp != NAN && isValid)
        break;

      Debug("Retries: ");
      Debug(retries);
    }
  }
  else
  {
    Debugln("SHT is not connected");    
  }
}

#define VOLTAGE_MEASUREMENTS 5

float readA0_Voltage_mV()
{
  int sensorValue = 0;
  for (int i =0; i <VOLTAGE_MEASUREMENTS;i++){
    sensorValue += analogRead(A0);
    delay(20);
  } 
  return sensorValue * (5.0 * 1000.0 / 1023.0) / VOLTAGE_MEASUREMENTS;  
}

#define SI1145_MEASUREMENTS 50
bool readSI1145()
{
//si1145data  
  mySI1145.init();
  //mySI1145.enableHighSignalVisRange();
  //mySI1145.enableHighSignalIrRange();
  
  /* choices: PS_TYPE, ALS_TYPE, PSALS_TYPE, ALSUV_TYPE, PSALSUV_TYPE || FORCE, AUTO, PAUSE */
  mySI1145.enableMeasurements(ALS_TYPE, FORCE);

   /* choose gain value: 0, 1, 2, 3, 4, 5, 6, 7 */ 
  mySI1145.setAlsVisAdcGain(0);

  //mySI1145.enableHighResolutionVis();
  Debugln("SI1145 - forced ALS");

  byte failureCode = 0;
  unsigned long amb_als = 0;
  unsigned long amb_ir = 0;
   
  for(int i=0; i<SI1145_MEASUREMENTS; i++){
    mySI1145.startSingleMeasurement();
    failureCode = mySI1145.getFailureMode();  // reads the response register
    if((failureCode&128))
      break;
    amb_als += mySI1145.getAlsVisData();
    amb_ir += mySI1145.getAlsIrData();
  }
  if((failureCode&128))
  {
    Debugln("error reading SI1145");
    return false;
  }
  else
  {
    amb_als /= SI1145_MEASUREMENTS;
    amb_ir /= SI1145_MEASUREMENTS;
    Debug("Ambient Light: ");
    Debugln(amb_als);
    Debug("Infrared Light: ");
    Debugln(amb_ir);  
    return true;
  }
}

void loop() {
  lcd_debug(0,0,0,0);
  wdt_reset();
  wdt_enable(WDTO_8S);

  digitalWrite(NRF_VCC_PIN, HIGH); // sets the digital pin on to power AM2301
  digitalWrite(TEMP_VCC_PIN, HIGH); // sets the digital pin on to power AM2301
  digitalWrite(UV_VCC_PIN, HIGH); // sets the digital pin on to power UV & Light sensor
  //for (int il = 0; il < 20;il++)
  //  delay(1000);

  Debug("Reading voltage ");
  //payload.voltage = Vcc::measure();
  payload.voltage = readA0_Voltage_mV();
  Debugln(payload.voltage);

  lcd_debug(0,0,0,1);
  readSI1145();

  Debugln("Reading temp");
  lcd_debug(0,0,1,0);
  readTemp();

  int p = sizeof(payload);
  int p2= sizeof(si1145data);
  //p = max(p,p2);
#ifdef DEBUG_MODE
  Debugln("");
  Debug("Size of payload is ");
  Debugln(p);
  if (p > 32) Debugln("!!Warning!! invalid payload size");
#endif

  lcd_debug(0,0,1,1);

  Debugln("Sending Radio packet");
  int radioReady = 1;
  int sensorReady = 1;
  int success = 0;
  //delay(100);
  // initialize the transceiver on the SPI bus
  if (!radio.begin()) {
    Debugln("radio hardware is not responding!!");
    lcd_debug(1,0,0,1);
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
    si1145data.nodeID = NODE_ID;

    Debugln("Payload size: ");
    Debugln(sizeof(payload));
    Debugln("SI1145 size: ");
    Debugln(sizeof(si1145data));    

    payload.payloadID = 0;
    si1145data.payloadID = 1;
    radio.powerUp(); //This will take up to 5ms for maximum compatibility
    delay(10);

    unsigned long start_timer = micros();                    // start the timer
    radio.writeBlocking(&payload, sizeof(payload), 2000);      // transmit & save the report
    bool report1 = radio.txStandBy(2000);
    //radio.writeBlocking(&si1145data, sizeof(si1145data), 2000);      // transmit & save the report
    bool report2 = true;
    //bool report2 = radio.txStandBy(2000);
    unsigned long end_timer = micros();                      // end the timer

    if (report1 && report2) {
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
  lcd_debug(0,1,0,1);
  radio.powerDown();
  digitalWrite(TEMP_VCC_PIN, LOW); // sets the digital pin 13 on
  digitalWrite(UV_VCC_PIN, LOW); // sets the digital pin on to power UV & Light sensor
  digitalWrite(NRF_VCC_PIN, LOW); // sets the digital pin on to power UV & Light sensor

  //delay(100);
  wdt_reset();

  // disable wdt while sleeping
  wdt_disable();

  Debugln("Starting sleep");
  lcd_debug(0,1,1,1);
#ifdef DEBUG_MODE
  int sleep_time = 1;
#else
  int sleep_time = (success == 1) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
#endif

  for (int il = 0; il < sleep_time; il++)
  {
    lcd_debug(1,0,1,0);

#ifdef DEBUG_MODE
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
#else
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
#endif
  }

  lcd_debug(1,1,1,1);

} // loop
