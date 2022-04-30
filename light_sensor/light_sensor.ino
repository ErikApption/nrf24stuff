

//#define DEBUG_MODE
#define WDT_ENABLED
//#define LED_DEBUG
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

#define LED1_PIN 2
#define LED2_PIN 3
#define LED3_PIN 4
#define LED4_PIN 5

#define SENSOR_VCC_PIN 9
#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 2
#define NODE_ID 3
#define DHT_PIN 5

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
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

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

TSLPayLoad tslPayload;

void setup() {

#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif

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

  pinMode(SENSOR_VCC_PIN, OUTPUT);
  pinMode(VCC_PIN, INPUT);
  Debugln("Ready...");
  lcd_debug(1,1,0,0);

  delayMS = 100;

  Wire.setTimeout(1000);
  //Wire.setClock(10000);  
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


bool readTSLLux()
{

  tslPayload.lux = 0;
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  //tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */  

  /* Initialise the sensor */
  //use tsl.begin() to default to Wire, 
  //tsl.begin(&Wire2) directs api to use Wire2, etc.
  if(!tsl.begin(&Wire))
  {
    /* There was a problem detecting the TSL2561 ... check your connections */
    Debugln("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");    
    return false;
  }  

  unsigned long start_timer = micros();
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);
  bool success =false;
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    Debug(event.light); Debugln(" lux");
    tslPayload.lux = event.light;
    success = true;
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Debugln("Sensor overload");
    tslPayload.lux = 0;
  }
  unsigned long end_timer = micros();
  tslPayload.readoutms = end_timer-start_timer;
  return success;
}

void loop() {

  Wire.setTimeout(1000);
  //Wire.setClock(10000);  

  lcd_debug(0,0,0,0);
  wdt_reset();
#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif

  Debug("Reading voltage ");
  tslPayload.voltage = Vcc::measure();
  //payload.voltage = readA0_Voltage_mV();
  Debugln(tslPayload.voltage);

  lcd_debug(0,0,0,1);
  Debug("Reading Lux ");  
  bool luxAvailable = readTSLLux();

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

    tslPayload.nodeID = NODE_ID;
    tslPayload.payloadID = 3;

    radio.powerUp(); //This will take up to 5ms for maximum compatibility
    delay(10);
    //radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
    unsigned long start_timer = micros();                    // start the timer

    radio.writeFast(&tslPayload, sizeof(tslPayload));
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
  lcd_debug(0,1,0,1);
  radio.powerDown();

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
  lcd_debug(1,1,1,1);

#ifdef WDT_ENABLED
  wdt_enable(WDTO_8S);
#endif


} // loop
