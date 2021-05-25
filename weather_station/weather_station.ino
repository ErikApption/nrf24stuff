
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
#include <SI1145_WE.h>

#define ONE_WIRE_BUS 6
#define TS_PIN 9
#define MAX_RADIO_RETRIES 10
#define NODE_ID 1


#define DEBUG_MODE
#define SLEEP_CYCLES_SUCCESS 150 //20 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
SI1145_WE mySI1145 = SI1145_WE();

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

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void configureGY1145Sensor(void)
{
  mySI1145.init();
  //mySI1145.enableHighSignalVisRange();
  //mySI1145.enableHighSignalIrRange();
  
  /* choices: PS_TYPE, ALS_TYPE, PSALS_TYPE, ALSUV_TYPE, PSALSUV_TYPE || FORCE, AUTO, PAUSE */
  mySI1145.enableMeasurements(ALS_TYPE, FORCE);

   /* choose gain value: 0, 1, 2, 3, 4, 5, 6, 7 */ 
  mySI1145.setAlsVisAdcGain(0);

  //mySI1145.enableHighResolutionVis();
  Serial.println("SI1145 - forced ALS");
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureLightSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

void setup() {

  
  #ifdef DEBUG_MODE
  Serial.begin(115200);
  while (!Serial) {
    // some boards need to wait to ensure access to serial over USB
  }
  #endif

  Serial.println(F("Starting Weather Station"));

  Serial.println(F("Configuring Transistor Pin"));
  pinMode(TS_PIN, OUTPUT);    // sets the digital pin 13 as output

  Serial.println(F("Ready..."));

} // setup

void readUVSensor(void) {
  byte failureCode = 0;
  payload.amb_als = 0;
  payload.amb_ir = 0;
   
  for(int i=0; i<50; i++){
    //mySI1145.clearAllInterrupts();
    //mySI1145.startSingleMeasurement();
    payload.amb_als += mySI1145.getAlsVisData();
    payload.amb_ir += mySI1145.getAlsIrData();
  }
  payload.amb_als /= 50;
  payload.amb_ir /= 50;
  Serial.print("Ambient Light: ");
  Serial.println(payload.amb_als);
  Serial.print("Infrared Light: ");
  Serial.println(payload.amb_ir);
  failureCode = mySI1145.getFailureMode();  // reads the response register
  if((failureCode&128)){   // if bit 7 is set in response register, there is a failure
    handleFailure(failureCode);
  }
  Serial.println("---------");
  delay(1000);
}


void loop() {


  digitalWrite(TS_PIN, HIGH); // sets the digital pin 13 on
  //for (int il = 0; il < 20;il++)
  //  delay(1000);
  delay(100);

  Serial.println(F("TSL Sensor"));
  if (tsl.begin()) 
  {
    Serial.println(F("Found a TSL2591 sensor"));
    /* Display some basic information on this sensor */
    #ifdef DEBUG_MODE
    //displaySensorDetails();
    #endif
  
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
      #ifdef DEBUG_MODE
      Serial.print(event.light); Serial.println(" lux");
      #endif
    }
    
  } 
  else 
  {
    Serial.println(F("No TSL sensor found ... check your wiring?"));
  }

  readUVSensor();

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
  int sleep_time = 1;
  #else
  int sleep_time = (success == 1)?SLEEP_CYCLES_SUCCESS:SLEEP_CYCLES; 
  #endif
  
  for (int il = 0; il < sleep_time; il++)
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF,
                  SPI_OFF, USART0_OFF, TWI_OFF);

  Debugln("Sleep completed");
  // to make this example readable in the serial monitor



} // loop

void handleFailure(byte code){
  String msg = "";
  switch(code){
    case SI1145_RESP_INVALID_SETTING:
      msg = "Invalid Setting";
      break;
    case SI1145_RESP_PS1_ADC_OVERFLOW:
      msg = "PS ADC Overflow";
      break;
    case SI1145_RESP_ALS_VIS_ADC_OVERFLOW:
      msg = "ALS VIS ADC Overflow";
      break;
    case SI1145_RESP_ALS_IR_ADC_OVERFLOW:
      msg = "ALS IR Overflow";
      break;
    case SI1145_RESP_AUX_ADC_OVERFLOW:
      msg = "AUX ADC Overflow";
      break;
    default:
      msg = "Unknown Failure";
      break;
  }
  Serial.println(msg); 
  mySI1145.clearFailure();
}
