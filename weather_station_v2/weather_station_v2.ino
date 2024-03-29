

//#define DEBUG_MODE
#define WDT_ENABLED
#define SI1145_ARDUINO
//#define LED_DEBUG
//#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include "SHT85.h"
//#include <LowPower.h>
#include <Vcc.h>
#include <Wire.h>
//#include <Adafruit_Sensor.h>
#include <avr/wdt.h>
#include <utility/twi.h>

#include <debug_stuff.h>
#include <payload.h>
#ifdef SI1145_ARDUINO
#include <Adafruit_SI1145.h>
#else
#include <SI1145_WE.h>
#endif
#include <EasyLed.h>

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

#define ERR_RADIO 4
#define ERR_TEMP_SENSOR 3
#define ERR_VIS_SENSOR 2
#define ERR_TRANS_FAILED 5
#define ERR_RETRIES_SENSOR 6
#define ERR_SUCCESS 1
#define ERR_CODE_CYCLES 80 // flash every 80 seconds * 80 = 6400 ~ 2 hours

#define SENSOR_VCC_ON_PIN 9
#define STATUS_LED 8
//#define TEMP_VCC_PIN 8
#define KEEPALIVE_PIN 10

#define VCC_PIN A1
#define MAX_RADIO_RETRIES 10
#define MAX_TEMP_RETRIES 4
#define NODE_ID 1 //weather
#define DHT_PIN 5

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
#define SLEEP_CYCLES 10 //80 seconds otherwise

#define I2C_TIMEOUT 500

int lastErrorCode = ERR_SUCCESS;
int lastErrorCodeCycles = ERR_CODE_CYCLES;

//Adafruit_VEML6075 uv = Adafruit_VEML6075();
//SI1145_WE mySI1145 = SI1145_WE(&Wire);
Adafruit_SI1145 uvs = Adafruit_SI1145();
uint32_t delayMS;
EasyLed led(STATUS_LED, EasyLed::ActiveLevel::High); // Use this for an active-low LED.
SHT85 sht;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7); // using pin 6 for the CE pin, and pin 7 for the CSN pin - weather station setup
//RF24 radio(9, 10); // using pin 6 for the CE pin, and pin 7 for the CSN pin - keyestudio nano shield

// Let these addresses be used for the pair
uint8_t receiver_address[6] = "1Node";
uint8_t nodes[][6] = { "2Node", "3Node", "4Node", "5Node" };
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

	pinMode(KEEPALIVE_PIN, OUTPUT);
	keep_alive();
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

	pinMode(SENSOR_VCC_ON_PIN, OUTPUT);
	pinMode(VCC_PIN, INPUT);
	Debugln("Ready...");

	delayMS = 100;

	Wire.setTimeout(1000);
	//Wire.setClock(10000);  
	Wire.begin();
	//Wire.setTimeout(I2C_TIMEOUT);

} // setup

const float TEMP_ERROR_TH = 20;
//float SI7021_temperature = NAN;
//float SI7021_humidity = NAN;
bool readTemp(int *retries)
{
	unsigned long start_timer = micros();
	Wire.setWireTimeout(25000ul,true);
	sht.begin(&Wire);

	payload.humidity = NAN;
	payload.temp = NAN;
	Debugln("Reading temperature");
	bool success = false;
	if (sht.isConnected())
	{
		sht.read();
		payload.temp = sht.getTemperature();
		float prevTemp = payload.temp - TEMP_ERROR_TH - 1;
		while (abs(prevTemp - payload.temp) > TEMP_ERROR_TH && (*retries) < MAX_TEMP_RETRIES)
		{
			sht.read();
			prevTemp = payload.temp;
			payload.temp = sht.getTemperature();
			payload.humidity = sht.getHumidity();
			Debug("Temp: ");
			Debugln(payload.temp);
			Debug("Hum: ");
			Debugln(payload.humidity);
			Debug("Retries: ");
			Debug(*retries);
			(*retries)++;
		}
		success = (*retries) < MAX_TEMP_RETRIES;
	}
	else
	{
		Debugln("SHT is not connected");

	}
	unsigned long end_timer = micros();
	payload.readoutms = end_timer - start_timer;
	Debug(" readoutms:"); Debug(payload.readoutms); Debug(" ");
	return success;
}

#define VOLTAGE_MEASUREMENTS 5

float readA0_Voltage_mV()
{
	int sensorValue = 0;
	for (int i = 0; i < VOLTAGE_MEASUREMENTS; i++) {
		sensorValue += analogRead(A0);
		delay(20);
	}
	return sensorValue * (5.0 * 1000.0 / 1023.0) / VOLTAGE_MEASUREMENTS;
}

#define SI1145_MEASUREMENTS 5

#ifdef SI1145_ARDUINO
bool readSI1145()
{
	unsigned long start_timer = micros();
	Wire.setWireTimeout(25000ul,true);
	if (!uvs.begin(&Wire))
	{
		return false;
	}

	Debugln("SI1145 - initialized3");

	byte failureCode = 0;
	unsigned long amb_als = 0;
	unsigned long amb_ir = 0;
	float uv = 0;

	si1145data.amb_als = 0;
	si1145data.amb_ir = 0;
	si1145data.uv = NAN;

	for (int i = 0; i < SI1145_MEASUREMENTS; i++) {
		Debug("readout ");
		Debugln(i);
		keep_alive();
		amb_als += (long)(uvs.readVisible());
		amb_ir += (long)(uvs.readIR());
		uv += (float)(uvs.readUV());
	}

	amb_als /= SI1145_MEASUREMENTS;
	amb_ir /= SI1145_MEASUREMENTS;
	uv /= SI1145_MEASUREMENTS;

	si1145data.amb_als = amb_als;
	si1145data.amb_ir = amb_ir;
	si1145data.uv = uv;

	Debug("Ambient Light: ");
	Debugln(si1145data.amb_als);
	Debug("Infrared Light: ");
	Debugln(si1145data.amb_ir);
	Debug("UV Index: ");
	Debugln(si1145data.uv);

	unsigned long end_timer = micros();
	si1145data.readoutms = end_timer - start_timer;
	Debug("Readout ms: ");
	Debugln(si1145data.readoutms);
	return true;
}

#else
bool readSI1145()
{
	unsigned long start_timer = micros();
	//si1145data  
	mySI1145.init();
	//mySI1145.enableHighSignalVisRange();
	//mySI1145.enableHighSignalIrRange();
	Debugln("SI1145 - initialized");
	mySI1145.enableHighSignalVisRange(); // Gain divided by 14.5
	mySI1145.enableHighSignalIrRange(); // Gain divided by 14.5
	/* choices: PS_TYPE, ALS_TYPE, PSALS_TYPE, ALSUV_TYPE, PSALSUV_TYPE || FORCE, AUTO, PAUSE */
	mySI1145.enableMeasurements(ALS_TYPE, FORCE);
	mySI1145.enableMeasurements(PSALSUV_TYPE, AUTO);
	Debugln("SI1145 - initialized2");

	/* choose gain value: 0, 1, 2, 3, 4, 5, 6, 7 */
	mySI1145.setAlsVisAdcGain(0);
	Debugln("SI1145 - initialized3");

	//mySI1145.enableHighResolutionVis();
	Debugln("SI1145 - forced ALS");

	byte failureCode = 0;
	unsigned long amb_als = 0;
	unsigned long amb_ir = 0;
	float uv = 0;

	si1145data.amb_als = 0;
	si1145data.amb_ir = 0;
	si1145data.uv = NAN;

	for (int i = 0; i < SI1145_MEASUREMENTS; i++) {
		Debug("readout ");
		Debugln(i);
		mySI1145.startSingleMeasurement();
		failureCode = mySI1145.getFailureMode();  // reads the response register
		if ((failureCode & 128))
			break;
		amb_als += mySI1145.getAlsVisData();
		amb_ir += mySI1145.getAlsIrData();
		uv += mySI1145.getUvIndex();
	}
	if ((failureCode & 128))
	{
		Debugln("error reading SI1145");
		unsigned long end_timer = micros();
		si1145data.readoutms = end_timer - start_timer;
		return false;
	}
	else
	{
		amb_als /= SI1145_MEASUREMENTS;
		amb_ir /= SI1145_MEASUREMENTS;
		uv /= SI1145_MEASUREMENTS;

		si1145data.amb_als = amb_als;
		si1145data.amb_ir = amb_ir;
		si1145data.uv = uv;

		Debug("Ambient Light: ");
		Debugln(si1145data.amb_als);
		Debug("Infrared Light: ");
		Debugln(si1145data.amb_ir);
		Debug("UV Index: ");
		Debugln(si1145data.uv);

		unsigned long end_timer = micros();
		si1145data.readoutms = end_timer - start_timer;
		Debug("Readout ms: ");
		Debugln(si1145data.readoutms);
		return true;
	}
}
#endif

void keep_alive()
{
	digitalWrite(KEEPALIVE_PIN, HIGH);
	delay(100);
	digitalWrite(KEEPALIVE_PIN, LOW);
}

void loop() {
	keep_alive();

	int error_code = ERR_SUCCESS;
	Wire.setTimeout(1000);
	//Wire.setClock(10000);  

	wdt_reset();
#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif


	digitalWrite(SENSOR_VCC_ON_PIN, HIGH);

	Debug("Reading voltage ");
	payload.voltage = Vcc::measure();
	//payload.voltage = readA0_Voltage_mV();
	Debugln(payload.voltage);

	bool si1145Available = false;
	for (int si = 0; si < 10; si++)
	{
		Debug("Reading SI1145 test");
		Debugln(si);
		si1145Available = readSI1145();
		if (si1145Available) break;
		delay(100);
	}
	if (!si1145Available)
	{
		error_code = ERR_VIS_SENSOR;
		led.flash(error_code);
		delay(200);
	}
	keep_alive();

	Debugln("Reading temp");
	int retries = 0;
	if (!readTemp(&retries))
	{
		if (retries > 0)
			error_code = ERR_RETRIES_SENSOR;
		else
			error_code = ERR_TEMP_SENSOR;
		led.flash(error_code);
	}

	int p = sizeof(payload);
	int p2 = sizeof(si1145data);
	//p = max(p,p2);
#ifdef DEBUG_MODE
	Debugln("");
	Debug("Size of payload is ");
	Debugln(p);
	if (p > 32) Debugln("!!Warning!! invalid payload size");
#endif
	keep_alive();

	Debugln("Sending Radio packet");
	int radioReady = 1;
	int sensorReady = 1;
	//delay(100);
	// initialize the transceiver on the SPI bus
	if (!radio.begin()) {
		Debugln("radio hardware is not responding!!");
		error_code = ERR_RADIO;
		led.flash(error_code);
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
		si1145data.nodeID = NODE_ID;

		Debugln("Payload size: ");
		Debugln(sizeof(payload));
		Debugln("SI1145 size: ");
		Debugln(sizeof(si1145data));

		payload.payloadID = 0;
		si1145data.payloadID = 1;
		radio.powerUp(); //This will take up to 5ms for maximum compatibility
		delay(10);
		//radio.setPayloadSize(sizeof(payload)); // float datatype occupies 4 bytes
		unsigned long start_timer = micros();                    // start the timer

		if (si1145Available)
		{
			Debug("sending si1145 packet - readoutms:");
			Debugln(si1145data.readoutms);
			radio.writeFast(&si1145data, sizeof(si1145data));
			radio.txStandBy();
			delay(200);
		}

		//radio.writeBlocking(&payload, sizeof(payload), 2000);      // transmit & save the report
		radio.writeFast(&payload, sizeof(payload));

		//radio.txStandBy();
		//radio.txStandBy(2000);
		//radio.setPayloadSize(sizeof(si1145data)); // float datatype occupies 4 bytes
		//radio.writeBlocking(&si1145data, sizeof(si1145data), 2000);
		keep_alive();

		bool fifoSuccess = radio.txStandBy(2000);// Using extended timeouts, returns 1 if success. Retries failed payloads for 1 seconds before returning 0.
		unsigned long end_timer = micros();                      // end the timer
		keep_alive();

		if (fifoSuccess) {
			Debug("Time to transmit = ");
			Debug(end_timer - start_timer);                 // print the timer result
			Debug(" us. Sent: ");
			Debugln(payload.temp);                               // print payload sent
		}
		else {
			Debug("Transmission failed or timed out - fifo:"); // payload was not delivered
			Debugln(fifoSuccess);
			error_code = ERR_TRANS_FAILED;
			led.flash(error_code);
			//radio.printPrettyDetails(); // (larger) function that prints human readable data`
		}
	}
	delay(100);

	radio.powerDown();
	digitalWrite(SENSOR_VCC_ON_PIN, LOW); // sets the digital pin on to power UV & Light sensor

	wdt_reset();

	// disable wdt while sleeping
	wdt_disable();

	Debugln("Starting sleep");

#ifdef DEBUG_MODE
	int sleep_cycles = 1;
#else
	int sleep_cycles = (error_code == ERR_SUCCESS) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES;
#endif

	if (error_code != ERR_SUCCESS)
	{
		lastErrorCode = error_code;
		lastErrorCodeCycles = ERR_CODE_CYCLES;
	}

	for (int il = 0; il < sleep_cycles; il++)
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
		keep_alive();
		if (il % 8 == 0) //every 64 seconds
		{
			if (lastErrorCodeCycles == 0)
			{
				lastErrorCode = error_code;
			}
			led.flash(lastErrorCode);
			if (lastErrorCodeCycles > 0)
				lastErrorCodeCycles--;
		}
}
	Debugln("End sleep");

#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif


} // loop
