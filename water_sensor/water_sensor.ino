

//#define DEBUG_MODE

//#define WDT_ENABLED

#define USE_BOARD_POWER

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <payload.h>
#include <debug_stuff.h>
#include <Vcc.h>
#include <avr/wdt.h>
#include <EasyLed.h>

#define ERR_RADIO 4
#define ERR_SENSOR 3
#define ERR_TRANS_FAILED 2
#define ERR_SUCCESS 1
#define ERR_CODE_CYCLES 80 // flash every 80 seconds * 80 = 6400 ~ 2 hours

#define RADIO_POWER_PIN 3
#define POWER_PIN 5
#define SENSOR_POWER_PIN A0
#define STATUS_LED 4
#define MAX_RADIO_RETRIES 10
#define NODE_ID 5 // water sensor

#define TX_TIMEOUT 10000 // 10 seconds

#define SLEEP_CYCLES_SUCCESS 75 // 10 minutes on success -
#define SLEEP_CYCLES_ERROR 10	// 80 seconds otherwise

int lastErrorCode = ERR_SUCCESS;
int lastErrorCodeCycles = ERR_CODE_CYCLES;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(6, 7);									 // using pin 5 for the CE pin, and pin 6 for the CSN pin
EasyLed led(STATUS_LED, EasyLed::ActiveLevel::High); // Use this for an active-low LED.

// Let these addresses be used for the pair
const uint8_t receiver_address[6] = "1Node";
const uint8_t nodes[][6] = {"2Node", "3Node", "4Node", "5Node", "6Node", "7Node"};
// It is very helpful to think of an address as a path instead of as
// an identifying device destination

// to use different addresses on a pair of radios, we need a variable to
// uniquely identify which address this radio will use to transmit

// Used to control whether this node is sending or receiving

// For this example, we'll be using a payload containing
// a single float number that will be incremented
// on every successful transmission
// float payload = 1.234;
AnalogPayload payload;

void setup()
{
#ifdef WDT_ENABLED
	wdt_disable();
#endif

#ifdef DEBUG_MODE
	Serial.begin(115200);
	while (!Serial)
	{
		// some boards need to wait to ensure access to serial over USB
	}
#endif

	pinMode(POWER_PIN, OUTPUT); // sets the digital pin 13 as output

#ifdef USE_BOARD_POWER
	pinMode(RADIO_POWER_PIN, OUTPUT); // sets the digital pin 13 as output
#endif

	// print example's introductory prompt
	Debug(F("Sensor "));
	Debugln(NODE_ID);

#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif
} // setup

#define SENSOR_READS 3

void Read_WaterSensor()
{
	int sensorValue = -1;
	for (int si = 0; si < SENSOR_READS; si++)
	{
		sensorValue += analogRead(SENSOR_POWER_PIN);

		delay(200);
#ifdef WDT_ENABLED
		wdt_reset();
#endif
	}
	payload.analogPayLoad = sensorValue / SENSOR_READS;
	Debug("A0 Readout: ");
	Debugln(payload.analogPayLoad);
}

void loop()
{

#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif
	int error_code = ERR_SUCCESS;
	wdt_reset();
	Debug(F("Reading voltage "));
	payload.voltage = Vcc::measure();
	Debugln(payload.voltage);

#ifdef USE_BOARD_POWER
	digitalWrite(RADIO_POWER_PIN, HIGH); // sets the digital pin 13 on
#endif

	digitalWrite(POWER_PIN, HIGH); // sets the digital pin 13 on
	delay(100);

	bool radioAvail = false;
	for (int rRetries = 0; rRetries < 10; rRetries++)
	{
		if (radio.begin())
		{
			radioAvail = true;
			break;
		}
		Debugln(F("radio hardware is not responding!!"));
		error_code = ERR_RADIO;
		led.flash(error_code);
		delay(1000);
#ifdef WDT_ENABLED
		wdt_reset();
#endif
	}
	if (radioAvail)
	{

		Debugln(F("powering up radio"));
		radio.powerUp();

		payload.nodeID = NODE_ID;
#ifdef USE_BOARD_POWER
		digitalWrite(SENSOR_POWER_PIN, HIGH); // sets the digital pin 13 on
#endif

		Debugln(F("reading sensor"));
		Read_WaterSensor();


		Debugln("Configuring radio");
		// role variable is hardcoded to RX behavior, inform the user of this
		// Debugln(F("*** PRESS 'T' to begin transmitting to the other node"));
		radio.setChannel(5);
		// radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
		//  Set the PA Level low to try preventing power supply related problems
		//  because these examples are likely run with nodes in close proximity to
		//  each other.
#ifdef DEBUG_MODE		
		radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
#else
		radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.
#endif

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

		payload.payloadID = 4;

		bool report = false;
		long transTime = 0;
		Debugln("Sending payload");

		unsigned long start_timer = micros();		// start the timer
		radio.writeFast(&payload, sizeof(payload)); // transmit & save the report
		report = radio.txStandBy(TX_TIMEOUT);
		Debugln("Payload size: ");
		Debugln(sizeof(payload));
		unsigned long end_timer = micros(); // end the timer
		transTime = end_timer - start_timer;
		if (!report)
		{
			Debugln("Transmission failed");
			delay(100);
		}

#ifdef WDT_ENABLED
		wdt_reset();
#endif

		if (report)
		{
			led.flash(ERR_SUCCESS);
			Debug(F("Transmission successful! ")); // payload was delivered
			Debug(F("Time to transmit = "));
			Debug(transTime); // print the timer result
			Debugln(F(" us. Sent: "));
			error_code = ERR_SUCCESS;
		}
		else
		{
			error_code = ERR_TRANS_FAILED;
			led.flash(error_code);
			Debugln(F("Transmission failed or timed out")); // payload was not delivered
			delay(10);
			// radio.printPrettyDetails(); // (larger) function that prints human readable data
		}

		radio.powerDown();
	}
#ifdef USE_BOARD_POWER
	digitalWrite(RADIO_POWER_PIN, LOW); // sets the digital pin 13 on
#endif

	digitalWrite(POWER_PIN, LOW); // sets the digital pin 13 on


#ifdef WDT_ENABLED
	wdt_reset();
	// disable wdt while sleeping
	wdt_disable();
#endif

	Debugln("Starting sleep");
#ifdef DEBUG_MODE
	int sleep_cycles = 1;
#else
	int sleep_cycles = (error_code == ERR_SUCCESS) ? SLEEP_CYCLES_SUCCESS : SLEEP_CYCLES_ERROR;
#endif

	if (error_code != ERR_SUCCESS)
	{
		lastErrorCode = error_code;
		lastErrorCodeCycles = ERR_CODE_CYCLES;
	}

	for (int il = 0; il < sleep_cycles; il++)
	{
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
		if (il % 4 == 0) // every 32 seconds
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

	Debugln("Sleep completed");
#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif

} // loop
