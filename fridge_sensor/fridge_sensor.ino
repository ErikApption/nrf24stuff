

#define DEBUG_MODE

#define WDT_ENABLED

#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <LowPower.h>
#include <payload.h>
#include <debug_stuff.h>
#include <Vcc.h>
#include <avr/wdt.h>
#include <EasyLed.h>
#include "AM232X.h"


#define ERR_RADIO 4
#define ERR_SENSOR 3
#define ERR_TRANS_FAILED 2
#define ERR_SUCCESS 1
#define ERR_CODE_CYCLES 80 // flash every 80 seconds * 80 = 6400 ~ 2 hours

#define RADIO_POWER_PIN 3
#define SENSOR_POWER_PIN 2
#define STATUS_LED 4
#define MAX_RADIO_RETRIES 10
#define NODE_ID 4 // fridge

#define TX_TIMEOUT 10000 // 10 seconds

#define SLEEP_CYCLES_SUCCESS 75 //10 minutes on success - 
#define SLEEP_CYCLES_ERROR 10          // 80 seconds otherwise

int lastErrorCode = ERR_SUCCESS;
int lastErrorCodeCycles = ERR_CODE_CYCLES;

AM232X AM2320;

// instantiate an object for the nRF24L01 transceiver
RF24 radio(8, 9);                                    // using pin 7 for the CE pin, and pin 8 for the CSN pin
EasyLed led(STATUS_LED, EasyLed::ActiveLevel::High); // Use this for an active-low LED.

// Let these addresses be used for the pair
const uint8_t receiver_address[6] = "1Node";
const uint8_t nodes[][6] = { "2Node", "3Node", "4Node", "5Node" };
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

	pinMode(RADIO_POWER_PIN, OUTPUT); // sets the digital pin 13 as output

	// print example's introductory prompt
	Debug(F("Sensor "));
	Debugln(NODE_ID);

#ifdef WDT_ENABLED
	wdt_enable(WDTO_8S);
#endif
} // setup

void loop()
{
	int error_code = ERR_SUCCESS;
	wdt_reset();
	Debug(F("Reading voltage "));
	payload.voltage = Vcc::measure();
	Debugln(payload.voltage);

	//digitalWrite(RADIO_POWER_PIN, HIGH); // sets the digital pin 13 on
	digitalWrite(SENSOR_POWER_PIN, HIGH); // sets the digital pin 13 on
	// for (int il = 0; il < 20;il++)
	delay(2000);

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
		wdt_reset();
	}
	if (radioAvail)
	{

		Debugln(F("powering up radio"));
		radio.powerUp();

		payload.nodeID = NODE_ID;

		Debugln(F("reading sensor"));
		int sensorReady = 1;
		for (int si = 0; si < 10; si++)
		{
			if (!AM2320.begin())
			{
				Debugln(F("sensor hardware is not responding!!"));
				sensorReady = 0;
				payload.temp = NAN;
				error_code = ERR_SENSOR;
				led.flash(error_code);
				delay(500);
				wdt_reset();
			}
			else
			{
				AM2320.wakeUp();
				delay(2000);
				payload.humidity = AM2320.getHumidity();
				payload.temp = AM2320.getTemperature();
				// payload.voltage = analogRead(A1);
				Debug("Temp: ");
				Debugln(payload.temp);
				Debug("Hum: ");
				Debugln(payload.humidity);
				break;
			}
		}
		digitalWrite(SENSOR_POWER_PIN, LOW); // sets the digital pin 13 on

		Debugln("Configuring radio");
		// role variable is hardcoded to RX behavior, inform the user of this
		// Debugln(F("*** PRESS 'T' to begin transmitting to the other node"));
		radio.setChannel(5);
		// radio.setAutoAck(false); //https://github.com/nRF24/RF24/issues/685
		//  Set the PA Level low to try preventing power supply related problems
		//  because these examples are likely run with nodes in close proximity to
		//  each other.
		radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.

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

		payload.payloadID = 0;

		bool report = false;
		long transTime = 0;
		Debugln("Sending payload");

		unsigned long start_timer = micros();       // start the timer
		radio.writeBlocking(&payload, sizeof(payload), TX_TIMEOUT); // transmit & save the report
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


		wdt_reset();

		if (report)
		{
			led.flash(ERR_SUCCESS);
			Debug(F("Transmission successful! ")); // payload was delivered
			Debug(F("Time to transmit = "));
			Debug(transTime); // print the timer result
			Debug(F(" us. Sent: "));
			Debugln(payload.temp); // print payload sent
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
	//digitalWrite(RADIO_POWER_PIN, LOW); // sets the digital pin 13 on

	wdt_reset();

	// disable wdt while sleeping
	wdt_disable();

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
		if (il % 4 == 0) //every 32 seconds
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
