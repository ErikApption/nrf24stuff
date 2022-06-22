#include <avr/sleep.h>

#define wdt_reset() __asm__ __volatile__ ("wdr") // this re-starts the watchdog clock/timer (ie. leaves current settings as is)

ISR(WDT_vect) {} // NB. MUST be present, otherwise ATTiny reboots instead of returning from Sleep!

#define INTERRUPT_PIN PCINT1  // This is PB1 per the schematic
#define INT_PIN PB1           // Interrupt pin of choice: PB1 (same as PCINT1) - Pin 6
#define MOSFET_PIN PB4           // PB4 - Pin 3
#define PCINT_VECTOR PCINT0_vect      // This step is not necessary - it's a naming thing for clarit

bool is_alive = false;

//https://www.instructables.com/ATtiny85-Watchdog-reboot-Together-With-SLEEP-Andor/

ISR(PCINT_VECTOR)
{
	is_alive = true;
}

void disable_watchdog()
{
	WDTCR |= (1 << WDCE) | (1 << WDE); WDTCR = 0x00; // disable watchdog
}

void enable_watchdog()
{
	//WDTCR = (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0); // set watchdog-timing to 4secs
	WDTCR = (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0); // set watchdog-timing to 8secs

	WDTCR |= (1 << WDE); // Set watchdog timer to re-boot if triggered to begin with (only add WDIE just before we start sleep)	

}

void setup() {
	byte boot = MCUSR; MCUSR = 0x00; // NB. MCUSR must be zeroed or watchdog will keep rebooting

	enable_watchdog();

	// put your setup code here, to run once:
	pinMode(MOSFET_PIN, OUTPUT);
	cli();                            // Disable interrupts during setup
	PCMSK |= (1 << INTERRUPT_PIN);    // Enable interrupt handler (ISR) for our chosen interrupt pin (PCINT1/PB1/pin 6)
	GIMSK |= (1 << PCIE);             // Enable PCINT interrupt in the general interrupt mask
	pinMode(INT_PIN, INPUT_PULLUP);   // Set our interrupt pin as input with a pullup to keep it stable
	sei();                            //last line of setup - enable interrupts after setup
	PowerCycle();
	wdt_reset();
}

void PowerCycle()
{
	digitalWrite(MOSFET_PIN, HIGH); // do this first
	delay(1000);
	digitalWrite(MOSFET_PIN, LOW); // do this first
}

void loop() {
	is_alive = false;
	enable_watchdog();
	wdt_reset();
	ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA		
	enable_watchdog(); //Setup watchdog to go off after 1sec		
	for (short i = 0; i < 3; i++)
	{
		wdt_reset(); // re-start timer here (if required)
		set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
		sleep_mode(); //Go to sleep! Wake up 1sec later and check water
	}
	ADCSRA |= (1 << ADEN); //Enable ADC
	if (!is_alive)
		PowerCycle();
}
