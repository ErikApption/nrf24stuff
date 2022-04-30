/*********************************************************************************************\
  Perform the actual reading + interpreting of data.
  \*********************************************************************************************/

float SI7021_temperature = NAN;
float SI7021_humidity = NAN;
int SI7021_Pin;

/*!
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
class InterruptLock {
public:
  InterruptLock() {
    noInterrupts();
  }
  ~InterruptLock() {
    interrupts();
  }
};

// Return the time difference as a signed value, taking into account the timers may overflow.
// Returned timediff is between -24.9 days and +24.9 days.
// Returned value is positive when "next" is after "prev"
inline int32_t timeDiff(const unsigned long prev, const unsigned long next) {
  return ((int32_t) (next - prev));
}

// Compute the number of milliSeconds passed since timestamp given.
// N.B. value can be negative if the timestamp has not yet been reached.
inline long timePassedSince(unsigned long timestamp) {
  return timeDiff(timestamp, millis());
}

inline long usecPassedSince(unsigned long timestamp) {
  return timeDiff(timestamp, micros());
}

// Check if a certain timeout has been reached.
inline bool timeOutReached(unsigned long timer) {
  return timePassedSince(timer) >= 0;
}

inline bool usecTimeOutReached(unsigned long timer) {
  return usecPassedSince(timer) >= 0;
}


boolean SI7021_waitState(int state)
{
  unsigned long timeout = micros() + 100;
  while (digitalRead(SI7021_Pin) != state)
  {
    if (usecTimeOutReached(timeout)) return false;
    delayMicroseconds(1);
  }
  
  return true;
}

/*********************************************************************************************\
  DHT sub to get an 8 bit value from the receiving bitstream
  \*********************************************************************************************/
int ReadSI7021_Data(void)
{
  byte i = 0;
  byte result = 0;
  for (i = 0; i < 8; i++)
  {
    if (!SI7021_waitState(1))  return -1;
    delayMicroseconds(35); // was 30
    if (digitalRead(SI7021_Pin))
      result |= (1 << (7 - i));
    if (!SI7021_waitState(0))  return -1;
  }
  return result;
}

int interrupt_error = 0;
bool ReadSI7021(int pin) {
  SI7021_temperature = NAN;
  SI7021_humidity = NAN;
  byte i;
  SI7021_Pin = pin;
  //Serial.print("Using pin");Serial.println(pin);
  byte dht_dat[5];
  bool readingAborted = false;

  {
    pinMode(SI7021_Pin, OUTPUT);
    delayMicroseconds(20);
    digitalWrite(SI7021_Pin, LOW);              // Pull low

    delayMicroseconds(500);

    pinMode(SI7021_Pin, INPUT_PULLUP);

    delayMicroseconds(20);

    //noInterrupts();
    InterruptLock lock;
    if (!SI7021_waitState(0)) {
      //interrupts();
      interrupt_error = 1;
      return false;
    }
    if (!SI7021_waitState(1)) {
      //interrupts();
      interrupt_error = 2;
      return false;
    }
    if (!SI7021_waitState(0)) {
      //interrupts();
      interrupt_error = 3;
      return false;
    }

    for (i = 0; i < 5 && !readingAborted; i++)
    {
      int data = ReadSI7021_Data();
      if (data == -1)
      { //P005_log(event, P005_error_protocol_timeout);
        Serial.println("SI7021 - protocol error, reading aborded");
        readingAborted = true;
      }
      else
      {
        dht_dat[i] = data;
      }
    }
  }
  //interrupts();
  if (readingAborted)
    return false;

  // Checksum calculation is a Rollover Checksum by design!
  byte dht_check_sum = (dht_dat[0] + dht_dat[1] + dht_dat[2] + dht_dat[3]) & 0xFF; // check check_sum
  if (dht_dat[4] != dht_check_sum)
  {
    Serial.println("SI7021 - checksum error");

    //P005_log(event, P005_error_checksum_error);
    return false;
  }


  if (dht_dat[2] & 0x80) // negative temperature
    SI7021_temperature = -0.1f * word(dht_dat[2] & 0x7F, dht_dat[3]);
  else
    SI7021_temperature = 0.1f * word(dht_dat[2], dht_dat[3]);
  SI7021_humidity = 0.1f * word(dht_dat[0], dht_dat[1]); // Humidity


  if (isnan(SI7021_temperature) || isnan(SI7021_humidity))
  {
    return false;
  }

  return true;
}
