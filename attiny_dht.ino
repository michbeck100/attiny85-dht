#include <readVcc.h>
#include <dht.h>
#include <avr/sleep.h>

dht DHT;

int MIN_V = 3200; // empty voltage (0%)

const int unit = 3; //Change the unit number here for multple sensors

//The values below have been identified by the documentation on weather1
#define SIGNAL_BREAK 456    //
#define SIGNAL_SHORT 1990   //
#define SIGNAL_LONG 3940    //    DHT Stuff, RF protocol
#define SIGNAL_STOP 9236    //
#define REPEATS 7           //
#define RFPIN 2       //Pin that the RF transmitter is connected to
#define DHT22_PIN 4  //Pin that the DHT data is connected to

int MODULE_ID = (unit);    //Used to identify the module in pimatic (1-255)
int CHANNEL = 1;      //Multiple channels can be used (can be 1 - 4)
int TEMP_UNIT = 'C';  //may be set to F
int ID = 123;          // KAKU address

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

/**
  This is a template of the binary message that will be send encoded
  by pulses of the RF transmitter
  Here is what the bits mean
  0101 | 11010000 | 00 | 00 | 000100001001 | 00111101
   ?        ID      BT   CH      Temp.        Humid.

  The temperature in this example is 26.5° (but sended at 265) and will
  be decoded in Pimatic again to the correct value.
  The value of the relative humidity is 65%
*/
byte message[] = {
  0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1
};

/*
  @setMessage(int val, int startPos, int endPos)
  convert an integer to binary representation and
  write the output to the locations specified in the
  message[]
*/
void setMessage(int val, int startPos, int endPos) {
  int result[12];
  int theValue = val;

  for (byte i = 0, j = 11; i < 12; ++i, j--) {
    result[j] = theValue & (1 << i) ? 1 : 0;
  }

  for (byte i = endPos, j = 11; i > startPos - 1; i--, j--) {
    message[i] = result[j];
  }
}

int watchdog_counter = 0;

//****************************************************************
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {

  byte bb;
  int ww;
  if (ii > 9 ) ii = 9;
  bb = ii & 7;
  if (ii > 7) bb |= (1 << 5);
  bb |= (1 << WDCE);
  ww = bb;


  MCUSR &= ~(1 << WDRF);
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

/*
  @getDHTvalues()
  wait for two seconds to read the values of the DHT and
  replace their counterparts in the message[]
*/
void getDHTvalues() {
  //delay(2000); //DHT needs 2 seconds to get started
  //int h = (int) dht.getHumidity();
  //int t = (int) (dht.getTemperature() * 10);

  int chk = DHT.read22(DHT22_PIN);
  float h = DHT.humidity;
  float t = DHT.temperature * 10;

  // Measure battery
  float batteryV = readVcc();
  int batteryOk = 1;
  if (batteryV < MIN_V) {
    batteryOk = 0;
  }

  //if the values could not be red
  if (isnan(h) || isnan(t)) {
    setMessage(0, 16, 27); //Temperature
    setMessage(0, 28, 35); //Humidity    
  } else {
    setMessage(t, 16, 27); //Temperature
    setMessage(h, 28, 35); //Humidity
  }
  setMessage(batteryOk, 12, 12); //Battery
}
/*
  @sendMessage()
  execute the getDHTvalues() method to get some input in the
  message[] and then send the message X times using the sendBit() method
  followed by the terminator pulse
*/
void sendMessage() {
  getDHTvalues();
  for (byte i = 0; i < REPEATS; i++) {
    for (byte i = 0; i < sizeof(message); i++) {
      sendBit(message[i]);
    }
    sendTerminator();
  }
}

/*
  @sendBit(byte b)
  takes a given byte which is either 1 or 0 and then sends it
  over the air using the breaks necessary for pimatic to recognize
  the profile of the a weather1 station
*/
void sendBit(byte b) {

  if (b == 0) {
    digitalWrite(RFPIN, HIGH);
    delayMicroseconds(SIGNAL_BREAK);
    digitalWrite(RFPIN, LOW);
    delayMicroseconds(SIGNAL_SHORT);
  }
  else {
    digitalWrite(RFPIN, HIGH);
    delayMicroseconds(SIGNAL_BREAK);
    digitalWrite(RFPIN, LOW);
    delayMicroseconds(SIGNAL_LONG);
  }
}

/*
  @sendTerminator()
  This method sends Arnold Schwarzenegger back in time. Just kidding,
  actually it does deliver the long break to show the end of the
  message[]
*/
void sendTerminator() {
  digitalWrite(RFPIN, HIGH);
  delayMicroseconds(SIGNAL_BREAK);
  digitalWrite(RFPIN, LOW);
  delayMicroseconds(SIGNAL_STOP);
  digitalWrite(RFPIN, HIGH);
  digitalWrite(RFPIN, LOW);
}

//****************************************************************
// Watchdog Interrupt Service / is executed when  watchdog timed out
ISR(WDT_vect) {
  watchdog_counter++;
}

//****************************************************************
// set system into the sleep state
// system wakes up when wtchdog is timed out
void system_sleep() {

  cbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter OFF

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();

  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out
  sbi(ADCSRA, ADEN);                   // switch Analog to Digitalconverter ON

}

void setup() {
  pinMode(RFPIN, OUTPUT);
  setMessage(MODULE_ID, 4, 11);
  setMessage(CHANNEL - 1, 14, 15);
  sbi(GIMSK, PCIE);
  setup_watchdog(6);
}

void loop() {
  if (watchdog_counter == 60) { //Set how long between wakeups here, in seconds
    watchdog_counter = 0;
    sendMessage();
    delay(10);
  }
  system_sleep();
  sei();
}
