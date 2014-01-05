#include <Wire.h>
#include <SD.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <RCSwitch.h>
#include <RTClib.h>
#include <DallasTemperature.h>

/**Pins
 * 0  - oboard tx
 * 1  - oboard rx
 * 2  - Bluesmird RX
 * 3  - Bluesmird TX
 * 4  - I2C
 * 5
 * 6
 * 7
 * 8
 * 9
 * 10 - Adafruit datalogger CS pin
 * 11
 * 12
 * 13 - Remote Control simulation pin
 */
// Define pins you're using for serial communication
// for the BlueSMiRF connection
// The TX pin on the arduino connects to the RX pin on the bluesmirf,
// and RX on arduino side, to TX on the bluesmirf
#define RXPIN 2
#define TXPIN 3
// I2C bus pin for the temperature sensors
#define I2CPIN 4
// SD Card pin - default on the adafruit logging board
#define SDPIN 10
// This is the pin connecting to the Remote control transmit
#define RCPIN 13



/* Read the internal voltage */
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

SoftwareSerial BlueSerial(RXPIN, TXPIN);
void setupBluetooth(void){
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);

  BlueSerial.begin(9600);
}

RCSwitch mySwitch = RCSwitch();
void setupRemoteControl(void){
  mySwitch.enableTransmit(RCPIN);
}

void setupSDCardReader(void){
  pinMode(SDPIN, OUTPUT);
  if (!SD.begin(SDPIN)) {
    Serial.println("Card failed, or not present");
  }
  Serial.println("card initialized.");
}

RTC_DS1307 RTC; // define the Real Time Clock object
void setupRTC(void){
  // connect to RTC
  if (!RTC.begin()) {
    Serial.println("RTC failed");
  }

  if (! RTC.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
}

OneWire ds(I2CPIN);
void setupSensors(void){
}

void setup(void) {
  Wire.begin();
  Serial.begin(9600);

  setupRTC();
  setupBluetooth();
  setupSDCardReader();
  setupRemoteControl();
  setupSensors();
}

void loop(void) {
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
  DateTime now = RTC.now();

 /*
  // Wait for command-line input
  if(Serial.available() > 0)
  {
    // Read off all bytes
    BlueSerial.print( (char) Serial.read());
  }

  if(BlueSerial.available() > 0)
  {
    // Read off all bytes
    Serial.print( (char) BlueSerial.read());
  }
 */

//  mySwitch.sendTriState("00000FFF0F0F");
//  delay(1000);
//  mySwitch.switchOn(1, 1);         // Switch 1st socket from 1st group on
//  delay(1000);
//  mySwitch.switchOff(1, 1);        // Switch 1st socket from 1st group off
//  delay(1000);



  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      ds.reset_search();
      return;
  }

  Serial.print("R=");
  for( i = 0; i < 8; i++) {
    Serial.print(addr[i], HEX);
    Serial.print(" ");
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\r\n");
      return;
  }

  if ( addr[0] == 0x10) {
      Serial.print("Device is a DS18S20 family device.\r\n");
  }
  else if ( addr[0] == 0x28) {
      Serial.print("Device is a DS18B20 family device.\r\n");
  }
  else {
      Serial.print("Device family is not recognized: 0x\r\n");
      Serial.println(addr[0],HEX);
      return;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end

  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.

  present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("P=");
  Serial.print(present,HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print( OneWire::crc8( data, 8), HEX);
  Serial.println();

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;


  if (SignBit) // If its negative
  {
     Serial.print("-");
  }
  Serial.print(Whole);
  Serial.print(".");
  if (Fract < 10)
  {
     Serial.print("0");
  }
  Serial.print(Fract);

  Serial.print("\r\n");

  Serial.print("Battery level: ");
  Serial.print(readVcc());
  Serial.print("\r\n");

  Serial.print("Date: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}
