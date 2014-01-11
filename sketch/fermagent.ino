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

void notice(String msg);
void  debug(String msg);
void   info(String msg);
void  error(String msg);
String strDate(DateTime);
void   outputSensor(String type, String id, String outstr);

/* Read the internal voltage */ long readVcc() {
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
    error("Card failed, or not present");
  }
  info("card initialized.");
}

RTC_DS1307 RTC; // define the Real Time Clock object
void setupRTC(void){
  // connect to RTC
  if (!RTC.begin()) {
    error("RTC failed");
  }

  if (! RTC.isrunning()) {
    DateTime builddate = DateTime(__DATE__, __TIME__);
    info("RTC is NOT running!");
    info("Attempt to set date to " + strDate(builddate));
    RTC.adjust(builddate);
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

void scanDSSensors(void){
  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;

  byte i;
  byte present = 0;
  byte data[12];
  byte addr[8];

  while (ds.search(addr)) {
    const String unknown = "Unknown";
    String type = unknown;
    String id = "";
    String msg = "";

    if ( OneWire::crc8( addr, 7) != addr[7]) {
      error("CRC is not valid!");
      continue;
    }

    switch(addr[0]){
      case 0x10: {
        const String str = "DS18S20";
        type = str;
        break;
      }     
      case 0x28: {
        const String str = "DS18B20";
        type = str;
        break;
      }
      default: {
        notice("-- Unknown device type");
        continue;
      }
    }
    

    ds.reset();
    ds.select(addr);
    ds.write(0x44,1);         // start conversion, with parasite power on at the end

    for( i = 0; i < 8; i++) {
      id += String(addr[i], HEX);
    }

    delay(1000);     // maybe 750ms is enough, maybe not
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE);         // Read Scratchpad

    msg = "P=" + String(present, HEX);
    msg += " D=";
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      msg += String(data[i], HEX);
    }
    msg += (" CRC=");
    msg += String(OneWire::crc8( data, 8), HEX);

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


    msg += " T=";
    if (SignBit) // If its negative
    {
      msg += "-";
    }
    msg += String(Whole) + ".";

    if (Fract < 10)
    {
      msg += "0";
    }
    msg += Fract;

    msg += "C";

    outputSensor(type, id, msg);
  }

  ds.reset_search();
  return;
}

void scanSensors(void) {
  outputSensor("Vcc","in", "V=" + String(readVcc()) + "mV");
  scanDSSensors();
}

String strDate(DateTime dt){
  String res = "";
  res += dt.year();
  res += '/';
  res += dt.month();
  res += '/';
  res += dt.day();
  res += ' ';
  res += dt.hour();
  res += ':';
  res += dt.minute();
  res += ':';
  res += dt.second();

  return res;
}

void loop(void) {
  scanSensors();

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

  delay(1000);
}

// How we output sensor values
void outputSensor(String type, String id, String outstr){
  DateTime now = RTC.now();
  Serial.println("++ " + strDate(now) + " " + type + "[" + id + "] " + outstr);
}

// These log to the serial port
void logbase(String lvl, String msg){
  DateTime now = RTC.now();
  Serial.println("-- " + strDate(now) + " " + lvl + ": " + msg);
}

void notice(String msg){logbase("NOTICE", msg);}
void  debug(String msg){logbase("DEBUG" , msg);}
void   info(String msg){logbase("INFO"  , msg);}
void  error(String msg){logbase("ERROR" , msg);}

