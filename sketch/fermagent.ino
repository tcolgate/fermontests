#include <Wire.h>
#include <SD.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <RCSwitch.h>
#include <RTClib.h>
#include <DallasTemperature.h>

// Define pins you're using for serial communication
// for the BlueSMiRF connection
// The TX pin on the arduino connects to the RX pin on the bluesmirf,
// and RX on arduino side, to TX on the bluesmirf
#define TXPIN 3
#define RXPIN 2
SoftwareSerial BlueSerial(RXPIN, TXPIN);

// This is the pin connecting to the Remote control transmit
#define RCPIN 13

#define I2CPIN 2

OneWire ds(I2CPIN); 

// Create an instance of the software serial object
RCSwitch mySwitch = RCSwitch();

void setup(void) {
 /*
  // Define the appropriate input/output pins
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);

  mySwitch.enableTransmit(RCPIN);  // Using Pin #10

  // Begin communicating with the bluetooth interface
  Serial.begin(9600);
  BlueSerial.begin(9600);
  
  // Say we are starting the serial com
  Serial.println("Serial start!");
  BlueSerial.println("Serial start!");
  */
  Serial.begin(9600);

}

void loop(void) {
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
      Serial.print("No more addresses.\r\n");
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
}
