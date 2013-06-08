#include <Wire.h>
#include <SD.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include "RCSwitch.h"
#include "RTClib.h"
#include "DallasTemperature.h"

// Define pins you're using for serial communication
// for the BlueSMiRF connection
// The TX pin on the arduino connects to the RX pin on the bluesmirf,
// and RX on arduino side, to TX on the bluesmirf
#define TXPIN 3
#define RXPIN 2
SoftwareSerial BlueSerial(RXPIN, TXPIN);

// This is the pin connecting to the Remote control transmit
#define RCPIN 13

// Create an instance of the software serial object
RCSwitch mySwitch = RCSwitch();

void setup(void) {
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
}

void loop(void) {
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

//  mySwitch.sendTriState("00000FFF0F0F");
//  delay(1000);  
//  mySwitch.switchOn(1, 1);         // Switch 1st socket from 1st group on
//  delay(1000);
//  mySwitch.switchOff(1, 1);        // Switch 1st socket from 1st group off
//  delay(1000);
}
