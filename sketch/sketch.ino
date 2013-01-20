#include <SD.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SoftwareSerial.h>

// Define pins you're using for serial communication
// for the BlueSMiRF connection
#define TXPIN 11
#define RXPIN 12
 
// Create an instance of the software serial object
SoftwareSerial BlueSerial(RXPIN, TXPIN);
void setup(void) {
  // Define the appropriate input/output pins
  pinMode(RXPIN, INPUT);
  pinMode(TXPIN, OUTPUT);

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
    // BlueSerial.print( Serial.read(), BYTE );
  }
}
