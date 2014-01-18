#include <Wire.h>
#include <SD.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <RCSwitch.h>
#include <RTClib.h>
#include <DallasTemperature.h>

/*
#include <avr/power.h>
#include <avr/sleep.h>

int sleepStatus = 0;             // variable to store a request for sleep
int count = 0;                   // counter

void sleepNow()
{
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
                
  power_adc_disable();
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();
  
  sleep_mode();            // here the device is actually put to sleep!!
 
  sleep_disable();         // first thing after waking from sleep:
                            // disable sleep...
  power_all_enable();
}
*/


/**Pins
 * 0  - oboard tx
 * 1  - oboard rx
 * 2  - Bluesmird RX
 * 3  - Bluesmird TX
 * 4  - DS OneWire control
 * 5  - DS OneWire Vcc (maybe)
 * 6
 * 7
 * 8
 * 9 -  Remote Control simulation pin
 * 10 - SPI Adafruit datalogger CS pin
 * 11 - SPI MOSI
 * 12 - SPI MISO
 * 13 - SPI SCK
 */
// Define pins you're using for serial communication
// for the BlueSMiRF connection
// The TX pin on the arduino connects to the RX pin on the bluesmirf,
// and RX on arduino side, to TX on the bluesmirf
#define RXPIN 2
#define TXPIN 3
// OneWire bus pin for the temperature sensors
#define ONEWIREPIN 4
#define ONEWIREPOWERPIN 5
// This is the pin connecting to the Remote control transmit
#define RCPIN 9
// SD Card pin - default on the adafruit logging board
#define SDPIN 10

void notice(String msg);
void  debug(String msg);
void   info(String msg);
void  error(String msg);
void   outputSensor(String type, String id, String outstr);

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

/* 
// set up variables using the SD utility library functions:
SdVolume volume;
SdFile root;
void cardInfo(void){
  // print the type of card
  debug("Card type: "));
  switch(SD.type()) {
    case SD_CARD_TYPE_SD1:
      debug("SD1"));
      break;
    case SD_CARD_TYPE_SD2:
      debug("SD2"));
      break;
    case SD_CARD_TYPE_SDHC:
      debug("SDHC"));
      break;
    default:
      debug("Unknown"));
  }

  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(SD)) {
    debug("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  debug("Volume type is FAT"));
  debug(String(volume.fatType(), DEC));
  
  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  volumesize /= 1024;
  volumesize /= 1024;
  debug(String("Volume size (Mbytes): ") + String(volumesize));

  //debug("Files found on the card (name, date and size in bytes): "));
  //root.openRoot(volume);
  //root.ls(LS_R | LS_DATE | LS_SIZE);
}
 */

#define USB_SERIAL 0
#define BT_SERIAL  1
#define CHAN_CNT   2
#define CHAN_SZ    64 
char  chan_buffs[CHAN_CNT][CHAN_SZ];
int   chan_curr[CHAN_CNT];

void setupChans(void){
  for(int i = 0 ; i < CHAN_CNT ; i++){
     chan_curr[i]  = 0;
  }
}

bool debugMsgs = false;
void cmd_debug(int argc, char** argv){
  if(argc != 1){
    if (debugMsgs){
      info(String("debug on"));
    } else {
      info(String("debug off"));
    };
  } else {
    if (String(argv[1]) == String("on")){
      debugMsgs = true;
      info(String("debug on"));
    } else {
      debugMsgs = false;
      info(String("debug off"));
    };
  }
};

void cmd_view(int argc, char** argv){
};

void cmd_poll(int argc, char** argv);


typedef void (*cmd_func)(int argc, char** argv);
typedef struct cmd_hash_item_s {
  String name;
  cmd_func f;
} cmd_hash_item;

cmd_hash_item cmd_hash[] = {
  {String("debug"),   cmd_debug},
  {String("view"), cmd_view},
  {String("poll"), cmd_poll},
  {NULL,NULL}
};

void run_cmd(int argc, char** argv){
  String cmd = String(argv[0]);

  debug(String("Command: ") + cmd);
  for(int i = 1; i <= argc; i++){
    debug(String("Arg[") + String(i,DEC) + "]: " + String(argv[i]));
  }

  for(int i = 0; cmd_hash[i].name != NULL; i++){
    if(cmd_hash[i].name == cmd){
      (cmd_hash[i].f)(argc, argv); 
      return;
    }
  }

  error(String("Unknown Command: ") + cmd);
}

#define ARGS_MAX  8
void dispatch_cmd(int chan){
  char* buff = chan_buffs[chan];
  int argc = 0;
  char* argv[ARGS_MAX];
  memset(argv,0,ARGS_MAX * sizeof(char*));
  argv[0] = buff;

  for(; *buff != (char) NULL; buff++){
    if (*buff == ' '){
      *buff = (char) NULL;
      buff++;
      argc++;
      argv[argc] = buff;
    }
  }

  run_cmd(argc,argv); 
}

void dispatch_error(int chan, String err){
  error(String("Error on chan[") + String(chan, DEC) + String("]: ") + err);
}

void dispatchByte(int chan, char c){
  int pos = chan_curr[chan];
  
  if (c == '\r'){
    chan_buffs[chan][pos] = (char) 0;
    dispatch_cmd(chan);
    chan_curr[chan] = 0;
    chan_buffs[chan][0] = 0;
  } else {
    if (chan_curr[chan] >= CHAN_SZ){
      dispatch_error(chan,String("Input too big"));
      chan_buffs[chan][0] = 0;
      chan_curr[chan] = 0;
    } else {
      chan_buffs[chan][pos] = c;
      chan_curr[chan] = pos + 1;
    }
  }
}

void setupSDCardReader(void){
  pinMode(SDPIN, OUTPUT);
  if (!SD.begin(SDPIN)) {
    error("Card failed, or not present");
  }
  info("card initialized.");
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
}

void printDate(int,DateTime);
RTC_DS1307 RTC; // define the Real Time Clock object
void setupRTC(void){
  // connect to RTC
  if (!RTC.begin()) {
    error("RTC failed");
  }

  if (! RTC.isrunning()) {
    DateTime builddate = DateTime(__DATE__, __TIME__);
    info("RTC not running!");
    info("Set date to ") ;
    printDate(USB_SERIAL, builddate);
    Serial.println();

    RTC.adjust(builddate);
  }
}

void setupSensors(void){
  pinMode(ONEWIREPOWERPIN, OUTPUT);
  digitalWrite(ONEWIREPOWERPIN, HIGH);

  digitalWrite(ONEWIREPOWERPIN, LOW);
}

void setup(void) {
  Wire.begin();
  Serial.begin(9600);

  setupRTC();
  setupChans();
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

  digitalWrite(ONEWIREPOWERPIN, HIGH);
  delay(5);
  OneWire ds(ONEWIREPIN);

  while (ds.search(addr)) {
    const String unknown = String("Unknown");
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
        notice("Unknown device type");
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

    msg = String("P=") + String(present, HEX);
    msg += " D=";
    for ( i = 0; i < 9; i++) {           // we need 9 bytes
      data[i] = ds.read();
      msg += String(data[i], HEX);
    }
    msg += String(" CRC=");
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

  digitalWrite(ONEWIREPOWERPIN, LOW);

  return;
}

void scanSensors(void) {
  outputSensor("Vcc","in", "V=" + String(readVcc()) + "mV");
  scanDSSensors();
}

void cmd_poll(int argc, char** argv){
  scanSensors();
};

void printDate(int chan,DateTime dt){
  if(chan == USB_SERIAL){
    Serial.print(dt.year());
    Serial.print('/');
    Serial.print(dt.month());
    Serial.print('/');
    Serial.print(dt.day());
    Serial.print(' ');
    Serial.print(dt.hour());
    Serial.print(':');
    Serial.print(dt.minute());
    Serial.print(':');
    Serial.print(dt.second());
  } else if (chan == BT_SERIAL) {
    BlueSerial.print(dt.year());
    BlueSerial.print('/');
    BlueSerial.print(dt.month());
    BlueSerial.print('/');
    BlueSerial.print(dt.day());
    BlueSerial.print(' ');
    BlueSerial.print(dt.hour());
    BlueSerial.print(':');
    BlueSerial.print(dt.minute());
    BlueSerial.print(':');
    BlueSerial.print(dt.second());
  }
}


void loop(void) {
  int idle_count = 0;
  bool idle = true;
  int incoming;
  //scanSensors();

  // Wait for command-line input
  while(Serial.available() > 0)
  {
    incoming = Serial.read();
    dispatchByte(USB_SERIAL, (char)incoming);
    idle_count = 0;
    idle = false;
  }

//  while(BlueSerial.available() > 0)
//  {
//    incoming = BlueSerial.read();
//    dispatchByte(BT_SERIAL,incoming) ;
//    idle_count = 0;
//    idle = false;
//  }

//  mySwitch.sendTriState("00000FFF0F0F");
//  delay(1000);
//  mySwitch.switchOn(1, 1);         // Switch 1st socket from 1st group on
//  delay(1000);
//  mySwitch.switchOff(1, 1);        // Switch 1st socket from 1st group off
//  delay(1000);

  if(idle){
    idle_count++;
    //delay(1000);
  };

  //if(idle && idle_count > 10){
  //sleepNow();
  //}
}

// How we output sensor values
// These log to the serial port
void logbase(String prefix, String lvl, String msg){
  DateTime now = RTC.now();
  Serial.print(prefix);
  printDate(USB_SERIAL,now);
  Serial.print(" ");
  Serial.print(lvl);
  Serial.print(": ");
  Serial.flush();
  Serial.println(msg);
  Serial.flush();
//  if( debugMsgs){
//   BlueSerial.println(String("# ") + strDate(now) + String(" ") + lvl + String(": ") + msg);
//  BlueSerial.flush();
//}
}

void outputSensor(String type, String id, String outstr){
  logbase("!" ,"+", type + String("[") + id + String("] ") + outstr);
}

void  debug(String msg){
  if( debugMsgs){
    logbase(String("!"),String("D") , msg);
  }
}

void notice(String msg){logbase(String("#"),String("N"), msg);}
void   info(String msg){logbase(String("#"),String("I"), msg);}
void  error(String msg){logbase(String("#"),String("E"), msg);}

