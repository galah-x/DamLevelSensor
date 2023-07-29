//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'dam sonar level sensor  Time-stamp: "2023-07-29 14:40:31 john"';

// currently using arduino-1.8.10

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <avr/wdt.h>       // watchdog
#include <LowPower.h>      // sleep mode
#include <SoftwareSerial.h>
#include "pinout.h"        // pin map
#include "addresses.h"     // radio addresses
#include "cal.h"           // adc + resistor cal values

// the F() macro puts strings into flash, not RAM. slower but lighter on RAM.
#define fVERSION  F("20230729")
#define fIAM      F("DamLevel")
#define fTEMP     F("Temp=")

// debugging
#define DOSERIAL
#define SERIAL_BAUD 115200

// #define DORADIO


Adafruit_BMP280 tempsensor;    // I2C temperature sensor is a bmp280
float temperature;
float batt_v;

// The moteino only has one hw serial port, which is used for debug.
// Going to use SW serial for the Sonar sensor, as its at 9600 baud.
// I know when its supposed to be on. And its RX only.
// However as the SW serial doesn't support an empty with only one port, I'll
// create a second one to switch to.
SoftwareSerial SonarComms (FROM_SONAR_TX, TO_SONAR_DUMMY_RX);
SoftwareSerial UnusedComms (SECOND_SWSERIAL_RX, TO_SONAR_DUMMY_RX);

#ifdef DORADIO

// Singleton instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, NODEID);
#endif

const uint8_t SHORT_RH_RF95_MAX_MESSAGE_LEN = 50;
uint8_t buf[SHORT_RH_RF95_MAX_MESSAGE_LEN];
char bufnum[8];    // want to allow -ddd.dd<null>
char bufnum2[8];

uint8_t loopcounter_2secs ; 
uint8_t loopcounter_mins ; 
uint8_t loopcounter_hours ; 

// needs the odd global variable
#include "functions.h"     // various named functions


void setup()
{
#ifdef DOSERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.print("0x");
  Serial.print(NODEID,HEX);
  Serial.print(" ");
  Serial.print(fIAM);
  Serial.print(" ");
  Serial.print(fVERSION);
  Serial.println();
#endif


  digitalWrite(SONAR_OUTSEL, 0);       //  average/processed not instantaneous
  // I played with this, 0 seemed a littles slower + better filtered.
  
  digitalWrite(SONAR_PRECHARGE,0);     //  off.
  digitalWrite(SONAR_POWER, 0);        //  off.
  digitalWrite(TO_SONAR_DUMMY_RX, 0);  //  whatever
  
  pinMode(FROM_SONAR_TX, INPUT);
  pinMode(SECOND_SWSERIAL_RX, INPUT_PULLUP);
  pinMode(TO_SONAR_DUMMY_RX, OUTPUT);
  pinMode(SONAR_OUTSEL,  OUTPUT);
  pinMode(SONAR_PRECHARGE,  OUTPUT);
  pinMode(SONAR_POWER,  OUTPUT);

  power_sonar(0);
  
  SonarComms.begin(9600);
  UnusedComms.begin(9600);
  // switch to a high input
  UnusedComms.listen();
  
  // blink the led a couple of times to show boot visually
  pinMode(LED, OUTPUT);
  delay(1000);
  Blink(LED, 100);
  delay (100);
  Blink(LED, 100);
  delay (100);

 loopcounter_2secs = 0; 
 loopcounter_mins = 0 ; 
 loopcounter_hours = 0 ; 

  
#ifdef DORADIO
#ifdef DOSERIAL      // note these serial calls are nested inside the DORADIO enable
  Serial.println(F("now init radio"));
#endif
  
  // initialize the radio if enabled
  if (!manager.init()) {
    while (1) {  // radio broken, just blink fast.
      Blink(LED, 100);
      delay (100);
    }
  }
  driver.setFrequency(915);  // to be sure, to be sure
  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  //set for pre-configured slow,long range
  driver.setTxPower(17);     //set for 50mw , +17dbm
  
#ifdef DOSERIAL
  Serial.println(F("inited radio"));
#endif

#endif
  
  if (!tempsensor.begin(0x76))  // ebay module is at address 0x76 not (adafruit) 0x77
    {
      while (1) {  // temp sensor broken, just blink med fast.
        Blink(LED, 200);
        delay (200);
      }
    }
  
#ifdef DOSERIAL
  Serial.println(F("initted temp sensor"));
#endif

#ifdef DOSERIAL
  serial_print_temp();
  Serial.flush();
#endif

  // Greet(header, CurrentMotorState);
#ifdef DOSERIAL
  Serial.println("greeted");
  Serial.flush();
#endif

  // LastPollTime = millis();
  wdt_enable(WDTO_8S);

#ifdef DOSERIAL
  Serial.println(F("done setup"));
  Serial.flush();
#endif
}

void loop(void)
{
  int16_t depth;
  loopcounter_2secs++ ;
  if (loopcounter_2secs > 29)  // 1 minute
    {
      loopcounter_2secs = 0;
      loopcounter_mins ++;
    }
  if (loopcounter_mins > 59)
    {
      loopcounter_mins=0;
      loopcounter_hours++;
    }

  if (loopcounter_2secs == 14) // every half minute
    {
#ifdef DOSERIAL
      serial_print_temp();
      Serial.print(loopcounter_mins);
      Serial.print(" ");
      Serial.println(loopcounter_hours);
      Serial.flush();
#endif
      // I observe it takes the Sonar about 1.05 seconds from power on to first message
      depth = sonar_depth(3000, 1); // timeout in 3000ms, pat dog while waiting.
      if (depth < 0)
	Serial.println("No valid depth received");
      else
	{
	  Serial.print("depth=");
	  Serial.println(depth);
	}
      Serial.flush();
    }
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  // pat the dog
  wdt_reset();
}

