//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'dam sonar level sensor  Time-stamp: "2023-07-30 10:42:13 john"';

// Im currently using arduino-1.8.10 as thats where some of the libraries are installed.
// for a moteino 328P with LoRa radio,  no USB. no flash.

#include <RHReliableDatagram.h>
#include <RH_RF95.h>         // last 2 for the NRF95 LoRa module 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h> // last 3 for the temp sensor
#include <avr/wdt.h>         // watchdog
#include <LowPower.h>        // sleep mode
#include <SoftwareSerial.h>  // for the Sonar module
#include "pinout.h"          // IO pin map
#include "addresses.h"       // radio addresses
#include "cal.h"             // adc + resistor cal values
// #include "functions.h"  is positioned after the local variable defines

// the F() macro puts strings into flash, not RAM.
// Lighter on RAM which on a 328P is more limiting
#define fVERSION  F("20230730")
#define fIAM      F("DamLevel")
#define fTEMP     F("Temp=")

// onboard HW serial is used for debug. Should not be defined for normal operation
// this is to the programmer connector.   The  arduino  Serial. object. 
// this #define does NOT affect SoftwareSerial to the Sonar module.
#define DOSERIAL
#define SERIAL_BAUD 115200

// The radio is used for normal operation. Generally turn radio on and serial off
// for normal installed use.
// Mainly controllable here since LoRa in my office affects the link to relay, LogRX,
// poorly. So I do most initial runup on serial debug.
// #define DORADIO

Adafruit_BMP280 tempsensor;    // I2C temperature sensor is a bmp280
float temperature;
float batt_v;

// The moteino only has one hw serial port, which is used for debug.
// Im going to use SW serial for the Sonar sensor, as
// (1) its slow at 9600 baud.
// (2) I know when its supposed to be on.
// (3) its RX only, all suiting the pin interrupt, half duplex SW model
// However, as the SW serial doesn't (appear to) support a queue empty when only
// one port exists, and as a powerdown of the sonar looks a lot like a start bit, I'll
// create a second imaginary serial port to switch to and from.
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

// functions.h needs the odd global variable, so its down here.
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
  // I played with this, 0 seemed a little slower + better filtered.
  
  digitalWrite(SONAR_PRECHARGE,0);     //  power off.
  digitalWrite(SONAR_POWER, 0);        //  power off.
  digitalWrite(TO_SONAR_DUMMY_RX, 0);  //  imaginary output pin for SWserial
  pinMode(FROM_SONAR_TX, INPUT);       // SWSerial input
  pinMode(SECOND_SWSERIAL_RX, INPUT_PULLUP); // imaginary second SWSERIAL input
  pinMode(TO_SONAR_DUMMY_RX, OUTPUT);  // imaginary SWSerial output
  pinMode(SONAR_OUTSEL,  OUTPUT);      // Sonar filtering control
  pinMode(SONAR_PRECHARGE,  OUTPUT);   // Sonar power 
  pinMode(SONAR_POWER,  OUTPUT);       // main Sonar power 
  pinMode(BATT_ADC, INPUT);            // battery voltage monitor
  
  power_sonar(0);
  SonarComms.begin(9600);
  UnusedComms.begin(9600);
  UnusedComms.listen();      // switch SWSerial to a high unused input
  
  // now blink the led a couple of times to show booting visually
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
  
  // initialize the radio if it should be enabled
  if (!manager.init()) {
    while (1) {  // radio broken, just blink fast.
      Blink(LED, 100);
      delay (100);
    }
  }
  driver.setFrequency(915);    // to be sure, to be sure
  driver.setModemConfig(RH_RF95::Bw31_25Cr48Sf512);  //set for pre-configured slow,long range
  driver.setTxPower(17);       //set for 50mw , +17dbm
  
#ifdef DOSERIAL
  Serial.println(F("inited radio"));
#endif

#endif   // end of DORADIO switch

  // initialize the temperature sensor 
  if (!tempsensor.begin(0x76))  // ebay module is at address 0x76 not (adafruit) 0x77
    {
      while (1) {  // temp sensor is broken, just blink med fast.
        Blink(LED, 200);
        delay (200);
      }
    }
  
#ifdef DOSERIAL
  Serial.println(F("initted temp sensor"));
#endif

#ifdef DOSERIAL
  serial_print_temp();
  Serial.print("batt=");
  batt_v = analogRead(BATT_ADC);
  Serial.print(batt_v * BATT_GAIN);
  Serial.println("V");
  Serial.flush();
#endif

  // this is likely the radio greeting
  // Greet(header, CurrentMotorState);
#ifdef DOSERIAL
  Serial.println("greeted");
  Serial.flush();
#endif

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
  // repeated action after a desired interval 
  if ((loopcounter_hours == 1) &&
      (loopcounter_mins == 0) &&
      (loopcounter_2secs == 0))
    {
      loopcounter_hours = 0;
      loopcounter_mins = 0;
      loopcounter_2secs = 0;
      
#ifdef DOSERIAL
      serial_print_temp();
      Serial.flush();
      // read battery before enabling sonar, which draws ~~15mA.
      Serial.print("batt=");
      batt_v = analogRead(BATT_ADC);
      Serial.print(batt_v * BATT_GAIN);
      Serial.println("V");
      Serial.flush();
      // I observed it takes the Sonar about 1.05 seconds from power on to first message
      depth = sonar_depth(3000, 1); // timeout in 3000ms, pat dog while waiting.
      if (depth < 0)
	Serial.println("No valid depth received");
      else
	{
	  Serial.print("depth=");
	  Serial.println(depth);
	}
      Serial.flush();
#endif
    }
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
  wdt_reset();    // pat the dog
}

