//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'dam sonar level sensor pinout Time-stamp: "2023-07-29 14:03:35 john"';

// basic moteino uses the atmega328
const byte LED         = 9;   // Moteino onboard LED is on D9

// these are the IOs used for the swing gate linear actuator
// for the prototype
const byte FROM_SONAR_TX = 3;      // input for sonar tx transmission
const byte TO_SONAR_DUMMY_RX = 7;  // sonar tx transmission
const byte SECOND_SWSERIAL_RX = 8; // switch SW serial to this when SONAR is off. 
const byte SONAR_OUTSEL = 4;  // OUTSEL , controls averaging
const byte SONAR_PRECHARGE = 5;  // Sonar Vdd via current limit resistor
const byte SONAR_POWER     = 6;  // Sonar VDD, controls averaging, with 100uf cap.

const byte BATT_ADC = A7;  // analog IO for measuring battery




