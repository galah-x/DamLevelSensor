//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'cal for dam sonar level sensor  Time-stamp: "2023-07-30 11:03:39 john"';

// BATTERY SENSE CALIBRATION 

// about : external /2,  10 bit adc, 3.3v onboard reference
// i'm going to multiply by adc reading by batt_gain to get voltage.
// BATT_GAIN = 2.0 * 3.3 / 1024.0  = 0.00644 (nominally, tweaked by a cal on the prototype.
const float BATT_GAIN  = 0.00644; 

// it seems the initial value is within a few 10s of millivolts at 4.00V on the battery. 
