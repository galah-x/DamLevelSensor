//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'functions : dam sonar level sensor  Time-stamp: "2023-07-30 11:17:49 john"';


// generally used to flash a led on for the period specified
void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

// read and print temperature to the serial port
void serial_print_temp(void)
{
  temperature = tempsensor.readTemperature();  
  Serial.print(fTEMP);  // say Temp=
  dtostrf(temperature, 3, 1, bufnum);
  sprintf((char*)buf, "%sC", bufnum);  
  Serial.println((char *)buf);
}

// turn the Sonar modle on and off.
// The board uses 2 outputs, with a 150 ohm resistor from the precharge to the Sonar
// power Vcc pin. There is also a 100uf cap across the Sonar Vcc - Gnd.
// While the moteino digital IOs can supply the ~15mA average current the
// module uses, they can't handle the peak sounder current. Hence the external cap.   
// Not sure if the module has internal caps, but this approach should limit peak
// moteino IO current and possible related ground bounce.
// 0 == off, 1 == on.
void power_sonar( uint8_t level)
{
  if (level == 0) 
    { // off
      pinMode(SONAR_POWER, INPUT);
      digitalWrite(SONAR_PRECHARGE, LOW);
    }
  else
    { // on
      pinMode(SONAR_POWER, INPUT);
      digitalWrite(SONAR_PRECHARGE, HIGH);
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
      digitalWrite(SONAR_POWER, HIGH);
      pinMode(SONAR_POWER, OUTPUT);
      LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
    }
}


// Return a sonar distance/depth (in mm)
// returns -1 if no valid (good csum) reading is received before timeout,
// Timeout is passed in in Milliseconds.
// This routine optionally pats the dog.
// the routine manages enable + disable of the sonar power.
// This routine switches between SW serial ports so as to ensure the rx queue is flushed 
// I observe just over 100ms after power is up before the first reading is sent from the
// Sonar.

int16_t sonar_depth(uint32_t timeout, uint8_t enable_pat)
{
  uint32_t start_time;
  uint8_t hbyte;
  uint8_t lbyte;
  uint8_t calc_csum;
  uint8_t  c;

  // the sonar sensor sends the 4 bytes in order,
  // Repeating every few hundred milliseconds. 
  // 0 = waiting for frame (0xff)
  // 1   waiting for hbyte
  // 2   waiting for lbyte
  // 3   waiting for csum
  uint8_t state_var;

  power_sonar(1);
  SonarComms.listen();
  state_var = 0;
  start_time = millis();
  while( (millis()-start_time) < timeout)
    {
      if (SonarComms.available())
	{
	  c = SonarComms.read();
	  switch (state_var)
	    {
	    case 0 : // wait for frame
	      if (c == 0xff)
		state_var = 1;
	      break;

	    case 1 : // wait for hbyte
	      hbyte = c;
	      state_var = 2;
	      break;
	      
	    case 2 : // wait for lbyte
	      lbyte = c;
	      state_var = 3;
	      break;
	      
	    case 3 : // wait for csum
	      state_var = 0;
	      calc_csum = 0xff + hbyte + lbyte;
	      if (calc_csum == c)
		{
		  // this is the normal routine exit, for a valid measurement
		  UnusedComms.listen();
		  power_sonar(0);
		  return (hbyte << 8) + lbyte;
		}
	      break;
	      
	    default : 
	      state_var = 0;
	      break;
	    }
	}
      if (enable_pat)
	wdt_reset();
    }
  // this is the errored return path.
  UnusedComms.listen();
  power_sonar(0);
  return -1;
}
  
