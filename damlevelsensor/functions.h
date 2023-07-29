//    -*- Mode: c++     -*-
// emacs automagically updates the timestamp field on save
// my $ver =  'functions : dam sonar level sensor  Time-stamp: "2023-07-29 14:34:03 john"';



void Blink(byte PIN, int DELAY_MS)
{
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

void serial_print_temp(void)
{
  temperature = tempsensor.readTemperature();  
  Serial.print(fTEMP);  // say Temp=
  dtostrf(temperature, 3, 1, bufnum);
  sprintf((char*)buf, "%sC", bufnum);  
  Serial.println((char *)buf);
}

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


// return a sonar distance/depth (in mm)
// manages enable + disable of the sonar,
// returns -1 if no valid reading before timeout, which is passed in units: msecs.
// this routine can pat the dog. 
int16_t sonar_depth(uint32_t timeout, uint8_t enable_pat)
{
  uint32_t start_time;
  uint8_t hbyte;
  uint8_t lbyte;
  uint8_t calc_csum;
  uint8_t  c;

  // 0 = waiting for frame (0xff)
  // 1 waiting for hbyte
  // 2 waiting for lbyte
  // 3 waiting for sum
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
  UnusedComms.listen();
  power_sonar(0);
  return -1;
}
  
