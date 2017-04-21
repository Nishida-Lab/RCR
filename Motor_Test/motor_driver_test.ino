//////////////////////////////////////////////////////////////
// Motor driver test program
// This program is to test the motor driver (Cytron MD10C).
// Author : RyodoTanaka
// Date   : 2015.4.8(Wed)
//////////////////////////////////////////////////////////////



// These parts are for manual pwm from here ----------
#define DUTY_RATE 0.99
#define PWM_FREQUENCY 20 //[kHz]

double pwm_delay;
// to here --------------------------------------------

void setup(void)
{
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  digitalWrite(8, HIGH);
  digitalWrite(9, LOW);

  // for manual pwm 
  // pwm_delay = 1000 / PWM_FREQUENCY;   
  
  // wait to start
  delay(1000);
 }

void loop(void)
{
  // auto pwm (about 490 [Hz])
  analogWrite(9, 150);

  // for manual pwm ---------------------------------------------
  /* digitalWrite(9,HIGH); */
  /* delayMicroseconds(pwm_delay * DUTY_RATE); */
  /* digitalWrite(9,LOW); */
  /* delayMicroseconds(pwm_delay * (1.0 - DUTY_RATE)); */
  // to here ----------------------------------------------------
}