

void setup() {
  pinMode(11,OUTPUT);  //LED pin (to blink in 50Hz frequency)
  pinMode(12,OUTPUT);  //LED pin 
//START TIMER SETUP
//TIMER SETUP for highly preceise timed measurements 
  cli();//stop all interrupts
  // turn on CTC mode
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaler 8
  TCCR1B |= (1 << CS11); 
  
  //initialize counter value to 0;
  TCNT1  = 0;
  
  // set timer count for 50Hz increments
  OCR1A = 39999;// = (16*10^6) / (50*8) - 1  
  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts
  //END TIMER SETUP
}

ISR(TIMER1_COMPA_vect) {//Interrupt at frequency of 50 Hz
 //write your timer code here
 digitalWrite(11,HIGH);
 delay(20);
 digitalWrite(11,LOW);

 //write your timer code here
 digitalWrite(12,HIGH);
 delay(2);
 digitalWrite(12,LOW);
}
void loop() {
//when the timer is over, your program will stop in the loop function and jump to the timer code. 
//After the timer code it will jump back to the point where it left the loop function
}
