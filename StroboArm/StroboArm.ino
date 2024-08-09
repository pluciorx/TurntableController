Servo myservo;

#define PIN_czujnik1 2
#define PIN_czyNapiecie 4
#define PIN_STROBO 10
#define PIN_SERVO 9

int pos = 0;    // variable to store the servo position
int pulseLength = 1000;
int aktualnaPozycja = 0;
bool IsStroboActive = false;

void setup() {

	pinMode(11, OUTPUT);  //LED pin (to blink in 50Hz frequency)
	pinMode(12, OUTPUT);  //LED pin 

	Serial.begin(9600);
	myservo.attach(PIN_SERVO);  // attaches the servo on pin 9 to the servo object
	pinMode(PIN_czujnik1, INPUT);
	pinMode(PIN_STROBO, INPUT);

	pinMode(PIN_czyNapiecie, OUTPUT);
	digitalWrite(PIN_czyNapiecie, LOW);
	myservo.write(0);
}

// the loop function runs over and over again until power down or reset
void loop() {
	pos = 0;
	//Serial.println(aktualnaPozycja);
	if (digitalRead(PIN_czujnik1) == HIGH) {
		cli();//stop all interrupts
		digitalWrite(PIN_czyNapiecie, HIGH);
		if (aktualnaPozycja >= 0) {
			for (pos = aktualnaPozycja; pos >= 0; pos -= 1) {
				myservo.write(pos);
				delay(50);
			}
			aktualnaPozycja = pos + 1;
			//Serial.println(aktualnaPozycja);
		}
		sei();
	}
	else {
		cli();
		pos = 0;
		digitalWrite(PIN_czyNapiecie, LOW);
		if (aktualnaPozycja <= 30) {
			for (pos = aktualnaPozycja; pos <= 30; pos += 1) {
				myservo.write(pos);
				delay(50);
			}
			aktualnaPozycja = pos + 1;
			//Serial.println(aktualnaPozycja);
		}
		sei();
	}
	if (digitalRead(PIN_STROBO) == LOW)
	{
		StopStrobo();
	}
	else
	{
		StartStrobo();
	}

}

void StopStrobo()
{
	cli();//stop all interrupts
	TIMSK1 |= ~(1 << OCIE1A);
	sei();//allow interrupts
}

void StartStrobo()
{
	cli();//stop all interrupts
	// turn on CTC mode
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCCR1B |= (1 << WGM12);
	// Set CS11 bit for prescaler 8
	TCCR1B |= (1 << CS11);

	//initialize counter value to 0;
	TCNT1 = 0;

	// set timer count for 50Hz increments
	OCR1A = 39999;// = (16*10^6) / (50*8) - 1  

	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect) {//Interrupt at frequency of 50 Hz
	//write your timer code here
	digitalWrite(11, HIGH);
	delay(20);
	digitalWrite(11, LOW);

	//write your timer code here
	digitalWrite(12, HIGH);
	delay(2);
	digitalWrite(12, LOW);
}