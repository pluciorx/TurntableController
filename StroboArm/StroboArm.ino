#include <PWMServo.h>

PWMServo myservo;

#define PIN_SENSOR_ARM 8
#define PIN_LED_ARM 4

#define PIN_SERVO 9
#define PIN_STROBO 10
#define PIN_LED1 11
#define PIN_LED2 12


int pos = 0;
int pulseLength = 1000;
int aktualnaPozycja = 0;
bool IsStroboActive = false;
bool _prevStrobeState = IsStroboActive;

void setup() {

	
	Serial.begin(9600);
	myservo.attach(PIN_SERVO);  
	pinMode(PIN_SENSOR_ARM, INPUT);

	pinMode(PIN_LED_ARM, OUTPUT);
	digitalWrite(PIN_LED_ARM, LOW);

	pinMode(PIN_STROBO, INPUT);
	pinMode(PIN_LED1, OUTPUT);
	pinMode(PIN_LED2, OUTPUT);

	myservo.write(0);
}

// the loop function runs over and over again until power down or reset
void loop() {
	pos = 0;
	//Serial.println(aktualnaPozycja);
	if (digitalRead(PIN_SENSOR_ARM) == HIGH) {
		cli();//stop all interrupts
		digitalWrite(PIN_LED_ARM, HIGH);
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
		digitalWrite(PIN_LED_ARM, LOW);
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
	if (digitalRead(PIN_STROBO) == HIGH && _prevStrobeState == false)
	{
		StartStrobo();
	}
	else
	{
		if (digitalRead(PIN_STROBO) == LOW)
		{
			StopStrobo();
		}
	}
}

void StopStrobo()
{
	_prevStrobeState = false;
	cli();//stop all interrupts
	TIMSK1 &= ~(1 << OCIE1A); // Disable Timer Compare Interrupt
	sei();//allow interrupts
}

void StartStrobo()
{
	_prevStrobeState = true;
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

ISR(TIMER1_COMPA_vect) {
	digitalWrite(11, HIGH);
	delay(20);
	digitalWrite(11, LOW);

	digitalWrite(12, HIGH);
	delay(2);
	digitalWrite(12, LOW);
}