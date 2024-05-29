#include <QuickPID.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Debounce.h>
#include <MX1508.h>


//IR Sensor
#define PIN_SPD_D0 3
#define PIN_SPD_A0 A0

//Buttons
#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7

Adafruit_Debounce btnMenuRight(PIN_BTN_RIGHT, HIGH);
Adafruit_Debounce btnMenuLeft(PIN_BTN_LEFT, HIGH);
Adafruit_Debounce btnMenuEnter(PIN_BTN_MID, HIGH);

//Motor
#define PINA 9
#define PINB 10
MX1508 motorA(PINA, PINB, FAST_DECAY, 2);
#define PWM_RESOLUTION 900

//PID
float Setpoint, Input, Output;
float Kp = 1, Ki = 0.0, Kd = 0.025;
QuickPID myPID(&Input, &Output, &Setpoint);

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
float prevSelectedSpeed = -1;
float prevMeasuredSpeed = -1;

bool isPlaying = false;
int currPWm;
volatile unsigned long numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0;

#define minPwm 550
#define maxPwm 800

float windowIntervalSec = 1; //2 seconds window - increase this if the no. of markers is less for better accuracy

#define NUM_MARKERS 54 //TO DO: Check this as per your setup 200

enum E_STATE {
	Idle,
	Starting,
	Running,	
	Stopping,
};

E_STATE _state;


void setup() {
	Serial.begin(115200);
	lcd.init(); // initialize the lcd	
	lcd.backlight();
	lcd.clear();
	
	numPulses = 0;
	//pinMode(PINA, OUTPUT);
	motorA.setPWM16(1, PWM_RESOLUTION); // prescaler at 1 , resolution 700, PWM frequency = 16Mhz/1/700~22000hz 

	pinMode(PIN_SPD_D0, INPUT_PULLUP); // declare ir as input	
	pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
	pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_BTN_MID, INPUT_PULLUP);
	
	btnMenuEnter.begin();
	btnMenuLeft.begin();
	btnMenuRight.begin();
	
	printSelectedSpeed(selectedSpeed);
	attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, RISING);

	curMillis = lastMillis = millis();
	revPerMin = 0;
	
	SetState(E_STATE::Idle);	
}

void  interruptRoutine() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	
	if (interrupt_time - last_interrupt_time > 15)
	{
		numPulses++;
	}
	last_interrupt_time = interrupt_time;

	/*if (digitalRead(PIN_SPD_D0) == HIGH) numPulses++;*/
	
}

void loop() {
	
	switch (_state)
	{
	case Idle:
	{
		printState("<-    Speed   ->");
		isPlaying = false;
		
		while (1)
		{
			printSelectedSpeed(selectedSpeed);
			updateButtons();
			if (btnMenuRight.justPressed()) {
				selectedSpeed = 45;				
			}
			if (btnMenuLeft.justPressed())	{
				selectedSpeed = 33.333; 				
			}

			Setpoint = selectedSpeed / (float)60 * (float)NUM_MARKERS;
			
			if (btnMenuEnter.justPressed())
			{
				SetState(E_STATE::Starting);
				Serial.println("Play Pressed");
				revPerMin = 0;
				Serial.print("Setpoint:"); Serial.println(Setpoint,3);
				btnMenuEnter.update();
				isPlaying = true;
				break;
			}			
		}		
	}break;
	case Starting:
	{	
		printState("    Starting    ");
		printMeasuredSpeed(0);
		
		isPlaying = true;
		SetState(E_STATE::Running);
		int x = 540;
		while (x < minPwm) 
		{
			measureSpeedOnly();
			motorA.motorGo(x);
			x++;			
			delay(50);			
		}
		Input = numPulses;
		//apply PID gains
		myPID.SetTunings(Kp, Ki, Kd);
		myPID.SetSampleTimeUs(50000);
		//turn the PID on
		myPID.SetMode(1);

	}break;
	case Running:
	{				
		printState("-    Speed     +");
		printMeasuredSpeed(0);
		while (isPlaying) 
		{
			measureSpeedOnly();
			Input = numPulses;
			myPID.Compute();
			Serial.print("Input:"); Serial.println(Input,3);	
			int newPwm = map(Output, 0, PWM_RESOLUTION, minPwm, maxPwm);

			if (newPwm != 0.0 || Output != 0.0) {
				Serial.print("NewPwm:"); Serial.println(Output, 5);
				Serial.print("Output:"); Serial.println(Output, 5);
			}
			HandleButtonsWhilePlaying();
		}
	}break;
	case Stopping:
	{
		printState("    Stopping    ");		
		int currPwm = motorA.getPWM();
		while (currPwm > 0)
		{
			measureSpeedOnly(100);
			if (currPwm < 150) currPwm = 0;
			motorA.motorGo(currPwm--);
			delay(100);
			
		}
		
		SetState(E_STATE::Idle);
	}
	}
}

static void  measureSpeedOnly()
{
	curMillis = millis();
	if (curMillis >= lastMillis + (windowIntervalSec * 1000)) {
		Input = numPulses;
		float revPerMin = 60 * (numPulses / (float)windowIntervalSec / NUM_MARKERS);
		lastMillis = curMillis;
		Serial.print("Pulses:"); Serial.println(numPulses);
	
		numPulses = 0;

		printMeasuredSpeed(revPerMin);
	}
}

static void measureSpeedOnly(long intervalMs)
{
	curMillis = millis();
	if (curMillis >= lastMillis + intervalMs) {
		float revPerMin = 60 * (numPulses / intervalMs / NUM_MARKERS);
		lastMillis = curMillis;
		Serial.print("Pulses for interval:"); Serial.println(numPulses);
		numPulses = 0;
		printMeasuredSpeed(revPerMin);
		
	}
}
//void measureSpeed()
//{
//	curMillis = millis();
//	if (curMillis >= lastMillis + (windowIntervalSec * 1000)) {
//		revPerMin = 60 * (numPulses / (float)windowIntervalSec / NUM_MARKERS);
//		lastMillis = curMillis;
//		Serial.print("Pulses:"); Serial.println(numPulses);
//		numPulses = 0;
//
//		printMeasuredSpeed(revPerMin);
//		float dev = abs(selectedSpeed - revPerMin);
//
//		Serial.print("Deviation:"); Serial.println(dev);
//		int mPwm = motorA.getPWM();
//		
//		if (dev > 0.5f)
//		{
//			
//			if (revPerMin < selectedSpeed && mPwm < maxPwm)
//				motorA.motorGo(motorA.getPWM() + 1);
//
//			if (revPerMin > selectedSpeed && mPwm > minPwm)
//				motorA.motorGo(motorA.getPWM() - 1);
//			windowIntervalSec = 0.1;
//		}
//		else {
//			sweetPwm = mPwm;
//			Serial.print("Using PWM Found:"); Serial.println(sweetPwm);
//			motorA.motorGo(sweetPwm);
//			windowIntervalSec = 2;
//			
//		}
//	}
//}

void HandleButtonsWhilePlaying()
{
	updateButtons();
	if (btnMenuRight.justPressed()) {
		selectedSpeed++;

		printSelectedSpeed(motorA.getPWM());
		motorA.motorGo(motorA.getPWM() + 1);
		delay(50);
		Serial.println(motorA.getPWM());
	}

	if (btnMenuLeft.justPressed()) {
		selectedSpeed--;
		printSelectedSpeed(motorA.getPWM());

		motorA.motorGo(motorA.getPWM() - 1);
		delay(50);
		Serial.println(motorA.getPWM());
	}

	if (btnMenuEnter.justPressed() && isPlaying)
	{
		Serial.println("Stop Pressed");
		SetState(E_STATE::Stopping);
		isPlaying = false;

	}
}

void updateButtons()
{
	btnMenuEnter.update();
	btnMenuLeft.update();
	btnMenuRight.update();
}

void printSelectedSpeed(double selectedSpeed)
{	
	if (prevSelectedSpeed != selectedSpeed) {
		char string[5];
		// Convert float to a string:
		dtostrf(selectedSpeed, 3, 2, string);
		lcd.setCursor(6, 1);
		lcd.print("     ");
		lcd.setCursor(6, 1);
		lcd.print(string);
		lcd.setCursor(12, 1);
		lcd.print("rpm");
		prevSelectedSpeed = selectedSpeed;
	}
}

void printMeasuredSpeed(float currenMeasuredtSpeed)
{
	if (prevMeasuredSpeed != currenMeasuredtSpeed )
	{
		Serial.print("Curr Speed:"); Serial.println(currenMeasuredtSpeed);
		lcd.setCursor(6, 1);
		lcd.print("      ");
		lcd.setCursor(6, 1);
		lcd.print(currenMeasuredtSpeed);
		lcd.setCursor(12, 1);
		lcd.print("rpm");
		prevMeasuredSpeed = currenMeasuredtSpeed;
	}
}


void printState(const char * text)
{
	Serial.println(text);	
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0,0 );
	lcd.print(text);

}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}