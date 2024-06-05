#include <ezButton.h>
#include <QuickPID.h>
#include <LiquidCrystal_I2C.h>
#include <MX1508.h>
#include <util/atomic.h>

//IR Sensor
#define PIN_SPD_D0 3

//Buttons
#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7

ezButton btnMenuRight(PIN_BTN_RIGHT, INPUT);
ezButton btnMenuLeft(PIN_BTN_LEFT, INPUT);
ezButton btnMenuEnter(PIN_BTN_MID);

//Motor
#define PINA 9
#define PINB 10

MX1508 motorA(PINA, PINB, SLOW_DECAY, 1);
#define PWM_RESOLUTION 800

//PID
float Setpoint, Input, Output;
float Kp = 1, Ki = 0.0, Kd = 0.025;
QuickPID myPID(&Input, &Output, &Setpoint);

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

double selectedSpeed = 33.33;
double prevSelectedSpeed = -1;
double prevMeasuredSpeed = -1;
double revPerSecondRequired = 0;
double markersPerSecondRequired = 0;
double markersPerWindowRequired = 0;


bool isPlaying = false;
bool isAvgFound = false;

int currPWm;
volatile long numPulses = 0;
volatile long prev_numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0;
int spotPwm[5];
int idxSpt = 0;

#define minPwm 0
#define maxPwm 150

#define SPD_MEASURE_INTERVAL 500 

#define NUM_MARKERS 54 //TO DO: 

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

	//pinMode(PIN_SPD_D0, ); // declare ir as input	
	pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
	pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_BTN_MID, INPUT_PULLUP);

	btnMenuEnter.setDebounceTime(100);
	btnMenuLeft.setDebounceTime(50);
	btnMenuRight.setDebounceTime(50);

	printSelectedSpeed(selectedSpeed);
	//attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, RISING);
	pinMode(PIN_SPD_D0, INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, FALLING);

	curMillis = lastMillis = millis();
	revPerMin = 0;

	SetState(E_STATE::Idle);
}

void  interruptRoutine() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();

	if (interrupt_time - last_interrupt_time >5 )
	{ 
		if (numPulses < NUM_MARKERS+1) numPulses++;
		
	}
	last_interrupt_time = interrupt_time;

}

void loop() {
	updateButtons();
	switch (_state)
	{
	case Idle:
	{
		printState("<-    Speed   ->");
		isPlaying = false;
		isAvgFound = false;
		while (!isPlaying)
		{
			updateButtons();

			if (btnMenuRight.isPressed()) {
				selectedSpeed = 45;
			}
			if (btnMenuLeft.isPressed()) {
				selectedSpeed = 33.333;
			}

			if (btnMenuEnter.isPressed())
			{

				Serial.println("Play Pressed");
				numPulses = 0;
				revPerMin = 0;

				isPlaying = true;
				break;
			}
			printSelectedSpeed(selectedSpeed);
		}
		
		SetState(E_STATE::Starting);
	}break;
	case Starting:
	{
		printState("    Counting    ");
		printBottomLineInt(0);
		long pwm = 150;
		motorA.motorGo(pwm);

		

		while (numPulses <= NUM_MARKERS)
		{
			if (prev_numPulses != numPulses)
			{
				printBottomLineInt(numPulses);
				
			}
			prev_numPulses = numPulses;
		}			
		motorA.motorGo(0);
		//motorA.stopMotor();
		Serial.print("ALL markers found:"); Serial.println(numPulses);
	
		 
		while (1)
		{
			btnMenuEnter.loop();
			if (btnMenuEnter.isPressed())
			{ 
				SetState(E_STATE::Starting);
				
				numPulses = 0;
				return;
			}

		}

		
		
		//prepare the required data
		revPerSecondRequired = selectedSpeed / 60;
		markersPerSecondRequired = revPerSecondRequired * NUM_MARKERS;
		markersPerWindowRequired = markersPerSecondRequired * SPD_MEASURE_INTERVAL / 1000;

		Serial.print("Rev's per/s req:"); Serial.println(revPerSecondRequired, 3);
		Serial.print("Markers per/s req:"); Serial.println(markersPerSecondRequired, 3);
		Serial.print("Window ms:"); Serial.println(SPD_MEASURE_INTERVAL);
		Serial.print("Pulses required per/windows:"); Serial.println(markersPerWindowRequired, 3);
		Serial.print("Max PWM:"); Serial.println(maxPwm);
		Serial.print("Min PWM:"); Serial.println(minPwm);
		Serial.print("Current PWM:"); Serial.println(motorA.getPWM());
		isPlaying = true;
		SetState(E_STATE::Stopping);

	}break;
	case Running:
	{
		printState("-     Speed    +");
		printMeasuredSpeed(0);



		motorA.motorGo(0);
		SetState(E_STATE::Stopping);
	}break;
	case Stopping:
	{
		printState("    Stopping    ");

		int currPwm = motorA.getPWM();
		while (currPwm > 0)
		{
			measureSpeedOnlyImpPerWindow(true);
			if (currPwm < minPwm) currPwm = 0;
			motorA.motorGo(currPwm--);
			delay(100);

		}
		numPulses = 0;
		isPlaying = false;
		SetState(E_STATE::Idle);
	}
	}
}

static void  measureSpeedOnlyImpPerWindow(bool isStopping)
{
	curMillis = millis();

	if (curMillis >= lastMillis + SPD_MEASURE_INTERVAL) {
		lastMillis = curMillis;

		int numberOfPulses = 0;
		// Calculate impulses per second
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			numberOfPulses = numPulses;
			numPulses = 0;
		}
		double impulsesPerSecond = numberOfPulses / ((double)SPD_MEASURE_INTERVAL / 1000);

		// Calculate rotations per second (RPS)
		double rotationsPerSecond = impulsesPerSecond / (double)NUM_MARKERS;

		// Calculate rotations per minute (RPM)
		double rotationsPerMinute = rotationsPerSecond * 60;
		Serial.print("Pulses measured:"); Serial.println(impulsesPerSecond, 3);
		Serial.print("Pulses requred:"); Serial.println(markersPerSecondRequired, 3);

		Serial.print("RPM measured:"); Serial.println(rotationsPerMinute, 3);

		double deviatoon = markersPerSecondRequired - impulsesPerSecond;
		Serial.print("Deviation:"); Serial.println(deviatoon, 2);

		printSelectedSpeed(rotationsPerMinute);

		if (isStopping) return;
		currPWm = motorA.getPWM();
		Serial.print("CurrPWM:"); Serial.println(currPWm);
		double absDev = abs(deviatoon);
		if (absDev <= 0.5)
		{
			Serial.print("Spot pwm:"); Serial.println(currPWm);
			Serial.print("Rpm:"); Serial.println(rotationsPerMinute);

			spotPwm[idxSpt] = currPWm;
			idxSpt++;
			if (idxSpt > 4)
			{
				idxSpt = 0;
				Serial.print("Found Average:");
				float avg = average(spotPwm, 5);
				Serial.println(avg);
				avg = round(avg);
				Serial.print("Round Average:");
				Serial.println(avg);

				motorA.motorGo(avg);

				numPulses = 0;
				isAvgFound = true;

				return;
			}
		}
		else if (absDev > 8) isAvgFound = false;

		if (!isAvgFound) {
			if (deviatoon > 1)
			{
				int adj = 1;
				if (deviatoon >= 4) adj = deviatoon / 2;

				int cap = min(maxPwm, motorA.getPWM() + adj);

				motorA.motorGo(cap);

			}
			if (deviatoon < -1)
			{
				int adj = 1;
				if (deviatoon <= -5) adj = abs(deviatoon / 2);
				int cap = max(minPwm, motorA.getPWM() - adj);
				motorA.motorGo(cap);
			}
		}

	}
}

float average(int* array, int len)  // assuming array is int.
{
	long sum = 0L;  // sum will be larger than an item, long for safety.
	for (int i = 0; i < len; i++)
		sum += array[i];
	return  ((float)sum) / len;  // average will be fractional, so float may be appropriate.
}

static void measureSpeedOnly(long intervalMs)
{
	curMillis = millis();
	if (curMillis >= lastMillis + intervalMs) {
		float revPerMin = 60 * (numPulses / intervalMs / NUM_MARKERS);
		lastMillis = curMillis;
		Serial.print("Pulses for interval:"); Serial.println(numPulses);
		numPulses = 0;
		Input = numPulses;
		printMeasuredSpeed(revPerMin);

	}
}


void HandleButtonsWhilePlaying()
{
	if (btnMenuEnter.isPressed() && isPlaying)
	{
		Serial.println("Stop Pressed");
		isPlaying = false;
	}
}

void updateButtons()
{
	btnMenuEnter.loop();
	btnMenuLeft.loop();
	btnMenuRight.loop();
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
	if (prevMeasuredSpeed != currenMeasuredtSpeed)
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

static void printBottomLineInt(int value)
{
	lcd.setCursor(1, 1);
	lcd.print("                ");
	lcd.setCursor(6, 1);
	lcd.print(value);
}

void printState(const char* text)
{
	Serial.println(text);
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 0);
	lcd.print(text);

}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}