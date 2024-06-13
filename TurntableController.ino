#include <ezButton.h>
#include <LiquidCrystal_I2C.h>
#include <MX1508.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

//IR Sensor
#define PIN_SPD_D0 3
#define PIN_SPD_A0 A0

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
MX1508 motorA(PINA, PINB, FAST_DECAY, 2);
#define PWM_RESOLUTION 1200

//PID
float Setpoint, Input, Output;
float Kp = 1, Ki = 0.0, Kd = 0.025;

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

double selectedSpeed = 33.33;

double prevMeasuredSpeed = -1;
double revPerSecondRequired = 0;
double markersPerSecondRequired = 0;
double markersPerWindowRequired = 0;

bool isPlaying = false;
bool isAvgFound = false;


int currPWm;
volatile unsigned long prev_numPulses, numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0;
int spotPwm[5];
int idxSpt = 0;

#define minPwm 900
#define maxPwm 1200

#define SPD_MEASURE_INTERVAL 1200 //2 seconds window - increase this if the no. of markers is less for better accuracy

#define NUM_MARKERS 54 //TO DO: Check this as per your setup 200

enum E_STATE {
	Idle,
	Starting,
	Running,
	Stopping
};
enum E_MODE {
	Auto33 = 0,
	Auto45 = 1,
	Manual = 2
};

E_STATE _state;
E_MODE _mode;
E_MODE prev_mode;

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

	attachInterrupt(digitalPinToInterrupt(3), interruptRoutine, FALLING);

	curMillis = lastMillis = millis();
	revPerMin = 0;
	_mode = E_MODE::Auto33;
	prev_mode = E_MODE::Auto45;
	SetState(E_STATE::Idle);
}

void  interruptRoutine() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();

	if (interrupt_time - last_interrupt_time > 5)
	{
		numPulses++;
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
		printMeasuredSpeed(0);
		isPlaying = false;
		isAvgFound = false;

		while (!isPlaying)
		{
			updateButtons();

			if (btnMenuLeft.isPressed()) {

				if ((int)_mode == 0)
				{
					_mode = E_MODE::Manual;
				}
				else {
					_mode = (E_MODE)((int)_mode - 1);
				}
				SetSelectedMode(_mode);
			}

			if (btnMenuRight.isPressed()) {

				if ((int)_mode == 2)
				{
					_mode = E_MODE::Auto33;
				}
				else
				{
					_mode = (E_MODE)((int)_mode + 1);
				}				
				SetSelectedMode(_mode);
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
		printState("    Starting    ");
		printMeasuredSpeed(0);

		int x = minPwm-10;
		while (x < minPwm)
		{
			motorA.motorGo(x);
			x++;
			delay(100);
		}
		//prepare the required data
		revPerSecondRequired = selectedSpeed / 60;
		markersPerSecondRequired = revPerSecondRequired * NUM_MARKERS;
		markersPerWindowRequired = markersPerSecondRequired * SPD_MEASURE_INTERVAL / 1000.0;

		Serial.print("Rev's per/s req:"); Serial.println(revPerSecondRequired, 3);
		Serial.print("Markers per/s req:"); Serial.println(markersPerSecondRequired, 3);
		Serial.print("Window ms:"); Serial.println(SPD_MEASURE_INTERVAL);
		Serial.print("Pulses required per/windows:"); Serial.println(markersPerWindowRequired, 3);
		Serial.print("Max PWM:"); Serial.println(maxPwm);
		Serial.print("Min PWM:"); Serial.println(minPwm);
		Serial.print("Current PWM:"); Serial.println(motorA.getPWM());
		isPlaying = true;
		SetState(E_STATE::Running);

	}break;
	case Running:
	{
		printState("-     Speed    +");
		printMeasuredSpeed(0);

		while (isPlaying)
		{
			updateButtons();
			switch (_mode)
			{
			case Auto33:
			case Auto45:
			{
				measureSpeedOnlyImpPerWindow(false);				
			}break;
			case Manual: {
				measureSpeedOnlyImpPerWindow(true);
				currPWm = motorA.getPWM();
				if (btnMenuLeft.isPressed()) {
					
					Serial.print("New pwm:"); Serial.println(currPWm--);
					
				}

				if (btnMenuRight.isPressed()) {
					Serial.print("New pwm:"); Serial.println(currPWm++);
					
				}
				motorA.motorGo(currPWm);
			}
				break;
			default:
				break;
			}

			HandleButtonsWhilePlaying();
		}
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

static void  measureSpeedOnlyImpPerWindow(bool displayOnly)
{
	curMillis = millis();

	if (curMillis >= lastMillis + SPD_MEASURE_INTERVAL) {
		lastMillis = curMillis;

		unsigned int numberOfPulses = 0;
		// Calculate impulses per second
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			numberOfPulses = numPulses;
			numPulses = 0;
		}
		int devP = prev_numPulses - numberOfPulses;
		if (isAvgFound && abs(devP) <= 2)
		{
			Serial.println("Volskwagen !");
			numberOfPulses = prev_numPulses;
		}
		prev_numPulses = numberOfPulses;

		double impulsesPerSecond = (double)numberOfPulses / ((double)SPD_MEASURE_INTERVAL / 1000);

		// Calculate rotations per second (RPS)
		double rotationsPerSecond = impulsesPerSecond / (double)NUM_MARKERS;

		// Calculate rotations per minute (RPM)
		double rotationsPerMinute = rotationsPerSecond * 60;
		Serial.print("Number of Pulses:"); Serial.println(numberOfPulses, 3);
		Serial.print("Pulses measured:"); Serial.println(impulsesPerSecond, 3);
		Serial.print("Pulses requred:"); Serial.println(markersPerSecondRequired, 3);
		Serial.print("RPM measured:"); Serial.println(rotationsPerMinute, 3);

		
		printMeasuredSpeed(rotationsPerMinute);

		if (displayOnly) return;

		double deviatoon = markersPerSecondRequired - impulsesPerSecond;
		Serial.print("Deviation:"); Serial.println(deviatoon, 3);

		currPWm = motorA.getPWM();
		Serial.print("CurrPWM:"); Serial.println(currPWm);
		double absDev = abs(deviatoon);
		if (absDev <= 0.02)
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

			if (deviatoon > 0.003)
			{
				int adj = 1;
				if (deviatoon >= 4) adj = deviatoon / 2;

				int cap = min(maxPwm, motorA.getPWM() + adj);

				motorA.motorGo(cap);

			}
			if (deviatoon <  0.003)
			{
				int adj = 1;
				if (deviatoon <= -4) adj = abs(deviatoon / 2);

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
void SetSelectedMode(E_MODE selectedMode)
{
	switch (selectedMode)
	{
	case Auto33:
	{
		selectedSpeed = 33.33;
	}break;
	case Auto45: {
		selectedSpeed = 45.0;
	} break;
	case Manual:
	{
	
	}
	break;
	default:
		break;
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
	if (prev_mode != _mode) {
		char string[5];
		// Convert float to a string:
		dtostrf(selectedSpeed, 3, 2, string);
		lcd.setCursor(6, 1);
		lcd.print("     ");
		lcd.setCursor(6, 1);
		lcd.print(string);
		
		switch (_mode)
		{
		case Auto33:
		case Auto45: {
			lcd.setCursor(0, 1);
			lcd.print("A");
			lcd.setCursor(12, 1);
			lcd.print("rpm");
		} break;
		case Manual:
		{
			lcd.setCursor(0, 1);
			lcd.print("M");
			lcd.setCursor(12, 1);
			lcd.print("rpm");
		}
		break;
		default:
			break;
		}

		prev_mode = _mode;
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