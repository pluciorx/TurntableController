#include <ezButton.h>
#include <LiquidCrystal_I2C.h>
#include <MX1508.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

//IR Sensor
#define PIN_SENSOR 2

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

LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
float prevMeasuredSpeed = -1;
float revPerSecondRequired = 0.0;
float markersPerSecondRequired = 0.0;
float markersPerWindowRequired = 0.0;

bool isPlaying = false;
bool isAvgFound = false;

int currPWm;
volatile unsigned int markersPerWindowActual = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;

int spotPwm[5];
int idxSpt = 0;
#define MAX_DEVITATION_MARKERS 3
#define PWM_RESOLUTION 16000
#define minPwm PWM_RESOLUTION * 0.75 
#define maxPwm PWM_RESOLUTION

int intervalFor33 = 1040;
//1040 1560 for 180
int intervalFor45 = 1137; //2 seconds window - increase this if the no. of markers is less for better accuracy
//568,1136 for (44.99) at 54 markers
int measureInterval = intervalFor33; //just a default setting

#define NUM_MARKERS 180 //TO DO: Check this as per your setup 200

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
	lcd.init();
	lcd.backlight();
	lcd.clear();

	markersPerWindowActual = 0;
	motorA.setPWM16(1, PWM_RESOLUTION); // prescaler at 1 , resolution 700, PWM frequency = 16Mhz/1/700~22000hz 

	pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
	pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_BTN_MID, INPUT_PULLUP);

	btnMenuEnter.setDebounceTime(100);
	btnMenuLeft.setDebounceTime(50);
	btnMenuRight.setDebounceTime(50);

	pinMode(PIN_SENSOR, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), interruptRoutine, RISING);

	curMillis = lastMillis = millis();
	_mode = E_MODE::Auto33;
	prev_mode = E_MODE::Auto45;
	SetState(E_STATE::Idle);
}

void  interruptRoutine() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();

	if (interrupt_time - last_interrupt_time > 4)
	{
		markersPerWindowActual++;
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
		printMeasuredSpeed(0, isAvgFound);

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
				markersPerWindowActual = 0;
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
		printMeasuredSpeed(0,false);

		int x = minPwm;
		while (x < minPwm+50)
		{
			motorA.motorGo(x);
			x+=5;
			delay(20);
		}
		//prepare the required data
		revPerSecondRequired = selectedSpeed / 60;
		markersPerSecondRequired = revPerSecondRequired * NUM_MARKERS;
	
		if (selectedSpeed == 33.33) measureInterval = intervalFor33;
		if (selectedSpeed == 45) measureInterval = intervalFor45;
		markersPerWindowRequired = markersPerSecondRequired * (measureInterval / 1000.0);

		Serial.print("Rev's per/s req:"); Serial.println(revPerSecondRequired, 3);
		Serial.print("Markers per/s req:"); Serial.println(markersPerSecondRequired, 3);
		Serial.print("Measure Interval ms:"); Serial.println(measureInterval);
		Serial.print("Pulses required per/windows:"); Serial.println(markersPerWindowRequired, 3);
		Serial.print("Max PWM:"); Serial.println(maxPwm);
		Serial.print("Min PWM:"); Serial.println(minPwm);
		Serial.print("Current PWM:"); Serial.println(motorA.getPWM());
		isPlaying = true;
		SetState(E_STATE::Running);

	}break;
	case Running:
	{
		switch (_mode)
		{
		case Auto33:
		case Auto45:
		{
			printState("      Speed     ");
		}break;
		case Manual: {
			printState("-     Speed    +");

		}break;
		default: {

		}break;
				 
		}
	    printMeasuredSpeed(0,false);

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
					if (abs(markersPerWindowRequired - markersPerWindowActual) > 20 )
					{
						measureSpeedOnlyImpPerWindow(false);
					}
					else 
					{					
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
				}break;			
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
			currPwm -= 20;
			motorA.motorGo(currPwm);
			delay(50);
		}
		markersPerWindowActual = 0;
		isPlaying = false;
		SetState(E_STATE::Idle);
	}
	}
}

static void  measureSpeedOnlyImpPerWindow(bool displayOnly)
{
	
	curMillis = millis();

	if (curMillis >= lastMillis + measureInterval) {
		lastMillis = curMillis;
		
		int numberOfPulses = 0;

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			numberOfPulses = markersPerWindowActual;
			markersPerWindowActual = 0;
		}

		// Calculate impulses per second
		double impulsesPerSecond = numberOfPulses / (measureInterval / 1000.0);
		// Calculate rotations per second (RPS)
		double rotationsPerSecond = impulsesPerSecond / NUM_MARKERS;

		// Calculate rotations per minute (RPM)
		double rotationsPerMinute = rotationsPerSecond * 60;

		int markersRequiredRounded = round(markersPerWindowRequired);
		int deviatoonMarkers = numberOfPulses - markersRequiredRounded;
		
		markersPerWindowActual = 0;
		int absDevMarkers = abs(deviatoonMarkers);
		float devSpd = abs(rotationsPerMinute - selectedSpeed);		
				
		printMeasuredSpeed(rotationsPerMinute,isAvgFound);

		if (displayOnly) return;

		Serial.println("");

		Serial.print(F("Markers required per interval:")); Serial.println(markersPerWindowRequired, 3);
		Serial.print(F("Markers round per interval:")); Serial.println(markersRequiredRounded);
		Serial.print(F("Markers counted per interval:")); Serial.println(numberOfPulses);
		Serial.print(F("Markers requred per 1s:")); Serial.println(markersPerSecondRequired, 3);
		Serial.print(F("Markers counted per 1s:")); Serial.println(impulsesPerSecond, 3);
		Serial.print(F("RPM calculated:")); Serial.println(rotationsPerMinute, 3);
		Serial.print(F("Deviation pulses:")); Serial.println(deviatoonMarkers);		
		Serial.print(F("ABS Deviation pulses:")); Serial.println(absDevMarkers);
		Serial.print(F("Deviation Speed:")); Serial.println(devSpd, 3);

		currPWm = motorA.getPWM();
		Serial.print(F("Current PWM:")); Serial.println(currPWm);		

		if (absDevMarkers == 0  && !isAvgFound)
		{
			Serial.print(F("Spot PWM:")); Serial.println(currPWm);
			spotPwm[idxSpt] = currPWm;
			idxSpt++;
			if (idxSpt > sizeof(spotPwm)-1)
			{
				idxSpt = 0;
				int avg = average(spotPwm, sizeof(idxSpt));
				Serial.print(F("Found Average:")); Serial.println(avg);								

				motorA.motorGo(avg);
				isAvgFound = true;

				return;
			}
		}
		
		if (absDevMarkers > MAX_DEVITATION_MARKERS)
		{
			isAvgFound = false;
			memset(spotPwm, 0, sizeof(spotPwm));
			idxSpt = 0;
		}
		
		if (!isAvgFound) {

			if (deviatoonMarkers > 1)
			{
				int adj = 1;
				if (deviatoonMarkers > MAX_DEVITATION_MARKERS*4)
				{
					
				}
				if (deviatoonMarkers > MAX_DEVITATION_MARKERS)
				{
					adj = deviatoonMarkers;
				}

				int cap = min(maxPwm, motorA.getPWM() - adj);

				motorA.motorGo(cap);

			}
			if (deviatoonMarkers < -1)
			{
				int adj = 1;
				if (deviatoonMarkers < -MAX_DEVITATION_MARKERS) adj = absDevMarkers;

				int cap = max(minPwm, motorA.getPWM() + adj);
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

void printMeasuredSpeed(float currenMeasuredSpeed,bool isAvgFound)
{
	if (prevMeasuredSpeed != currenMeasuredSpeed)
	{		
		if (isAvgFound && abs(currenMeasuredSpeed - selectedSpeed) <= 0.65)
		{
			currenMeasuredSpeed = selectedSpeed;			
		}
		
		lcd.setCursor(6, 1);
		lcd.print("      ");
		lcd.setCursor(6, 1);
		lcd.print(currenMeasuredSpeed);
		lcd.setCursor(12, 1);		
		lcd.print("rpm");

		prevMeasuredSpeed = currenMeasuredSpeed;
		
		if (isAvgFound && currenMeasuredSpeed == selectedSpeed) {

			lcd.print("*");
		}
		else lcd.print(" ");
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