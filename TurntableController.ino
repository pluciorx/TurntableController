#include <ezButton.h>
#include <LiquidCrystal_I2C.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.

//IR Sensor
#define PIN_SENSOR 3

//MOSFET GATE PIN
#define PIN_EN 9

//Buttons
#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7

ezButton btnMenuRight(PIN_BTN_RIGHT, INPUT);
ezButton btnMenuLeft(PIN_BTN_LEFT, INPUT);
ezButton btnMenuEnter(PIN_BTN_MID);

LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
volatile float prevMeasuredSpeed = -1;
float revPerSecondRequired = 0.0;
float markersPerSecondRequired = 0.0;
float markersPerWindowRequired = 0.0;

volatile bool isPlaying = false;
volatile bool isStabilised = false;

volatile unsigned int markersPerWindowActual = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;

#define MCP_ADDR 0x28 //(40)
#define POT0 0x10 //
#define POT1 0x11
#define POT0_Default 250

#define minPOT 80  
#define maxPOT 170

volatile int currentPVal = maxPOT;

int measureInterval;
double Kp;   
double Ki;   
double Kd;   

//--------------------CALIBRATION---------------------
//33.33 PID definitions 
double Kp33 = 1.35;   // Increased for faster response
double Ki33 = 0.02;   // Increased to reduce steady-state error
double Kd33 = 0.01;   // Introduced for damping oscillations
int measureInterval33 = 350;

//45 definitions
double Kp45 = 1.2;  // Increased for faster response
double Ki45 = 0.05;   // Increased to reduce steady-state error
double Kd45 = 0.04;  // Introduced for damping oscillations
int measureInterval45 = 200;

//--------------------CALIBRATION---------------------

double previousError = 0;
double integral = 0;
const double integralLimit = 100.0; // Limit for the integral term

int stableCount = 0;
const int stabilityThreshold = 10;  // Number of consecutive stable intervals needed
const double acceptableError = 0.3; // Acceptable error range for RPM

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
	Manual33 = 2,
	Manual45 = 3
};

volatile E_STATE _state;
volatile E_MODE _mode;
volatile E_MODE prev_mode;

void setup() {
	Serial.begin(115200);
	lcd.init();
	lcd.backlight();
	lcd.clear();

	markersPerWindowActual = 0;

	pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
	pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_BTN_MID, INPUT_PULLUP);

	btnMenuEnter.setDebounceTime(100);
	btnMenuLeft.setDebounceTime(50);
	btnMenuRight.setDebounceTime(50);

	pinMode(PIN_SENSOR, INPUT_PULLUP);

	//pinMode(PIN_EN, INPUT_PULLUP);
	pinMode(PIN_EN, OUTPUT);
	disableOutput();
	setSpedForP0(POT0_Default);
	setSpedForP1(255);

	attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), interruptRoutine, RISING);

	curMillis = lastMillis = millis();
	_mode = E_MODE::Auto33;
	prev_mode = E_MODE::Auto45;
	stopMotor();
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
		disableOutput();
		printState("<-    Speed   ->");
		isPlaying = false;
		isStabilised = false;
		printMeasuredSpeed(0, false);
		SetSelectedMode(E_MODE::Auto33);
		while (!isPlaying)
		{
			updateButtons();

			if (btnMenuLeft.isPressed()) {
				Serial.println("Btn Left Pressed");
				if (_mode != E_MODE::Auto33)
				{
					_mode = E_MODE::Auto33;
				}
				else
					if (_mode == E_MODE::Auto33) {
						_mode = E_MODE::Manual33;
					}
				SetSelectedMode(_mode);
			}

			if (btnMenuRight.isPressed()) {
				Serial.println("Btn Right Pressed");
				if (_mode != E_MODE::Auto45)
				{
					_mode = E_MODE::Auto45;
				}
				else
					if (_mode == E_MODE::Auto45) {
						_mode = E_MODE::Manual45;
					}
				SetSelectedMode(_mode);
			}

			if (btnMenuEnter.isPressed())
			{
				Serial.println("Play Pressed");
				markersPerWindowActual = 0;
				isPlaying = true;
				printMeasuredSpeed(0, false);
				break;
			}
			printSelectedSpeed(selectedSpeed);

		}
		SetState(E_STATE::Starting);
	}break;
	case Starting:
	{
		enableOutput();
		printState("    Starting    ");
		printMeasuredSpeed(0, false);
		int x = minPOT;

		printState("   Stabilising ");
		int target;
		switch (_mode)
		{
		case Auto33:
		case Manual33:
		{
			Kp = Kp33;
			Ki = Ki33;
			Kd = Kd33;
			measureInterval = measureInterval33;
			
		}break;
		case Auto45:
		case Manual45:
		{
			Kp = Kp45; 
			Ki = Ki45; 
			Kd = Kd45; 
			measureInterval = measureInterval45;
			
		} break;
		default:
			break;
		}
		revPerSecondRequired = selectedSpeed / 60;
		markersPerSecondRequired = revPerSecondRequired * NUM_MARKERS;
		
		markersPerWindowRequired = markersPerSecondRequired * (measureInterval / 1000.0);

		Serial.print("Rev's per/s req:"); Serial.println(revPerSecondRequired, 3);
		Serial.print("Markers per/s req:"); Serial.println(markersPerSecondRequired, 3);
		Serial.print("Measure Interval ms:"); Serial.println(measureInterval);
		Serial.print("Pulses required per/windows:"); Serial.println(markersPerWindowRequired, 3);
		Serial.print("Max POT Value:"); Serial.println(minPOT);
		Serial.print("Min POT Value:"); Serial.println(maxPOT);
		Serial.print("Current P1 Value:"); Serial.println(currentPVal);		
		Serial.print("Target POT:"); Serial.println(target);
		Serial.print("Kp:"); Serial.println(Kp);
		Serial.print("Ki:"); Serial.println(Ki);
		Serial.print("Kd:"); Serial.println(Kd);

		markersPerWindowActual = 0;

		while (!isStabilised)
		{
			updateButtons();
			measureSpeedOnlyImpPerWindow(false);

			if (btnMenuEnter.isPressed())
			{
				Serial.println("Stop");
				SetState(E_STATE::Stopping);
				return;
			}
		}

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
		case Manual33:
		case Manual45: {
			printState("-     Speed    +");

		}break;
		default: {

		}break;

		}

		while (isPlaying)
		{
			updateButtons();
			switch (_mode)
			{
			case Auto33:
			{
				measureSpeedOnlyImpPerWindow(false);
			}break;
			case Auto45:
			{
				measureSpeedOnlyImpPerWindow(false);
			}break;
			case Manual33:
			case Manual45:
			{
				if (abs(markersPerWindowRequired - markersPerWindowActual) > 20)
				{
					measureSpeedOnlyImpPerWindow(false);
				}
				else
				{
					if (btnMenuLeft.isPressed()) {
						Serial.print("New pwm:"); Serial.println(currentPVal--);
					}

					if (btnMenuRight.isPressed()) {
						Serial.print("New pwm:"); Serial.println(currentPVal++);
					}
					setSpedForP1(currentPVal);
					measureSpeedOnlyImpPerWindow(true);
				}
			}break;
			}
			HandleButtonsWhilePlaying();
		}
		SetState(E_STATE::Stopping);
	}break;
	case Stopping:
	{
		stopMotor();

		printState("    Stopping    ");
		setSpedForP0(POT0_Default);
		while (currentPVal > minPOT)
		{
			measureSpeedOnlyImpPerWindow(true);
			currentPVal -= 2;
								
		}

		markersPerWindowActual = 0;
		isPlaying = false;
		SetState(E_STATE::Idle);
	}
	}
}

void measureSpeedOnlyImpPerWindow(bool displayOnly) {
	static unsigned long lastMillis = 0;
	unsigned long curMillis = millis();

	if (curMillis >= lastMillis + measureInterval) {
		lastMillis = curMillis;

		int numberOfPulses = 0;

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			numberOfPulses = markersPerWindowActual;
			markersPerWindowActual = 0;
		}

		double impulsesPerSecond = numberOfPulses / (measureInterval / 1000.0);
		double rotationsPerSecond = impulsesPerSecond / NUM_MARKERS;
		double rotationsPerMinute = rotationsPerSecond * 60;

		double setpoint = selectedSpeed; // Desired RPM
		double error = setpoint - rotationsPerMinute;

		// Calculate PID terms
		integral += error * (measureInterval / 1000.0);
		integral = constrain(integral, -integralLimit, integralLimit); // Prevent integral windup
		double derivative = (error - previousError) / (measureInterval / 1000.0);

		// Calculate the new potentiometer value
		double output = Kp * error + Ki * integral + Kd * derivative;
		previousError = error;

		// Adjust new potentiometer value for reversed control
		int newPot = constrain(currentPVal - (int)output, minPOT, maxPOT);
		setSpedForP0(newPot);

		// Update currentPVal to the new potentiometer value
		currentPVal = newPot;

		printMeasuredSpeed(rotationsPerMinute, isStabilised);
		Serial.println("");
		Serial.print(F("Markers required per interval: ")); Serial.println(markersPerWindowRequired, 3);
		Serial.print(F("Markers counted per interval: ")); Serial.println(numberOfPulses);
		Serial.print(F("Markers required per 1s: ")); Serial.println(markersPerSecondRequired, 3);
		Serial.print(F("Markers counted per 1s: ")); Serial.println(impulsesPerSecond, 3);
		Serial.print(F("Error: ")); Serial.println(error);		
		Serial.print(F("Current P0 Value: ")); Serial.println(currentPVal);

		if (displayOnly) return;

		// Check stability
		if (abs(error) <= acceptableError) {
			stableCount++;
		}
		else {
			stableCount = 0;
			isStabilised = false;
		}

		if (stableCount >= stabilityThreshold) {
			isStabilised = true;
		}
	}
}

void setSpedForP1(int value)
{
	writePot(MCP_ADDR, POT1, value);
	
}
void setSpedForP0(int value)
{
	writePot(MCP_ADDR, POT0, value);
	
}

void writePot(uint8_t address, uint8_t pot, uint16_t val) {
	Wire.beginTransmission(address);
	Wire.write((pot & 3) << 4 | ((val >> 8) & 3));
	Wire.write(val & 0xFF);
	Wire.endTransmission();
}


void stopMotor()
{
	disableOutput();
}

void disableOutput()
{
	digitalWrite(PIN_EN, LOW);
}
void enableOutput()
{
	digitalWrite(PIN_EN, HIGH);
}

double average(int* array, int size) {
	long sum = 0;
	for (int i = 0; i < size; i++) {
		sum += array[i];
	}
	return sum / size;
}

void SetSelectedMode(E_MODE selectedMode)
{
	switch (selectedMode)
	{
	case Auto33:
	case Manual33:
	{
		selectedSpeed = 33.33;
	}break;
	case Auto45:
	case Manual45:
	{
		selectedSpeed = 45.0;
	} break;

	default:
		break;
	}
	Serial.print("Selected mode:"); Serial.println(selectedSpeed);
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
		case Manual33:
		case Manual45:
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

void printMeasuredSpeed(float currenMeasuredSpeed, bool isStabilised)
{
	Serial.print(F("RPM: ")); Serial.print(currenMeasuredSpeed);
	Serial.print(F(" Is stable: ")); Serial.println(isStabilised);

	if (prevMeasuredSpeed != currenMeasuredSpeed)
	{
		/*if (isStabilised && abs(#error-1) <= 0.1)
		{
			currenMeasuredSpeed = selectedSpeed;
		}*/

		lcd.setCursor(6, 1);
		lcd.print("      ");
		lcd.setCursor(6, 1);
		lcd.print(currenMeasuredSpeed);
		lcd.setCursor(12, 1);
		lcd.print("rpm");

		prevMeasuredSpeed = currenMeasuredSpeed;

		if (isStabilised) {

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