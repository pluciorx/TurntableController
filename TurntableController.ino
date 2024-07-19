#include <ButtonControl.h>  //https://github.com/fellipecouto/ButtonControl
#include <LiquidCrystal_I2C.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.
#include <EEPROM.h>
//IR Sensor
#define PIN_SENSOR 3

//MOSFET GATE PIN
#define PIN_EN 9

//Buttons
#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7
#define BTN_DEBOUNCE_MS 80

ButtonControl  btnMenuRight(PIN_BTN_RIGHT);
ButtonControl  btnMenuLeft(PIN_BTN_LEFT);
ButtonControl  btnMenuEnter(PIN_BTN_MID);

LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
double rotationsPerMinuteMeasured;
volatile float prevMeasuredSpeed = -1;
float revPerSecondRequired = 0.0;
float markersPerSecondRequired = 0.0;
float markersPerWindowRequired = 0.0;

volatile bool isPlaying = false;
volatile bool isStabilised = false;
#define EE_ADDR_VW 6
int IsUltraPrecisionEnabled = false;

#define EE_ADDR_STROBE 10
int IsStrobeEnabled = false;

volatile unsigned int markersPerWindowActual = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;

#define MCP_ADDR 0x28 //(40)
#define POT0 0x10 //
#define POT1 0x11
#define POT0_Default 250

int minPot, maxPot;
int measureInterval;
double Kp;
double Ki;
double Kd;

//--------------------CALIBRATION---------------------
//33.33 PID definitions 
#define Kp33 0.05  // Increased for faster response
#define Ki33 0.01   // Increased to reduce steady-state error
#define Kd33 0.15   // Introduced for damping oscillations
#define measureInterval33 200
int minPOT33 = 130;
#define maxPOT33 184
#define EE_ADDR_33 0

//45 definitions
#define Kp45 1.5  // Increased for faster response
#define Ki45 0.08  // Increased to reduce steady-state error
#define Kd45 0.002  // Introduced for damping oscillations
#define measureInterval45  200
int minPOT45 = 140;
#define maxPOT45 150
#define EE_ADDR_45 4

//--------------------CALIBRATION---------------------
volatile int currentPVal = maxPOT33;

double previousError = 0;
double integral = 0;
const double integralLimit = 100.0; // Limit for the integral term

int stableCount = 0;
const int stabilityThreshold = 20;  // Number of consecutive stable intervals needed
const double acceptableError = 0.1; // Acceptable error range for RPM

#define NUM_MARKERS 180 //TO DO: Check this as per your setup 200

enum E_STATE {
	Idle,
	Starting,
	Running,
	Stopping,
	Setup
};

enum E_MODE {
	Auto33 = 0,
	Auto45 = 1,
	Manual33 = 2,
	Manual45 = 3
};

enum E_SETUP {
	Min33 = 0,
	Min45 = 1,
	UltraPrecision = 2,
	Strobe = 3,
	Exit = 4
};

volatile E_STATE _state;
volatile E_MODE _mode;
volatile E_MODE prev_mode;

void setup() {
	Serial.begin(115200);
	lcd.init();
	lcd.backlight();
	lcd.clear();

	Serial.println("");
	Serial.println("Turntable v 1.0");

	markersPerWindowActual = 0;

	pinMode(PIN_SENSOR, INPUT_PULLUP);

	//pinMode(PIN_EN, INPUT_PULLUP);
	pinMode(PIN_EN, OUTPUT);
	EnableEngine(false);
	setSpeedForP0(POT0_Default);
	setSpeedForP1(255);	

	attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), interruptRoutine, RISING);
	printState(" Machina czasu ");

	int eMin33 = readIntFromEEPROM(EE_ADDR_33);
	int eMin45 = readIntFromEEPROM(EE_ADDR_45);

	minPOT33 = eMin33 > 0 ? eMin33 : minPOT33;
	minPOT45 = eMin45 > 0 ? eMin45 : minPOT45;
	IsUltraPrecisionEnabled = readIntFromEEPROM(EE_ADDR_VW) > 0 ? true : false;
	IsStrobeEnabled = readIntFromEEPROM(EE_ADDR_STROBE) > 0 ? true : false;
	

	Serial.print("Strobe:"); Serial.println(IsStrobeEnabled);
	Serial.print("Ultra Precision:"); Serial.println(IsUltraPrecisionEnabled);
	Serial.print("Min33 Value:"); Serial.println(minPOT33);
	Serial.print("Min45 Value:"); Serial.println(minPOT45);
	
	//code from Artur as is
	
	//END TIMER SETUP

	curMillis = lastMillis = millis();
	while (1)
	{
		curMillis = millis();

		if (millis() >= lastMillis + 2000) {
			lastMillis = curMillis;

			SetSelectedMode(E_MODE::Auto33);
			SetState(E_STATE::Idle);

			Serial.println("Going normal mode...");
			break;
		}
		if (btnMenuEnter.click())
		{
			SetState(E_STATE::Setup);
			Serial.println("Entering Calibration");
			break;
		}
	}

}

void enableStrobe(bool isEnabled)
{
	if (IsStrobeEnabled)
	{

	}
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
	//updateButtons();
	switch (_state)
	{
	case Idle:
	{
		EnableEngine(false);
		printState("<-    Speed   ->");
		isPlaying = false;
		isStabilised = false;
		
		SetSelectedMode(_mode);

		while (!isPlaying)
		{
			if (btnMenuLeft.click()) {
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

			if (btnMenuRight.click()) {
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

			if (btnMenuEnter.click())
			{
				Serial.println("Play Pressed");
				markersPerWindowActual = 0;
				isPlaying = true;
				printMeasuredSpeed(0, false);
				break;
			}
		}
		SetState(E_STATE::Starting);
	}break;
	case Setup:
	{
		HandleSetup();
		SetState(E_STATE::Idle);
	}break;
	case Starting:
	{
		
		EnableEngine(true);
		printState("    Starting    ");
		printMeasuredSpeed(0, false);
		int x = minPot;

		printState("   Stabilising ");

		switch (_mode)
		{
		case Auto33:
		case Manual33:
		{
			setPIDParams(Kp33, Ki33, Kd33, measureInterval33, minPOT33, maxPOT33);

		}break;
		case Auto45:
		case Manual45:
		{
			setPIDParams(Kp45, Ki45, Kd45, measureInterval45, minPOT45, maxPOT45);
			
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
		Serial.print("Max POT Value:"); Serial.println(minPot);
		Serial.print("Min POT Value:"); Serial.println(maxPot);
		Serial.print("Current P1 Value:"); Serial.println(currentPVal);
		Serial.print("Kp:"); Serial.println(Kp);
		Serial.print("Ki:"); Serial.println(Ki);
		Serial.print("Kd:"); Serial.println(Kd);

		markersPerWindowActual = 0;

		while (!isStabilised)
		{
			calculateAndApplySpeed(false);

			if (btnMenuEnter.click())
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
		case Manual45:
		{
			printState("-     Speed    +");
		}break;
		default: {
		}break;

		}

		while (isPlaying)
		{
			HandleButtonsWhilePlaying();
			calculateAndApplySpeed(false);
		}
		SetState(E_STATE::Stopping);
	}break;
	case Stopping:
	{
		setSpeedForP0(POT0_Default);
		EnableEngine(false);

		printState("    Stopping    ");

		while (rotationsPerMinuteMeasured > 5)
		{
			calculateAndApplySpeed(true);

		}
		
		SetState(E_STATE::Idle);
	}
	}
}

void calculateAndApplySpeed(bool displayOnly) {
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
		rotationsPerMinuteMeasured = rotationsPerSecond * 60;

		double setpoint = selectedSpeed; // Desired RPM

		double error = setpoint - rotationsPerMinuteMeasured;

		// Calculate PID terms
		integral += error * (measureInterval / 1000.0);
		integral = constrain(integral, -integralLimit, integralLimit); // Prevent integral windup
		double derivative = (error - previousError) / (measureInterval / 1000.0);

		// Calculate the new potentiometer value
		double output = Kp * error + Ki * integral + Kd * derivative;
		previousError = error;

		// Adjust new potentiometer value for reversed control
		int newPot = constrain(currentPVal + (int)output, minPot, maxPot); // Reverse logic by adding output

		// Update currentPVal to the new potentiometer value
		currentPVal = newPot;
		printMeasuredSpeed(rotationsPerMinuteMeasured, isStabilised);
		Serial.println("");
		Serial.print(F("Requested Speed:")); Serial.println(selectedSpeed);
		Serial.print(F("Markers required per interval: ")); Serial.println(markersPerWindowRequired, 3);
		Serial.print(F("Markers counted per interval: ")); Serial.println(numberOfPulses);
		Serial.print(F("Markers required per 1s: ")); Serial.println(markersPerSecondRequired, 3);
		Serial.print(F("Markers counted per 1s: ")); Serial.println(impulsesPerSecond, 3);
		Serial.print(F("Error: ")); Serial.println(error);
		Serial.print(F("Current P0 Value: ")); Serial.println(currentPVal);

		if (displayOnly) return;
		
	    setSpeedForP0(newPot);

		// Check stability
		if (abs(error) <= acceptableError) {
			stableCount++;
		}
		else {
			stableCount = 0;
		}

		if (stableCount >= stabilityThreshold) {
			isStabilised = true;
		}
	}
}

void setSpeedForP1(int value)
{
	writePot(MCP_ADDR, POT1, value);

}

void setSpeedForP0(int value)
{
	writePot(MCP_ADDR, POT0, value);

}

void writePot(uint8_t address, uint8_t pot, uint16_t val) {
	Wire.beginTransmission(address);
	Wire.write((pot & 3) << 4 | ((val >> 8) & 3));
	Wire.write(val & 0xFF);
	Wire.endTransmission();
}

void EnableEngine(bool isEnabled)
{
	enableStrobe(isEnabled);
	digitalWrite(PIN_EN, isEnabled);
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
	prev_mode = _mode;
	_mode = selectedMode;
	printSelectedMode(selectedSpeed);
	Serial.print("Selected mode:"); Serial.println(selectedSpeed);
}

void HandleButtonsWhilePlaying()
{
	if (btnMenuEnter.click() && isPlaying)
	{
		Serial.println("Stop Pressed");
		isPlaying = false;
	}
	if (_mode == E_MODE::Manual33 || _mode == E_MODE::Manual45)
	{
		if (btnMenuLeft.click()) {

			Serial.print("New speed:"); Serial.println(selectedSpeed -= 0.05);
		}

		if (btnMenuRight.click()) {
			Serial.print("New speed:"); Serial.println(selectedSpeed += 0.05);
		}
	}
}

void printSelectedMode(double selectedSpeed)
{
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
	{
		lcd.setCursor(0, 1);
		lcd.print("A33");
		lcd.setCursor(12, 1);
		lcd.print("rpm");
	}break;
	case Auto45: {
		lcd.setCursor(0, 1);
		lcd.print("A45");
		lcd.setCursor(12, 1);
		lcd.print("rpm");
	} break;
	case Manual33:
	{
		lcd.setCursor(0, 1);
		lcd.print("M33");
		lcd.setCursor(12, 1);
		lcd.print("rpm");
	}break;
	case Manual45:
	{
		lcd.setCursor(0, 1);
		lcd.print("M45");
		lcd.setCursor(12, 1);
		lcd.print("rpm");
	}
	break;
	default:
		break;
	}
}

void printMeasuredSpeed(float currenMeasuredSpeed, bool isStabilised)
{
	Serial.print(F("RPM: ")); Serial.print(currenMeasuredSpeed);
	Serial.print(F(" Is stable: ")); Serial.println(isStabilised);
	bool isVW = false;
	if (isStabilised && IsUltraPrecisionEnabled && abs(currenMeasuredSpeed - selectedSpeed) < 1.9) {
		isVW = true;
		currenMeasuredSpeed = selectedSpeed;
	}
	else
		if (abs(currenMeasuredSpeed - selectedSpeed) >= 1.9) isVW = false;

	if (prevMeasuredSpeed != currenMeasuredSpeed)
	{
		lcd.setCursor(6, 1);
		lcd.print("      ");
		lcd.setCursor(6, 1);
		lcd.print(currenMeasuredSpeed);
		lcd.setCursor(12, 1);
		lcd.print("rpm");

		prevMeasuredSpeed = currenMeasuredSpeed;
	}
	lcd.setCursor(15, 1);
	if (isVW) {

		lcd.print("*");
	}
	else lcd.print(" ");
}

void writeIntIntoEEPROM(int address, int number)
{
	byte byte1 = number >> 8;
	byte byte2 = number & 0xFF;
	EEPROM.write(address, byte1);
	EEPROM.write(address + 1, byte2);
}

int readIntFromEEPROM(int address)
{
	byte byte1 = EEPROM.read(address);
	byte byte2 = EEPROM.read(address + 1);
	return (byte1 << 8) + byte2;
}
/// <summary>
/// This prints the top line of the LCD
/// </summary>
/// <param name="text"></param>
void printState(const char* text)
{
	Serial.println(text);
	lcd.setCursor(0, 0);
	lcd.print("                ");
	lcd.setCursor(0, 0);
	lcd.print(text);
}

void printMenu(const E_SETUP menuState)
{
	lcd.setCursor(0, 1);
	lcd.print("                ");
	lcd.setCursor(1, 1);

	switch (menuState)
	{
	case Min33:
	{
		lcd.print("Min33 = ");
		printMenuValue(minPOT33);
	}
	break;
	case Min45:
	{
		lcd.print("Min45 = ");
		printMenuValue(minPOT45);
	}break;
	case UltraPrecision:
	{
		lcd.print("Ultra = ");
		printMenuValue(IsUltraPrecisionEnabled ? "True" : "False");
	}break;
	case Strobe:
	{
		lcd.print("Strobe = ");
		printMenuValue(IsStrobeEnabled ? "True" : "False");
	}break;
	case Exit:
	{
		lcd.print("     Exit    ");
	}break;
	default:
		break;
	}
}

void setPIDParams(double _Kp, double _Ki, double _Kd, int _measureInterval,int _minPot, int _maxPot)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	measureInterval = _measureInterval;
	minPot = _minPot;
	maxPot = _maxPot;
}

void printMenuValue(int value)
{
	lcd.setCursor(9, 1);
	lcd.print("       ");
	lcd.setCursor(9, 1);
	lcd.print(value);
}
void printMenuValue(const char* value)
{
	lcd.setCursor(10, 1);
	lcd.print("      ");
	lcd.setCursor(10, 1);
	lcd.print(value);
}
void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;

}

bool handleValueEditing(int& value, E_SETUP s_mode)
{
	bool valueChanged = false;
	switch (s_mode)
	{
	case Min33:
	case Min45:
	{
		if (btnMenuLeft.click() && value > 0) {
			value--; // Decrease the value with limit check
			valueChanged = true;
		}
		if (btnMenuRight.click() && value < 254) {
			value++; // Increase the value
			valueChanged = true;
		}
	}
	break;
	case Strobe: 	
	case UltraPrecision: // only 0 and 1 are allowed
	{
		if (btnMenuLeft.click() && value > 0) {
			value--; // Decrease the value with limit check
			valueChanged = true;
		}
		if (btnMenuRight.click() && value < 1) {
			value++; // Increase the value
			valueChanged = true;
		}
	}
	break;
	case Exit:
		break;
	default:
		break;
	}

	return valueChanged;
}

// Function to handle setup editing
void handleSetupEditing(const char* setupName, int& value, E_SETUP s_mode)
{
	printState(setupName);
	printMenuValue(value); // Show the initial value
	int lastValue = value;

	while (1)
	{
		
		if (handleValueEditing(value, s_mode))
		{
			printMenuValue(value);
			Serial.print(setupName);
			Serial.print(" value: ");
			Serial.println(value);
		}

		if (btnMenuEnter.click())
		{
			Serial.print("Saving ");
			Serial.print(setupName);
			Serial.print(" value...");
			Serial.println(value);
			int addr = 0;
			if (s_mode == E_SETUP::Min33)
			{
				addr = EE_ADDR_33;
				minPOT33 = value;
			}

			if (s_mode == E_SETUP::Min45)
			{
				addr = EE_ADDR_45;
				minPOT45 = value;
			}

			if (s_mode == E_SETUP::UltraPrecision)
			{
				addr = EE_ADDR_VW;
				IsUltraPrecisionEnabled = value > 0 ? true : false;
			}

			if (s_mode == E_SETUP::Strobe)
			{
				addr = EE_ADDR_STROBE;
				IsStrobeEnabled = value > 0 ? true : false;
			}
			writeIntIntoEEPROM(addr, value);

			printState("Saved");

			printState("   Calibration   ");
			break;
		}
	}
}

void HandleSetup()
{
	printState("   Calibration   ");
	E_SETUP setupType = E_SETUP::Min33;

	printMenu(setupType);
	printMenuValue(minPOT33); // Show the initial value for Min33

	while (1)
	{	
		if (btnMenuLeft.click())
		{
			if (setupType > E_SETUP::Min33)
			{
				setupType = static_cast<E_SETUP>(setupType - 1);
				printMenu(setupType);
			}
		}
		if (btnMenuRight.click())
		{
			if (setupType < E_SETUP::Exit)
			{
				setupType = static_cast<E_SETUP>(setupType + 1);
				printMenu(setupType);
			}
		}

		if (btnMenuEnter.click())
		{
			switch (setupType)
			{
			case Exit:
				return; // Exit the function
			case Min33:
				handleSetupEditing("     Min33   ", minPOT33, setupType);
				break;
			case Min45:
				handleSetupEditing("     Min45   ", minPOT45, setupType);
				break;
			case UltraPrecision:
			{
				handleSetupEditing("Ultra Precision", IsUltraPrecisionEnabled, setupType);
			}break;
			case Strobe:
			{
				handleSetupEditing("     Strobe", IsStrobeEnabled, setupType);
			}break;
			default:
				break;
			}

			//printMenu(setupType);
			//if (setupType == E_SETUP::Min33)
			//{
			//	printMenuValue(minPOT33); // Show the current value for Min33
			//}
			//else if (setupType == E_SETUP::Min45)
			//{
			//	printMenuValue(minPOT45); // Show the current value for Min45
			//}
		}
	}
}