#include <ButtonControl.h>  //https://github.com/fellipecouto/ButtonControl
#include <LiquidCrystal_I2C.h>
#include <util/atomic.h> // this library includes the ATOMIC_BLOCK macro.
#include <EEPROM.h>
//IR Sensor
#define PIN_SENSOR 2

//Engine enablePIN (SHORT A6 WITH THE PIN BELOW)
#define PIN_EN PIN_A3
//PIN ENGINE state
#define PIN_EN_STATE 8
#define PIN_EN_STROBO 10
#define DEBUG 1

//Buttons
#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7
#define BTN_DEBOUNCE_MS 50

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
#define POT0_Default 1 //Lower value = higher span of the voltage available.

int minPot, maxPot;
int measureInterval;
double Kp;
double Ki;
double Kd;

//--------------------CALIBRATION---------------------
//33.33 PID definitions 
#define Kp33 0.70  // Increased for faster response
#define Ki33 0.00   // Increased to reduce steady-state error
#define Kd33 0.60   // Introduced for damping oscillations
#define measureInterval33 200
int maxPOT33 = 255;
#define minPOT33 1
#define EE_ADDR_33 0

//45 definitions
#define Kp45 0.8  // Increased for faster response
#define Ki45 0.001  // Increased to reduce steady-state error
#define Kd45 0.6  // Introduced for damping oscillations
#define measureInterval45  100
int maxPOT45 = 255;
#define minPOT45 1
#define EE_ADDR_45 4

//--------------------CALIBRATION---------------------
volatile int currentPVal = maxPOT33;

double previousError = 0;
double integral = 0;
const double integralLimit = 100; // Limit for the integral term

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
	Max33 = 0,
	Max45 = 1,
	UltraPrecision = 2,
	Strobe = 3,
	Exit = 4
};

E_STATE _state;
E_MODE _mode;

void setup() {
	Serial.begin(115200);
	lcd.init();
	lcd.backlight();
	lcd.clear();

	Serial.println();
	Serial.println(F("Turntable v 1.1"));

	markersPerWindowActual = 0;

	pinMode(PIN_SENSOR, INPUT_PULLUP);
	pinMode(PIN_EN_STATE, INPUT);
	pinMode(PIN_EN_STROBO, OUTPUT);
	
	//pinMode(PIN_EN, INPUT_PULLUP);
	pinMode(PIN_EN, OUTPUT);
	
	digitalWrite(PIN_EN, LOW);
	EnableEngine(false);
	setSpeedForP0(POT0_Default);
	setSpeedForP1(1);

	attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), interruptRoutine, RISING);
	printState(F("  Machina czasu "));

	int eMax33 = readIntFromEEPROM(EE_ADDR_33);
	int eMax45 = readIntFromEEPROM(EE_ADDR_45);

	maxPOT33 = eMax33 > 0 ? eMax33 : maxPOT33;
	maxPOT45 = eMax45 > 0 ? eMax45 : maxPOT45;
	IsUltraPrecisionEnabled = readIntFromEEPROM(EE_ADDR_VW) > 0 ? true : false;
	IsStrobeEnabled = readIntFromEEPROM(EE_ADDR_STROBE) > 0 ? true : false;

	Serial.print(F("Strobe:")); Serial.println(IsStrobeEnabled);
	Serial.print(F("Ultra Precision:")); Serial.println(IsUltraPrecisionEnabled);
	Serial.print(F("Max33 Value:")); Serial.println(maxPOT33);
	Serial.print(F("Max45 Value:")); Serial.println(maxPOT45);

	curMillis = lastMillis = millis();
	while (1)
	{
		curMillis = millis();

		if (millis() >= lastMillis + 1500) {
			lastMillis = curMillis;

			SetSelectedMode(E_MODE::Auto33);
			SetState(E_STATE::Idle);

			Serial.println(F("Going normal mode..."));
			break;
		}
		if (btnMenuEnter.click())
		{
			SetState(E_STATE::Setup);
			Serial.println(F("Entering Calibration"));
			break;
		}
	}

}

void enableStrobe(bool isEnabled)
{
	if (IsStrobeEnabled)
	{
		digitalWrite(PIN_EN_STROBO, isEnabled);
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
	Serial.println("Idle...");
	//updateButtons();
	switch (_state)
	{
		case Idle:
		{
			EnableEngine(false);
			printState(F("<-    Tryb    ->"));
			isPlaying = false;
			isStabilised = false;

			SetSelectedMode(_mode);

			while (!isPlaying)
			{
				if (btnMenuLeft.click()) {
					Serial.println(F("Btn Left Pressed"));
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
					Serial.println(F("Btn Right Pressed"));
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
					Serial.println(F("Play Pressed"));
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
			printState(F("    Start    "));
			printMeasuredSpeed(0, false);
			int x = minPot;

			printState(F("  Stabilizacja "));

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
			Serial.print(F("Machina czasu FW for PCB rev 1.0a:")); Serial.println(revPerSecondRequired, 3);
			Serial.print(F("Rev's per/s req:")); Serial.println(revPerSecondRequired, 3);
			Serial.print(F("Markers per/s req:")); Serial.println(markersPerSecondRequired, 3);
			Serial.print(F("Measure Interval ms:")); Serial.println(measureInterval);
			Serial.print(F("Pulses required per/windows:")); Serial.println(markersPerWindowRequired, 3);
			Serial.print(F("Max POT Value:")); Serial.println(maxPot);
			Serial.print(F("Min POT Value:")); Serial.println(minPot);
			Serial.print(F("Current P1 Value:")); Serial.println(currentPVal);
			Serial.print(F("Kp:")); Serial.println(Kp);
			Serial.print(F("Ki:")); Serial.println(Ki);
			Serial.print(F("Kd:")); Serial.println(Kd);

			markersPerWindowActual = 0;

			while (!isStabilised )
			{				
				calculateAndApplySpeed(false);

				if (btnMenuEnter.click() || digitalRead(PIN_EN_STATE) == LOW)
				{
					Serial.println(F("Stop"));
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
				printState(F("      Gram    "));
			}break;
			case Manual33:
			case Manual45:
			{
				printState(F("-     Gram    +"));
			}break;
			default: {
			}break;

			}

			while (isPlaying)
			{
				if (digitalRead(PIN_EN_STATE) == LOW) break;
				HandleButtonsWhilePlaying();
				calculateAndApplySpeed(false);
			}
			SetState(E_STATE::Stopping);
		}break;
		case Stopping:
		{			
			
			EnableEngine(false);

			printState(F("   Zatrzymanie  "));

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

#ifdef DEBUG
		Serial.println();		
		Serial.print(F("Markers required per interval: ")); Serial.println(markersPerWindowRequired, 3);
		Serial.print(F("Markers counted per interval: ")); Serial.println(numberOfPulses);		
		Serial.print(F("Error: ")); Serial.println(error);
		Serial.print(F("Current P0 Value: ")); Serial.println(currentPVal);
#endif // DEBUG
		if (displayOnly) return;

		setSpeedForP1(newPot);

		// Check stability
		if (abs(error) <= acceptableError) {
			stableCount++;
		}
		else {
			stableCount = 0;
		}

		if (stableCount >= stabilityThreshold) {
			if (!isStabilised) Serial.println("Stable rpm reached.");
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
	_mode = selectedMode;
	printSelectedMode(selectedSpeed);
	
}

void HandleButtonsWhilePlaying()
{
	if (btnMenuEnter.click() && isPlaying)
	{
		Serial.println(F("Stop Pressed"));
		isPlaying = false;
	}
	if (_mode == E_MODE::Manual33 || _mode == E_MODE::Manual45)
	{
		if (btnMenuLeft.click()) {

			Serial.print(F("New speed:")); Serial.println(selectedSpeed -= 0.05);
		}

		if (btnMenuRight.click()) {
			Serial.print(F("New speed:")); Serial.println(selectedSpeed += 0.05);
		}
	}
}

void printSelectedMode(double selectedSpeed)
{
	Serial.print(F("Selected mode:")); Serial.println(selectedSpeed);
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
		lcd.print(F("A33"));
		lcd.setCursor(12, 1);
		lcd.print(F("rpm"));
	}break;
	case Auto45: {
		lcd.setCursor(0, 1);
		lcd.print(F("A45"));
		lcd.setCursor(12, 1);
		lcd.print(F("rpm"));
	} break;
	case Manual33:
	{
		lcd.setCursor(0, 1);
		lcd.print(F("M33"));
		lcd.setCursor(12, 1);
		lcd.print(F("rpm"));
	}break;
	case Manual45:
	{
		lcd.setCursor(0, 1);
		lcd.print(F("M45"));
		lcd.setCursor(12, 1);
		lcd.print(F("rpm"));
	}
	break;
	default:
		break;
	}
}

void printMeasuredSpeed(float currenMeasuredSpeed, bool isStabilised)
{
#ifdef DEBUG

	Serial.print(F("RPM: ")); Serial.print(currenMeasuredSpeed);
	Serial.print(F(" Is stable: ")); Serial.println(isStabilised);

#endif // DEBUG
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
		lcd.print(F("rpm"));

		prevMeasuredSpeed = currenMeasuredSpeed;
	}
	lcd.setCursor(15, 1);
	if (isVW) {

		lcd.print(F("*"));
	}
	else lcd.print(F(" "));
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
void printState(const __FlashStringHelper* text)
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
	case Max33:
	{
		lcd.print(F("Max33 = "));
		printMenuValue(maxPOT33);
	}
	break;
	case Max45:
	{
		lcd.print(F("Max45 = "));
		printMenuValue(maxPOT45);
	}break;
	case UltraPrecision:
	{
		lcd.print(F("Ultra = "));
		printMenuValue(IsUltraPrecisionEnabled ? "Tak" : "Nie");
	}break;
	case Strobe:
	{
		lcd.print("Strobo = ");
		printMenuValue(IsStrobeEnabled ? "Tak" : "Nie");
	}break;
	case Exit:
	{
		lcd.print(F("     Wyjdz    "));
	}break;
	default:
		break;
	}
}

void setPIDParams(double _Kp, double _Ki, double _Kd, int _measureInterval, int _minPot, int _maxPot)
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
	case Max33:
	case Max45:
	{
		if (btnMenuLeft.fastClick() && value > -1) {
			value--; // Decrease the value with limit check
			valueChanged = true;
		}
		if (btnMenuRight.fastClick() && value < 255) {
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
void handleSetupEditing(const __FlashStringHelper* setupName, int& value, E_SETUP s_mode)
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
			Serial.print(F(" value: "));
			Serial.println(value);
		}

		if (btnMenuEnter.click())
		{
			Serial.print(F("Saving "));
			Serial.print(setupName);
			Serial.print(F(" value: "));
			Serial.println(value);
			int addr = 0;
			if (s_mode == E_SETUP::Max33)
			{
				addr = EE_ADDR_33;
				maxPOT33 = value;
			}

			if (s_mode == E_SETUP::Max45)
			{
				addr = EE_ADDR_45;
				maxPOT45 = value;
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

			printState(F("Zapisane"));

			printState(F("   Ustawienia   "));
			break;
		}
	}
}

void HandleSetup()
{
	printState(F("   Ustawienia   "));
	E_SETUP setupType = E_SETUP::Max33;

	printMenu(setupType);
	printMenuValue(maxPOT33); // Show the initial value for Min33

	while (1)
	{
		if (btnMenuLeft.click())
		{
			if (setupType > E_SETUP::Max33)
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
			case Max33:
				handleSetupEditing(F("     Max33   "), maxPOT33, setupType);
				break;
			case Max45:
				handleSetupEditing(F("     Max45   "), maxPOT45, setupType);
				break;
			case UltraPrecision:
			{
				handleSetupEditing(F("Ultra Precision"), IsUltraPrecisionEnabled, setupType);
			}break;
			case Strobe:
			{
				handleSetupEditing(F("     Strobo"), IsStrobeEnabled, setupType);
			}break;
			default:
				break;
			}
		}
	}
}