#include <TB9051FTGMotorCarrier.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Debounce.h>

#define PINA 9
#define PINB 10

#define PIN_SPD_D0 3
#define PIN_SPD_A0 A0

#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7

Adafruit_Debounce btnMenuRight(PIN_BTN_RIGHT, LOW);
Adafruit_Debounce btnMenuLeft(PIN_BTN_LEFT, LOW);
Adafruit_Debounce btnMenuEnter(PIN_BTN_MID, LOW);

static TB9051FTGMotorCarrier driver{ PINA, PINB };

static float throttlePercent{ 0.0f };

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
bool isPlaying = false;
volatile unsigned int numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0.0f;
int sweetPwm = 0;
#define minPwm 34
#define maxPwm 90
#define DEFAULT_INTERVAL 1

float windowIntervalSec = DEFAULT_INTERVAL; //2 seconds window - increase this if the no. of markers is less for better accuracy

#define NUM_MARKERS 55 //TO DO: Check this as per your setup 200

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
	
	lastMillis = millis();
	numPulses = 0;

	driver.enable();
	driver.setOutput(throttlePercent);

	pinMode(PIN_SPD_D0, INPUT_PULLUP); // declare ir as input	
		
	btnMenuEnter.begin();
	btnMenuLeft.begin();
	btnMenuRight.begin();
	
	attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, RISING);

	curMillis = lastMillis = millis();
	revPerMin = 0.0f;
	printSelectedSpeed(selectedSpeed);
	SetState(E_STATE::Idle);	
}

void  interruptRoutine() {
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	
	if (interrupt_time - last_interrupt_time > 3)
	{
		numPulses++;
	}
	last_interrupt_time = interrupt_time;
}

void loop() {
	
	switch (_state)
	{
	case Idle:
	{
		printState("(-)   Speed  (+)");
		
		isPlaying = false;
		windowIntervalSec = DEFAULT_INTERVAL;
		while (1)
		{
			
			updateButtons();
			if (btnMenuRight.justPressed()) {
				selectedSpeed = 45;
				printSelectedSpeed(selectedSpeed);
			}
			if (btnMenuLeft.justPressed())
			{
				selectedSpeed = 33.33;
				printSelectedSpeed(selectedSpeed);
			}

			if (btnMenuEnter.justPressed())
			{
				SetState(E_STATE::Starting);
				Serial.println("Play Pressed");
				revPerMin = 0;
				btnMenuEnter.update();
				isPlaying = true;
				break;
			}
			
		}
		

	}break;
	case Starting:
	{	
		printState("(-)   ....   (-)");
		int pwm = 53;
		motorA.motorGo(pwm);
		digitalWrite()
		lcd.setCursor(0, 0);
		lcd.print("    Starting     ");
		while (revPerMin < selectedSpeed - 4)
		{
			motorA.motorGo(pwm++);
			delay(400);
			measureSpeed();
		}
		Serial.print("Reached speed:"); Serial.println(selectedSpeed);
		isPlaying = true;
		SetState(E_STATE::Running);

	}break;
	case Running:
	{		

		printState("(-)   (||)   (+)");

		while (isPlaying) {

			measureSpeed();
			
			updateButtons();
			if (btnMenuRight.justPressed()) {
				selectedSpeed++;
				sweetPwm = 0;
				printSelectedSpeed(selectedSpeed);
				delay(500);
			}

			if (btnMenuLeft.justPressed()){
				selectedSpeed--;
				sweetPwm = 0;
				printSelectedSpeed(selectedSpeed);
				delay(500);
			}

			if (btnMenuEnter.justPressed() && isPlaying)
			{
				Serial.println("Stop Pressed");				
				SetState(E_STATE::Stopping);
				isPlaying = false;		
				
			}		
		}
	}break;
	case Stopping:
	{
		printState("    Stopping    ");
		printSelectedSpeed(0);
				
		int currPwm = motorA.getPWM();
		while (currPwm > 0)
		{
			if (currPwm < 38) currPwm = 0;
			motorA.motorGo(currPwm--);
			delay(300);		
		}
		printSelectedSpeed(0);
		SetState(E_STATE::Idle);
	}
	}
}

void measureSpeed()
{
	curMillis = millis();
	if (curMillis >= lastMillis + (windowIntervalSec * 1000)) {
		revPerMin = 60 * ((float)numPulses / (float)windowIntervalSec / NUM_MARKERS);
		lastMillis = curMillis;
		numPulses = 0;
		printMeasuredSpeed(revPerMin);

		float dev = abs(selectedSpeed - revPerMin);

		Serial.print("Deviation:"); Serial.println(dev);
		int mPwm = motorA.getPWM();
		
		if (dev > 0.3f)
		{			
			if (revPerMin < selectedSpeed && mPwm < maxPwm)
				motorA.motorGo(motorA.getPWM() + 1);

			if (revPerMin > selectedSpeed && mPwm > minPwm)
				motorA.motorGo(motorA.getPWM() - 1);
			windowIntervalSec = 0.1;
		}
		else {
			sweetPwm = mPwm;
			Serial.print("Using PWM Found:"); Serial.println(sweetPwm);
			motorA.motorGo(sweetPwm);
			windowIntervalSec = 2;
			
		}
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
	char string[5];  
	// Convert float to a string:
	dtostrf(selectedSpeed, 3, 2, string);  
	lcd.setCursor(6, 0);
	lcd.print("      ");
	lcd.setCursor(6, 0);
	lcd.print(string);
	lcd.setCursor(12, 0);
	lcd.print("rpm");  
	
}

void printMeasuredSpeed(float currentSpeed)
{
	Serial.print("Curr Speed:"); Serial.println(currentSpeed);	
	lcd.setCursor(0, 0);
	lcd.println("                ");
	lcd.setCursor(6, 0);
	lcd.print(currentSpeed);
    lcd.setCursor(12, 0);
	lcd.print("rpm");
}


void printState(const char * text)
{	
	Serial.println(text);
	lcd.setCursor(0,1);
	lcd.print(text);
}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}