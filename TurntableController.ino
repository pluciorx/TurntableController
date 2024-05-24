#include <LiquidCrystal_I2C.h>
#include <Adafruit_Debounce.h>
#include <MX1508.h>


#define PINA 9
#define PINB 9
#define NUMPWM 2

#define PIN_SPD_D0 3
#define PIN_SPD_A0 A0

#define PIN_BTN_LEFT 5
#define PIN_BTN_RIGHT 6
#define PIN_BTN_MID 7

Adafruit_Debounce btnMenuRight(PIN_BTN_RIGHT, LOW);
Adafruit_Debounce btnMenuLeft(PIN_BTN_LEFT, LOW);
Adafruit_Debounce btnMenuEnter(PIN_BTN_MID, LOW);

MX1508 motorA(PINA, PINB, FAST_DECAY, 2);
int PWMResolution = 90;

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
#define GREY     0xC618

//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

float selectedSpeed = 33.33;
bool isPlaying = false;
volatile unsigned int numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0.0f;
int sweetPwm = 0;
#define minPwm 50
#define maxPwm 90


float windowIntervalSec = 0.4f; //2 seconds window - increase this if the no. of markers is less for better accuracy

#define NUM_MARKERS 198 //TO DO: Check this as per your setup 200

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

	motorA.setPWM16(2, PWMResolution); // prescaler at 8, resolution 1000, PWM frequency = 16Mhz/8/1000=2000Hz

	pinMode(PIN_SPD_D0, INPUT_PULLUP); // declare ir as input	
	
	btnMenuEnter.begin();
	btnMenuLeft.begin();
	btnMenuRight.begin();
	
	printSelectedSpeed(selectedSpeed);
	attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, RISING);

	curMillis = lastMillis = millis();
	revPerMin = 0.0f;

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
		printState("<-    Speed   ->");
		isPlaying = false;
		
		while (1)
		{
			printSelectedSpeed(selectedSpeed);
			updateButtons();
			if (btnMenuRight.justPressed()) {
				selectedSpeed = 45;
			}
			if (btnMenuLeft.justPressed())
			{
				selectedSpeed = 33.33;
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
		printState("Starting...");
		int pwm = 50;
				
		while (revPerMin < selectedSpeed - 5)
		{
			motorA.motorGo(pwm++);
			delay(200);
			measureSpeed();
		}
		Serial.print("Reached speed:"); Serial.println(selectedSpeed);
		isPlaying = true;
		SetState(E_STATE::Running);

	}break;
	case Running:
	{		
		printState("-    Speed     +");

		while (isPlaying) {

			measureSpeed();
			
			updateButtons();
			if (btnMenuRight.justPressed()) {
				selectedSpeed++;
				sweetPwm = 0;
				printSelectedSpeed(selectedSpeed);
			}

			if (btnMenuLeft.justPressed()){
				selectedSpeed--;
				sweetPwm = 0;
				printSelectedSpeed(selectedSpeed);
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
		printState("Stopping");
		
		int currPwm = motorA.getPWM();
		while (currPwm > 0)
		{
			if (currPwm < 38) currPwm = 0;
			motorA.motorGo(currPwm--);
			//if spd = 0 then break;
			delay(100);
			measureSpeed();
		}
		
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
		
		if (dev > 0.5f)
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
	lcd.setCursor(6, 1);
	lcd.print("     ");
	lcd.setCursor(6, 1);
	lcd.print(string);
	lcd.setCursor(12, 1);
	lcd.print("rpm");  
}

void printMeasuredSpeed(float currentSpeed)
{
	Serial.print("Curr Speed:"); Serial.println(currentSpeed);	
	lcd.setCursor(6, 1);
	lcd.print("     ");
	lcd.setCursor(6, 1);
	lcd.print(currentSpeed);
		lcd.setCursor(12, 1);
	lcd.print("rpm");
}


void printState(const char * text)
{	
	Serial.println(text);

	lcd.setCursor(0,0 );
	lcd.print(text);
}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}