#include <Adafruit_Debounce.h>
#include "Ucglib.h"  // Include Ucglib library to drive the display
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

#define cs 12
#define dc 9
#define rst 8

Ucglib_ST7735_18x128x160_HWSPI ucg(8, 10, 12);  // (A0=8, CS=10, RESET=9)

float selectedSpeed = 33.33;
bool isPlaying = false;
volatile unsigned int numPulses = 0;
unsigned long lastMillis = 0;
unsigned long curMillis = 0;
float revPerMin = 0.0f;
int sweetPwm = 0;
int minPwm = 50;
int maxPwm = 80;


#define windowIntervalSec 0.5 //2 seconds window - increase this if the no. of markers is less for better accuracy

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
	ucg.begin(UCG_FONT_MODE_SOLID);  // It writes a background for the text. This is the recommended option

	lastMillis = millis();
	numPulses = 0;

	ucg.clearScreen();  // Clear the screen
	// Set display orientation:
	ucg.setRotate90();  // Put 90, 180 or 270, or co
	ucg.setFont(ucg_font_10x20_tr);

	ucg.setColor(0, 255, 255, 255);  // Set color (0,R,G,B)
	ucg.setColor(1, 0, 0, 0);  // Set color of text background (1,R,G,B)
	ucg.setPrintPos(0, 24);  // Set position (x,y)
	ucg.print("   Turntable   ");  // Print text or value

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
		printState("Speed:");
		isPlaying = false;
		printMeasuredSpeed(0);
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
		printState("Starting:");
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
		printState("Playing:");

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
		Serial.print("Morot PWM:"); Serial.println(motorA.getPWM());

		if (dev > 0.5f)
		{
			if (revPerMin < selectedSpeed && motorA.getPWM() < maxPwm)
				motorA.motorGo(motorA.getPWM() + 1);

			if (revPerMin > selectedSpeed && motorA.getPWM() > minPwm)
				motorA.motorGo(motorA.getPWM() - 1);
			delay(100);
		}
		else {
			sweetPwm = motorA.getPWM();
			Serial.print("Using PWM Found:"); Serial.println(sweetPwm);
			motorA.motorGo(sweetPwm);

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
	Serial.print("Speed:"); Serial.println(selectedSpeed);
	char string[5];  // Create a character array of 10 characters
	// Convert float to a string:
	dtostrf(selectedSpeed, 3, 2, string);  // (<variable>,<amount of digits we are going to use>,<amount of decimal digits>,<string name>)
	ucg.setFont(ucg_font_9x18_tr);  // Set font
	ucg.setColor(0, 255, 255, 0);  // Set color (0,R,G,B)
	ucg.setColor(1, 0, 0, 0);  // Set color of text background (1,R,G,B)
	ucg.setPrintPos(80, 45);  // Set position (x,y)
	ucg.print(string);  // Print text or value
	ucg.setPrintPos(125, 45);  // Set position (x,y)
	ucg.print("rpm");  // Print text or value
}

void printMeasuredSpeed(float currentSpeed)
{
	Serial.print("Curr Speed:"); Serial.println(currentSpeed);	
	//Serial.print("Curr Speed:"); Serial.println(string);
	ucg.setFont(ucg_font_9x18_tr);  // Set font
	ucg.setColor(0, 0, 0, 0);  // Set color (0,R,G,B)
	ucg.drawBox(70, 70, 50, 20);
	ucg.setColor(0, 255, 255, 0);  // Set color (0,R,G,B)
	ucg.setColor(1, 0, 0, 0);  // Set color of text background (1,R,G,B)
	ucg.setPrintPos(70, 90);  // Set position (x,y)
	ucg.print(currentSpeed);  // Print text or value
	ucg.setPrintPos(125, 90);  // Set position (x,y)
	ucg.print("rpm");  // Print text or value
}


void printState(const char * text)
{	
	Serial.println(text);
	ucg.setColor(0, 0, 0, 0);  // Set color (0,R,G,B)
	ucg.drawBox(0, 30, 80, 20);
	ucg.setFont(ucg_font_9x18_tr);  // Set font
	ucg.setColor(0, 255, 255, 255);  // Set color (0,R,G,B)
	ucg.setColor(1, 0, 0, 0);  // Set color of text background (1,R,G,B)
	ucg.setPrintPos(2, 45);  // Set position (x,y)
	ucg.print(text);  // Print text or value
}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}