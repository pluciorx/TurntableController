#include <FastPwmPin.h>
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

bool isPlaying = false;
volatile unsigned int numPulses = 0;


#define minPwm 34
#define maxPwm 90
#define DEFAULT_INTERVAL 1

float windowIntervalSec = DEFAULT_INTERVAL; //2 seconds window - increase this if the no. of markers is less for better accuracy

#define NUM_MARKERS 55 //TO DO: Check this as per your setup 200
unsigned long frequency = 22000L;
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

	driver.enable();
	//driver.setOutput(throttlePercent);

	pinMode(PIN_SPD_D0, INPUT_PULLUP); // declare ir as input	
		
	btnMenuEnter.begin();
	btnMenuLeft.begin();
	btnMenuRight.begin();
	
	attachInterrupt(digitalPinToInterrupt(PIN_SPD_D0), interruptRoutine, RISING);

	printSelectedSpeed(throttlePercent);

	pinMode(3, OUTPUT);

	pinMode(PINA, OUTPUT);

	FastPwmPin::enablePwmPin(PINA, frequency, 75);

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
		printBottomLine("(-)   Speed  (+)");
		
		isPlaying = false;
		windowIntervalSec = DEFAULT_INTERVAL;
		throttlePercent = 0;
		while (1)
		{
			
			updateButtons();
			if (btnMenuRight.justPressed()) {
				throttlePercent = 0.2;
				printSelectedSpeed(throttlePercent);
			}
			if (btnMenuLeft.justPressed())
			{
				throttlePercent = 0.1;
				printSelectedSpeed(throttlePercent);
			}

			if (btnMenuEnter.justPressed())
			{
				SetState(E_STATE::Running);
				Serial.println("Play Pressed");
				
				btnMenuEnter.update();
				isPlaying = true;
				break;
			}
			
		}
		

	}break;
	case Starting:
	{	
		printBottomLine("(-)   ....   (-)");		
		lcd.setCursor(0, 0);
		lcd.print("    Starting     ");
	
	
		isPlaying = true;
		SetState(E_STATE::Running);

	}break;
	case Running:
	{		

		printBottomLine("(-)   (||)   (+)");

		while (isPlaying) {

			driver.setOutput(throttlePercent);
			
			updateButtons();
			if (btnMenuRight.justPressed()) {
				throttlePercent +=0.01;
				frequency += 500L;
				printSelectedSpeed(throttlePercent);
				delay(50);
			}

			if (btnMenuLeft.justPressed()){
				throttlePercent -=0.01;
				frequency -= 500L;
				printSelectedSpeed(throttlePercent);
				delay(50);
			}

			if (btnMenuEnter.justPressed() && isPlaying)
			{
				Serial.println("Stop Pressed");				
				SetState(E_STATE::Stopping);
				isPlaying = false;	
				digitalWrite(PINA, LOW);
				
			}	
			FastPwmPin::enablePwmPin(PINA, frequency, 75);
		}
	}break;
	case Stopping:
	{
		printBottomLine("    Stopping    ");
		
		SetState(E_STATE::Idle);
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


void printBottomLine(const char * text)
{	
	Serial.println(text);
	lcd.setCursor(0,1);
	lcd.print(text);
}

void SetState(E_STATE newState)
{
	_state = _state != newState ? newState : _state;
}