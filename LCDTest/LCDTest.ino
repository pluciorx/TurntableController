//Circuit
// Arduino Uno  -->   TCRT5000
// 5v           --->   VCC
// Grnd         --->   Grnd
// A0           --->   A0
// D8           --->   D0


const int pinIRd = 4;
const int pinIRa = A0;
const int pinLED = 13;
int IRvalueA = 0;
int IRvalueD = 0;
int numPulses = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(pinIRd, INPUT);
    pinMode(pinIRa, INPUT);
    pinMode(pinLED, OUTPUT);
    PCMSK2 |= 0b00010000;
   // attachInterrupt(digitalPinToInterrupt(pinIRd), interruptRoutine, CHANGE);
}

void  interruptRoutine() {
    numPulses++;
}

ISR(PCINT2_vect)
{
    numPulses++;
}

void loop()
{
    Serial.print("Analog Reading=");
    Serial.print(IRvalueA);
    Serial.print("\t Digital Reading=");
    Serial.println(IRvalueD);

    if (IRvalueD == LOW) {
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
        digitalWrite(LED_BUILTIN, LOW);
    }


    delay(500);

    Serial.println(numPulses);


}