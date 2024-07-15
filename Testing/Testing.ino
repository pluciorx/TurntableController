#include <Wire.h> //include Wire.h library

#define MCP_ADDR 0x28 //(40)
#define POT0 0x10
#define POT1 0x11

#define maxPOT 215

#define MCP_ADDR 0x28 //(40)
#define fet 9

void setup()
{

	Wire.begin(); // Wire communication begin
	Serial.begin(9600); // The baudrate of Serial monitor is set in 9600
	//while (!Serial); // Waiting for Serial Monitor
	Serial.println("\nI2C Scanner");
	pinMode(fet, OUTPUT);
	digitalWrite(fet, LOW);

	writePot(MCP_ADDR, POT1, 127); //r1

	writePot(MCP_ADDR, POT0, 127);
	Serial.println("Ready.");

	/*while (1)
	{
		
		digitalWrite(fet, LOW);
		setSpedForP1(220);
		delay(2000);
		digitalWrite(fet, HIGH);
		delay(2000);

	}*/


}
char incomingByte = 0; // for serial debugging
int oldstate,state = 0;
bool newData;
String readString;

void loop()
{
	char buffer[] = { ' ',' ',' ',' ',' ',' ',' ' }; // Receive up to 7 bytes
	while (!Serial.available()); // Wait for characters
	Serial.readBytesUntil('\n', buffer, 7);
	int state = atoi(buffer);
	//if (state == -1) digitalWrite(fet, LOW);
	//if (state == -2) digitalWrite(fet, HIGH);
	if (state >= 0 ) setSpedForP0(state);
	if (state < 0) setSpedForP1(abs(state)); //fine tunning

}



void writePot(uint8_t address, uint8_t pot, uint16_t val) {
	Wire.beginTransmission(address);
	Wire.write((pot & 3) << 4 | ((val >> 8) & 3));
	Wire.write(val & 0xFF);
	Wire.endTransmission();
}



void setSpedForP1(int value)
{
	Serial.println(value);
	writePot(MCP_ADDR, POT1, value);
	
}
void setSpedForP0(int value)
{
	Serial.println(value);
	writePot(MCP_ADDR, POT0, value);
	
}