#include <Wire.h> //include Wire.h library

#define MCP_ADDR 0x28 //(40)
#define POT0 0x10
#define POT1 0x11

#define maxPOT 215
volatile int x = 50;

void setup()
{
	Wire.begin(); // Wire communication begin
	Serial.begin(9600); // The baudrate of Serial monitor is set in 9600
	while (!Serial); // Waiting for Serial Monitor
	Serial.println("\nI2C Scanner");
	writePot(MCP_ADDR, POT0, 128); //r1

	writePot(MCP_ADDR, POT1, 1);
	Serial.println("Ready.");
}

void loop()
{

	//delay(5000);
	Serial.println("Starting");
	byte error, address; //variable for error and I2C address
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address);
		error = Wire.endTransmission();

		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");
			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknown error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");
	if (x< maxPOT) writePot(MCP_ADDR, POT1, x++);
	Serial.println(x);
	delay(250); // wait 5 seconds for the next I2C scan
}

void writePot(uint8_t address, uint8_t pot, uint16_t val) {
	Wire.beginTransmission(address);
	Wire.write((pot & 3) << 4 | ((val >> 8) & 3));
	Wire.write(val & 0xFF);
	Wire.endTransmission();
}