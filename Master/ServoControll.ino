#include "SDLogging.h"
//#include "DepthSensor.h"
#include "PositionSensor.h"
#include "ServoControler.h"

#include <Adafruit_NeoPixel.h>

// pin that enables SD card (4 for ethernet shield)
const int chipSelect = SDCARD_SS_PIN;
bool SDPresent = false;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Initialise depth depthSensor
MS5837 depthSensor;
bool depthPresent;

// pin declarations
byte LEDpin = 88;
Adafruit_NeoPixel strip(1, LEDpin, NEO_GRB + NEO_KHZ800);

#define buttonPin A11

byte sw1pin = 49;	// PID or proportional
byte sw2pin = 47;	// ignore sensors
byte sw3pin = 45;
byte sw4pin = 43;

#define ContGreenLED A7 
#define ContYellowLED A6

// Servo pinout
// O_R_B
// | | |
// S + -
byte tServoPin = 27;
byte bServoPin = 29;
byte lServoPin = 31;
byte rServoPin = 33;
byte motorPin = 35;


// control pins
#define HorizContPin A12
#define VertContPin A9
#define ThrottleContPin A10

// PID or regular proportional control
bool PropMode = true;

void setup() {

	//================================================================
	// pin setup
	pinMode(buttonPin, INPUT_PULLUP); // button, switches normally high
	pinMode(sw1pin, INPUT_PULLUP); 
	pinMode(sw2pin, INPUT_PULLUP); 
	pinMode(sw3pin, INPUT_PULLUP); 
	pinMode(sw4pin, INPUT_PULLUP); 

	pinMode(HorizContPin, INPUT);
	pinMode(VertContPin, INPUT);
	pinMode(ThrottleContPin, INPUT);


	pinMode(ContGreenLED, OUTPUT);
	pinMode(ContYellowLED, OUTPUT);

	strip.begin();
	strip.setPixelColor(0, 0, 255, 0);
	strip.show(); // Initialize Pixel to green

	digitalWrite(ContGreenLED, HIGH);

	// if swich 2 is flipped only do proportional control
	if(!digitalRead(sw2pin)){
		SetupServos();
		stopPID();
		ServoPropLoop();
	}

	// check to see if recalibration is needed, if button is pressed
	bool forceCal = false;
	if(!digitalRead(buttonPin)){
		forceCal = true;
		// wipeEEPROM();
		Serial.println("\n___________Erasing Calibration Data_____________\n");
		while (!digitalRead(buttonPin)){
			delay(500);
		}
		
	}

	//================================================================
	// Initialise Serial and SD

	// Open serial communications and wait for port to open:
	Serial.begin(115200);

	// see if the card is present and can be initialized:
	Serial.print("Initializing SD card...");
	if (!SD.begin(chipSelect)) {
		Serial.println("Card failed, or not present\nPress button to skip");
		// don't do anything more:
		while(1){ // error flash
			digitalWrite(ContGreenLED, LOW);
			digitalWrite(ContYellowLED, LOW);
			strip.setPixelColor(0, 255, 0, 0);
			strip.show(); // set pixel red
			delay(100);
			digitalWrite(ContYellowLED, HIGH);
			strip.setPixelColor(0, 0, 255, 0);
			strip.show(); // set pixel green
			delay(100);
			if(!digitalRead(buttonPin)){
				SDPresent == false;
				break;
			} 
		analogWrite(ContGreenLED,HIGH);
		}
	} else SDPresent == true;
	
	CSVinit();
	Serial.println("card initialized.");

	//================================================================
	// Initialise Sensors

	Serial.println("Orientation Sensor Test"); Serial.println("");
	
	/* Initialise the orientation sensor */
	if(!bno.begin()) {
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1){ // error flash
			digitalWrite(ContGreenLED, LOW);
			digitalWrite(ContYellowLED, LOW);
			strip.setPixelColor(0, 255, 0, 0);
			strip.show(); // set pixel red
			delay(100);
			strip.setPixelColor(0, 200, 50, 200);
			digitalWrite(ContGreenLED, HIGH);
			digitalWrite(ContYellowLED, HIGH);
			strip.show(); // set pixel purple
			delay(100);
		}
	}

	delay(1000);

	// Initialise depth depthSensor
	depthPresent = initDepthSensor(depthSensor);

	//================================================================
	// read calibration data
	//readEEPROMcal(bno, forceCal);                                     // reset when done with tinkering to reenable calibrations
	//wipeEEPROM();

	/* Display some basic information on this orientation sensor */
	displaySensorDetails(bno);

	/* Optional: Display current status */
	displaySensorStatus(bno);

	bno.setExtCrystalUse(true);

	// sweep servos 
	Serial.println("\nSweeping Servos\n");
	SetupServos();
	SweepServos();

	// if proportional mode, disable PID
	//if(digitalRead(sw1pin)){
		stopPID();
	//}

	digitalWrite(ContGreenLED,HIGH);
}

void loop() {

	/* Get a new depthSensor event */
	sensors_event_t event;
	bno.getEvent(&event);

	/* Display the floating point data */
	Serial.print("X: ");
	Serial.print(event.orientation.x, 4);
	Serial.print("\tY: ");
	Serial.print(event.orientation.y, 4);
	Serial.print("\tZ: ");
	Serial.print(event.orientation.z, 4);
	Serial.print("\tDepth: ");
	Serial.print(depthSensor.depth());


	// /* Optional: Display calibration status */
	displayCalStatus(bno);

	// /* Optional: Display depthSensor status (debug only) */
	// //displaySensorStatus();

	int* pos;

	if(!PropMode){
		// get orientation data and sent to PID controller
		double Orient[] = {event.gyro.x, event.gyro.y, event.orientation.z};
		pos = ServoPIDUpdate(Orient);
	
		// if button pressed set all servos straight back
		if(!digitalRead(buttonPin)){
			resetPID();
			SetServos(90,90,90,90);
			delay(500);
		}

		// print PID 
		Serial.print("  Ki,x,y,z: ");
		Serial.print(XPID.getIntegral());
		Serial.print(", ");
		Serial.print(YPID.getIntegral());
		Serial.print(", ");
		Serial.print(ZPID.getIntegral());
	} else{ // proportional mode
		pos = ServoPropControl();

		// when button pressed set servos to straight back for 500ms
		if(!digitalRead(buttonPin)){
			SetServos(90,90,90,90);
			delay(500);
		}
	}
	

	// /* New line for the next sample */
	Serial.println("");
	
	// log the shit to CSV
	writeCSV(bno, depthSensor, pos);
	
	/* Wait the specified delay before requesting nex data */
	delay(BNO055_SAMPLERATE_DELAY_MS);

}
