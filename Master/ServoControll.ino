#include "SDLogging.h"
//#include "DepthSensor.h"
#include "PositionSensor.h"
#include "ServoControler.h"

//#include <Servo.h>

// pin that enables SD card (4 for ethernet shield)
const int chipSelect = 4;
bool SDPresent = false;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Initialise depth depthSensor
MS5837 depthSensor;
bool depthPresent;

// pin declarations
byte GCalPin = 43;
byte ACalPin = 45;
byte MCalPin = 47;
byte buttonPin = 41;

// Servo pinout
// O_R_B
// | | |
// S + -
byte tServoPin = 6;
byte bServoPin = 5;
byte lServoPin = 3;
byte rServoPin = 2;
byte motorPin = 1;

// control pins
byte HorizContPin = 8;
byte VertContPin = 9;
byte ThrottleContPin = 10;

// PID or regular proportional control
bool PropMode = true;

void setup() {

  //================================================================
  // pin setup
  pinMode(buttonPin, INPUT_PULLUP); // button normally high

  pinMode(HorizContPin, INPUT);
  pinMode(VertContPin, INPUT);
  pinMode(ThrottleContPin, INPUT);

  pinMode(GCalPin, OUTPUT);
  pinMode(ACalPin, OUTPUT);
  pinMode(MCalPin, OUTPUT);
  digitalWrite(GCalPin, HIGH);
  digitalWrite(ACalPin, HIGH);
  digitalWrite(MCalPin, HIGH);
  delay(500);
  digitalWrite(GCalPin, LOW);
  digitalWrite(ACalPin, LOW);
  digitalWrite(MCalPin, LOW);

  // check to see if recalibration is needed, if button is pressed
  bool forceCal = false;
  if(!digitalRead(buttonPin)){
    forceCal = true;
    wipeEEPROM();
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
      digitalWrite(MCalPin, LOW);
      delay(100);
      digitalWrite(MCalPin, HIGH);
      delay(100);
      if(!digitalRead(buttonPin)){
        SDPresent == false;
        break;
      } 
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
      digitalWrite(ACalPin, LOW);
      delay(100);
      digitalWrite(ACalPin, HIGH);
      delay(100);
    }
  }

  delay(1000);

  // Initialise depth depthSensor
  depthPresent = initDepthSensor(depthSensor);

  //================================================================
  // read calibration data
  //readEEPROMcal(bno, forceCal);                                     // reset when done with tinkering
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
  stopPID();

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
    double Orient[] = {event.gyro.x, event.gyro.y, event.orientation.z};
    pos = ServoPIDUpdate(Orient);
  
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

    if(!digitalRead(buttonPin)){
      SetServos(90,90,90,90);
      delay(500);
    }
  }
  

  // /* New line for the next sample */
  Serial.println("");
  
  writeCSV(bno, depthSensor, pos);
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
