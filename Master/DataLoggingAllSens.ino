#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MS5837.h"

#include "DepthSensor.h"
#include "DataLogging.h"
#include "PositionSensor.h"

// pin that enables SD card (4 for ethernet shield)
const int chipSelect = 4;


/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/* Set the delay between fresh samples for calibration */
#define BNO055_CALIBRATE_DELAY_MS (500)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Initialise depth depthSensor
MS5837 depthSensor;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // see if the card is present and can be initialized:
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  
  Serial.println("card initialized.");

  CSVinit();

  //================================================================
  // Initialise Sensors

    Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the orientation sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  // Initialise depth depthSensor
  initDepthSensor();

  //================================================================
  // read calibration data
  readEEPROMcal(true);
  wipeEEPROM();


  /* Display some basic information on this depthSensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

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

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display depthSensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");
  
  writeCSV();
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
