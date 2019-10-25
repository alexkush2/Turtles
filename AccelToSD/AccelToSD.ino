#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// pin that enables SD card (4 for ethernet shield)
const int chipSelect = 4;

// name of file to save data in
#define FILE_NAME "accel.csv"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/* Set the delay between fresh samples for calibration */
#define BNO055_CALIBRATE_DELAY_MS (500)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// read cal data from eeprom, force == true to make system recalibrate
void readEEPROMcal(bool force = false){
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  
    /*
    *  Look for the sensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&sensor);
    if (bnoID != sensor.sensor_id){
        Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
        delay(500);
    }
    else{
        Serial.println("\nFound Calibration for this sensor in EEPROM.");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);

        displaySensorOffsets(calibrationData);

        Serial.println("\n\nRestoring Calibration data to the BNO055...");
        bno.setSensorOffsets(calibrationData);

        Serial.println("\n\nCalibration data loaded into BNO055");
        foundCalib = true;

        if (force == true){
          foundCalib == false;
        }
    }
    sensors_event_t event;
    bno.getEvent(&event);
    /* always recal the mag as It goes out of calibration very often */
    if (foundCalib && force == false){
        Serial.println("Move sensor slightly to calibrate magnetometers");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            delay(BNO055_CALIBRATE_DELAY_MS);
        }
    }
    else{
        Serial.println("Please Calibrate Sensor: ");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);

            Serial.print("X: ");
            Serial.print(event.orientation.x, 4);
            Serial.print("\tY: ");
            Serial.print(event.orientation.y, 4);
            Serial.print("\tZ: ");
            Serial.print(event.orientation.z, 4);

            /* Optional: Display calibration status */
            displayCalStatus();

            /* New line for the next sample */
            Serial.println("");

            /* Wait the specified delay before requesting new data */
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }
    }

    Serial.println("\nFully calibrated!");
    Serial.println("--------------------------------");
    Serial.println("Calibration Results: ");
    adafruit_bno055_offsets_t newCalib;
    bno.getSensorOffsets(newCalib);
    displaySensorOffsets(newCalib);

    Serial.println("\n\nStoring calibration data to EEPROM...");

    eeAddress = 0;
    bno.getSensor(&sensor);
    bnoID = sensor.sensor_id;

    EEPROM.put(eeAddress, bnoID);

    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, newCalib);
    Serial.println("Data stored to EEPROM.");

    Serial.println("\n--------------------------------\n");
    delay(500);
}

void wipeEEPROM(){
  EEPROM.put(0, long(0));
  Serial.println("EEPROM Cal Erased");
}

void writeCSV(){

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);

  // get euler, gyro, and accelerometer data vectos
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  uint8_t systemCal, gyroCal, accelCal, magCal = 0;
  bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(millis());   // time
    dataFile.print(", ");
    dataFile.print(euler.x());  // positional vector
    dataFile.print(", ");
    dataFile.print(euler.y());
    dataFile.print(", ");
    dataFile.print(euler.z());
    dataFile.print(", ");
    dataFile.print(gyro.x());   // gyro vector
    dataFile.print(", ");
    dataFile.print(gyro.y());
    dataFile.print(", ");
    dataFile.print(gyro.z());
    dataFile.print(", ");
    dataFile.print(accel.x());  // accelerometer vector 
    dataFile.print(", ");
    dataFile.print(accel.y());
    dataFile.print(", ");
    dataFile.print(accel.z());
    dataFile.print(", ");
    dataFile.print(systemCal);  // calibrations
    dataFile.print(", ");
    dataFile.print(gyroCal);
    dataFile.print(", ");
    dataFile.print(accelCal);
    dataFile.print(", ");
    dataFile.print(magCal);
    dataFile.println("");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening data file");
  }
}

void CSVinit(){
  removeCSV();
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Time, xOrient, yOrient, zOrient, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, CalS, CalA, CalG, CalM");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening data file");
  }
}

void removeCSV(){
  if(SD.exists(FILE_NAME)){
    SD.remove(FILE_NAME);
  }
}

void displaySensorDetails(void){
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void){
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData){
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

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
  // Initialise Sensor

    Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  //================================================================
  // read calibration data
  readEEPROMcal(true);
  wipeEEPROM();




  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);

}

void loop() {

    /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");
  
  writeCSV();
  
  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
