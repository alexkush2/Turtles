#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "MS5837.h"

extern bool depthPresent;
extern bool SDPresent;
//Dont Forget -----
// pin that enables SD card (4 for ethernet shield)
//const int chipSelect = 4;

// name of file to save data in
#define FILE_NAME "accel.csv"

void writeCSV(Adafruit_BNO055 bno, MS5837 depthSensor){
  // if no sd card break
  //if(!SDPresent) return;

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);

  // get euler, gyro, and accelerometer data vectos
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  uint8_t systemCal, gyroCal, accelCal, magCal = 0;
  bno.getCalibration(&systemCal, &gyroCal, &accelCal, &magCal);

  // if depth sensor isn't present depth = 0
  int depth = 0;
  if(depthPresent){
    depth = depthSensor.depth();
  } 

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
    dataFile.print(", ");
    dataFile.print(depth);  // depth
    dataFile.println("");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening data file");
  }
}

// Delete CSV file
void removeCSV(){
  if(SD.exists(FILE_NAME)){
    SD.remove(FILE_NAME);
  }
}

// initialize CSV file on SD card
void CSVinit(){
  // check sd exits
  //if(!SDPresent) return;

  // delete old file
  removeCSV();
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open(FILE_NAME, FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println("Time, xOrient, yOrient, zOrient, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, CalS, CalA, CalG, CalM, Depth");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening data file");
  }
}

// saves the calibration offsets to the SD card
void logSensorOffsets(const adafruit_bno055_offsets_t &calibData){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("CalData.txt", FILE_WRITE);

  dataFile.println("---------------Sensor Calibration Offsets---------------\n");
  dataFile.print("Accelerometer: ");
  dataFile.print(calibData.accel_offset_x); Serial.print(" ");
  dataFile.print(calibData.accel_offset_y); Serial.print(" ");
  dataFile.print(calibData.accel_offset_z); Serial.print(" ");

  dataFile.print("\nGyro: ");
  dataFile.print(calibData.gyro_offset_x); Serial.print(" ");
  dataFile.print(calibData.gyro_offset_y); Serial.print(" ");
  dataFile.print(calibData.gyro_offset_z); Serial.print(" ");

  dataFile.print("\nMag: ");
  dataFile.print(calibData.mag_offset_x); Serial.print(" ");
  dataFile.print(calibData.mag_offset_y); Serial.print(" ");
  dataFile.print(calibData.mag_offset_z); Serial.print(" ");

  dataFile.print("\nAccel Radius: ");
  dataFile.print(calibData.accel_radius);

  dataFile.print("\nMag Radius: ");
  dataFile.print(calibData.mag_radius);
  
  dataFile.close();
}