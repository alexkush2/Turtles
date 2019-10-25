#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include "DepthSensor.h"

//Dont Forget -----
// pin that enables SD card (4 for ethernet shield)
//const int chipSelect = 4;

// name of file to save data in
#define FILE_NAME "accel.csv"


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
    dataFile.println("Time, xOrient, yOrient, zOrient, xGyro, yGyro, zGyro, xAccel, yAccel, zAccel, CalS, CalA, CalG, CalM, Depth");
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