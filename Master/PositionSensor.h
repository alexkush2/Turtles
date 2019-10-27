#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>


// dont forget

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/* Set the delay between fresh samples for calibration */
#define BNO055_CALIBRATE_DELAY_MS (500)

// // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// //                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

extern byte GCalPin;
extern byte ACalPin;
extern byte MCalPin;

void displaySensorDetails(Adafruit_BNO055 bno){
  sensor_t depthSensor;
  bno.getSensor(&depthSensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(depthSensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(depthSensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(depthSensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(depthSensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(depthSensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(depthSensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(Adafruit_BNO055 bno){
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

void displayCalStatus(Adafruit_BNO055 bno){
  /* Get the four calibration values (0..3) */
  /* Any depthSensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system){
    Serial.print("! ");
  } else if(gyro==3){
    // turn on Gyro Cal LED
    digitalWrite(GCalPin, HIGH);
  } //else if(mag==3){
  //   digitalWrite(MCalPin, HIGH);
  // } else if (accel==3){
  //   digitalWrite(ACalPin, HIGH);
  // }
  
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

// read cal data from eeprom, force == true to make system recalibrate
void readEEPROMcal(Adafruit_BNO055 bno, bool force = false){
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t depthSensor;

  
    /*
    *  Look for the depthSensor's unique ID at the beginning oF EEPROM.
    *  This isn't foolproof, but it's better than nothing.
    */
    bno.getSensor(&depthSensor);
    if (bnoID != depthSensor.sensor_id){
        Serial.println("\nNo Calibration Data for this depthSensor exists in EEPROM");
        delay(500);
    }
    else{
        Serial.println("\nFound Calibration for this depthSensor in EEPROM.");
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
        Serial.println("Move depthSensor slightly to calibrate magnetometers");
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
            displayCalStatus(bno);

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
    bno.getSensor(&depthSensor);
    bnoID = depthSensor.sensor_id;

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