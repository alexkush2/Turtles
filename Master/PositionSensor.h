#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <SPI.h>
// #include <SdFat.h>
// #include <Adafruit_SPIFlash.h>
// #include <EEPROM.h>

// // setup flash
// Adafruit_FlashTransport_QSPI flashTransport(PIN_QSPI_SCK, PIN_QSPI_CS, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);
// Adafruit_SPIFlash flash(&flashTransport);

// // file system object from SdFat
// FatFileSystem fatfs;
// File CalFile;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// //                                   id, address
// Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// extern byte GCalPin;
// extern byte ACalPin;
// extern byte MCalPin;
extern byte buttonPin;

// displays sensor specs and shit to serial monitor
void displaySensorDetails(Adafruit_BNO055 bno){
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

// displays sensor status and tests and shit to serial monitor
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

void updateCalStatusLEDS(Adafruit_BNO055 bno){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  if(gyro==3){
    // turn on Gyro Cal LED
    // turn on Gyro Cal LED
    strip.setPixelColor(0, 255, 120, 50);
    strip.show(); // set pixel orange
  }
  // } else digitalWrite(GCalPin, LOW);
  // if(mag==3){
  //   // turn on magnetometer cal LED                // need to sort all this shit out! dont have good cal error display
  //   digitalWrite(MCalPin, HIGH);
  // } else digitalWrite(MCalPin, LOW);
  // if (accel==3){
  //   // turn on Accelerometer Cal LED
  //   digitalWrite(ACalPin, HIGH);
  // } else digitalWrite(ACalPin, LOW);
}

// prints the calibration status to the serial monitor and updates LEDS
void displayCalStatus(Adafruit_BNO055 bno){
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system){
    Serial.print("! ");
  }

  updateCalStatusLEDS(bno);
  
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

// prints sensor calibration ofsets to the serial monitor
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

// // read cal data from eeprom, force == true to make system recalibrate
// void readFlashcal(Adafruit_BNO055 bno, bool force = false){
//   int eeAddress = 0;
//   long bnoID;
//   bool foundCalib = false;

//   uint8_t calibrationData;
//   sensor_t sensor;

//   // Init external flash
//   flash.begin();
//   // Init file system on the flash
//   fatfs.begin(&flash);

//   // re-open the file for reading:
//   CalFile = fatfs.open("ID");
//   if (myFile) {
//     char str[] = myFile.read();
//     bnoID = atoi(str);     // convert this part to an integer
//     // close the file:
//     myFile.close();
//   } else {
//     // if the file didn't open, print an error:
//     Serial.println("error opening cal");
//     foundCalib = false;
//   }
  
//     /*
//     *  Look for the sensor's unique ID at the beginning oF EEPROM.
//     *  This isn't foolproof, but it's better than nothing.
//     */
//     bno.getSensor(&sensor); // calibration data was not found
//     if (bnoID != sensor.sensor_id || force == true){
//         Serial.println("\nNo Calibration Data for this sensor exists in flash");
//         delay(500);
//     }
//     else{ // calibration data was found in eeprom
//         Serial.println("\nFound Calibration for this sensor in EEPROM.");

//         // open the file for reading:
//         CalFile = fatfs.open("Cal");
//         if (myFile) {
//           char str[] = myFile.read();
//           calibrationData = atoi(str);     // convert this part to an integer
//           // close the file:
//           myFile.close();
//           foundCalib = true;
//         } else {
//           // if the file didn't open, print an error:
//           Serial.println("error opening cal");
//           foundCalib = false;
//         }

//         displaySensorOffsets(calibrationData);

//         Serial.println("\n\nRestoring Calibration data to the BNO055...");
//         bno.setSensorOffsets(calibrationData);

//         Serial.println("\n\nCalibration data loaded into BNO055");
//         foundCalib = true;

//         // if force flag was set in arguments force the system to update cal
//         if (force == true){
//           foundCalib == false;
//         }
//     }
//     sensors_event_t event;
//     bno.getEvent(&event);
//     /* always recal the mag as It goes out of calibration very often */
//     if (foundCalib && force == false){
//         Serial.println("Move sensor slightly to calibrate magnetometers");
//         bool status = 0;
//         while (!bno.isFullyCalibrated()) {
//           // flash LEDS while magnetometer needs cal
//           //digitalWrite(GCalPin, status); digitalWrite(ACalPin, status); digitalWrite(MCalPin, status);
//           status = !status;
//           bno.getEvent(&event);
//           delay(BNO055_SAMPLERATE_DELAY_MS);
//         }
//     }
//     else{
//         Serial.println("Please Calibrate Sensor: ");
//         delay(2000);
//         //while (!bno.isFullyCalibrated())  
//         while(digitalRead(buttonPin)==HIGH)
//         {
//             bno.getEvent(&event);

//             Serial.print("X: ");
//             Serial.print(event.orientation.x, 4);
//             Serial.print("\tY: ");
//             Serial.print(event.orientation.y, 4);
//             Serial.print("\tZ: ");
//             Serial.print(event.orientation.z, 4);

//             /* Optional: Display calibration status */
//             displayCalStatus(bno);

//             /* New line for the next sample */
//             Serial.println("");

//             /* Wait the specified delay before requesting new data */
//             delay(BNO055_SAMPLERATE_DELAY_MS);
//         }
//     }

//     Serial.println("\nFully calibrated!");
//     Serial.println("--------------------------------");
//     Serial.println("Calibration Results: ");
//     adafruit_bno055_offsets_t newCalib;
//     bno.getSensorOffsets(newCalib);
//     displaySensorOffsets(newCalib);

//     Serial.println("\n\nStoring calibration data to flash...");

//     bno.getSensor(&sensor);

//     // write BNO id number to file ID
//     myFile = fatfs.open("ID", FILE_WRITE);
//     // if the file opened okay, write to it:
//     if (myFile) {
//       myFile.print(sensor);
//       // close the file:
//       myFile.close();
//     } else {
//       // if the file didn't open, print an error:
//       Serial.println("error opening ID");
//     }

//     // write calibration data to file Cal
//     myFile = fatfs.open("Cal", FILE_WRITE);
//     // if the file opened okay, write to it:
//     if (myFile) {
//       myFile.print(newCalib);
//       // close the file:
//       myFile.close();
//     } else {
//       // if the file didn't open, print an error:
//       Serial.println("error opening ID");
//     }

//     Serial.println("Data stored to EEPROM.");

//     Serial.println("\n--------------------------------\n");
//     logSensorOffsets(newCalib);
//     delay(500);
// }
// // read cal data from eeprom, force == true to make system recalibrate
// void readEEPROMcal(Adafruit_BNO055 bno, bool force = false){
//   int eeAddress = 0;
//   long bnoID;
//   bool foundCalib = false;
//   // read from eeprom to look for id
//   EEPROM.get(eeAddress, bnoID);

//   adafruit_bno055_offsets_t calibrationData;
//   sensor_t sensor;
  
//     /*
//     *  Look for the sensor's unique ID at the beginning oF EEPROM.
//     *  This isn't foolproof, but it's better than nothing.
//     */
//     bno.getSensor(&sensor); // calibration data was not found
//     if (bnoID != sensor.sensor_id || force == true){
//         Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
//         delay(500);
//     }
//     else{ // calibration data was found in eeprom
//         Serial.println("\nFound Calibration for this sensor in EEPROM.");
//         eeAddress += sizeof(long);
//         EEPROM.get(eeAddress, calibrationData);

//         displaySensorOffsets(calibrationData);

//         Serial.println("\n\nRestoring Calibration data to the BNO055...");
//         bno.setSensorOffsets(calibrationData);

//         Serial.println("\n\nCalibration data loaded into BNO055");
//         foundCalib = true;

//         // if force flag was set in arguments force the system to update cal
//         if (force == true){
//           foundCalib == false;
//         }
//     }
//     sensors_event_t event;
//     bno.getEvent(&event);
//     /* always recal the mag as It goes out of calibration very often */
//     if (foundCalib && force == false){
//         Serial.println("Move sensor slightly to calibrate magnetometers");
//         bool status = 0;
//         while (!bno.isFullyCalibrated()) {
//           // flash LEDS while magnetometer needs cal
//           digitalWrite(GCalPin, status); digitalWrite(ACalPin, status); digitalWrite(MCalPin, status);
//           status = !status;
//           bno.getEvent(&event);
//           delay(BNO055_SAMPLERATE_DELAY_MS);
//         }
//     }
//     else{
//         Serial.println("Please Calibrate Sensor: ");
//         delay(2000);
//         //while (!bno.isFullyCalibrated())  
//         while(digitalRead(buttonPin)==HIGH)
//         {
//             bno.getEvent(&event);

//             Serial.print("X: ");
//             Serial.print(event.orientation.x, 4);
//             Serial.print("\tY: ");
//             Serial.print(event.orientation.y, 4);
//             Serial.print("\tZ: ");
//             Serial.print(event.orientation.z, 4);

//             /* Optional: Display calibration status */
//             displayCalStatus(bno);

//             /* New line for the next sample */
//             Serial.println("");

//             /* Wait the specified delay before requesting new data */
//             delay(BNO055_SAMPLERATE_DELAY_MS);
//         }
//     }

//     Serial.println("\nFully calibrated!");
//     Serial.println("--------------------------------");
//     Serial.println("Calibration Results: ");
//     adafruit_bno055_offsets_t newCalib;
//     bno.getSensorOffsets(newCalib);
//     displaySensorOffsets(newCalib);

//     Serial.println("\n\nStoring calibration data to EEPROM...");

//     eeAddress = 0;
//     bno.getSensor(&sensor);
//     bnoID = sensor.sensor_id;

//     EEPROM.put(eeAddress, bnoID);

//     eeAddress += sizeof(long);
//     EEPROM.put(eeAddress, newCalib);
//     Serial.println("Data stored to EEPROM.");

//     Serial.println("\n--------------------------------\n");
//     logSensorOffsets(newCalib);
//     delay(500);
// }

// void wipeEEPROM(){
//   EEPROM.put(0, long(0));
//   Serial.println("EEPROM Cal Erased");
// }

// bool wipeFlash(){
//   // Initialize flash library and check its chip ID.
//   if (!flash.begin()) {
//     Serial.println("Error, failed to initialize flash chip!");
//     return(false);
//   }
//   if (!flash.eraseChip()) {
//     Serial.println("Failed to erase chip!");
//     return(true);
//   }

//   flash.waitUntilReady();
//   Serial.println("Successfully erased chip!");
//   return(true);
// }