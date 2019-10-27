#include <Wire.h>
#include "MS5837.h"


/* Dont forget to add

MS5837 depthSensor;

in global declarations to initialize the sensor object

*/


// connects to sensor and sets up basic parameters
void initDepthSensor(MS5837 depthSensor){
    Wire.begin();

    // Initialize pressure sensor
    // Returns true if initialization was successful
    // We can't continue with the rest of the program unless we can initialize the sensor
    while (!depthSensor.init()) {
        Serial.println("Depth sensor init failed!");
        Serial.println("Are SDA/SCL connected correctly?");
        Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
        Serial.println("\n\n\n");
        delay(1000);
    }
    
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}