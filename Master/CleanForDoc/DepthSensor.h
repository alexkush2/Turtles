#include <Wire.h>
#include "MS5837.h"
#include <Adafruit_NeoPixel.h>

#define ContGreenLED A7 
#define ContYellowLED A6

// load external pins and objects
extern Adafruit_NeoPixel strip;

#define buttonPin A11

/* Dont forget to add

MS5837 depthSensor;

in global declarations to initialize the sensor object

*/

// flash GCalPin 100ms at a time as an error
void errDepthDelay1sec(){
    for(int i; i<5; i++){
        delay(100);
        strip.setPixelColor(0, 0, 0, 255);
        strip.show(); // set pixel blue
        digitalWrite(ContGreenLED, HIGH);
		digitalWrite(ContYellowLED, HIGH);
        delay(100);
        strip.setPixelColor(0, 255, 0, 0);
        strip.show(); // set pixel red 
        digitalWrite(ContGreenLED, LOW);
		digitalWrite(ContYellowLED, LOW);
    }
}

// connects to sensor and sets up basic parameters
bool initDepthSensor(MS5837 depthSensor){
    Wire.begin();

    // Initialize pressure sensor
    // Returns true if initialization was successful
    // will wait 5 sec for sensor to start or button is pressed
    byte i = 0;
    while (!depthSensor.init()) {
        i++;
        Serial.println("Depth sensor init failed!");
        Serial.println("Are SDA/SCL connected correctly?");
        Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
        Serial.println("\n\n\n");
        // flash GCalPin LED
        errDepthDelay1sec();
        if(i == 5) return false;
        if(digitalRead(buttonPin)==LOW) return false;
    }
    
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)
    return true;
}

