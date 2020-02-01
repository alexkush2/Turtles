// test all servos by sweeping when button is pressed

#include <Servo.h>

byte buttonPin = 41;

// Servo pinout
// O_R_B
// | | |
// S + -

byte tServoPin = 6;
byte bServoPin = 5;
byte lServoPin = 4;
byte rServoPin = 3;



void setup(){
  pinMode(buttonPin, INPUT_PULLUP);

  // Open serial communications and wait for port to open:
  Serial.begin(115200);

  // attach servos
  tServo.attach(tServoPin);
  bServo.attach(bServoPin);
  lServo.attach(lServoPin);
  rServo.attach(rServoPin);

}

void loop(){
    tServo.write(90);
    bServo.write(90);
    lServo.write(90);
    rServo.write(90);

    if(!digitalRead(buttonPin)){
        // set servos to 0 position
        tServo.write(0);
        bServo.write(0);
        lServo.write(0);
        rServo.write(0);
        delay(50);

        // sweep to 180 degrees
        int i=0;
        while(i<=180){
            tServo.write(i);
            bServo.write(i);
            lServo.write(i);
            rServo.write(i);
            i++;
            delay(10);
        }
        //  return servos to rest
        delay(50);
        tServo.write(90);
        bServo.write(90);
        lServo.write(90);
        rServo.write(90);
        
      }
}