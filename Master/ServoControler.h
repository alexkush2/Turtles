#include <Servo.h>

extern byte tServopin;
extern byte bServopin;
extern byte lServopin;
extern byte rServopin;

// declare servos
Servo tServo;
Servo bServo;
Servo lServo;
Servo rServo;

void SetupServos(){
    tServo.attach(tServopin);
    bServo.attach(bServopin);
    lServo.attach(lServopin);
    rServo.attach(rServopin);
}

void SweepServos(){
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