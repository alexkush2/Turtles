#include <Servo.h>
#include <AutoPID.h>

// max deflection from the 90 degree set point of fins
#define maxServoSwing 30
#define minServoSwing -30

// int ContGreenLED = A7;
// int ContYellowLED = A6;
#define ContGreenLED A7 
#define ContYellowLED A6

// Servo pinout
// O_R_B
// | | |
// S + -
byte tServoPin = 27;
byte bServoPin = 29;
byte lServoPin = 31;
byte rServoPin = 33;
byte motorPin = 35;

// // control pins
// byte HorizContPin = A12;
// byte VertContPin = A9;
// byte ThrottleContPin = A10;

// control pins
#define HorizContPin A12
#define VertContPin A9
#define ThrottleContPin A10

#define buttonPin A11

// declare servos
Servo tServo;
Servo bServo;
Servo lServo;
Servo rServo;
// servo to control motor throttle
Servo Throttle;


// manually set servos
void SetServos(int t=90, int b=90,int l=90,int r=90){
	// set servos to 0 position
	tServo.write(t);
	bServo.write(b);
	lServo.write(l);
	rServo.write(r);
}



void setup() {

	//================================================================
	// pin setup
	pinMode(buttonPin, INPUT_PULLUP); // button, switches normally high

	pinMode(HorizContPin, INPUT);
	pinMode(VertContPin, INPUT);
	pinMode(ThrottleContPin, INPUT);


	pinMode(ContGreenLED, OUTPUT);
	pinMode(ContYellowLED, OUTPUT);

	tServo.attach(tServoPin);
	bServo.attach(bServoPin);
	lServo.attach(lServoPin);
	rServo.attach(rServoPin);
	Throttle.attach(motorPin,1000,2000);

    digitalWrite(ContGreenLED, HIGH);
}

void loop(){
    // read inputs and map to range of min/max swings, then set servos
	int top = 90+map(analogRead(HorizContPin),0,1023,minServoSwing,maxServoSwing); 
	int bot = 90-map(analogRead(HorizContPin),0,1023,minServoSwing,maxServoSwing); 
	int left = 90+map(analogRead(VertContPin),0,1023,minServoSwing,maxServoSwing); 
	int right = 90-map(analogRead(VertContPin),0,1023,minServoSwing,maxServoSwing);
	SetServos(top,bot,left,right);

	int throt = map(analogRead(ThrottleContPin),0,1023,180,0);
	Throttle.write(throt);

    delay(100);
}