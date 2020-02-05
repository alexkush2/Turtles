#include <Servo.h>
#include <AutoPID.h>

extern byte tServoPin;
extern byte bServoPin;
extern byte lServoPin;
extern byte rServoPin;
extern byte motorPin;

#define maxServoSwing 30
#define minServoSwing -30

float Kp = 1;
float Ki = 1;
float Kd = 0;

// declare servos
Servo tServo;
Servo bServo;
Servo lServo;
Servo rServo;

Servo Throttle;

extern byte HorizContPin;
extern byte VertContPin;
extern byte ThrottleContPin;


static double in[3], out[3];
double setPoint[] = {0,0,0};

AutoPID XPID(&in[0], &setPoint[0], &out[0], minServoSwing, maxServoSwing, Kp,Ki,Kd);
AutoPID YPID(&in[1], &setPoint[1], &out[1], minServoSwing, maxServoSwing, Kp,Ki,Kd);
AutoPID ZPID(&in[2], &setPoint[2], &out[2], minServoSwing, maxServoSwing, Kp,Ki,Kd);

// initialize servo objects
void SetupServos(){
    tServo.attach(tServoPin);
    bServo.attach(bServoPin);
    lServo.attach(lServoPin);
    rServo.attach(rServoPin);
    Throttle.attach(motorPin);
}

// manualy set servos
void SetServos(int t=90, int b=90,int l=90,int r=90){
    // set servos to 0 position
    tServo.write(t);
    bServo.write(b);
    lServo.write(l);
    rServo.write(r);
}

void SweepServos(){
    // set servos to 0 position
    SetServos(90-minServoSwing, 90-minServoSwing, 90-minServoSwing, 90-minServoSwing);
    delay(50);

    // sweep to 180 degrees
    int i=90-minServoSwing;
    while(i<=maxServoSwing){
        SetServos(i,i,i,i);
        i++;
        delay(10);
    }
    //  return servos to rest
    delay(50);
    SetServos(90,90,90,90);
}


// void ServoPIDSetup(){
//     static double in[3], out[3], setPoint[3];
//     setPoint[] = [90,90,0];
//     // static AutoPID XPID(&in[0], &setPoint[0], &out[0], minServoSwing, maxServoSwing, Kp,Ki,Kd);
//     // static AutoPID YPID(&in[1], &setPoint[1], &out[1], minServoSwing, maxServoSwing, Kp,Ki,Kd);
//     static AutoPID ZPID(&in[2], &setPoint[2], &out[2], minServoSwing, maxServoSwing, Kp,Ki,Kd);
// }

// write Servo updates from PID calculations
int* SetServosPID(double out[]){
    int top = 90-int(out[2])+int(out[1]); // im making this all up as i go
    int bot = 90+int(out[2])+int(out[1]); // no idea, seriously probably all wrong
    int left = 90 + int(out[0]); // controls is really fucking hard
    int right = 90-int(out[0]);
    SetServos(top,bot,left,right);
    int pos[4] = {top,bot,left,right};
    return pos;
}

// run PID calculations
int* ServoPIDUpdate(double input[3]){    
    in[0] = input[0] + map(analogRead(HorizContPin),0,1023,minServoSwing,maxServoSwing); // x
    in[1] = input[1] + map(analogRead(VertContPin),0,1023,minServoSwing,maxServoSwing); // y
    in[2] = input[2]; // z
    XPID.run();
    YPID.run();
    ZPID.run();
    return SetServosPID(out);
}

// reset currently calcluated PID parameters
void resetPID(){
    XPID.reset();
    YPID.reset();
    ZPID.reset();
}

// stop PID controllers
void stopPID(){
    XPID.stop();
    YPID.stop();
    ZPID.stop();
}

// proportional control of servos
int* ServoPropControl(){
    int top = 90+map(analogRead(HorizContPin),0,1023,minServoSwing,maxServoSwing); 
    int bot = 90-map(analogRead(HorizContPin),0,1023,minServoSwing,maxServoSwing); 
    int left = 90+map(analogRead(VertContPin),0,1023,minServoSwing,maxServoSwing); 
    int right = 90-map(analogRead(VertContPin),0,1023,minServoSwing,maxServoSwing);
    SetServos(top,bot,left,right);
    int pos[4] = {top,bot,left,right};
    return pos;
}