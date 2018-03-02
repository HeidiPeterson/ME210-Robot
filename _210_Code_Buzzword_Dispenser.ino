#include <Servo.h>

Servo gateServo;  // create servo object to control a servo
IntervalTimer gateTimer;

const int gatePin = 10;
const int openBack = 15;
const int openFront = 175;
const int closed = 90;
const int oneBallTime = 250000;
const int twoBallTime = 500000;
int pos = closed;    // variable to store the servo position
char serialValue;  //value to store the Serial input

void setup() {
  Serial.begin(9600);  // initialize serial
  gateServo.attach(gatePin);  // attaches the servo on pin 9 to the servo object
  gateServo.write(closed); 
}

void loop() {
  if (Serial.available()) {
    serialValue = Serial.read();
    Serial.println(serialValue);
    if(serialValue == 'f') openFrontGate();
    if(serialValue == 'b') openBackGate();
  }
}

void openFrontGate() {
  gateTimer.begin(closeGate, twoBallTime);
  gateServo.write(openFront);  
}

void openBackGate() {
  gateTimer.begin(closeGate, oneBallTime);
  gateServo.write(openBack);  
}

void closeGate() {
  gateServo.write(closed);  
  gateTimer.end();
}

