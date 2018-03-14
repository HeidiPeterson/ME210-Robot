//Aidan Biggar, Ben Fearon, Heidi Peterson, and Leila Taleghani
//ME 210: Introduction to Mechatronics
//Winter 2018
//This is the final code for our robot, Hot Wheels.
//The strategy follows our final state transition diagram, which can be found
//on our project website: https://hotwh33ls.weebly.com/

#include <Metro.h>
#include <Servo.h>
/*---------------Module Defines-----------------------------*/
//hysteresis thresholds used for line sensing
#define BLACK_THRESHOLD_LOW    10
#define BLACK_THRESHOLD_HIGH   95
#define GREY_THRESHOLD_LOW     110
#define GREY_THRESHOLD_HIGH    250

#define SWITCH_THRESHOLD 500 //threshold for limit switch

#define TURN_INTERVAL 300  //interval before the robot is allowed to stop turning
//interval before the robot is allowed to come to a stop after the start of a turn
#define STOP_INTERVAL 1500 
//interval before the robot releases a buzzword after hitting a grey line
#define GREY_INTERVAL 100
//multiplier used to adjust right motor speed to compensate for friction in right wheel
#define MOTOR_OFFSET 1.65
//servo values for different postions of the gate
#define OPEN_BACK 15
#define OPEN_FRONT 175
#define CLOSED 90
//time intervals for opening servo gate
#define BUZZWORD_0 100000 
#define BUZZWORD_1 280000
#define BUZZWORD_2 500000
#define BUZZWORD_3 1000000

/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_REVERSE, STATE_STOP, STATE_TURN
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;

//Teensy pins to connect to L293 enable and direction pins
const int enablePinLeft = 5;       //left motor enable pin 
const int leftMotorDirection = 6;  //left motor direction pin 
const int enablePinRight = 3;      //right motor enable pin 
const int rightMotorDirection = 4; //right motor direction pin

//Teensy pins to connect to OPB704WZ phototransistor outupts
const int photoA9 = A9;
const int photoA8 = A8;
const int photoA7 = A7;
const int photoA6 = A6;
const int photoA5 = A5;

//initialize variables to store phototransistor output values
int frontLineSensorLeft = 0;
int frontLineSensorRight = 0;
int backLineSensorLeft = 0;
int backLineSensorRight = 0;
int sideLineSensor = 0;

//set motor speeds
const double shiftConstant = 1.15;   //Multiplier used to adjust speed when shifting left and right
const double speedAdjust = 1.05;   //multiplier used to adjust speed when shifting left and right
float leftMotorSpeed = 650;  //left motor speed
//right motor speed (higher than left to compensate for friction)
float rightMotorSpeed = leftMotorSpeed * MOTOR_OFFSET;
const float rightMotorSpeedDefault = rightMotorSpeed; //right motor speed default
const float leftMotorSpeedDefault = leftMotorSpeed; //right motor speed default
const int leftTurnSpeed = 573;
const int rightTurnSpeed = 556;

//set up buzzword dispenser servo
Servo gateServo;  // create servo object to control a servo

const int switchPin = 14; //Teensy pin used for limit switch
const int gatePin = 10; //Teensy pin used for servo
int buzzwordTimeInterval = BUZZWORD_1; //time interval for how long gate is open
char compass = 'W'; //direction robot is moving - robot starts out moving west
int greyCounter = 0; //counter tracks nummber of grey lines the robot has crossed

//timers
IntervalTimer gateTimer; //controls how long the buzzword gate is open
//controls how long before the a buzzword is dropped after hitting a grey line
IntervalTimer greyTimer;
//controls how long before the robot is allowed to stop turning
static Metro turnTimer = Metro(TURN_INTERVAL); 
//controls how long before the robot is allowed to come to a stop after the start of a turn
static Metro stopTimer = Metro(STOP_INTERVAL); 

void setup() {
  Serial.begin(9600);
  initializePins();
  initializeServo();
  state = STATE_MOVE_FORWARD;
  delay(250);
}

void loop() {
  switch (state) {
    case STATE_MOVE_FORWARD:
      stateForward ();
      break;
    case STATE_MOVE_REVERSE:
      stateReverse();
      break;
    case STATE_STOP:
      stateStop();
      break;
    case STATE_TURN:
      stateTurn();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

/*-----------------------Initialization Functions--------------------------------------------*/
void initializeServo() {
  gateServo.attach(gatePin);  //attach the servo on pin 9 to the servo object
  gateServo.write(CLOSED);    //close gate
}

void initializePins() {
  pinMode(leftMotorDirection, OUTPUT); //set direction pin as output
  pinMode(rightMotorDirection, OUTPUT); //set direction pin as output
  digitalWrite(leftMotorDirection, HIGH); //initialize direction pin 1 to HIGH
  digitalWrite(rightMotorDirection, LOW); //initialize direction pin 2 to LOW
  analogWrite(enablePinLeft, 0); //initialize enable pin 1 to 0
  analogWrite(enablePinRight, 0); //initialize enable pin 2 to 0
}

void readLineSensors() {
  //Teensy reads value from phototransistors in OPB704WZ tape sensors
  frontLineSensorLeft = analogRead(photoA8);
  frontLineSensorRight = analogRead(photoA9);
  backLineSensorLeft = analogRead(photoA6);
  backLineSensorRight = analogRead(photoA7);
  sideLineSensor = analogRead(photoA5);
}

/*-----------------------------Servo Functions-----------------------------------------------*/
//opens the front gate for a given time interval
void openFrontGate() {
  gateTimer.begin(closeGate, buzzwordTimeInterval);
  gateServo.write(OPEN_FRONT);
  greyTimer.end();
}

//opens the back gate for a given time interval
void openBackGate() {
  gateTimer.begin(closeGate, buzzwordTimeInterval);
  gateServo.write(OPEN_BACK);
  greyTimer.end();
}

//closes both gates and increases grey counter
void closeGate() {
  gateServo.write(CLOSED);
  gateTimer.end();
  greyCounter ++;
}

/*------------------------------State Functions-----------------------------------------------*/
void stateForward () {
  readLineSensors();
  runMotorsForward();
  readSwitches();
  if (blackTape(sideLineSensor)) {
    //if the robot is moving west and the side sensor hits black tape, turn robot to the right
    if (compass == 'W') {
      compass = 'N';
      leftMotorSpeed = leftTurnSpeed;
      rightMotorSpeed = rightTurnSpeed;
      turnTimer.reset();
      stopTimer.reset();
      state = STATE_TURN;
    }
    //if the robot is moving north and the side sensor hits black tape after the stop timer 
    //has expired, stop robot
    else if (compass == 'N' && timerExpired(stopTimer)) {
      state = STATE_STOP;
    }
  }
  if (greyTape(frontLineSensorLeft) and greyTape(frontLineSensorRight)) {
    //if the robot hits the first grey line, deposit one buzzword in Funding Round A
    if (greyCounter == 0) {
      buzzwordTimeInterval = BUZZWORD_1;
    }
    //if the robot hits the second grey line, deposit zero buzzwords in the patent office
    else if (greyCounter == 1) {
      buzzwordTimeInterval = BUZZWORD_0;
    }
    //if the robot hits the third grey line, deposit three buzzwords in Funding Round B
    else if (greyCounter == 2) {
      buzzwordTimeInterval = BUZZWORD_3;
    }
    greyTimer.begin(openFrontGate, GREY_INTERVAL);
    adjustSpeedGrey();
  }
}

void stateReverse() {
  readLineSensors();
  runMotorsReverse();
}

void stateStop() {
  readLineSensors();
  leftMotorSpeed = 0;
  rightMotorSpeed = 0;
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, LOW);
}

void stateTurn() {
  readLineSensors();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, HIGH);
  //If the front right tape sensor hits a black line after the turn timer expired, move forward
  if (blackTape(frontLineSensorRight) and timerExpired(turnTimer)) {
    leftMotorSpeed = leftMotorSpeedDefault;
    rightMotorSpeed = rightMotorSpeedDefault;
    state = STATE_MOVE_FORWARD;
  }

}

/*----------------------------------Limit Switch Function-----------------------------------*/

//if the limit switch is hit, stop the robot
void readSwitches(){
  if(analogRead(switchPin) < SWITCH_THRESHOLD){
    state = STATE_STOP;
  }
}

/*-------------------------------------Motor Functions--------------------------------------*/
//run the motors forward: Motor 1 is positive, Motor 2 is negative
void runMotorsForward () {
  //set direction to Forward
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, LOW);
  //adjust speed
  adjustSpeedForward ();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
}

//run the motors backward: Motor 1 is negative, Motor 2 is positive
void runMotorsReverse() {
  //set direction to reverse
  digitalWrite(leftMotorDirection, LOW);
  digitalWrite(rightMotorDirection, HIGH);
  //adjust speed
  adjustSpeedReverse();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
}

//Use the front tape sensors to line follow while moving forward
void adjustSpeedForward () {
  //if the front left tape sensor hits a black line, shift left
  if (blackTape(frontLineSensorLeft)) {
    shiftLeft();
  }
  //if the front right tape sensor hits a black line, shift right
  else if (blackTape(frontLineSensorRight)) {
    shiftRight();
  }
  //otherwise, set motors to default speeds
  else {
    leftMotorSpeed = leftMotorSpeedDefault;
    rightMotorSpeed = rightMotorSpeedDefault;
  }
}

//Use the back tape sensors to line follow while moving backward
void adjustSpeedReverse() {
  //if the back left tape sensor hits a black line, shift left
  if (blackTape(backLineSensorLeft)) {
    shiftLeft();
    //if the back right tape sensor hits a black line, shift right
  } else if (blackTape(backLineSensorRight)) {
    shiftRight();
  }
  //otherwise, set motors to default speeds
  else {
    leftMotorSpeed = leftMotorSpeedDefault;
    rightMotorSpeed = rightMotorSpeedDefault;
  }
}

//while the front tape sensors are over the grey tape, uses the back tape sensors to line 
//follow while moving forward
void adjustSpeedGrey() {
  //if the back right tape sensor hits a black line, shift left
  if (blackTape(backLineSensorRight)) {
    shiftLeft();
    //if the back left tape sensor hits a black line, shift right
  } else if (blackTape(backLineSensorLeft)) {
    shiftRight();
  }
  //otherwise, set motors to default speeds
  else {
    leftMotorSpeed = leftMotorSpeedDefault;
    rightMotorSpeed = rightMotorSpeedDefault;
  }
}

//shift right by increasing the left motor speed and decreasing the right motor speed
void shiftRight() {
  rightMotorSpeed = max(1, rightMotorSpeed / speedAdjust);
  leftMotorSpeed = min(leftMotorSpeedDefault * shiftConstant, leftMotorSpeed * speedAdjust);
}

//shift left by increasing the right motor speed and decreasing the left motor speed
void shiftLeft() {
  leftMotorSpeed = max(1, leftMotorSpeed / speedAdjust);
  rightMotorSpeed = min(rightMotorSpeedDefault * shiftConstant, rightMotorSpeed * speedAdjust);
}

/*-----------------------------------Tape Sensing Functions---------------------------------*/
//returns true if the given line sensor is over black tape
unsigned char blackTape(int lineSensor) {
  if (lineSensor > BLACK_THRESHOLD_LOW and lineSensor < BLACK_THRESHOLD_HIGH)
    return true;
  else return false;
}

//returns true if the given line sensor is over grey tape
unsigned char greyTape(int lineSensor) {
  if (lineSensor > GREY_THRESHOLD_LOW and lineSensor < GREY_THRESHOLD_HIGH)
    return true;
  else return false;
}

/*-----------------------------------Timer Functions----------------------------------------*/
//returns true if the given timer has expired
unsigned char timerExpired(Metro timer) {
  return timer.check();
}
