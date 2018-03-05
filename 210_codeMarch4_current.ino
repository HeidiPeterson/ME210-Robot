#include <Metro.h>
#include <Servo.h>
/*---------------Module Defines-----------------------------*/
//NEED TO FILL IN THESE VALUES
//low and high values defined for hysteresis


#define BLACK_THRESHOLD_LOW    10
#define BLACK_THRESHOLD_HIGH   95
#define GREY_THRESHOLD_LOW     110
#define GREY_THRESHOLD_HIGH    250
#define GREEN_THRESHOLD_LOW
#define GREEN_THRESHOLD_HIGH
//#define BLACK_SIDE_THRESHOLD_LOW  50
//#define BLACK_SIDE_THRESHOLD_HIGH 100
#define TURN_INTERVAL 300


//Teensy pins to connect to L293 enable and direction pins
const int enablePinLeft = 5;    //set the enable pin to motor
const int leftMotorDirection = 6;  //set the direction pin to motor
const int enablePinRight = 3;
const int rightMotorDirection = 4;

//Teensy pins to connect to OPB704WZ phototransistor outupt
const int photoA9 = A9;
const int photoA8 = A8;
const int photoA7 = A7;
const int photoA6 = A6;
const int photoA5 = A5;

//initialize variable to store phototransistor output value
int frontLineSensorLeft = 0;
int frontLineSensorRight = 0;
int backLineSensorLeft = 0;
int backLineSensorRight = 0;
int sideLineSensor = 0;

//set motor speed
double speedAdjust = 1.25;
double motorOffset = 1.45;
double turnConstant = 1.5;
int leftMotorSpeed = 100;
double greyInterval = 120000/leftMotorSpeed;
#define GREY_INTERVAL greyInterval
int rightMotorSpeed = leftMotorSpeed * motorOffset;
int motor_baseline_right = rightMotorSpeed;
int motor_baseline_left = leftMotorSpeed;



//Set up ball dispenser servo
Servo gateServo;  // create servo object to control a servo
IntervalTimer gateTimer;

const int gatePin = 10;
const int openBack = 15;
const int openFront = 175;
const int closed = 90;
const int oneBallTime = 250000;
const int twoBallTime = 500000;
int pos = closed;    // variable to store the servo position

//set up turn and grey timers
static Metro turnTimer = Metro(TURN_INTERVAL);
static Metro greyTimer = Metro(GREY_INTERVAL);

/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_REVERSE, STATE_STOP, STATE_TURN, STATE_GREY
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
int counter = 0;

void setup() {
  Serial.begin(9600);
  initializePins();
  initializeServo();
  state = STATE_MOVE_FORWARD;
  delay(250);
}

/*-----------------------Initialization Functions-----------------------------------------------*/
void initializeServo() {
  gateServo.attach(gatePin);  // attaches the servo on pin 9 to the servo object
  gateServo.write(closed);
}

void initializePins() {
  pinMode(leftMotorDirection, OUTPUT); //set direction pin as output
  pinMode(rightMotorDirection, OUTPUT); //set direction pin as output
  digitalWrite(leftMotorDirection, HIGH); //initialize direction pin 1 to HIGH
  digitalWrite(rightMotorDirection, LOW); //initialize direction pin 2 to LOW
  analogWrite(enablePinLeft,0);
  analogWrite(enablePinRight,0);
}

void readLineSensors() {
  //Teensy reads value from phototransistor in OPB704WZ
  frontLineSensorLeft = analogRead(photoA8);
  frontLineSensorRight = analogRead(photoA9);
  backLineSensorLeft = analogRead(photoA6);
  backLineSensorRight = analogRead(photoA7);
  sideLineSensor = analogRead(photoA5);
  Serial.print("FL sensor: ");
  Serial.print(frontLineSensorLeft);
  Serial.print(" FR sensor: ");
  Serial.print(frontLineSensorRight);
  Serial.print(" RL sensor: ");
  Serial.print(backLineSensorLeft);
  Serial.print(" RR sensor: ");
  Serial.print(backLineSensorRight);
  Serial.print(" S sensor: ");
  Serial.print(sideLineSensor);
  Serial.print(" R Motor: ");
  Serial.print(rightMotorSpeed);
  Serial.print(" L Motor: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" state: ");
  Serial.println(state);
}
/*-----------------------------------------------------------------------------------------------*/

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
    case STATE_GREY:
      stateGrey();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

/*-----------------------Servo Functions-----------------------------------------------*/
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
/*---------------------------------------------------------------------------------------------*/



/*-----------------------State Functions-----------------------------------------------*/

void stateForward () {
  readLineSensors();
  runMotorsForward ();
  if (blackTape(sideLineSensor)) {
    leftMotorSpeed = (leftMotorSpeed/1.5) * turnConstant;
    rightMotorSpeed = (leftMotorSpeed * motorOffset)/1.5;
    turnTimer.reset();
    state = STATE_TURN;
  }
  if (greyTape(frontLineSensorLeft) and greyTape(frontLineSensorRight)) {
    runMotorsGrey();
    greyTimer.reset();
    state = STATE_GREY;
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
  digitalWrite(leftMotorDirection, HIGH); //initialize direction pin 1 to HIGH
  digitalWrite(rightMotorDirection, HIGH); //initialize direction pin 2 to LOW
  if (blackTape(frontLineSensorRight) and timerExpired(turnTimer)){
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
    state = STATE_MOVE_FORWARD;
  }
  
}

void stateGrey() {
  Serial.println("grey");
  readLineSensors();
  runMotorsGrey();
  if (timerExpired(greyTimer)) {
    runMotorsForward();
    openBackGate();
    state = STATE_MOVE_FORWARD;
  }
}
/*---------------------------------------------------------------------------------------------*/



/*-----------------------Running Motors Functions-----------------------------------------------*/
void runMotorsForward () {
  //Run the motors Forward : Motor 1 is positive, Motor 2 is negative
  //Set direction to Forward
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, LOW);

  //Adjust speed
  adjustSpeedForward ();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
}

void runMotorsReverse() {
  //Run the motors Forward : Motor 1 is negative, Motor 2 is positive

  //set driection to Reverse
  digitalWrite(leftMotorDirection, LOW);
  digitalWrite(rightMotorDirection, HIGH);

  //Adjust speed
  adjustSpeedReverse();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
}

//Line follow with the back line sensors
void runMotorsGrey() {
  digitalWrite(leftMotorDirection, HIGH);
  digitalWrite(rightMotorDirection, LOW);

  //Adjust speed
  adjustSpeedGrey();
  analogWrite(enablePinLeft, leftMotorSpeed);
  analogWrite(enablePinRight, rightMotorSpeed);
}

void adjustSpeedForward () {
  if (blackTape(frontLineSensorLeft) || blackTape(backLineSensorRight)) {
    shiftLeft();
  }
  else if (blackTape(frontLineSensorRight) || blackTape(backLineSensorLeft)) {
    shiftRight();
  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void adjustSpeedReverse() {
  if (blackTape(backLineSensorLeft)|| blackTape(backLineSensorRight)) {
    shiftRight();
  } else if (blackTape(backLineSensorRight)|| blackTape(backLineSensorRight)) {
    shiftLeft();
  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void adjustSpeedGrey() {
if (blackTape(backLineSensorLeft)) {
    shiftRight();
    Serial.print("shift right");
  } else if (blackTape(backLineSensorRight)) {
    shiftLeft();
    Serial.print("shift left");
  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void shiftRight() {
  //  delay(40);
  //  Serial.print("  shifting right  ");
  rightMotorSpeed = max(0, rightMotorSpeed / speedAdjust);
  leftMotorSpeed = min(1023, leftMotorSpeed * speedAdjust);
}

void shiftLeft() {
  //  delay(40);
  //  Serial.print("  shifting left  ");
  leftMotorSpeed = max(0, leftMotorSpeed / speedAdjust);
  rightMotorSpeed = min(1023, rightMotorSpeed * speedAdjust);
}
/*---------------------------------------------------------------------------------------------*/



/*-----------------------------------Tape Sensing Functions-------------------------------------*/
unsigned char blackTape(int lineSensor) {
  if (lineSensor > BLACK_THRESHOLD_LOW and lineSensor < BLACK_THRESHOLD_HIGH)
    return true;
  else return false;
}

unsigned char greyTape(int lineSensor) {
  if (lineSensor > GREY_THRESHOLD_LOW and lineSensor < GREY_THRESHOLD_HIGH)
    return true;
  else return false;
}

//unsigned char blackSideTape(int lineSensor) {
//  if (lineSensor > BLACK_SIDE_THRESHOLD_LOW and lineSensor < BLACK_SIDE_THRESHOLD_HIGH)
//    return true;
//  else return false;
//}
/*---------------------------------------------------------------------------------------------*/



/*-----------------------------------Timer Functions-------------------------------------*/
unsigned char timerExpired(Metro timer) {
  return timer.check();
}
/*---------------------------------------------------------------------------------------------*/
