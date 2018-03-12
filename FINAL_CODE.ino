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
#define TURN_INTERVAL 300
#define STOP_INTERVAL 1500


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
double rightConst = 1.15;
double turnSpeedDecrease = 1.7;
double speedAdjust = 1.05;
double motorOffset = 1.65;
double turnConstant = 1.5;
float leftMotorSpeed = 650;
double greyInterval = 100;
#define GREY_INTERVAL greyInterval
float rightMotorSpeed = leftMotorSpeed * motorOffset;
float motor_baseline_right = rightMotorSpeed;
float motor_baseline_left = leftMotorSpeed;

//Set up ball dispenser servo
Servo gateServo;  // create servo object to control a servo
IntervalTimer gateTimer;

const int gatePin = 10;
const int openBack = 15;
const int openFront = 175;
const int closed = 90;
const int noBallTime = 100000;
const int oneBallTime = 280000;
const int twoBallTime = 500000;
const int threeBallTime = 1000000;
int ballTimeInterval = oneBallTime;

//set up turn and grey timers
static Metro turnTimer = Metro(TURN_INTERVAL);
static Metro stopTimer = Metro(STOP_INTERVAL);
IntervalTimer greyTimer;

/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_REVERSE, STATE_STOP, STATE_TURN
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
char compass = 'W';
int greyCounter = 0;

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
  analogWrite(enablePinLeft, 0);
  analogWrite(enablePinRight, 0);
}

void readLineSensors() {
  //Teensy reads value from phototransistor in OPB704WZ
  frontLineSensorLeft = analogRead(photoA8);
  frontLineSensorRight = analogRead(photoA9);
  backLineSensorLeft = analogRead(photoA6);
  backLineSensorRight = analogRead(photoA7);
  sideLineSensor = analogRead(photoA5);
}
/*-----------------------------------------------------------------------------------------------*/

void loop() {
//  Serial.print(state);
//  Serial.print(leftMotorSpeed);
//  Serial.println(rightMotorSpeed);
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
    //Serial.println(sideLineSensor);
      stateTurn();
      break;
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

/*-----------------------Servo Functions-----------------------------------------------*/
void openFrontGate() {
  gateTimer.begin(closeGate, ballTimeInterval);
  gateServo.write(openFront);
  greyTimer.end();
}

void openBackGate() {
  gateTimer.begin(closeGate, ballTimeInterval);
  gateServo.write(openBack);
  greyTimer.end();
}

void closeGate() {
  gateServo.write(closed);
  gateTimer.end();
  greyCounter ++;
}
/*---------------------------------------------------------------------------------------------*/



/*-----------------------State Functions-----------------------------------------------*/

void stateForward () {
  readLineSensors();
  runMotorsForward ();
  readSwitches();
  if (blackTape(sideLineSensor)) {
    if (compass == 'W') {
      compass = 'N';
      leftMotorSpeed = 573;
      rightMotorSpeed = 556;
      turnTimer.reset();
      stopTimer.reset();
      state = STATE_TURN;
    }
    else if (compass == 'N' && timerExpired(stopTimer)) {
      state = STATE_STOP;
    }
  }
  if (greyTape(frontLineSensorLeft) and greyTape(frontLineSensorRight)) {
    //Serial.print("Grey Counter: ");
    //Serial.println(greyCounter);
    if (greyCounter == 0) {
      ballTimeInterval = oneBallTime;
      greyTimer.begin(openFrontGate, GREY_INTERVAL);
      adjustSpeedGrey();
    }
    else if (greyCounter == 1) {
      ballTimeInterval = noBallTime;
      greyTimer.begin(openFrontGate, GREY_INTERVAL);
      adjustSpeedGrey();
    }
    else if (greyCounter == 2) {
      ballTimeInterval = threeBallTime;
      greyTimer.begin(openFrontGate, GREY_INTERVAL);
      adjustSpeedGrey();
    }
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
  if (blackTape(frontLineSensorRight) and timerExpired(turnTimer)) {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
    state = STATE_MOVE_FORWARD;
  }

}

/*---------------------------------------------------------------------------------------------*/

void readSwitches(){
  //Serial.println(analogRead(14));
  if(analogRead(14)<500){
    state = STATE_STOP;
  }
}

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

void adjustSpeedForward () {
  if (blackTape(frontLineSensorLeft) /*|| blackTape(backLineSensorLeft)*/) {
    shiftLeft();
  }
  else if (blackTape(frontLineSensorRight) /*|| blackTape(backLineSensorRight)*/) {
    shiftRight();

  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void adjustSpeedReverse() {
  if (blackTape(backLineSensorLeft) || blackTape(backLineSensorRight)) {
    shiftRight();
  } else if (blackTape(backLineSensorRight) || blackTape(backLineSensorRight)) {
    shiftLeft();
  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void adjustSpeedGrey() {
  if (blackTape(backLineSensorRight)) {
    shiftRight();
  } else if (blackTape(backLineSensorLeft)) {
    shiftLeft();
  }
  else {
    leftMotorSpeed = motor_baseline_left;
    rightMotorSpeed = motor_baseline_right;
  }
}

void shiftRight() {
  rightMotorSpeed = max(1, rightMotorSpeed / speedAdjust);
  leftMotorSpeed = min(motor_baseline_left * rightConst, leftMotorSpeed * speedAdjust);
}

void shiftLeft() {
  leftMotorSpeed = max(1, leftMotorSpeed / speedAdjust);
  rightMotorSpeed = min(motor_baseline_right * rightConst, rightMotorSpeed * speedAdjust);
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

/*-----------------------------------PID Control-------------------------------------*/
//void pidControl(){
//  error = frontLineSEnsorLeft - frontLineSensorRight;
//  motorSpeed = Kp * error + Kd * (error - lastError);
//}
/*---------------------------------------------------------------------------------------------*/
