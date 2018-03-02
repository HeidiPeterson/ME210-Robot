//Lab 2 Part 4
//Drives two DC motors
//Speed of motor is set in code
//CURRENTLY WHAT THIS DOES:
//TEENSY READS VALUES FROM THE PHOTOTRANSISTOR
//RUNS MOTORS 

/*---------------Module Defines-----------------------------*/
//NEED TO FILL IN THESE VALUES
//low and high values defined for hysteresis

#define BLACK_THRESHOLD_LOW    40 //placeholder value
#define BLACK_THRESHOLD_HIGH   450  //placeholder value
#define GREY_THRESHOLD_LOW
#define GREY_THRESHOLD_HIGH
#define GREEN_THRESHOLD_LOW
#define GREEN_THRESHOLD_HIGH

//Teensy pins to connect to L293 enable and direction pins
const int enablePin1 = 5;    //set the enable pin to motor
const int directionPin1 = 6;  //set the direction pin to motor
const int enablePin2 = 3;
const int directionPin2 = 4; 

//Teensy pins to connect to OPB704WZ phototransistor outupt
const int photoA9 = A9;    
const int photoA8 = A8;
const int photoA7 = A7;
const int photoA6 = A6;
const int photoA5 = A5;

////Direction pin initializations
//int directionOutput1 = HIGH;  //initialize direction pin to HIGH
//int directionOutput2 = LOW;  

//initialize variable to store phototransistor output value
int frontLineSensorLeft = 0;
int frontLineSensorRight = 0;
int backLineSensorLeft = 0;
int backLineSensorRight = 0;
int sideLineSensor = 0;

//set motor speed
float leftMotorSpeed = 500;
float rightMotorSpeed = 500;
float speedAdjust = 1.5;

/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_WEST, STATE_MOVE_EAST, STATE_MOVE_NORTH
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
int counter = 0;

void setup() {
  Serial.begin(9600);
  initializePins();
  state = STATE_MOVE_WEST;

}

void initializePins(){
  pinMode(directionPin1, OUTPUT); //set direction pin as output
  pinMode(directionPin2, OUTPUT); //set direction pin as output
  digitalWrite(directionPin1, LOW); //initialize direction pin 1 to HIGH
  digitalWrite(directionPin2, HIGH); //initialize direction pin 2 to LOW
}

void readLineSensors(){
//Teensy reads value from phototransistor in OPB704WZ
  frontLineSensorRight = analogRead(photoA7);
  frontLineSensorLeft = analogRead(photoA5);
  backLineSensorLeft = analogRead(photoA6);
  backLineSensorRight = analogRead(photoA9);     
  sideLineSensor = analogRead(photoA5); 
  //Serial.println(frontLineSensorLeft);
 //Serial.println(frontLineSensorRight);
}

void loop() {

 switch (state) {
    case STATE_MOVE_WEST:
       delay(20);
      stateWest();
      break;
    case STATE_MOVE_EAST:
      stateEast();
      break;
//    case STATE_MOVE_NORTH:
//      stateNorth();
//      break;  
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

void stateWest(){
   readLineSensors();
   runMotorsWest();
//   if (blackTape(sideLineSensor)) state = STATE_MOVE_EAST;  
}

void stateEast(){
   runMotorsEast();
}

void runMotorsWest(){
  //Run the motors West: Motor 1 is positive, Motor 2 is negative
  adjustSpeedWest();
  analogWrite(enablePin1, leftMotorSpeed);
  digitalWrite(directionPin1, HIGH);
  analogWrite(enablePin2, rightMotorSpeed);
  digitalWrite(directionPin2, LOW);
//    Serial.println(rightMotorSpeed);
//  Serial.println(leftMotorSpeed);

}

void runMotorsEast(){
  //Run the motors West: Motor 1 is negative, Motor 2 is positive
  adjustSpeedEast();
  analogWrite(enablePin1, leftMotorSpeed);
  digitalWrite(directionPin1, HIGH);
  analogWrite(enablePin2, rightMotorSpeed);
  digitalWrite(directionPin2, LOW);
}

void adjustSpeedWest(){
//  Serial.println("debug2");
  if (blackTape(frontLineSensorLeft)) {
//    Serial.println("blackatpe");
    shiftLeft();

  } 
  else if (blackTape(frontLineSensorRight)){
    shiftRight();
  }
}

void adjustSpeedEast(){
  if (blackTape(backLineSensorLeft)) {
    shiftRight();
  } else if (blackTape(backLineSensorRight)){
    shiftLeft();
  }
}

void shiftLeft(){
  rightMotorSpeed = rightMotorSpeed*speedAdjust;
  leftMotorSpeed = leftMotorSpeed/speedAdjust;
    Serial.println("shift left");

}

void shiftRight(){
  leftMotorSpeed = leftMotorSpeed*speedAdjust;
  rightMotorSpeed = rightMotorSpeed/speedAdjust;
    Serial.println("shift right");

}

unsigned char blackTape(int lineSensor){
  if (lineSensor > BLACK_THRESHOLD_LOW and lineSensor < BLACK_THRESHOLD_HIGH)
  return true;
  else return false;
}
