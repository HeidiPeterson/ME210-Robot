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
#define BLACK_THRESHOLD_HIGH   400 //placeholder value
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
//int directionOutput1 = LOW;  //initialize direction pin to HIGH
//int directionOutput2 = HIGH;  

//initialize variable to store phototransistor output value
int frontLineSensorLeft = 0;
int frontLineSensorRight = 0;
int backLineSensorLeft = 0;
int backLineSensorRight = 0;
int sideLineSensor = 0;

//set motor speed
int leftMotorSpeed = 750;
int rightMotorSpeed = 750;
int motor_baseline = 750;
double speedAdjust = 1.25;


/*---------------State Definitions--------------------------*/
typedef enum {
  STATE_MOVE_FORWARD, STATE_MOVE_REVERSE, STATE_MOVE_NORTH
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
int counter = 0;

void setup() {
  Serial.begin(9600);
  initializePins();
  state = STATE_MOVE_FORWARD;
  
}

void initializePins(){
  pinMode(directionPin1, OUTPUT); //set direction pin as output
  pinMode(directionPin2, OUTPUT); //set direction pin as output
  digitalWrite(directionPin1, HIGH); //initialize direction pin 1 to HIGH
  digitalWrite(directionPin2, LOW); //initialize direction pin 2 to LOW
}

void readLineSensors(){
//Teensy reads value from phototransistor in OPB704WZ
  frontLineSensorLeft = analogRead(photoA8);
  frontLineSensorRight = analogRead(photoA9);
  backLineSensorLeft = analogRead(photoA6);
  backLineSensorRight = analogRead(photoA7);     
  sideLineSensor = analogRead(photoA5); 
  //Serial.println(frontLineSensorLeft);
}

void loop() {
 switch (state) {
    case STATE_MOVE_FORWARD:
      stateForward ();
      break;
    case STATE_MOVE_REVERSE:
      stateReverse();
      break;
//    case STATE_MOVE_NORTH:
//      stateNorth();
//      break;  
    default:    // Should never get into an unhandled state
      Serial.println("What is this I do not even...");
  }
}

void stateForward (){
  Serial.print("right: ");
  Serial.print(rightMotorSpeed);
  Serial.print("  left: ");
  Serial.print(leftMotorSpeed);
  Serial.print("  right_sensor: ");
  Serial.print(frontLineSensorRight);
  Serial.print("  left_sensor: ");
  Serial.println(frontLineSensorLeft);
   readLineSensors();
   runMotorsForward ();
   
//   if (blackTape(sideLineSensor)) state = STATE_MOVE_REVERSE;  
}

void stateReverse(){
   runMotorsReverse();
}

void runMotorsForward (){
  //Run the motors Forward : Motor 1 is positive, Motor 2 is negative
  //Set direction to Forward 
  digitalWrite(directionPin1, HIGH);
  digitalWrite(directionPin2, LOW);

  //Adjust speed
  adjustSpeedForward ();
  analogWrite(enablePin1, leftMotorSpeed);
  analogWrite(enablePin2, rightMotorSpeed);
}

void runMotorsReverse(){
  //Run the motors Forward : Motor 1 is negative, Motor 2 is positive
  
  //set driection to Reverse
  digitalWrite(directionPin1, HIGH);
  digitalWrite(directionPin2, LOW);

  //Adjust speed
  adjustSpeedReverse();  
  analogWrite(enablePin1, leftMotorSpeed);
  analogWrite(enablePin2, rightMotorSpeed);
}

void adjustSpeedForward (){
  if (blackTape(frontLineSensorLeft)) {
    shiftLeft();
  }
  else if (blackTape(frontLineSensorRight)){
    shiftRight();
  } 
  else {
    leftMotorSpeed = motor_baseline;
    rightMotorSpeed = motor_baseline;
  }
}

void adjustSpeedReverse(){
  if (blackTape(backLineSensorLeft)) {
    shiftLeft();
  } else if (blackTape(backLineSensorRight)){
    shiftRight();
  }
}

void shiftRight(){
  delay(40);
  Serial.print("  shifting right  ");
  rightMotorSpeed = max(0,rightMotorSpeed/speedAdjust);
  leftMotorSpeed = min(1023,leftMotorSpeed*speedAdjust);
}

void shiftLeft(){
  delay(40);
  Serial.print("  shifting left  ");
  leftMotorSpeed = max(0,leftMotorSpeed/speedAdjust);
  rightMotorSpeed = min(1023,rightMotorSpeed*speedAdjust);
}

unsigned char blackTape(int lineSensor){
  if (lineSensor > BLACK_THRESHOLD_LOW and lineSensor < BLACK_THRESHOLD_HIGH)
  return true;
  else return false;
}
