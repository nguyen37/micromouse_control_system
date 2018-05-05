#include <SharpDistSensor.h> //IR sensor library

void rightEncoderEvent(void);
void rightEncoderEvent(void);
//void move_forward(void);
//void move_backward(void);
void right_turn(void);
void left_turn(void);
void go_straight(void);

// pins for the encoder inputs
#define RH_ENCODER_A 2
#define RH_ENCODER_B 4
#define LH_ENCODER_A 5
#define LH_ENCODER_B 3
volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;

volatile unsigned long RH_totalcount = 0;
volatile unsigned long RH_current_count = 0;
//pins for motor controls
#define RH_MOTOR_A 11
#define RH_MOTOR_B 10
#define RH_MOTOR_EN 9
#define LH_MOTOR_A 8
#define LH_MOTOR_B 7
#define LH_MOTOR_EN 6
int SPEED = 100;     //SET SPEED OF MOTORS HERE


//IR SENSOR SETUP:
//pins for IR sensors
const byte RH_IR_pin = A0;
const byte LH_IR_pin = A1;
const byte CN_IR_pin = A2;
//Window size of the median filter (odd number, 1 = no filtering)
const byte mediumFilterWindowSize = 5; 
// Create an object instance of the SharpDistSensor class
SharpDistSensor RH_IR(RH_IR_pin, mediumFilterWindowSize);
SharpDistSensor LH_IR(LH_IR_pin, mediumFilterWindowSize);
SharpDistSensor CN_IR(CN_IR_pin, mediumFilterWindowSize);
const float polyCoefficients[] = {449.65, -5.354, 0.0279, -7E-5, 6E-8};
const byte nbCoefficients = 5;
const unsigned int minVal = 67; // ~ 200mm    67
const unsigned int maxVal = 400; // ~20mm     362
unsigned int RH_dist_val = 0;
unsigned int LH_dist_val = 0;
unsigned int CN_dist_val = 0;

////////////////////////////           FOR PID       ////////////////////////////
int ERROR_CONSTANT_P = 6; //P = 6; D = 1
int ERROR_CONSTANT_I = 0;
int ERROR_CONSTANT_D = 1;
float proportional_feedback = 0;
float integral_feedback = 0;
float derivative_feedback = 0;
float old_proportional_feedback = 0;
float error = 0;
volatile float adjustAmount = 0;
volatile double LH_current_count = 0;
volatile double desired_tick_val = 0;
int deltaMillis=0, oldMillis=0;

// variables to store the number of encoder pulses for each motor

void setup() {
  //intialize encoders
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  //initialize motors
  pinMode(RH_MOTOR_A, OUTPUT);
  pinMode(RH_MOTOR_B, OUTPUT);
  pinMode(LH_MOTOR_A, OUTPUT);
  pinMode(LH_MOTOR_B, OUTPUT);
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_B), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  RH_IR.setPolyFitCoeffs(nbCoefficients, polyCoefficients, minVal, maxVal);
  LH_IR.setPolyFitCoeffs(nbCoefficients, polyCoefficients, minVal, maxVal);
  CN_IR.setPolyFitCoeffs(nbCoefficients, polyCoefficients, minVal, maxVal);
  Serial.begin(9600); 
}

void loop() {
  //move_forward();
  get_dist();
  Serial.print("right: ");
  Serial.print(RH_dist_val);
  Serial.print("left: ");
  Serial.print(LH_dist_val);
  Serial.print("center: ");
  Serial.println(CN_dist_val);
  // Wait some time
  delay(10);
}

// encoder event for the interrupt call
void leftEncoderEvent() {
    leftCount++;
}
// encoder event for the interrupt call
void rightEncoderEvent() {
  RH_totalcount++; //begin PID for encoder ticks
}



void move_forward() {
  digitalWrite(RH_MOTOR_A, LOW);
  digitalWrite(RH_MOTOR_B, HIGH);
  digitalWrite(LH_MOTOR_A, HIGH);
  digitalWrite(LH_MOTOR_B, LOW);
  int  driveR = SPEED + adjustAmount;
  int driveL = SPEED;
  if((driveR) >= 255) {
    driveR = 255;
  }
  if((driveR) <= 5) {
    driveR = 5;
  }
  analogWrite(RH_MOTOR_EN, driveR);
  analogWrite(LH_MOTOR_EN, driveL);
  //debug:
  //Serial.print("Right Motor speed = ");
  //Serial.println(driveR);
  //Serial.print("RH motor clicks = ");
  Serial.print(RH_totalcount);
  Serial.print(" ");
  //Serial.print("Left Motor speed = ");
  //Serial.println(driveL);
  //Serial.print("   LH motor clicks = ");
  Serial.println(leftCount);
  //Serial.print("INTEGRAL: ");
  //Serial.println(integral_feedback);
  //Serial.print("Adjust Amount: ");
  //Serial.println(adjustAmount);
}

void get_dist() {
  //read IR sensor values:
  RH_dist_val = RH_IR.getDist();
  LH_dist_val = LH_IR.getDist();
  CN_dist_val = CN_IR.getDist();
}
