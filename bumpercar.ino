#include "Robojax_L298N_DC_motor.h"
#include <Arduino.h>
// motor 1 settings
#define CHA 0

#define ENA 5 // this pin must be PWM enabled pin if Arduino board is used
#define IN1 18
#define IN2 23


// motor 2 settings
#define IN3 4
#define IN4 17
#define ENB 16 // this pin must be PWM enabled pin if Arduino board is used

#define CHB 1

const int CCW = 2; // do not change
const int CW  = 1; // do not change

#define motor1 1 // do not change
#define motor2 2 // do not change



const int motorA_in1 = 18;
const int motorA_in2 = 23;
const int motorA_speed = 5; // PWM pin

// Motor B (Right) connections
const int motorB_in3 = 4;
const int motorB_in4 = 17;
const int motorB_speed = 16; // PWM pin

// for single motor
//Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, true);  

// for two motors without debug information // Watch video instruciton for this line: https://youtu.be/2JTMqURJTwg
Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA,  IN3, IN4, ENB, CHB);

// fore two motors with debut information
//Robojax_L298N_DC_motor robot(IN1, IN2, ENA, CHA, IN3, IN4, ENB, CHB, true);

// Joystick pins (digital)

const int joystickLeftUpPin = GPIO_NUM_13;
const int joystickLeftDownPin = GPIO_NUM_12;
const int joystickRightUpPin = GPIO_NUM_14;
const int joystickRightDownPin = GPIO_NUM_27;


pinMode(motorA_in1, OUTPUT);
pinMode(motorA_in2, OUTPUT);
pinMode(motorA_speed, OUTPUT); 
pinMode(motorB_in3, OUTPUT);
pinMode(motorB_in4, OUTPUT);
pinMode(motorB_speed, OUTPUT); 
  

//const int rightB = GPIO_NUM_27;
//const int rightF = GPIO_NUM_14;
//const int leftB = GPIO_NUM_12;
//const int leftF = GPIO_NUM_13;


// TTGO LORA SX1278 ESP32 0.96 OLED 128Mt bit (16MB) 433Mhz

void setup() {
  Serial.begin(115200);
  robot.begin();
  //L298N DC Motor by Robojax.com
  // Set joystick pins as inputs with internal pull-up resistors
  pinMode(joystickLeftUpPin, INPUT_PULLUP);
  pinMode(joystickLeftDownPin, INPUT_PULLUP);
  pinMode(joystickRightUpPin, INPUT_PULLUP);
  pinMode(joystickRightDownPin, INPUT_PULLUP);


  gpio_pullup_en(GPIO_NUM_27);
  gpio_pulldown_dis(GPIO_NUM_27); 

  gpio_pullup_en(GPIO_NUM_14);
  gpio_pulldown_dis(GPIO_NUM_14); 

  gpio_pullup_en(GPIO_NUM_12);
  gpio_pulldown_dis(GPIO_NUM_12); 

  gpio_pullup_en(GPIO_NUM_13);
  gpio_pulldown_dis(GPIO_NUM_13); 


  robot.brake(motor1);
  robot.brake(motor2);

}

void loop() {

  Serial.begin(115200);
  // Read joystick states (LOW when pressed because of pull-up resistors)
  bool leftUp = digitalRead(joystickLeftUpPin) == LOW;
  bool leftDown = digitalRead(joystickLeftDownPin) == LOW;
  bool rightUp = digitalRead(joystickRightUpPin) == LOW;
  bool rightDown = digitalRead(joystickRightDownPin) == LOW;

  // Control the motors
  controlLeftMotor(leftUp, leftDown);
  controlRightMotor(rightUp, rightDown);

  // Print joystick states for debugging (optional)
  Serial.print("Left Up: ");
  Serial.print(leftUp);
  Serial.print(", Left Down: ");
  Serial.print(leftDown);
  Serial.print(", Right Up: ");
  Serial.print(rightUp);
  Serial.print(", Right Down: ");
  Serial.println(rightDown);

  delay(20); // Small delay for stability
}

// Control the left motor based on joystick input
void controlLeftMotor(bool up, bool down) {
  const int motorSpeed = 255;
  if (up) {
    // Move forward
    digitalWrite(motorA_in1, HIGH);
    digitalWrite(motorA_in2, LOW);
    analogWrite(motorA_speed, motorSpeed);
  } else if (down) {
    // Move backward
    digitalWrite(motorA_in1, LOW);
    digitalWrite(motorA_in2, HIGH);
    analogWrite(motorA_speed, motorSpeed);
  } else {
    // Stop
    digitalWrite(motorA_in1, LOW);
    digitalWrite(motorA_in2, LOW);
    analogWrite(motorA_speed, 0);
  }
}

// Control the right motor based on joystick input
void controlRightMotor(bool up, bool down) {
  const int motorSpeed = 255;
  if (up) {
    // Move forward
    digitalWrite(motorB_in3, HIGH);
    digitalWrite(motorB_in4, LOW);
    analogWrite(motorB_speed, motorSpeed);
  } else if (down) {
    // Move backward
    digitalWrite(motorB_in3, LOW);
    digitalWrite(motorB_in4, HIGH);
    analogWrite(motorB_speed, motorSpeed);
  } else {
    // Stop
    digitalWrite(motorB_in3, LOW);
    digitalWrite(motorB_in4, LOW);
    analogWrite(motorB_speed, 0);
  }
}
