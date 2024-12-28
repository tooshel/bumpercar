#include <Arduino.h>

// Board this works for
// TTGO LORA SX1278 ESP32 0.96 OLED 128Mt bit (16MB) 433Mhz

// Motor A (Left) connections
const int motorA_in1 = 18;
const int motorA_in2 = 23;
const int motorA_speed = 5; // PWM pin

// Motor B (Right) connections
const int motorB_in3 = 4;
const int motorB_in4 = 17;
const int motorB_speed = 16; // PWM pin

// Joystick pins (digital)
const int joystickLeftUpPin = GPIO_NUM_13;
const int joystickLeftDownPin = GPIO_NUM_12;
const int joystickRightUpPin = GPIO_NUM_14;
const int joystickRightDownPin = GPIO_NUM_27;


void setup() {
  Serial.begin(115200);

  // Set joystick pins as inputs with internal pull-up resistors
  pinMode(joystickLeftUpPin, INPUT_PULLUP);
  pinMode(joystickLeftDownPin, INPUT_PULLUP);
  pinMode(joystickRightUpPin, INPUT_PULLUP);
  pinMode(joystickRightDownPin, INPUT_PULLUP);

  // This is some sort of voodoo from an esp32? not sure if I even need it
  gpio_pullup_en(GPIO_NUM_27);
  gpio_pulldown_dis(GPIO_NUM_27); 

  gpio_pullup_en(GPIO_NUM_14);
  gpio_pulldown_dis(GPIO_NUM_14); 

  gpio_pullup_en(GPIO_NUM_12);
  gpio_pulldown_dis(GPIO_NUM_12); 

  gpio_pullup_en(GPIO_NUM_13);
  gpio_pulldown_dis(GPIO_NUM_13); 


  pinMode(motorA_in1, OUTPUT);
  pinMode(motorA_in2, OUTPUT);
  pinMode(motorA_speed, OUTPUT); 
  pinMode(motorB_in3, OUTPUT);
  pinMode(motorB_in4, OUTPUT);
  pinMode(motorB_speed, OUTPUT); 
}

void loop() {

//  Serial.begin(115200);
  
  // Read joystick states (LOW when pressed because of pull-up resistors)
  bool leftUp = digitalRead(joystickLeftUpPin) == LOW;
  bool leftDown = digitalRead(joystickLeftDownPin) == LOW;
  bool rightUp = digitalRead(joystickRightUpPin) == LOW;
  bool rightDown = digitalRead(joystickRightDownPin) == LOW;

  // Control the motors
  controlLeftMotor(leftUp, leftDown);
  controlRightMotor(rightUp, rightDown);

  // Print joystick states for debugging (optional)
//  Serial.print("JOYSTICKS -- ");
//  Serial.print("Left Up: ");
//  Serial.print(leftUp);
//  Serial.print(", Left Down: ");
//  Serial.print(leftDown);
//  Serial.print(", Right Up: ");
//  Serial.print(rightUp);
//  Serial.print(", Right Down: ");
//  Serial.println(rightDown);

  delay(10); // Small delay for stability
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
