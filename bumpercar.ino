/*
 * Library Example for L298N Module to control DC motors
 * 
 * This code is to control single motor. For two motor control, please open L298N_DC_2_Motors
 * this code is ready for ESP32
 * Watch video instructions for this code:  https://youtu.be/2JTMqURJTwg
 * 
 * Written by Ahmad Shamshiri on Dec 24, 2019 
 * in Ajax, Ontario, Canada. www.robojax.com
 * 
  Need wiring diagram from this code: Purchase My course on Udemy.com http://robojax.com/L/?id=62
 * 
  * 
 * Get this code and other Arduino codes from Robojax.com
Learn Arduino step by step in structured course with all material, wiring diagram and library
all in once place. Purchase My course on Udemy.com http://robojax.com/L/?id=62

If you found this tutorial helpful, please support me so I can continue creating 
content like this. You can support me on Patreon http://robojax.com/L/?id=63

or make donation using PayPal http://robojax.com/L/?id=64

 *  * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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


//const int rightB = 17;
//const int rightF = 14;
//const int leftB = 12;
//const int leftF = 13;

// Joystick pins (digital)

const int joystickLeftUpPin = GPIO_NUM_13;
const int joystickLeftDownPin = GPIO_NUM_12;
const int joystickRightUpPin = GPIO_NUM_14;
const int joystickRightDownPin = GPIO_NUM_27;



const int rightB = GPIO_NUM_27;
const int rightF = GPIO_NUM_14;
const int leftB = GPIO_NUM_12;
const int leftF = GPIO_NUM_13;


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

  delay(50); // Small delay for stability
}

// Control the left motor based on joystick input
void controlLeftMotor(bool up, bool down) {
  const int motorSpeed = 200;
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
  const int motorSpeed = 200;

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

//
//
//
//
//
//
//  int rightBState = digitalRead(rightB); // Read the state of the button
//  
//  // LOW means the button is pressed (connected to ground)
//  if (rightBState == LOW) {
//    Serial.println("rightBState is pressed!");
//    robot.rotate(motor2, 100, CCW);
//  } else {
//    robot.brake(motor2);
//    Serial.println("rightBState is NOT pressed!");
//  }
//
//  int rightFState = digitalRead(rightF); // Read the state of the button
//
//  // LOW means the button is pressed (connected to ground)
//  if (rightFState == LOW) {
//    Serial.println("rightFState is pressed!");
//    robot.rotate(motor2, 100, CCW);
//  } else {
//    robot.brake(motor2);
//    Serial.println("rightFState is NOT pressed!");
//  }


//  int leftFState = digitalRead(leftF); // Read the state of the button
//
//  // LOW means the button is pressed (connected to ground)
//  if (leftFState == LOW) {
//    Serial.println("Button is pressed!");
//    robot.rotate(motor2, 100, CW);//run motor1 at 60% speed in CW direction
//  } else {
//    robot.brake(motor2);
//    Serial.println("leftFState is not pressed!");
//  }
//
//  int leftBState = digitalRead(leftB); // Read the state of the button
//
//  // LOW means the button is pressed (connected to ground)
//  if (leftBState == LOW) {
//    Serial.println("Button is pressed!");
//    robot.rotate(motor2, 100, CCW);//run motor1 at 60% speed in CW direction
//  } else {
//    robot.brake(motor2);
//    Serial.println("leftBState is not pressed!");
//  }
//
//  delay(100); // Small delay for stability


  
//  robot.rotate(motor1, 100, CW);//run motor1 at 60% speed in CW direction
//  robot.rotate(motor2, 100, CCW);//run motor1 at 60% speed in CW direction
//  
//  delay(2000);
//
//  robot.brake(motor1);
//  robot.brake(motor2); 
//  delay(2000);









//
//  robot.rotate(motor1, 100, CW);//run motor1 at 60% speed in CW direction
//  delay(3000);
//  
//  robot.rotate(motor2, 100, CCW);//run motor1 at 60% speed in CW direction
//  
//  robot.brake(1);
//  robot.brake(2);   
//  delay(2000);  
//
//  for(int i=0; i<=100; i++)
//  {
//    robot.rotate(motor1, i, CW);// turn motor1 with i% speed in CW direction (whatever is i) 
//    delay(100);
//  }
//  delay(2000);
//  
//  robot.brake(1);
//  delay(2000);  
//  
//  for(int i=0; i<=100; i++)
//  {
//    robot.rotate(motor2, i, CW);// turn motor1 with i% speed in CW direction (whatever is i) 
//    delay(100);
//  }
//  delay(2000);
//  
//  robot.brake(2);
//  delay(2000);    
//  // Robojax L298N Library. Watch video instruciton https://youtu.be/2JTMqURJTwg

//}
