#include "Wire.h"
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

SoftwareSerial HC05(3, 2);  // RX, TX

char command;

MPU6050 mpu(Wire);
unsigned long previous_time = 0;  // Timer to calculate delta time

// PID Constants (Tune these for best performance)
float Kp = 150;
float Ki = 20;
float Kd = 0.01;

float setpoint = 0;  // Desired pitch angle (robot balanced at 0°)
float previous_error = 0.0;
float integral = 0.0;

float forward_offset = 0.0; // Tilt target forward by _°, adjust for speed
int forward_offset_multiplier = 0;

// Motor Driver Pins (L298)
#define ENA 9  // Left Motor Speed (PWM)
#define IN1 5    // Left Motor Direction
#define IN2 6
#define ENB 10    // Right Motor Speed (PWM)
#define IN3 8    // Right Motor Direction
#define IN4 7

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  HC05.begin(9600);  //Set the baud rate to your Bluetooth module.
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {} // Stop if MPU6050 not connected

  mpu.setAccOffsets(0.05,-0.01,-0.08);
  mpu.setGyroOffsets(0.19, 0.94, -0.90);
  Serial.println("IMU Ready!\n");

  // Initialize Motor Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Ensure motors start OFF
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);



}


// Function to control motors with L298
void driveMotors(float PID_value) {
  int speed = abs(PID_value);  // Convert PID output to positive for PWM
  if (speed < 40)
    speed = 0;
  else
  speed = map(speed,0,255,80,255);
  
  if (PID_value > 0) {  // Move Forward
    analogWrite(ENA, speed);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    
    analogWrite(ENB, speed);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } 
  else {  // Move Backward
    analogWrite(ENA, speed);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    
    analogWrite(ENB, speed);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

void loop() {
  mpu.update();  // Read sensor values and update angles

  unsigned long current_time = millis();
  float dt = (current_time - previous_time) / 1000.0;  // Time in seconds

//   Serial.print("X: ");
// Serial.print(mpu.getAngleX());
// Serial.print("  Y: ");
// Serial.print(mpu.getAngleY());
// Serial.print("  Z: ");
// Serial.println(mpu.getAngleZ());

if (HC05.available() > 0) {
    command = HC05.read();

    // Stop(); //initialize with motors stoped

    Serial.print("Received: ");
    Serial.println(command);

    
      switch (command) {
        case 'F':
          forward_offset_multiplier = 1;
          break;
        case 'B':
          forward_offset_multiplier = -1;
          break;
        case '0':
          forward_offset = 0;
          break;
        case '1':
          forward_offset = 0.2;
          break;
        case '2':
          forward_offset = 0.5;
          break;
        case '3':
          forward_offset = 0.9;
          break;
        case '4':
          forward_offset = 1.2;
          break;
        case '5':
          forward_offset = 1.7;
          break;
        case '6':
          forward_offset = 2.0;
          break;
        case '7':
          forward_offset = 2.3;
          break;
        case '8':
          forward_offset = 2.6;
          break;
        case '9':
          forward_offset = 2.8;
          break;
        case 'q':
          forward_offset = 3.0;
          break;
        case 'S':
          forward_offset_multiplier = 0;
          break;
      }
      Serial.print('Mult: ');
      Serial.print(forward_offset_multiplier);
      Serial.print(', offs: ');
      Serial.println(forward_offset);
}

  if (dt >= 0.01) {  // Run every 10ms

    float pitch = mpu.getAngleY();  // Use Y-axis as pitch

     float target = setpoint + forward_offset_multiplier * forward_offset;

    // Calculate PID
    float error = target - pitch;
    integral += error * dt;  // dt = 10ms = 0.01s
    float derivative = (error - previous_error) / dt;
    float PID_value = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previous_error = error;

    // Limit PID output
    if (PID_value > 255) PID_value = 255;
    if (PID_value < -255) PID_value = -255;

    // Optional: Stop if bot tilts too far
    if (abs(pitch) > 45) {
      stopMotors();
    } else {
      driveMotors(PID_value);  // Control motors based on PID output
    }

    previous_time = current_time;  // Update time
  }
  delay(25);
}


// Function to stop both motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}