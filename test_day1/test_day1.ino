#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long previous_time = 0;  // Timer to calculate delta time

// PID Constants (Tune these for best performance)
float Kp = 65;
float Ki = 40;
float Kd = 0.03;

float setpoint = 0;  // Desired pitch angle (robot balanced at 0°)
float previous_error = 0.0;
float integral = 0.0;

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
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {} // Stop if MPU6050 not connected

  mpu.setAccOffsets(-0.08,-0.02,-0.09);
  mpu.setGyroOffsets(0.07, 0.09, -1.07);
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

  if (dt >= 0.01) {  // Run every 10ms

    float pitch = -mpu.getAngleX();  // Use Y-axis as pitch

    // Calculate PID
    float error = setpoint - pitch;
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