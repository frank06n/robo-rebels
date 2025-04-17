#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);
unsigned long previous_time = 0;  // Timer to calculate delta time

// PID Constants (Tune these for best performance)
float Kp = 65;
float Ki = 35;
float Kd = 0.03;

float setpoint = 0;  // Desired pitch angle (robot balanced at 0°)
float previous_error = 0.0;
float integral = 0.0;

float forward_offset = 0.7; // Tilt target forward by 3°, adjust for speed

// Motor Driver Pins (L298)
#define ENA 9  // Left Motor Speed (PWM)
#define IN1 5    // Left Motor Direction
#define IN2 6
#define ENB 10    // Right Motor Speed (PWM)
#define IN3 7    // Right Motor Direction
#define IN4 8

void setup()
{
    Serial.begin(9600);
    Wire.begin();
    Wire.setClock(400000);

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0)
    {
    } // Stop if MPU6050 not connected

    mpu.setAccOffsets(-0.08, -0.02, -0.09);
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

void loop()
{
    mpu.update(); // Read sensor values and update angles

    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0; // Time in seconds

    if (dt >= 0.01)
    { // Run every 10ms

        float pitch = mpu.getAngleY(); // Use Y-axis as pitch

        // 🚀 Forward motion by offsetting setpoint
        float target = setpoint;

        if (millis() / 500 % 2 == 0)
        {
            target += forward_offset;
        }

        // Calculate PID
        float error = target - pitch;
        integral += error * dt; // dt = 10ms = 0.01s
        float derivative = (error - previous_error) / dt;
        float PID_value = (Kp * error) + (Ki * integral) + (Kd * derivative);
        previous_error = error;

        // Limit PID output
        if (PID_value > 255)
            PID_value = 255;
        if (PID_value < -255)
            PID_value = -255;

        // 👇 Add rotation for 2 seconds
        int rotation = 0;

        // Optional: Stop if bot tilts too far
        if (abs(pitch) > 45)
        {
            stopMotors();
        }
        else
        {
            driveMotors(PID_value, rotation); // Control motors based on PID output
        }

        previous_time = current_time; // Update time
    }
    delay(25);
}

// Function to control motors with L298
void driveMotors(float PID_value, int rotation = 0)
{
    int baseSpeed = abs(PID_value); // Convert PID output to positive for PWM
    int leftSpeed = 0;
    int rightSpeed = 0;

    if (baseSpeed < 40)
        baseSpeed = 0;
    leftSpeed = baseSpeed + rotation;
    rightSpeed = baseSpeed - rotation;
    leftSpeed = map(leftSpeed, 0, 255, 80, 255);
    rightSpeed = map(rightSpeed, 0, 255, 80, 255);

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    if (PID_value > 0)
    { // Move Forward
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        analogWrite(ENB, rightSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }
    else
    { // Move Backward
        analogWrite(ENA, leftSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        analogWrite(ENB, rightSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
}

// Function to stop both motors
void stopMotors()
{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
