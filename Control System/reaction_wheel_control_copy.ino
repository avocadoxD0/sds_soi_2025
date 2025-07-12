/*
  Description:
  This code implements a PID-based angular velocity detumbling algorithm for a satellite
  using a reaction wheel. The MPU9250 provides angular velocity data, and a DC motor 
  is driven through a DRV8833 motor driver using PWM directly on the IN1 and IN2 pins.

  Microcontroller: Arduino Nano
  IMU Sensor: MPU9250 (I2C)
  Motor Driver: DRV8833
  Motor Type: DC Motor
  Control Mode: Angular Velocity Detumbling (Z-axis)
*/

#include <Wire.h>
#include <MPU9250_asukiaaa.h> // Install this library via Library Manager

MPU9250_asukiaaa mySensor;

// Pin definitions
const int IN1 = A1; // DRV8833 IN1 pin (PWM-capable)
const int IN2 = A2; // DRV8833 IN2 pin (PWM-capable)

// PID control variables
float Kp = 50.0;
float Ki = 0.5;
float Kd = 2.0;

float targetRate = 0.0087; // Target angular velocity (rad/s) ≈ 0.5 deg/s
float error = 0, prevError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// Timing
unsigned long lastTime = 0;
float dt = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  pinMode(IN!, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("Reaction wheel control started.");
}

void loop() {
  // Read gyro data
  mySensor.accelUpdate();
  mySensor.gyroUpdate();
  float angularRateZ = mySensor.gyroZ(); // deg/s
  float angularRateZ_rad = radians(angularRateZ); // rad/s

  // Time step calculation
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.01; // prevent div by zero
  lastTime = currentTime;

  // PID control computation
  error = angularRateZ_rad - targetRate;
  integral += error * dt;
  derivative = (error - prevError) / dt;
  pidOutput = Kp * error + Ki * integral + Kd * derivative;
  prevError = error;

  // Convert PID output to PWM signal
  int pwmValue = constrain(abs(pidOutput) * 255, 0, 255);

  if (pidOutput > 0) {
    analogWrite(IN1, pwmValue); // Spin one direction
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, pwmValue); // Spin opposite direction
  }

  // Debug output
  Serial.print("ω (rad/s): ");
  Serial.print(angularRateZ_rad, 4);
  Serial.print(" | PWM: ");
  Serial.println(pwmValue);

  delay(20); // ~50 Hz control loop
}