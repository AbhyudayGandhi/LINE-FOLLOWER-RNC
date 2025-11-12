#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 mpu(Wire);

const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
const byte serialEn = 4;    // Connect UART output enable of LSA08 to pin 2
const byte junctionPulse = 3;  // Pin connected to Junction Pulse (Input)
const byte dir1 = 8;   // Motor A direction
const byte dir2 = 9;   // Motor A direction
const byte dir3 = 7;   // Motor B direction
const byte dir4 = 5;   // Motor B direction
const byte pwm1 = 10;  // Motor A PWM
const byte pwm2 = 6;   // Motor B PWM

float initialYaw = 0;   // Store initial yaw angle
float targetAngle = 80; // 90 degrees right turn
bool hasTurned = false; // Flag to check if the turn has completed

void setup() {
  // Motor pins setup
  pinMode(serialEn, OUTPUT);   // Setting serialEn as digital output pin
  pinMode(junctionPulse, INPUT); 
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(dir3, OUTPUT);
  pinMode(dir4, OUTPUT);
  pinMode(pwm1, OUTPUT);
  pinMode(pwm2, OUTPUT);

  // Set up MPU6050
  Wire.begin();
  mpu.begin();
  mpu.calcOffsets();  // Assuming the robot is stationary during calibration

  // Begin serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  int Jpulse = digitalRead(junctionPulse);  // Read the junction pulse
  Serial.println(Jpulse);  // Print pulse for debugging

  if (Jpulse == 1) {  // If a junction pulse is detected
    moveStraight(250);  // Move straight for 250ms
    turnLeftWithMPU();  // Turn right using MPU6050 sensor
  }
  delay(100);  // Add a delay to prevent rapid retriggering
}

void moveStraight(int duration) {
  digitalWrite(dir1, HIGH);  // Motor A forward
  digitalWrite(dir2, LOW);
  digitalWrite(dir3, HIGH);  // Motor B forward
  digitalWrite(dir4, LOW);

  analogWrite(pwm1, 150);  // Set motor speed (adjust as needed)
  analogWrite(pwm2, 150);

  delay(duration);  // Move straight for the specified duration
  stopMotors();     // Stop after moving straight
}

void turnLeftWithMPU() {
  mpu.update();  // Update MPU6050 data

  if (!hasTurned) {
    float currentYaw = mpu.getAngleZ();  // Get the yaw (Z-axis rotation)

    // If it's the start of the turn, record the initial yaw
    if (initialYaw == 0) {
      initialYaw = currentYaw;
    }

    // Calculate the angle difference to see if the robot turned 90 degrees to the left
    float angleTurned = initialYaw - currentYaw;

    // If the angle difference is less than 90 degrees, keep turning left
    while (abs(angleTurned) < targetAngle) {
      turnLeft();  // Perform the left turn
      mpu.update();  // Continuously update MPU6050 data
      currentYaw = mpu.getAngleZ();  // Update current yaw
      angleTurned = initialYaw - currentYaw;  // Calculate angle turned

      Serial.print("Yaw: ");
      Serial.println(currentYaw);
    }

    // Stop the robot after turning 90 degrees
    stopMotors();
    Serial.println("Turn Complete");
    hasTurned = true;  // Set flag to indicate the turn is done
  }
}

void turnLeft() {
  digitalWrite(dir1, HIGH);   // Motor A backward
  digitalWrite(dir2, LOW);
  digitalWrite(dir3, LOW);  // Motor B forward
  digitalWrite(dir4, HIGH);

  analogWrite(pwm1, 60);    // Adjust motor speed as needed
  analogWrite(pwm2, 60);
}

void stopMotors() {
  analogWrite(pwm1, 0);
  analogWrite(pwm2, 0);
}

