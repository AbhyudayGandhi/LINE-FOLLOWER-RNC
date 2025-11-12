// PID Constants (unchanged)
#include <MPU6050_light.h>
#include <Wire.h>

MPU6050 mpu(Wire);

// PID Constants
const float Kp = 2.20;
const float Kd = 2.52;
const int Ki = 0.7;
const int setPoint = 35;
const int lowSpeed = 0;
const int baseSpeed = 100;
const int maxSpeed = 255;
int integral = 0;
int end_count = 0;
int count = 0;

// Pin definitions
const byte rx = 0;
const byte tx = 1;
const byte serialEn = 17;
const byte junctionPulse = 4;
const byte dir1 = 9;
const byte dir2 = 8;
const byte dir3 = 7;
const byte dir4 = 5;
const byte pwm1 = 10;
const byte pwm2 = 6;
const byte led = 11;
const byte toggleSwitch = 12;

// Turn control parameters
float initialYaw = 0;
float targetAngle = 80;
bool hasTurned = false;
const int TURN_SPEED = 60;
const int FORWARD_SPEED = 150;
const int TURN_TIMEOUT = 810;
const float ANGLE_TOLERANCE = 2.0;
volatile bool rightPrio = true;

// Path control
String rawPath = "";       // Store the raw path from dry run
String optimizedPath = ""; // Store the optimized path for actual run
String tempPath = "";
bool dryRunMode = true;    // Toggle between dry run and actual run
bool executingTurn = false;

// Interrupt variables
volatile bool interruptFlag = false; // Flag to track interrupt
volatile bool dryRunDone = false;
volatile unsigned long lastInterruptTime = 0; // To track debounce timing

volatile bool rerunFlag = false;           // Flag to track rerun interrupt
volatile unsigned long lastRerunTime = 0;  // To track debounce timing for pin 3

// New interrupt handler for pin 3
void handleRerunInterrupt()
{
    unsigned long currentTime = millis(); // Get the current time in milliseconds
    if (currentTime - lastRerunTime > 200)
    {                                     // Check if 200ms have passed (debounce)
        rerunFlag = true;                 // Set rerun flag
        lastRerunTime = currentTime;      // Update last rerun time
    }
}

// Interrupt handler
void handleInterrupt()
{
    unsigned long currentTime = millis(); // Get the current time in milliseconds
    if (currentTime - lastInterruptTime > 200)
    {                                    // Check if 200ms have passed
        interruptFlag = true;            // Set flag
        lastInterruptTime = currentTime; // Update the last interrupt time
    }
}

void setup()
{
    // Pin setup
    pinMode(serialEn, OUTPUT);
    pinMode(junctionPulse, INPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);
    pinMode(dir3, OUTPUT);
    pinMode(dir4, OUTPUT);
    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(toggleSwitch, INPUT);

    digitalWrite(serialEn, HIGH);
    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, LOW);

    // Interrupt setup
    pinMode(2, INPUT_PULLUP);                                           // Set pin 2 as input with internal pull-up
    attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, RISING); // Trigger on rising edge

    pinMode(3, INPUT_PULLUP);                                            // Set pin 3 as input with internal pull-up
    attachInterrupt(digitalPinToInterrupt(3), handleRerunInterrupt, RISING); // Trigger on rising edge

    Wire.begin();
    Serial.begin(9600);

    byte status = mpu.begin();
    while (status != 0)
    {
        Serial.println("MPU6050 initialization failed!");
        delay(1000);
        status = mpu.begin();
    }

    Serial.println("MPU6050 initialized!");
    delay(1000);
    mpu.calcOffsets();
    Serial.println("Offsets calculated!");

    Serial.println("Starting in Dry Run Mode with RSL preference...");
}

void loop()
{
    // Check for interrupt to switch to actual run
    if (rerunFlag)
    {
        noInterrupts();     // Temporarily disable interrupts
        rerunFlag = false;  // Reset rerun flag
        interrupts();       // Re-enable interrupts

        Serial.println("Rerunning Actual Run Mode");

        // Rerun the actual run
        optimizedPath = tempPath;
        stopMotors();
        delay(2000);
        actual_run();
    }

    if (interruptFlag)
    {
        noInterrupts();        // Temporarily disable interrupts
        interruptFlag = false; // Reset flag
        interrupts();          // Re-enable interrupts

        // Switch to actual run mode
        dryRunMode = false;
        optimizePath(); // Optimize path before starting actual run

        Serial.println("Switching to Actual Run Mode");
        Serial.print("Optimized Path: ");
        Serial.println(optimizedPath);
    }

    // Run either dry run or actual run
    if (dryRunMode)
    {
        dry_run();
    }
    else
    {
        actual_run();
    }
}

void dry_run()
{
    int pinState = digitalRead(toggleSwitch); // Read the state of pin 12
    if (pinState == HIGH)
    {
        rightPrio = true; // Blink once if pin 12 is HIGH
    }
    else
    {
        rightPrio = false; // Blink three times if pin 12 is LOW
    }
    digitalWrite(serialEn, LOW);
    while (Serial.available() <= 0)
        ;
    int positionVal = Serial.read();
    int Jpulse = digitalRead(junctionPulse);
    digitalWrite(serialEn, HIGH);
    if (rightPrio)
    {
        if (Jpulse == 1 && !executingTurn)
        {
            count++;
            executingTurn = true;
            if (positionVal > 40)
            {
                // Right path detected
                moveStraight(250);
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0)
                    ;
                positionVal = Serial.read();
                Jpulse = digitalRead(junctionPulse);
                digitalWrite(serialEn, HIGH);
                if (Jpulse == 1)
                {
                    rawPath += 'E';
                    stopMotors();
                    optimizePath();
                    digitalWrite(led,HIGH);
                    while (!interruptFlag)
                    {
                        Serial.println(optimizedPath);
                    }
                }
                else
                {
                    performTurn(true);
                    rawPath += 'R';
                    end_count = 0;
                    Serial.println("Turning Right");
                }
            }
            else if (positionVal >= 25 && positionVal <= 40)
            {
                // Right path detected
                moveStraight(250);
                end_count = 0;
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0)
                    ;
                positionVal = Serial.read();
                Jpulse = digitalRead(junctionPulse);
                digitalWrite(serialEn, HIGH);
                if (Jpulse == 1)
                {
                    rawPath += 'E';
                    stopMotors();
                    optimizePath();
                    digitalWrite(led, HIGH);
                    while (!interruptFlag)
                    {
                        Serial.println(optimizedPath);
                    }
                }
                else
                {
                    performTurn(true);
                    rawPath += 'R';
                    end_count = 0;
                    Serial.println("Turning Right");
                }
                Serial.println("Turning Right");
            }
            else
            {
                // Check if straight path exists
                moveStraight(150); // Move forward a bit
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0)
                    ;
                int straightCheck = Serial.read();
                digitalWrite(serialEn, HIGH);

                if (straightCheck != 255)
                {
                    // Straight path exists
                    moveStraight(100); // Continue forward
                    digitalWrite(serialEn, LOW);
                    while (Serial.available() <= 0)
                        ;
                    positionVal = Serial.read();
                    Jpulse = digitalRead(junctionPulse);
                    if (Jpulse == 1)
                    {
                        moveStraight(150);
                        rawPath += 'E';
                        stopMotors();
                        digitalWrite(led,HIGH);
                        optimizePath();
                        while (!interruptFlag)
                        {
                            Serial.println(optimizedPath);
                        }
                    }
                    rawPath += 'S';
                    end_count += 1;
                    Serial.println("Going Straight");
                }
                else
                {
                    // No straight path - turn left
                    moveStraight(100);
                    digitalWrite(serialEn, LOW);
                    while (Serial.available() <= 0)
                        ;
                    positionVal = Serial.read();
                    Jpulse = digitalRead(junctionPulse);
                    digitalWrite(serialEn, HIGH);
                    if (Jpulse == 1)
                    {
                        moveStraight(150);
                        rawPath += 'E';
                        stopMotors();
                        digitalWrite(led,HIGH);
                        optimizePath();
                        while (!interruptFlag)
                        {
                            Serial.println(optimizedPath);
                        }
                    }
                    else
                    {
                        performTurn(false);
                        rawPath += 'L';
                        end_count = 0;
                        Serial.println("Turning Left");
                    }
                    // Forward before turn
                }
            }

            Serial.print("Current raw path: ");
            Serial.println(rawPath);

            // Check if we're back at start
            if (HasFinished())
            {
                Serial.println("Back at start! Optimizing path...");
                optimizePath();

                Serial.print("Optimized path: ");
                Serial.println(optimizedPath);
                delay(2000); // Pause before starting actual run
            }

            executingTurn = false;
            hasTurned = false;
            initialYaw = 0;
            // delay(500);
        }
        else if (positionVal == 255)
        {
            // No line detected - U-turn
            moveStraight(220);
            performUTurn();
            rawPath += 'B';
            end_count = 0;
            Serial.println("U-Turn - No line detected");
        }

        followLine(positionVal);
    }
    else
    {
        if (Jpulse == 1 && !executingTurn)
        {
            count++;
            executingTurn = true;
            if (positionVal < 30)
            {
                // Left path detected
                moveStraight(250);
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0);
                positionVal = Serial.read();
                Jpulse = digitalRead(junctionPulse);
                digitalWrite(serialEn, HIGH);
                if (Jpulse == 1)
                {
                    rawPath += 'E';
                    digitalWrite(led,HIGH);
                    stopMotors();
                    optimizePath();
                    while (!interruptFlag)
                    {
                        Serial.println(optimizedPath);
                    }
                }
                else
                {
                    performTurn(false);
                    rawPath += 'L';
                    end_count = 0;
                    Serial.println("Turning Left");
                }
            }
            else if (positionVal >= 30 && positionVal <= 40)
            {
                // Right path detected
                moveStraight(250);
                end_count = 0;
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0)
                    ;
                positionVal = Serial.read();
                Jpulse = digitalRead(junctionPulse);
                digitalWrite(serialEn, HIGH);
                if (Jpulse == 1)
                {
                    rawPath += 'E';
                    stopMotors();
                    optimizePath();
                    digitalWrite(led, HIGH);
                    while (!interruptFlag)
                    {
                        Serial.println(optimizedPath);
                    }
                }
                else
                {
                    performTurn(false);
                    rawPath += 'L';
                    end_count = 0;
                    Serial.println("Turning Left");
                }
                Serial.println("Turning Left");
            }
            else
            {
                // Check if straight path exists
                moveStraight(150); // Move forward a bit
                digitalWrite(serialEn, LOW);
                while (Serial.available() <= 0)
                    ;
                int straightCheck = Serial.read();
                digitalWrite(serialEn, HIGH);

                if (straightCheck != 255)
                {
                    // Straight path exists
                    moveStraight(100); // Continue forward
                    digitalWrite(serialEn, LOW);
                    while (Serial.available() <= 0)
                        ;
                    positionVal = Serial.read();
                    Jpulse = digitalRead(junctionPulse);
                    digitalWrite(serialEn, HIGH);
                    if (Jpulse == 1)
                    {
                        rawPath += 'E';
                        stopMotors();
                        optimizePath();
                        digitalWrite(led,HIGH);
                        while (!interruptFlag)
                        {
                            Serial.println(optimizedPath);
                        }
                    }
                    else
                    {
                    rawPath += 'S';
                    end_count += 1;
                    Serial.println("Going Straight");
                    }
                }
                else
                {
                    // No straight path - turn left
                    moveStraight(100);
                    digitalWrite(serialEn, LOW);
                    while (Serial.available() <= 0)
                        ;
                    positionVal = Serial.read();
                    Jpulse = digitalRead(junctionPulse);
                    digitalWrite(serialEn, HIGH);
                    if (Jpulse == 1)
                    {
                        moveStraight(150);
                        rawPath += 'E';
                        stopMotors();
                        optimizePath();
                        digitalWrite(led,HIGH);
                        while (!interruptFlag)
                        {
                            Serial.println(optimizedPath);
                        }
                    }
                    else
                    {
                        performTurn(true);
                        rawPath += 'R';
                        end_count = 0;
                        Serial.println("Turning right");
                    }
                    // Forward before turn
                }
            }

            Serial.print("Current raw path: ");
            Serial.println(rawPath);

            // Check if we're back at start
            if (HasFinished())
            {
                Serial.println("Back at start! Optimizing path...");
                optimizePath();

                Serial.print("Optimized path: ");
                Serial.println(optimizedPath);
                delay(2000); // Pause before starting actual run
            }

            executingTurn = false;
            hasTurned = false;
            initialYaw = 0;
            // delay(500);
        }
        else if (positionVal == 255)
        {
            // No line detected - U-turn
            moveStraight(220);
            performUTurn();
            rawPath += 'B';
            end_count = 0;
            Serial.println("U-Turn - No line detected");
        }

        followLine(positionVal);

    }
}

void actual_run()
{
    digitalWrite(led, LOW);
    if (optimizedPath.length() == 0)
    {
        stopMotors();
        Serial.println("Path completed!");
        return;
    }

    digitalWrite(serialEn, LOW);
    while (Serial.available() <= 0)
        ;
    int positionVal = Serial.read();
    int Jpulse = digitalRead(junctionPulse);
    digitalWrite(serialEn, HIGH);

    if (Jpulse == 1 && !executingTurn)
    {
        executingTurn = true;

        char nextMove = optimizedPath.charAt(0);
        optimizedPath.remove(0, 1);

        Serial.print("Executing move: ");
        Serial.println(nextMove);

        switch (nextMove)
        {
        case 'R':
            moveStraight(250);
            performTurn(true);
            break;
        case 'L':
            moveStraight(250);
            performTurn(false);
            break;
        case 'S':
            moveStraight(250);
            break;
        case 'E':
            moveStraight(100);
            stopMotors();
            digitalWrite(led, HIGH);
            while (1)
                ;
        }

        executingTurn = false;
        hasTurned = false;
        initialYaw = 0;
        // delay(500);
    }
    // else if (positionVal == 255) {
    //   executingTurn = true;

    //   char nextMove = optimizedPath.charAt(0);
    //   optimizedPath.remove(0, 1);

    //   Serial.print("Executing move: ");
    //   Serial.println(nextMove);

    //   switch(nextMove) {
    //     case 'B':
    //       performUTurn();
    //       break;
    //   }

    //   executingTurn = false;
    //   hasTurned = false;
    //   initialYaw = 0;
    //   delay(500);
    // }

    followLine(positionVal);
}

bool checkStraight()
{
    moveStraight(50);
    digitalWrite(serialEn, LOW);
    while (Serial.available() <= 0)
        ;
    int reading = Serial.read();
    digitalWrite(serialEn, HIGH);
    moveBackward(100);
    return (reading != 255);
}

bool checkLeft()
{
    turnLeftMotors();
    delay(100);
    digitalWrite(serialEn, LOW);
    while (Serial.available() <= 0)
        ;
    int reading = Serial.read();
    digitalWrite(serialEn, HIGH);
    turnRightMotors(); // Return to center
    delay(100);
    stopMotors();
    return (reading != 255);
}

void optimizePath()
{
    // Convert path with B (backward/U-turn) to optimal path
    String temp = rawPath;

    while (temp.indexOf("B") != -1)
    {
        while (temp.indexOf("RBL") != -1)
            temp.replace("RBL", "B");
        while (temp.indexOf("RBR") != -1)
            temp.replace("RBR", "S");
        while (temp.indexOf("RBS") != -1)
            temp.replace("RBS", "L");
        while (temp.indexOf("LBR") != -1)
            temp.replace("LBR", "B");
        while (temp.indexOf("LBL") != -1)
            temp.replace("LBL", "S");
        while (temp.indexOf("LBS") != -1)
            temp.replace("LBS", "R");
        while (temp.indexOf("SBR") != -1)
            temp.replace("SBR", "L");
        while (temp.indexOf("SBS") != -1)
            temp.replace("SBS", "B");
        while (temp.indexOf("SBL") != -1)
            temp.replace("SBL", "R");
    }

    optimizedPath = temp;
    tempPath = optimizedPath;
}

bool HasFinished()
{
    // Add your own logic to detect when robot is back at start
    // This could be based on:
    // 1. A specific marker or sensor at start
    // 2. Pattern recognition in the path
    // 3. Number of junctions encountered
    // For now, returning false to avoid premature termination
    return false; // Modify based on your maze setup
}

void performUTurn()
{

    float initialYaw = 0;
    float currentYaw, angleTurned;

    // Get the initial yaw angle from the IMU
    mpu.update();
    initialYaw = mpu.getAngleZ();

    // Turn one motor forward and one motor backward
    moveStraight(100);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, TURN_SPEED);
    analogWrite(pwm2, TURN_SPEED);

    while (true)
    {
        mpu.update();
        currentYaw = mpu.getAngleZ();

        // Calculate the angle turned
        angleTurned = currentYaw - initialYaw;

        // Check if the robot has turned 180 degrees
        if (abs(angleTurned) >= 175.0 - ANGLE_TOLERANCE)
        {
            break;
        }
    }

    // Stop the motors
    stopMotors();
}

void moveBackward(int duration)
{
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, FORWARD_SPEED);
    analogWrite(pwm2, FORWARD_SPEED);
    delay(duration);
    stopMotors();
}
void turnRightMotors()
{
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, LOW);
    analogWrite(pwm1, TURN_SPEED);
    analogWrite(pwm2, TURN_SPEED);
}

void turnLeftMotors()
{
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, LOW);
    digitalWrite(dir4, HIGH);
    analogWrite(pwm1, TURN_SPEED);
    analogWrite(pwm2, TURN_SPEED);
}

void followLine(int positionVal)
{
    static int lastError = 0;

    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, LOW);

    int error = positionVal - setPoint;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    rightMotorSpeed = constrain(rightMotorSpeed, lowSpeed, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, lowSpeed, maxSpeed);

    if (rightMotorSpeed < lowSpeed)
    {
        digitalWrite(dir1, LOW);
        digitalWrite(dir2, HIGH);
        rightMotorSpeed = -rightMotorSpeed;
    }
    if (leftMotorSpeed < lowSpeed)
    {
        digitalWrite(dir3, LOW);
        digitalWrite(dir4, HIGH);
        leftMotorSpeed = -leftMotorSpeed;
    }

    analogWrite(pwm1, rightMotorSpeed);
    analogWrite(pwm2, leftMotorSpeed);
}

void handleNoLineDetected()
{
    static int lastError = 0;
    delay(50);

    digitalWrite(serialEn, LOW);
    while (Serial.available() <= 0)
        ;
    int positionVal = Serial.read();
    digitalWrite(serialEn, HIGH);

    if (positionVal == 255)
    {
        if (lastError > 0)
        {
            turnRightMotors();
            delay(30);
        }
        else if (lastError < 0)
        {
            turnLeftMotors();
            delay(30);
        }
        else
        {
            digitalWrite(dir1, LOW);
            digitalWrite(dir2, HIGH);
            digitalWrite(dir3, LOW);
            digitalWrite(dir4, HIGH);
            analogWrite(pwm1, 150);
            analogWrite(pwm2, 150);
            delay(50);
        }
    }
}

void moveStraight(int duration)
{
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, LOW);
    analogWrite(pwm1, FORWARD_SPEED);
    analogWrite(pwm2, FORWARD_SPEED);
    delay(duration);
    stopMotors();
}

void moveStraightActual(int duration)
{
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);
    digitalWrite(dir3, HIGH);
    digitalWrite(dir4, LOW);
    analogWrite(pwm1, FORWARD_SPEED);
    analogWrite(pwm2, FORWARD_SPEED);
    delay(50);
}

void stopMotors()
{
    digitalWrite(pwm1, LOW);
    digitalWrite(pwm2, LOW);
}
void performTurn(bool turnRight)
{
    unsigned long startTime = millis();
    mpu.update();
    initialYaw = mpu.getAngleZ();
    float currentYaw, angleTurned;

    while ((millis() - startTime) < TURN_TIMEOUT)
    {
        mpu.update();
        currentYaw = mpu.getAngleZ();

        if (turnRight)
        {
            angleTurned = currentYaw - initialYaw;
        }
        else
        {
            angleTurned = initialYaw - currentYaw;
        }

        if (abs(angleTurned) >= (targetAngle - ANGLE_TOLERANCE))
        {
            break;
        }

        if (turnRight)
        {
            turnRightMotors();
        }
        else
        {
            turnLeftMotors();
        }

        Serial.print("Angle Turned: ");
        Serial.println(angleTurned);
    }

    stopMotors();
}
