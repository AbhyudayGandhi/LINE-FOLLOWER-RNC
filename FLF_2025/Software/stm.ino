#include <Arduino.h>
#include <Wire.h> // Include the I2C library
#include <string.h> // Include for C-style string functions
#include <stdlib.h> // Include for atof and atoi

// Set WEBSERVER to 1 to enable I2C communication with the ESP32
#define WEBSERVER 0

// Set USB_DEBUG to 1 to enable detailed Serial output for debugging
#define USB_DEBUG 0

// Set LINE_SENSOR_MODE to 1 for Raw Mode (calculate position from 8 bits)
#define LINE_SENSOR_MODE 1

// --- Line Analysis Algorithm Selection ---
// Use one of the following three options to choose the line analysis algorithm.
#define ALGO_MOST_CENTERED 1
#define ALGO_LARGEST       2
#define ALGO_NORMAL        3

// Set the desired algorithm here:
#define LINE_ANALYSIS_ALGORITHM ALGO_MOST_CENTERED

// --- I2C Definitions ---
#if WEBSERVER
const int I2C_SLAVE_ADDR = 0x08; // The I2C address for this STM32 device
volatile bool newCommandReceived = false; // Flag to indicate a new command has arrived
char command_buffer[32]; // Buffer to store incoming commands
#endif

// --- Pin Definitions ---
#define MOTOR_A_PWM PA1
#define MOTOR_A_IN1 PB8
#define MOTOR_A_IN2 PB9
#define MOTOR_B_PWM PA6
#define MOTOR_B_IN1 PA5
#define MOTOR_B_IN2 PA4
#define UEN_PIN PA15
#define STBY_PIN PB12
#define JUNCTION_PULSE PB4

// --- Serial Baud Rates ---
const int SENSOR_BAUD_RATE = 230400;
const int DEBUG_BAUD_RATE = 115200;

// --- PID and Motor Control Variables ---
float Kp = 5.150;
float Kd = 4.550;
int SLOW_SPEED = 130;
int baseSpeed = 180;
int maxSpeed = 255;
int setPoint = 35;
int minSpeed = -255;
int junctionSpeed = 120;

int lastError = 0;
int error = 0;

unsigned long previousTime = 0;

#if WEBSERVER
void receiveEvent(int howMany);
void processCommand(char* command);
#endif

// --- State variables for line tracking and memory ---
const int LAST_END_LEFT = 0;
const int LAST_END_RIGHT = 1;
const int LAST_END_UNKNOWN = 2;
int last_end = LAST_END_UNKNOWN;
const int LAST_END_UPDATE_THRESHOLD = 30; // NOT USED

// --- Timeout for last_end memory ---
unsigned long last_end_update_time = 0;
unsigned int LAST_END_TIMEOUT_MS = 110; 

// --- Grace period for handling dashed lines ---
unsigned long line_lost_time = 0;
unsigned int DASHED_LINE_GRACE_PERIOD_MS = 180;

// --- Turn Speeds for Line Lost ---
int LINE_LOST_TURN_SPEED = 150;
int LINE_LOST_PIVOT_SPEED = -150;

enum LineFeature {
  NORMAL_LINE,
  JUNCTION,
  LINE_LOST
};

struct LineAnalysisResult {
  int position;
  LineFeature featureType;
  bool isComplex;
};

void stopmotors();
void pidTurn(int error, bool isComplex);
void motorspeed(int rightmotorspeed, int leftmotorspeed);
void handleLineLost();
LineAnalysisResult analyzeSensorData(uint8_t rawSensorByte);

void setup() {
    Serial.begin(DEBUG_BAUD_RATE);
    delay(1500);
    Serial.println("Initializing USB Debug Serial...");

#if WEBSERVER
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent); // Register receive event ISR
    Serial.print("Initialized I2C Slave on address 0x");
    Serial.println(I2C_SLAVE_ADDR, HEX);
    Serial.println("I2C Pins: PB7 (SDA), PB6 (SCL)");
#endif

    Serial1.begin(SENSOR_BAUD_RATE);
    Serial.println("Initializing Hardware Serial1 for Line Sensor...");

    Serial.println("Initializing GPIO Pins...");
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);
    pinMode(UEN_PIN, OUTPUT);
    digitalWrite(UEN_PIN, LOW);
    pinMode(JUNCTION_PULSE, INPUT);

    Serial.println("Setup Complete.");
    previousTime = micros();
}

LineAnalysisResult analyzeSensorData(uint8_t rawSensorByte) {
    LineAnalysisResult result;
    result.position = 255;
    result.isComplex = false;

    // Handle junction (all sensors on) and line lost (all sensors off) cases first,
    // as they are common to all algorithms.
    if (rawSensorByte == 0xFF) {
        result.featureType = JUNCTION;
        result.position = 35; // Center position
        return result;
    }
    if (rawSensorByte == 0x00) {
        result.featureType = LINE_LOST;
        return result;
    }

    int weights[8] = {0, 10, 20, 30, 40, 50, 60, 70};

#if LINE_ANALYSIS_ALGORITHM == ALGO_MOST_CENTERED
    // --- ALGORITHM 1: MOST CENTERED BLOCK ---
    // This algorithm finds all contiguous blocks of sensors and follows the one
    // closest to the center. Good for ignoring noise or splits at the edge of the sensor array.
    uint8_t sensors[8];
    for (int i = 0; i < 8; i++) {
        sensors[i] = (rawSensorByte >> i) & 0x01;
    }
    struct Block { int start; int len; };
    Block blocks[8];
    int block_count = 0;

    for (int i = 0; i < 8; ) {
        if (sensors[i] == 1) {
            blocks[block_count].start = i;
            blocks[block_count].len = 0;
            while (i < 8 && sensors[i] == 1) {
                blocks[block_count].len++;
                i++;
            }
            block_count++;
        } else {
            i++;
        }
    }
    
    if (block_count > 1) {
        result.isComplex = true;
    }

    if (block_count == 0) { // Should not happen due to initial check, but for safety
        result.featureType = LINE_LOST;
        return result;
    }

    int most_centered_block_idx = 0;
    if (block_count > 1) {
        float min_dist_to_center = 100.0;
        for (int i = 0; i < block_count; i++) {
            float block_center = (float)blocks[i].start + ((float)blocks[i].len - 1.0) / 2.0;
            float dist_to_center = abs(block_center - 3.5);
            if (dist_to_center < min_dist_to_center) {
                min_dist_to_center = dist_to_center;
                most_centered_block_idx = i;
            }
        }
    }

    int weighted_sum = 0;
    int sum_of_readings = 0;
    for (int i = blocks[most_centered_block_idx].start; i < blocks[most_centered_block_idx].start + blocks[most_centered_block_idx].len; i++) {
        weighted_sum += weights[i];
        sum_of_readings++;
    }
    if (sum_of_readings > 0) {
        result.position = weighted_sum / sum_of_readings;
    }

#elif LINE_ANALYSIS_ALGORITHM == ALGO_LARGEST
    // --- ALGORITHM 2: LARGEST BLOCK ---
    // This algorithm finds all contiguous blocks of sensors and follows the largest one.
    // Useful for tracks where a thinner line might split off from the main, wider track.
    uint8_t sensors[8];
    for (int i = 0; i < 8; i++) {
        sensors[i] = (rawSensorByte >> i) & 0x01;
    }
    struct Block { int start; int len; };
    Block blocks[8];
    int block_count = 0;

    for (int i = 0; i < 8; ) {
        if (sensors[i] == 1) {
            blocks[block_count].start = i;
            blocks[block_count].len = 0;
            while (i < 8 && sensors[i] == 1) {
                blocks[block_count].len++;
                i++;
            }
            block_count++;
        } else {
            i++;
        }
    }
    
    if (block_count > 1) {
        result.isComplex = true;
    }

    if (block_count == 0) { // Should not happen due to initial check, but for safety
        result.featureType = LINE_LOST;
        return result;
    }

    int largest_block_idx = 0;
    for (int i = 1; i < block_count; i++) {
        if (blocks[i].len > blocks[largest_block_idx].len) {
            largest_block_idx = i;
        }
    }

    int weighted_sum = 0;
    int sum_of_readings = 0;
    for (int i = blocks[largest_block_idx].start; i < blocks[largest_block_idx].start + blocks[largest_block_idx].len; i++) {
        weighted_sum += weights[i];
        sum_of_readings++;
    }
    if (sum_of_readings > 0) {
        result.position = weighted_sum / sum_of_readings;
    }

#elif LINE_ANALYSIS_ALGORITHM == ALGO_NORMAL
    // --- ALGORITHM 3: NORMAL WEIGHTED AVERAGE ---
    // This is the standard algorithm. It calculates a weighted average across all active sensors.
    // It's simple and effective but can be confused by gaps or splits in the line.
    int weighted_sum = 0;
    int sum_of_readings = 0;

    for (int i = 0; i < 8; i++) {
        if ((rawSensorByte >> i) & 0x01) {
            weighted_sum += weights[i];
            sum_of_readings++;
        }
    }

    if (sum_of_readings > 0) {
        result.position = weighted_sum / sum_of_readings;
    }
    // This algorithm does not detect complex lines, so isComplex is always false.
    result.isComplex = false;

#endif

    result.featureType = NORMAL_LINE;
    return result;
}

void loop() {
#if WEBSERVER
    if (newCommandReceived) {
        processCommand(command_buffer);
        newCommandReceived = false; // Reset the flag
    }
#endif

    if (last_end != LAST_END_UNKNOWN && millis() - last_end_update_time > LAST_END_TIMEOUT_MS) {
        last_end = LAST_END_UNKNOWN;
    }

    if (Serial1.available()) {
#if LINE_SENSOR_MODE == 1
        uint8_t rawSensorByte = Serial1.read();
        LineAnalysisResult lineState = analyzeSensorData(rawSensorByte);

        switch (lineState.featureType) {
            case NORMAL_LINE: {
                line_lost_time = 0;
                error = lineState.position - setPoint;
                bool on_left_edge = (rawSensorByte & 0b00000001);
                bool on_right_edge = (rawSensorByte & 0b10000000);

                if (on_left_edge) {
                    last_end = LAST_END_LEFT;
                    last_end_update_time = millis();
                } else if (on_right_edge) {
                    last_end = LAST_END_RIGHT;
                    last_end_update_time = millis();
                }
                pidTurn(error, lineState.isComplex);
                break;
            }
            case JUNCTION: {
                line_lost_time = 0;
                motorspeed(junctionSpeed, junctionSpeed);
                break;
            }
            case LINE_LOST: {
                if (last_end == LAST_END_UNKNOWN) {
                    if (line_lost_time == 0) {
                        line_lost_time = millis();
                    } else if (millis() - line_lost_time > DASHED_LINE_GRACE_PERIOD_MS) {
                        handleLineLost();
                    }
                } else {
                    handleLineLost();
                }
                break;
            }
        }
#elif LINE_SENSOR_MODE == 2
        // To be implemented
#endif
    }
}

void handleLineLost() {
    if (last_end == LAST_END_RIGHT) {
        motorspeed(LINE_LOST_PIVOT_SPEED, LINE_LOST_TURN_SPEED);
    } else if (last_end == LAST_END_LEFT) {
        motorspeed(LINE_LOST_TURN_SPEED, LINE_LOST_PIVOT_SPEED);
    }
}

void pidTurn(int error, bool isComplex) {
    unsigned long currentTime = micros();
    float deltaTime = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    if (deltaTime <= 0.0 || deltaTime > 0.3) {
        deltaTime = 0.0;
    }

    float derivative = 0;
    if (deltaTime > 0) {
        derivative = (error - lastError) / deltaTime;
    }

    int motorSpeed = round((Kp * error) + (Kd * derivative));
    lastError = error;

    int dynamicBaseSpeed = baseSpeed;
    if (isComplex) {
        dynamicBaseSpeed = SLOW_SPEED + 1;
    }

    int currentSpeed = map(abs(error), 0, setPoint / 1.7, dynamicBaseSpeed, SLOW_SPEED);
    currentSpeed = constrain(currentSpeed, SLOW_SPEED, dynamicBaseSpeed);

    int leftMotorSpeed = constrain(currentSpeed + motorSpeed, -maxSpeed, maxSpeed);
    int rightMotorSpeed = constrain(currentSpeed - motorSpeed, -maxSpeed, maxSpeed);

    motorspeed(rightMotorSpeed, leftMotorSpeed);
}

void stopmotors() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
}

#if WEBSERVER
void receiveEvent(int howMany) {
    if (howMany > 0 && !newCommandReceived) {
        int i = 0;
        while (Wire.available() && i < sizeof(command_buffer) - 1) {
            char c = Wire.read();
            if (c == '\n' || c == '\r') continue; // Ignore newlines
            command_buffer[i] = c;
            i++;
        }
        command_buffer[i] = '\0'; 
        newCommandReceived = true;
    }
}

void processCommand(char* command) {
    char* separator = strchr(command, ':');

    if (separator != NULL) {
        // Split the string into two parts by replacing ':' with a null terminator
        *separator = '\0';
        char* valueStr = separator + 1;

#if USB_DEBUG
        Serial.print("I2C CMD RX: '");
        Serial.print(command);
        Serial.print("' VAL: '");
        Serial.print(valueStr);
        Serial.println("'");
#endif

        if (strcmp(command, "KP") == 0) {
            Kp = atof(valueStr);
        } else if (strcmp(command, "KD") == 0) {
            Kd = atof(valueStr);
        } else if (strcmp(command, "BS") == 0) {
            baseSpeed = atoi(valueStr);
        } else if (strcmp(command, "MS") == 0) {
            maxSpeed = atoi(valueStr);
        } else if (strcmp(command, "MN") == 0) {
            minSpeed = atoi(valueStr);
        } else if (strcmp(command, "SS") == 0) {
            SLOW_SPEED = atoi(valueStr);
        } else if (strcmp(command, "SP") == 0) {
            setPoint = atoi(valueStr);
        } else if (strcmp(command, "LT") == 0) {
            LAST_END_TIMEOUT_MS = atoi(valueStr);
        } else if (strcmp(command, "DG") == 0) {
            DASHED_LINE_GRACE_PERIOD_MS = atoi(valueStr);
        } else if (strcmp(command, "TS") == 0) {
            LINE_LOST_TURN_SPEED = atoi(valueStr);
        } else if (strcmp(command, "PS") == 0) {
            LINE_LOST_PIVOT_SPEED = atoi(valueStr);
        } else if (strcmp(command, "JS") == 0) {
            junctionSpeed = atoi(valueStr);
        }
    }
}
#endif

void motorspeed(int rightmotorspeed, int leftmotorspeed) {
    if (rightmotorspeed > 0) {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
    } else if (rightmotorspeed < 0) {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, constrain(abs(rightmotorspeed), 0, 255));

    if (leftmotorspeed > 0) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else if (leftmotorspeed < 0) {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
    }
    analogWrite(MOTOR_A_PWM, constrain(abs(leftmotorspeed), 0, 255));
}
