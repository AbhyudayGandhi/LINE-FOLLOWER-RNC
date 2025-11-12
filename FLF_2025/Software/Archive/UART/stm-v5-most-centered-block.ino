#include <Arduino.h>

// Set WEBSERVER to 0 to disable ESP32 communication
#define WEBSERVER 0

// Set USB_DEBUG to 1 to enable detailed Serial output for debugging
#define USB_DEBUG 0

// Set LINE_SENSOR_MODE to 1 for Raw Mode (calculate position from 8 bits)
#define LINE_SENSOR_MODE 1

#if WEBSERVER
#include <SoftwareSerial.h>
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

#if WEBSERVER
#define ESP_RX_PIN PB7     // SL
#define ESP_TX_PIN PB6
#endif

const int SENSOR_BAUD_RATE = 230400; // 5
const int DEBUG_BAUD_RATE = 115200;

#if WEBSERVER
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);
const int ESP_BAUD_RATE = 9600;
#endif

// --- PID and Motor Control Variables ---
float Kp = 4.4;
float Kd = 6;
int SLOW_SPEED = 165;         // The minimum speed for navigating sharp curves
int baseSpeed = 170;          // Keep below 180
int maxSpeed = 255;
int setPoint = 35;
int minSpeed = -255;          // not used

int lastError = 0;
int error = 0;

unsigned long previousTime = 0;  // Time of the last PD calculation (microseconds)

#if WEBSERVER
String received = "";        // Buffer for incoming ESP32 data
bool readingSerial = false;  // Flag indicating if currently reading a command
bool validPrefix = false;    // Flag indicating if the first 2 chars form a valid prefix
#endif

// --- State variables for line tracking and memory ---
const int LAST_END_LEFT = 0;
const int LAST_END_RIGHT = 1;
const int LAST_END_UNKNOWN = 2; // Represents an expired or unknown state
int last_end = LAST_END_UNKNOWN;  // Where the line was last seen. Start as unknown.
const int LAST_END_UPDATE_THRESHOLD = 30;  // Minimum error magnitude to update last_end

// --- Timeout for last_end memory ---
unsigned long last_end_update_time = 0; // Timestamp for the last update
unsigned int LAST_END_TIMEOUT_MS = 200; // "Forget" the last direction after x ms of not seeing a turn.

// --- Grace period for handling dashed lines ---
unsigned long line_lost_time = 0; // Timestamp for when the line was first lost
unsigned int DASHED_LINE_GRACE_PERIOD_MS = 150; // Continue straight for 150ms over gaps

// --- Turn Speeds for Line Lost ---
int LINE_LOST_TURN_SPEED = 255;
int LINE_LOST_PIVOT_SPEED = -255;

// Defines the types of features the robot can identify on the track.
enum LineFeature {
  NORMAL_LINE,   // A standard line that PID can follow.
  JUNCTION,      // A cross-intersection or T-junction (all sensors are on).
  LINE_LOST      // No line is detected by any sensor.
};


struct LineAnalysisResult {
  int position;
  LineFeature featureType;
  bool isComplex; // True if more than one block of sensors is detected
};


void stopmotors();
void pidTurn(int error, bool isComplex);
void motorspeed(int rightmotorspeed, int leftmotorspeed);
void handleLineLost();
LineAnalysisResult analyzeSensorData(uint8_t rawSensorByte);


#if WEBSERVER
bool isValidPrefix(String prefix);
void processCommand(String commandString);
#endif

void setup() {
    // Always begin USB Serial
    Serial.begin(DEBUG_BAUD_RATE);
    delay(1500);
    Serial.println("Initializing USB Debug Serial...");

#if WEBSERVER
    espSerial.begin(ESP_BAUD_RATE);
    Serial.println("Initializing Software Serial for ESP32 comms (PB7=RX, PB6=TX)...");
#endif

    // Always initialize Hardware Serial for the Line Sensor
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
    result.position = 255; // Default invalid position
    result.isComplex = false; // Default to not complex

    if (rawSensorByte == 0xFF) {
        result.featureType = JUNCTION;
        result.position = 35; // Center position
        return result;
    }
    if (rawSensorByte == 0x00) {
        result.featureType = LINE_LOST;
        return result;
    }

    // --- Convert byte to array ---
    uint8_t sensors[8];
    for (int i = 0; i < 8; i++) {
        sensors[i] = (rawSensorByte >> i) & 0x01;
    }

    // --- Block Finding Logic ---
    int weights[8] = {0, 10, 20, 30, 40, 50, 60, 70};

    struct Block { int start; int len; };
    Block blocks[8]; // Max 8 blocks (e.g., 10101010)
    int block_count = 0;

    // Step 1: Find all contiguous blocks of 'on' sensors
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

    if (block_count == 0) {
        result.featureType = LINE_LOST;
        return result;
    }

    // Step 2: Find the most centered block
    // This logic replaces the previous "find largest block" logic.
    // It calculates which block is closest to the center of the sensor array (index 3.5)
    // and selects that one to follow. This is more robust for handling forks in the line.
    int most_centered_block_idx = 0;
    if (block_count > 1) {
        float min_dist_to_center = 100.0; // Initialize with a large value
        for (int i = 0; i < block_count; i++) {
            // Calculate the center of the current block
            float block_center = (float)blocks[i].start + ((float)blocks[i].len - 1.0) / 2.0;
            // Calculate the distance from the array's absolute center (3.5)
            float dist_to_center = abs(block_center - 3.5);
            if (dist_to_center < min_dist_to_center) {
                min_dist_to_center = dist_to_center;
                most_centered_block_idx = i;
            }
        }
    }

    // Step 3: Calculate position based on the chosen (most centered) block
    int weighted_sum = 0;
    int sum_of_readings = 0;
    for (int i = blocks[most_centered_block_idx].start; i < blocks[most_centered_block_idx].start + blocks[most_centered_block_idx].len; i++) {
        // We know sensors[i] is 1 inside the block, so we can just add the weight
        weighted_sum += weights[i];
        sum_of_readings++;
    }
    if (sum_of_readings > 0) {
        result.position = weighted_sum / sum_of_readings;
    }

    // If we've gotten this far, we have a normal line reading
    result.featureType = NORMAL_LINE;
    
    return result;
}


void loop() {
    // Check if the last_end memory has expired due to driving straight for too long
    if (last_end != LAST_END_UNKNOWN && millis() - last_end_update_time > LAST_END_TIMEOUT_MS) {
#if USB_DEBUG
        Serial.println("last_end memory expired. Resetting to UNKNOWN.");
#endif
        last_end = LAST_END_UNKNOWN;
    }

    if (Serial1.available()) {
#if LINE_SENSOR_MODE == 1
        uint8_t rawSensorByte = Serial1.read();
        LineAnalysisResult lineState = analyzeSensorData(rawSensorByte);

        switch (lineState.featureType) {
            case NORMAL_LINE: {
                // If we found the line, reset the dashed line grace period timer
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
                // Reset dashed line timer at junctions as well
                line_lost_time = 0;
#if USB_DEBUG
                Serial.println("Action: Junction Detected. Moving straight and slow.");
#endif
                motorspeed(120, 120);
                break;
            }
            case LINE_LOST: {
                // --- Dashed Line and Line Lost Logic ---
                // Only consider dashed lines if our memory has expired (we're likely on a straight).
                if (last_end == LAST_END_UNKNOWN) {
                    if (line_lost_time == 0) {
                        line_lost_time = millis();
#if USB_DEBUG
                        Serial.println("Line Lost (state UNKNOWN) - Starting grace period for dashed line...");
#endif
                        // Coast forward by continuing previous motor speeds, handled by pidTurn not being called.
                    } else if (millis() - line_lost_time > DASHED_LINE_GRACE_PERIOD_MS) {
                        // Grace period expired. The line is truly lost.
#if USB_DEBUG
                        Serial.println("Grace period expired. Action: Line Lost, UNKNOWN.");
#endif
                        handleLineLost();  // does absolutly nothing in this case
                    }
                    // Within grace period: do nothing, let robot coast.
                } else {
                    // If last_end is known (LEFT or RIGHT).
                    // Skip the grace period and react immediately.
#if USB_DEBUG
                    Serial.print("Line Lost (state KNOWN) - Reacting immediately. Last end: ");
                    Serial.println(last_end == LAST_END_RIGHT ? "RIGHT" : "LEFT");
#endif
                    handleLineLost(); // Will turn immediately
                }
                break;
            }
        }

#elif LINE_SENSOR_MODE == 2
        // To be implemented
#endif
    
    } else {
        // No sensor data available. Do nothing.
    }

// --- ESP32 Command Processing ---
#if WEBSERVER
    if (espSerial.available() > 0 || readingSerial) {
        readingSerial = true;
        while (espSerial.available() > 0) {
            char ch = (char)espSerial.read();
            if (ch == '\n') {
                if (validPrefix) {
                    processCommand(received);
                }
                received = "";
                readingSerial = false;
                validPrefix = false;
                break;
            }
            received += ch;
            if (received.length() == 2) {
                if (isValidPrefix(received)) {
                    validPrefix = true;
                } else {
                    validPrefix = false;
                    received = "";
                    readingSerial = false;
                    break;
                }
            }
        }
    }
#endif
    // --- End ESP32 Command Processing ---

}  // End loop()

void handleLineLost() { // quite simple as of now
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
#if USB_DEBUG
        Serial.println("WARN: Invalid deltaTime. Skipping D-term.");
#endif
    }

    float derivative = 0;
    if (deltaTime > 0) {
        derivative = (error - lastError) / deltaTime;
    }

    int motorSpeed = round((Kp * error) + (Kd * derivative));
    lastError = error;

    int dynamicBaseSpeed = baseSpeed;
    // If a complex line is detected (e.g., a split in the line or noise),
    // reduce the base speed to navigate the section more carefully.
    if (isComplex) {
        dynamicBaseSpeed = SLOW_SPEED + 15; 
#if USB_DEBUG
        Serial.println("Complex line detected, slowing down.");
#endif
    }

    // The robot's forward speed is now proportional to the tracking error,
    int currentSpeed = map(abs(error), 0, setPoint / 1.5, dynamicBaseSpeed, SLOW_SPEED);
    currentSpeed = constrain(currentSpeed, SLOW_SPEED, dynamicBaseSpeed);

    int leftMotorSpeed = constrain(currentSpeed + motorSpeed, -maxSpeed, maxSpeed);
    int rightMotorSpeed = constrain(currentSpeed - motorSpeed, -maxSpeed, maxSpeed);

#if USB_DEBUG
    if (error > 0) {
        Serial.println("Turning Right (PID)");
    } else if (error < 0) {
        Serial.println("Turning Left (PID)");
    }
#endif
    motorspeed(rightMotorSpeed, leftMotorSpeed);
}

void stopmotors() {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_A_PWM, 0);
    analogWrite(MOTOR_B_PWM, 0);
#if USB_DEBUG
    Serial.println("STOPMOTORS called");
#endif
}

#if WEBSERVER
bool isValidPrefix(String prefix) { 
    return prefix == "KP" || prefix == "KD" || prefix == "MN" || prefix == "BS" || prefix == "MS" ||
           prefix == "SS" || prefix == "SP" || prefix == "LT" || prefix == "DG" || prefix == "TS" ||
           prefix == "PS";
}

void processCommand(String commandString) {
    commandString.trim();
    int colonIndex = commandString.indexOf(':');

    if (colonIndex > 0 && commandString.length() > colonIndex + 1) {
        String command = commandString.substring(0, colonIndex);   // e.g., "KP"
        String valueStr = commandString.substring(colonIndex + 1);  // e.g., "1.23"

#if USB_DEBUG
        Serial.print("  Processing Parsed Command: ");
        Serial.print(command);
        Serial.print(", Value: ");
        Serial.println(valueStr);
#endif

        if (command.equalsIgnoreCase("KP")) {
            Kp = valueStr.toFloat();
#if USB_DEBUG
            Serial.print("  -> Updated Kp = ");
            Serial.println(Kp);
#endif
        } else if (command.equalsIgnoreCase("KD")) {
            Kd = valueStr.toFloat();
#if USB_DEBUG
            Serial.print("  -> Updated Kd = ");
            Serial.println(Kd);
#endif
        } else if (command.equalsIgnoreCase("BS")) {
            baseSpeed = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Base Speed = ");
            Serial.println(baseSpeed);
#endif
        } else if (command.equalsIgnoreCase("MS")) {
            maxSpeed = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Max Speed = ");
            Serial.println(maxSpeed);
#endif
        } else if (command.equalsIgnoreCase("MN")) {
            minSpeed = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated minspeed = ");
            Serial.println(minSpeed);
#endif
        } else if (command.equalsIgnoreCase("SS")) {
            SLOW_SPEED = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Slow Speed = ");
            Serial.println(SLOW_SPEED);
#endif
        } else if (command.equalsIgnoreCase("SP")) {
            setPoint = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Set Point = ");
            Serial.println(setPoint);
#endif
        } else if (command.equalsIgnoreCase("LT")) {
            LAST_END_TIMEOUT_MS = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Last End Timeout (ms) = ");
            Serial.println(LAST_END_TIMEOUT_MS);
#endif
        } else if (command.equalsIgnoreCase("DG")) {
            DASHED_LINE_GRACE_PERIOD_MS = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Dash Grace Period (ms) = ");
            Serial.println(DASHED_LINE_GRACE_PERIOD_MS);
#endif
        } else if (command.equalsIgnoreCase("TS")) {
            LINE_LOST_TURN_SPEED = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Line Lost Turn Speed = ");
            Serial.println(LINE_LOST_TURN_SPEED);
#endif
        } else if (command.equalsIgnoreCase("PS")) {
            LINE_LOST_PIVOT_SPEED = valueStr.toInt();
#if USB_DEBUG
            Serial.print("  -> Updated Line Lost Pivot Speed = ");
            Serial.println(LINE_LOST_PIVOT_SPEED);
#endif
        } else {
// This case shouldn't happen if isValidPrefix worked, but good to have
#if USB_DEBUG
            Serial.println("  -> Error: Unknown command type in processCommand");
#endif
        }
    } else {
#if USB_DEBUG
        Serial.println("  -> Error: Invalid command format found in processCommand: " + commandString);
#endif
    }
}

#endif  // WEBSERVER

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
