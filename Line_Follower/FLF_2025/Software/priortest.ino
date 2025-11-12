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
#define ALGO_MOST_CENTERED 1
#define ALGO_LARGEST       2
#define ALGO_NORMAL        3
#define LINE_ANALYSIS_ALGORITHM ALGO_MOST_CENTERED

// --- Line Priority Selection ---
#define PRIO_NONE          0 
#define PRIO_LEFT          1 
#define PRIO_RIGHT         2 
#define LINE_PRIORITY 2

#define MASK_LEFT_TURN  0b00000111
#define MASK_RIGHT_TURN 0b11100000
#define MASK_CENTER_SENSORS 0b00011100
// --- NEW: Bitmasks for circle exit patterns (left and right) ---
#define CIRCLE_EXIT_LEFT_MASK 0b00011111
#define CIRCLE_EXIT_RIGHT_MASK 0b00111111

// --- I2C Definitions ---
#if WEBSERVER
const int I2C_SLAVE_ADDR = 0x08;
volatile bool newCommandReceived = false;
char command_buffer[32];
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

// --- Turn Commit Logic ---
const unsigned int PRIORITY_TURN_DURATION_MS = 70; // TUNE THIS VALUE FOR 90 DEGREES
unsigned long turn_commit_end_time = 0;
enum TurnDirection { NONE, LEFT, RIGHT };
TurnDirection committed_turn_direction = NONE;

unsigned long last_junction_time = 0;
const unsigned int JUNCTION_LOST_LINE_TIMEOUT_MS = 100;
unsigned long same_direction_timer = 0;
const unsigned int SAME_DIRECTION_TIMEOUT_MS = 2000;
int last_error_direction = 0;
// --- NEW: Branch and Circle Exit detection state variables ---
bool is_branch_detected = false; 
bool is_circle_exit_detected = false;

// --- PID and Motor Control Variables ---
float Kp = 5.150;
float Kd = 4.550;
int SLOW_SPEED = 90;
int baseSpeed = 180;
int maxSpeed = 255;
int setPoint = 35;
int minSpeed = -255;
int junctionSpeed = 100;
int lastError = 0;
int error = 0;
unsigned long previousTime = 0;
int currentSpeed = 0; // Added for the speed threshold logic

#if WEBSERVER
void receiveEvent(int howMany);
void processCommand(char* command);
#endif

// --- State variables for line tracking and memory ---
const int LAST_END_LEFT = 0;
const int LAST_END_RIGHT = 1;
const int LAST_END_UNKNOWN = 2;
int last_end = LAST_END_UNKNOWN;
unsigned long last_end_update_time = 0;
unsigned int LAST_END_TIMEOUT_MS = 110; 
unsigned long line_lost_time = 0;
unsigned int DASHED_LINE_GRACE_PERIOD_MS = 180;

const int DEBOUNCE_FRAME_COUNT = 4;
int priority_left_frame_count = 0;
int priority_right_frame_count = 0;
unsigned long time_after_junction = 0;

// --- Turn Speeds ---
int LINE_LOST_TURN_SPEED = 170;
int LINE_LOST_PIVOT_SPEED = -170;
int PRIORITY_TURN_SPEED = 170;
int PRIORITY_PIVOT_SPEED = -170;

enum LineFeature {
  NORMAL_LINE,
  JUNCTION,
  LINE_LOST,
  FORCED_PRIORITY_TURN_LEFT,
  FORCED_PRIORITY_TURN_RIGHT
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
    delay(850);
    Serial.println("Initializing USB Debug Serial...");
#if WEBSERVER
    Wire.begin(I2C_SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Serial.print("Initialized I2C Slave on address 0x");
    Serial.println(I2C_SLAVE_ADDR, HEX);
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
      is_branch_detected = false; // Reset the flags for each new reading
    is_circle_exit_detected = false;

    if (rawSensorByte == 0xFF) {
        result.featureType = JUNCTION;
        result.position = 35;
         is_branch_detected = true; // A junction is a branch
        return result;
           

    }
    if (rawSensorByte == 0x00) {
        result.featureType = LINE_LOST;
        return result;
    }
    // --- NEW: Check for both left and right circle exit patterns ---
    if (((rawSensorByte & CIRCLE_EXIT_LEFT_MASK) == CIRCLE_EXIT_LEFT_MASK) ||
        ((rawSensorByte & CIRCLE_EXIT_RIGHT_MASK) == CIRCLE_EXIT_RIGHT_MASK)) {
        is_circle_exit_detected = true;
    }

    int weights[8] = {0, 10, 20, 30, 40, 50, 60, 70};

#if (LINE_ANALYSIS_ALGORITHM == ALGO_MOST_CENTERED) || (LINE_ANALYSIS_ALGORITHM == ALGO_LARGEST)
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

    if (block_count == 0) {
        result.featureType = LINE_LOST;
        return result;
    }

    if (block_count == 1) {
        // Use block start/length to avoid false triggers from curves/short edge contacts.
        int bstart = blocks[0].start;
        int blen = blocks[0].len;

        // --- NEW: If both extreme edge sensors are active, treat as straight.
        // This prevents trying to turn when path goes toward both sides.
        if ((rawSensorByte & 0b10000001) == 0b10000001) {
            // both leftmost and rightmost sensors active -> consider it not a priority turn
            priority_left_frame_count = 0;
            priority_right_frame_count = 0;
        } else {
            #if LINE_PRIORITY == PRIO_LEFT
            // Require the block to be touching the left edge (start==0), be reasonably wide,
            // and match the left mask to reduce false positives from small edge contacts/curves.
            if (bstart == 0 && blen >= 3 && (rawSensorByte & MASK_LEFT_TURN) == MASK_LEFT_TURN) {
                priority_left_frame_count++;
            } else {
                priority_left_frame_count = 0;
            }
            if (priority_left_frame_count >= DEBOUNCE_FRAME_COUNT) {
                priority_left_frame_count = 0;
                result.featureType = FORCED_PRIORITY_TURN_LEFT;
                return result;
            }
            #endif

            #if LINE_PRIORITY == PRIO_RIGHT
            // Require the block to be touching the right edge (end==8), be reasonably wide,
            // and match the right mask. Using mask check prevents spurious equality matches.
            if ((bstart + blen) == 8 && blen >= 3 && (rawSensorByte & MASK_RIGHT_TURN) == MASK_RIGHT_TURN) {
                priority_right_frame_count++;
            } else {
                priority_right_frame_count = 0;
            }
            if (priority_right_frame_count >= DEBOUNCE_FRAME_COUNT) {
                priority_right_frame_count = 0;
                result.featureType = FORCED_PRIORITY_TURN_RIGHT;
                return result;
            }
            #endif
        }
    } else {
        // If it's not a single line, it can't be a valid turn, so reset counters
        priority_left_frame_count = 0;
        priority_right_frame_count = 0;
    }

    int selected_block_idx = 0;
    if (block_count > 1) {
        #if LINE_ANALYSIS_ALGORITHM == ALGO_MOST_CENTERED
        float min_dist_to_center = 100.0;
        for (int i = 0; i < block_count; i++) {
            float block_center = (float)blocks[i].start + ((float)blocks[i].len - 1.0) / 2.0;
            float dist_to_center = abs(block_center - 3.5);
            if (dist_to_center < min_dist_to_center) {
                min_dist_to_center = dist_to_center;
                selected_block_idx = i;
            }
        }
        #elif LINE_ANALYSIS_ALGORITHM == ALGO_LARGEST
        for (int i = 1; i < block_count; i++) {
            if (blocks[i].len > blocks[selected_block_idx].len) {
                selected_block_idx = i;
            }
        }
        #endif
    }

    int weighted_sum = 0;
    int sum_of_readings = 0;
    for (int i = blocks[selected_block_idx].start; i < blocks[selected_block_idx].start + blocks[selected_block_idx].len; i++) {
        weighted_sum += weights[i];
        sum_of_readings++;
    }
    if (sum_of_readings > 0) {
        result.position = weighted_sum / sum_of_readings;
    }

#elif LINE_ANALYSIS_ALGORITHM == ALGO_NORMAL
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
    result.isComplex = false;
#endif

    result.featureType = NORMAL_LINE;
    return result;
}

void loop() {
    if (committed_turn_direction != NONE) {
        if (millis() < turn_commit_end_time) {
            if (committed_turn_direction == LEFT) {
                motorspeed(PRIORITY_TURN_SPEED, PRIORITY_PIVOT_SPEED);
            } else { // RIGHT
                motorspeed(PRIORITY_PIVOT_SPEED, PRIORITY_TURN_SPEED);
            }
            return; 
        } else {
        committed_turn_direction = NONE; 
    }
    }
    // --- NEW: STUCK CHECK LOGIC ---
    if (same_direction_timer > 0 && millis() - same_direction_timer > SAME_DIRECTION_TIMEOUT_MS && is_circle_exit_detected ) {
        // If we have been turning in the same direction for too long, override the priority and force a turn.
        // We do this by checking the last recorded direction and forcing the opposite.
        if (last_error_direction == 1) { // Was turning left, so force right
            committed_turn_direction = RIGHT;
        } else { // Was turning right, so force left
            committed_turn_direction = LEFT;
        }
        turn_commit_end_time = millis() + PRIORITY_TURN_DURATION_MS;
        
        // Reset the timer and direction for the next run.
        same_direction_timer = 0;
        last_error_direction = 0;
        is_circle_exit_detected = false; 

        return; // Skip the rest of the loop to execute the forced turn.
    }
    
#if WEBSERVER
    if (newCommandReceived) {
        processCommand(command_buffer);
        newCommandReceived = false;
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
               // <<< NEW: Record the time we saw the junction
                last_junction_time = millis();
                motorspeed(junctionSpeed, junctionSpeed);
                   // No priority at junctions, follow center line(old)
                break;
            }
            case LINE_LOST: {
               // <<< NEW: Check if we lost the line right after a junction
                if (last_junction_time > 0 && millis() - last_junction_time < JUNCTION_LOST_LINE_TIMEOUT_MS) {
                    // This is a T-junction turn, execute based on priority
                    #if LINE_PRIORITY == PRIO_LEFT
                        committed_turn_direction = LEFT;
                        turn_commit_end_time = millis() + PRIORITY_TURN_DURATION_MS;
                        motorspeed(PRIORITY_TURN_SPEED, PRIORITY_PIVOT_SPEED);
                    #elif LINE_PRIORITY == PRIO_RIGHT
                        committed_turn_direction = RIGHT;
                        turn_commit_end_time = millis() + PRIORITY_TURN_DURATION_MS;
                        motorspeed(PRIORITY_PIVOT_SPEED, PRIORITY_TURN_SPEED);
                    #endif
                    last_junction_time = 0; // Consume the event
                    break; 
                }

                // Original logic for dashed lines or losing the line
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
            case FORCED_PRIORITY_TURN_LEFT: {
                committed_turn_direction = LEFT;
                turn_commit_end_time = millis() + PRIORITY_TURN_DURATION_MS;
                motorspeed(PRIORITY_TURN_SPEED, PRIORITY_PIVOT_SPEED);
                break;
            }
            case FORCED_PRIORITY_TURN_RIGHT: {
                committed_turn_direction = RIGHT;
                turn_commit_end_time = millis() + PRIORITY_TURN_DURATION_MS;
                motorspeed(PRIORITY_PIVOT_SPEED, PRIORITY_TURN_SPEED);
                break;
            }
        }
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

     // --- NEW: Update the direction timer only on NORMAL_LINE feature ---
    if (abs(error) > 5) { // Only update if there's a significant error
        int current_error_direction = (error > 0) ? 1 : -1;
        if (current_error_direction != last_error_direction) {
            same_direction_timer = millis(); // Reset timer if direction changes
            last_error_direction = current_error_direction;
        }
    } else {
        // If the robot is going straight, reset the timer
        same_direction_timer = 0;
    }

    int dynamicBaseSpeed = baseSpeed;
    if (isComplex) {
        dynamicBaseSpeed = SLOW_SPEED + 1;
    }

    currentSpeed = map(abs(error), 0, setPoint / 1.75, dynamicBaseSpeed, SLOW_SPEED);
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
            if (c == '\n' || c == '\r') continue;
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
        *separator = '\0';
        char* valueStr = separator + 1;
#if USB_DEBUG
        Serial.print("I2C CMD RX: '"); Serial.print(command); Serial.print("' VAL: '"); Serial.print(valueStr); Serial.println("'");
#endif
        if (strcmp(command, "KP") == 0) { Kp = atof(valueStr); }
        else if (strcmp(command, "KD") == 0) { Kd = atof(valueStr); }
        else if (strcmp(command, "BS") == 0) { baseSpeed = atoi(valueStr); }
        else if (strcmp(command, "MS") == 0) { maxSpeed = atoi(valueStr); }
        else if (strcmp(command, "MN") == 0) { minSpeed = atoi(valueStr); }
        else if (strcmp(command, "SS") == 0) { SLOW_SPEED = atoi(valueStr); }
        else if (strcmp(command, "SP") == 0) { setPoint = atoi(valueStr); }
        else if (strcmp(command, "LT") == 0) { LAST_END_TIMEOUT_MS = atoi(valueStr); }
        else if (strcmp(command, "DG") == 0) { DASHED_LINE_GRACE_PERIOD_MS = atoi(valueStr); }
        else if (strcmp(command, "TS") == 0) { LINE_LOST_TURN_SPEED = atoi(valueStr); }
        else if (strcmp(command, "PS") == 0) { LINE_LOST_PIVOT_SPEED = atoi(valueStr); }
        else if (strcmp(command, "JS") == 0) { junctionSpeed = atoi(valueStr); }
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