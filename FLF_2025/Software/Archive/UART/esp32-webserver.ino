#include <HardwareSerial.h>
#include <WebServer.h>
#include <WiFi.h>

const char* ssid = "bus247";
const char* password = "ONETWOEIGHT";

// --- Web Server ---
WebServer server(80);

// ESP32-S3-ZERO
HardwareSerial ControlSerial(0);
const int uartBaud = 9600;

// Initialize with some default values that match the STM32 code
float currentKp = 4.4;
float currentKd = 6.0;
int currentMinSpeed = -255; // This was unused in STM32, but keeping it
int currentBaseSpeed = 170;
int currentMaxSpeed = 255;
int currentSlowSpeed = 165;
int currentSetPoint = 35;
int currentLastEndTimeout = 200;
int currentDashGracePeriod = 150;
int currentTurnSpeed = 255;
int currentPivotSpeed = -255;


// Set to 1 to enable USB Serial debug messages, 0 to disable
#define USB_DEBUG 0

// --- WiFi Reconnection ---
unsigned long previousWifiCheckMillis = 0;
const long wifiCheckInterval = 10000;  // Check WiFi status every 10 seconds

// --- Helper Function to send command to STM32 ---
void sendCommandToSTM32(String command, String value) {
    String fullCommand = command + ":" + value + "\n";
#if USB_DEBUG
    Serial.print("Sending to STM32: ");  // Debug print to USB Serial
    Serial.print(fullCommand);
#endif
    ControlSerial.print(fullCommand);  // Send via UART0
}

// Function to generate the main HTML page
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>FLF Control</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<style>";
    html += "body { font-family: sans-serif; background-color: #f4f4f4; margin: 20px; }";
    html += "h1 { color: #333; text-align: center; }";
    html += "form { background: #fff; padding: 20px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); max-width: 400px; margin: 20px auto; }";
    html += "label { display: block; margin-bottom: 8px; font-weight: bold; color: #555; }";
    html += "input[type='number'] { width: 95%; padding: 10px; margin-bottom: 15px; border: 1px solid #ccc; border-radius: 4px; }";
    html += "input[type='submit'] { background-color: #007bff; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 1em; width: 100%; }";
    html += "input[type='submit']:hover { background-color: #0056b3; }";
    html += ".current-vals { background-color: #e9ecef; padding: 15px; border-radius: 5px; margin-top: 20px; max-width: 400px; margin: 20px auto; font-size: 0.9em; }";
    html += ".current-vals p { margin: 5px 0; }";
    html += "</style>";
    html += "</head><body>";
    html += "<h1>FLF Controller Settings</h1>";

    // Display current values
    html += "<div class='current-vals'>";
    html += "<h2>Current Settings:</h2>";
    html += "<p>Kp: " + String(currentKp, 3) + "</p>";  // Show 3 decimal places
    html += "<p>Kd: " + String(currentKd, 3) + "</p>";
    html += "<p>Base Speed: " + String(currentBaseSpeed) + "</p>";
    html += "<p>Max Speed: " + String(currentMaxSpeed) + "</p>";
    html += "<p>Slow Speed: " + String(currentSlowSpeed) + "</p>";
    html += "<p>Set Point: " + String(currentSetPoint) + "</p>";
    html += "<p>Last End Timeout (ms): " + String(currentLastEndTimeout) + "</p>";
    html += "<p>Dash Grace Period (ms): " + String(currentDashGracePeriod) + "</p>";
    html += "<p>Line Lost Turn Speed: " + String(currentTurnSpeed) + "</p>";
    html += "<p>Line Lost Pivot Speed: " + String(currentPivotSpeed) + "</p>";
    html += "</div>";

    // Input Form
    html += "<form action='/set' method='POST'>";  // Submit data to /set path

    html += "<label for='kp'>Kp:</label>";
    html += "<input type='number' id='kp' name='kp' step='0.01' value='" + String(currentKp, 3) + "' required><br>";

    html += "<label for='kd'>Kd:</label>";
    html += "<input type='number' id='kd' name='kd' step='0.01' value='" + String(currentKd, 3) + "' required><br>";

    html += "<label for='base_speed'>Base Speed (0-255):</label>";
    html += "<input type='number' id='base_speed' name='base_speed' step='1' min='0' max='255' value='" + String(currentBaseSpeed) + "' required><br>";

    html += "<label for='max_speed'>Max Speed (0-255):</label>";
    html += "<input type='number' id='max_speed' name='max_speed' step='1' min='0' max='255' value='" + String(currentMaxSpeed) + "' required><br>";

    html += "<label for='slow_speed'>Slow Speed (0-255):</label>";
    html += "<input type='number' id='slow_speed' name='slow_speed' step='1' min='0' max='255' value='" + String(currentSlowSpeed) + "' required><br>";

    html += "<label for='set_point'>Set Point (0-70):</label>";
    html += "<input type='number' id='set_point' name='set_point' step='1' min='0' max='70' value='" + String(currentSetPoint) + "' required><br>";

    html += "<label for='last_end_timeout'>Last End Timeout (ms):</label>";
    html += "<input type='number' id='last_end_timeout' name='last_end_timeout' step='10' value='" + String(currentLastEndTimeout) + "' required><br>";

    html += "<label for='dash_grace_period'>Dash Grace Period (ms):</label>";
    html += "<input type='number' id='dash_grace_period' name='dash_grace_period' step='10' value='" + String(currentDashGracePeriod) + "' required><br>";

    html += "<label for='turn_speed'>Line Lost Turn Speed (-255 to 255):</label>";
    html += "<input type='number' id='turn_speed' name='turn_speed' step='1' min='-255' max='255' value='" + String(currentTurnSpeed) + "' required><br>";

    html += "<label for='pivot_speed'>Line Lost Pivot Speed (-255 to 255):</label>";
    html += "<input type='number' id='pivot_speed' name='pivot_speed' step='1' min='-255' max='255' value='" + String(currentPivotSpeed) + "' required><br>";

    html += "<input type='submit' value='Update Settings'>";
    html += "</form>";

    html += "</body></html>";

    server.send(200, "text/html", html);
}

// Function to handle the form submission
void handleSet() {
    if (server.hasArg("kp")) {
        float newKp = server.arg("kp").toFloat();
        if (newKp != currentKp) {
            currentKp = newKp;
            sendCommandToSTM32("KP", String(currentKp, 3));
        }
    }
    if (server.hasArg("kd")) {
        float newKd = server.arg("kd").toFloat();
        if (newKd != currentKd) {
            currentKd = newKd;
            sendCommandToSTM32("KD", String(currentKd, 3));
        }
    }
    if (server.hasArg("base_speed")) {
        int newBaseSpeed = server.arg("base_speed").toInt();
        if (newBaseSpeed != currentBaseSpeed) {
            currentBaseSpeed = newBaseSpeed;
            sendCommandToSTM32("BS", String(currentBaseSpeed));
        }
    }
    if (server.hasArg("max_speed")) {
        int newMaxSpeed = server.arg("max_speed").toInt();
        if (newMaxSpeed != currentMaxSpeed) {
            currentMaxSpeed = newMaxSpeed;
            sendCommandToSTM32("MS", String(currentMaxSpeed));
        }
    }
    if (server.hasArg("slow_speed")) {
        int newSlowSpeed = server.arg("slow_speed").toInt();
        if (newSlowSpeed != currentSlowSpeed) {
            currentSlowSpeed = newSlowSpeed;
            sendCommandToSTM32("SS", String(currentSlowSpeed));
        }
    }
    if (server.hasArg("set_point")) {
        int newSetPoint = server.arg("set_point").toInt();
        if (newSetPoint != currentSetPoint) {
            currentSetPoint = newSetPoint;
            sendCommandToSTM32("SP", String(currentSetPoint));
        }
    }
    if (server.hasArg("last_end_timeout")) {
        int newLastEndTimeout = server.arg("last_end_timeout").toInt();
        if (newLastEndTimeout != currentLastEndTimeout) {
            currentLastEndTimeout = newLastEndTimeout;
            sendCommandToSTM32("LT", String(currentLastEndTimeout));
        }
    }
    if (server.hasArg("dash_grace_period")) {
        int newDashGracePeriod = server.arg("dash_grace_period").toInt();
        if (newDashGracePeriod != currentDashGracePeriod) {
            currentDashGracePeriod = newDashGracePeriod;
            sendCommandToSTM32("DG", String(currentDashGracePeriod));
        }
    }
    if (server.hasArg("turn_speed")) {
        int newTurnSpeed = server.arg("turn_speed").toInt();
        if (newTurnSpeed != currentTurnSpeed) {
            currentTurnSpeed = newTurnSpeed;
            sendCommandToSTM32("TS", String(currentTurnSpeed));
        }
    }
    if (server.hasArg("pivot_speed")) {
        int newPivotSpeed = server.arg("pivot_speed").toInt();
        if (newPivotSpeed != currentPivotSpeed) {
            currentPivotSpeed = newPivotSpeed;
            sendCommandToSTM32("PS", String(currentPivotSpeed));
        }
    }

    // Redirect back to the root page to show updated values
    server.sendHeader("Location", "/");
    server.send(303);  // 303 See Other redirect
}

// --- Setup Function ---
void setup() {
    Serial.begin(115200);  // USB Serial for debugging
    delay(500);
    Serial.println("\nESP32-S3 Controller Setup");

    ControlSerial.begin(uartBaud);
    Serial.printf("Control UART (Serial0) Initialized at %d baud on TX=D6, RX=D7.\n", uartBaud);

    // --- Connect to WiFi ---
    Serial.printf("Connecting to %s ", ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect to WiFi. Halting.");
        // Consider a non-halting behavior if device needs to function partially offline
        while (true) {
            delay(1000);
        }
    } else {
        Serial.println("\nWiFi connected!");
        Serial.print("IP Address: http://");
        Serial.println(WiFi.localIP());
    }

    server.on("/", HTTP_GET, handleRoot);   
    server.on("/set", HTTP_POST, handleSet);

    server.begin();
    Serial.println("HTTP server started");
}

void checkWiFi() {
    unsigned long currentMillis = millis();
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousWifiCheckMillis >= wifiCheckInterval)) {
        Serial.print(currentMillis);
        Serial.println("ms: WiFi disconnected! Attempting reconnection...");
        WiFi.disconnect();  // Force disconnect before reconnecting
        WiFi.reconnect();   // Attempt to reconnect using stored credentials
        // Or use WiFi.begin(ssid, password); if reconnect fails
        previousWifiCheckMillis = currentMillis;
    }
    // Simple periodic status check without blocking
    else if (currentMillis - previousWifiCheckMillis >= wifiCheckInterval) {
        previousWifiCheckMillis = currentMillis;  // Update timestamp even if connected
    }
}

void loop() {
    checkWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
    }

    // Optional: Check for any messages *from* STM32 (if needed)
    // if (ControlSerial.available() > 0) {
    //    String msgFromSTM = ControlSerial.readStringUntil('\n');
    //    #if USB_DEBUG
    //    Serial.print("Received from STM32: ");
    //    Serial.println(msgFromSTM);
    //    #endif
    // }

    delay(10);  // Small delay for stability and to yield time
}
