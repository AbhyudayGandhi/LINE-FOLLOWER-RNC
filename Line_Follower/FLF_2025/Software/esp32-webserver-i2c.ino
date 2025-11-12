#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h> // Include the I2C library

const char* ssid = "bus247";
const char* password = "ONETWOEIGHT";

// --- Web Server ---
WebServer server(80);

// --- I2C Definitions ---
// SCL GP12 SDA GP13
const int STM32_I2C_ADDRESS = 0x08; // The I2C address of the STM32 slave

// Initialize with some default values that match the STM32 code
float currentKp = 4.4;
float currentKd = 6.0;
int currentMinSpeed = -255;
int currentBaseSpeed = 170;
int currentMaxSpeed = 255;
int currentSlowSpeed = 165;
int currentLastEndTimeout = 200;
int currentDashGracePeriod = 150;
int currentTurnSpeed = 255;
int currentPivotSpeed = -255;
int currentJunctionSpeed = 120;

// Set to 1 to enable USB Serial debug messages, 0 to disable
#define USB_DEBUG 0

// --- WiFi Reconnection ---
unsigned long previousWifiCheckMillis = 0;
const long wifiCheckInterval = 10000; // Check WiFi status every 10 seconds

// --- Helper Function to send command to STM32 via I2C ---
void sendCommandToSTM32(String command, String value) {
    String fullCommand = command + ":" + value; // No newline needed, STM32 handles it.
    
    Wire.beginTransmission(STM32_I2C_ADDRESS);
    Wire.print(fullCommand);
    byte error = Wire.endTransmission();
    
#if USB_DEBUG
    if (error == 0) {
        Serial.print("I2C Success. Sent: ");
        Serial.println(fullCommand);
    } else {
        Serial.print("I2C Send Error: ");
        Serial.print(error);
        Serial.print(" (Failed to send: ");
        Serial.print(fullCommand);
        Serial.println(")");
    }
#endif
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

    html += "<div class='current-vals'>";
    html += "<h2>Current Settings:</h2>";
    html += "<p>Kp: " + String(currentKp, 3) + "</p>";
    html += "<p>Kd: " + String(currentKd, 3) + "</p>";
    html += "<p>Base Speed: " + String(currentBaseSpeed) + "</p>";
    html += "<p>Max Speed: " + String(currentMaxSpeed) + "</p>";
    html += "<p>Slow Speed: " + String(currentSlowSpeed) + "</p>";
    html += "<p>Junction Speed: " + String(currentJunctionSpeed) + "</p>";
    html += "<p>Last End Timeout (ms): " + String(currentLastEndTimeout) + "</p>";
    html += "<p>Dash Grace Period (ms): " + String(currentDashGracePeriod) + "</p>";
    html += "<p>Line Lost Turn Speed: " + String(currentTurnSpeed) + "</p>";
    html += "<p>Line Lost Pivot Speed: " + String(currentPivotSpeed) + "</p>";
    html += "</div>";

    html += "<form action='/set' method='POST'>";
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
    html += "<label for='junction_speed'>Junction Speed (0-255):</label>";
    html += "<input type='number' id='junction_speed' name='junction_speed' step='1' min='0' max='255' value='" + String(currentJunctionSpeed) + "' required><br>";
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

void updateIntParam(const char* argName, const char* cmdCode, int& currentValue) {
    if (server.hasArg(argName)) {
        int newValue = server.arg(argName).toInt();
        if (newValue != currentValue) {
            currentValue = newValue;
            sendCommandToSTM32(cmdCode, String(newValue));
        }
    }
}

void updateFloatParam(const char* argName, const char* cmdCode, float& currentValue) {
    if (server.hasArg(argName)) {
        float newValue = server.arg(argName).toFloat();
        if (newValue != currentValue) {
            currentValue = newValue;
            sendCommandToSTM32(cmdCode, String(newValue, 3));
        }
    }
}

void handleSet() {
    updateFloatParam("kp", "KP", currentKp);
    updateFloatParam("kd", "KD", currentKd);
    updateIntParam("base_speed", "BS", currentBaseSpeed);
    updateIntParam("max_speed", "MS", currentMaxSpeed);
    updateIntParam("slow_speed", "SS", currentSlowSpeed);
    updateIntParam("junction_speed", "JS", currentJunctionSpeed);
    updateIntParam("last_end_timeout", "LT", currentLastEndTimeout);
    updateIntParam("dash_grace_period", "DG", currentDashGracePeriod);
    updateIntParam("turn_speed", "TS", currentTurnSpeed);
    updateIntParam("pivot_speed", "PS", currentPivotSpeed);

    server.sendHeader("Location", "/");
    server.send(303);
}

// --- Setup Function ---
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\nESP32-S3 Controller Setup");

    Wire.begin(13, 12); 
    Serial.println("I2C Master Initialized on pins SDA=13, SCL=12.");

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
        while (true) { delay(1000); }
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
        Serial.println("WiFi disconnected! Attempting reconnection...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousWifiCheckMillis = currentMillis;
    }
    else if (currentMillis - previousWifiCheckMillis >= wifiCheckInterval) {
        previousWifiCheckMillis = currentMillis;
    }
}

void loop() {
    checkWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
    }
    
    delay(10);
}
