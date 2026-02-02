
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <ESP32Servo.h>

// WiFi Credentials
const char* ssid = "SMARTBOT";
const char* password = "1122334455";

// Motor Control Pins (L298N)
#define MOTOR_A1 16
#define MOTOR_A2 17
#define MOTOR_B1 5
#define MOTOR_B2 18
#define PWM_PIN 32   // Speed control (also used for water sensor)
#define PUMP_PIN 27  // Water pump control

// Ultrasonic Sensor Pins
#define TRIG_PIN 26
#define ECHO_PIN 25

// TCS34725 Color Sensor (I2C)
#define TCS34725_ADDRESS 0x29

// Battery Monitoring
#define BATTERY_PIN 33       // Voltage divider input (4.7k + 1k)
#define WATER_SENSOR_PIN 32  // Analog water sensor input

// Web Server
WebServer server(80);

// Global variables
int currentSpeed = 200;
volatile bool manualMode = true;  // Default to manual mode
volatile bool lastManualCommand = false;
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT = 1000;  // Stop after 1 second of no command

// Ultrasonic sensor
const int OBSTACLE_DISTANCE = 25;  // Stop if obstacle within 50cm
bool obstacleDetected = false;

// Manual mode states
enum ManualState { STOPPED,
                   MOVING_FORWARD,
                   MOVING_BACKWARD,
                   TURNING_LEFT,
                   TURNING_RIGHT };
ManualState currentManualState = STOPPED;
unsigned long turnEndTime = 0;
const unsigned long TURN_DURATION = 200;  // 500ms for left/right turns

// Color detection
enum Color { NO_COLOR,
             RED,
             GREEN,
             BLUE,
             WHITE,
             BLACK };
Color detectedColor = NO_COLOR;
bool colorSensorFound = false;

// Battery monitoring
float batteryVoltage = 0.0;
int batteryPercentage = 0;
const float MAX_BATTERY_VOLTAGE = 12.6;  // 3S LiPo fully charged
const float MIN_BATTERY_VOLTAGE = 9.0;   // 3S LiPo discharged

// Water sensor
int waterSensorValue = 0;
bool waterAvailable = false;
const int WATER_THRESHOLD = 500;  // Above this = water available

// Color sensor readings
uint16_t r = 0, g = 0, b = 0, c = 0;

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize battery monitoring pin
  pinMode(BATTERY_PIN, INPUT);

  // Initialize water sensor pin
  pinMode(WATER_SENSOR_PIN, INPUT);

  // Initialize I2C for TCS34725
  Wire.begin(21, 22);     // SDA=21, SCL=22 on ESP32
  Wire.setClock(100000);  // 100kHz

  // Initialize color sensor
  initColorSensor();

  // Set initial speed
  analogWrite(PWM_PIN, currentSpeed);

  // Ensure pump is off initially
  digitalWrite(PUMP_PIN, LOW);

  // Stop motors initially
  stopMotor();

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Start WiFi Access Point
  Serial.println("Creating Access Point...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Setup server routes
  server.on("/", HTTP_GET, []() {
    if (!handleFileRead("/index.html")) {
      server.send(404, "text/plain", "File not found");
    }
  });

  server.on("/control", HTTP_GET, handleControl);
  server.on("/speed", HTTP_GET, handleSpeed);
  server.on("/pump", HTTP_GET, handlePump);
  server.on("/mode", HTTP_GET, handleMode);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/sensorinfo", HTTP_GET, handleSensorInfo);
  server.on("/distance", HTTP_GET, handleDistance);

  // Serve static files
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "File not found");
    }
  });

  server.begin();
  Serial.println("HTTP server started");
  Serial.println("Starting in MANUAL mode");
}

void loop() {
  server.handleClient();

  if (manualMode) {
    // Check ultrasonic sensor in manual mode
    checkObstacle();

    // Handle manual state machine
    handleManualState();

    // Manual mode: check if command timeout has occurred (only for non-continuous states)
    if (currentManualState != MOVING_FORWARD && currentManualState != MOVING_BACKWARD) {
      if (lastManualCommand && (millis() - lastCommandTime > COMMAND_TIMEOUT)) {
        stopMotor();
        currentManualState = STOPPED;
        lastManualCommand = false;
        Serial.println("Manual mode timeout - stopped motors");
      }
    }
  } else {
    // Auto mode: follow color markers
    autoFollowColors();
  }

  // Update sensor readings every loop
  updateBatteryVoltage();
  updateWaterSensor();

  delay(50);  // Small delay for stability
}

bool initColorSensor() {
  Serial.println("Initializing TCS34725 color sensor...");

  // Check if sensor is present
  Wire.beginTransmission(TCS34725_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("TCS34725 found!");

    // Enable the sensor
    write8(TCS34725_ADDRESS, 0x80 | 0x00, 0x01);  // Power ON
    delay(3);
    write8(TCS34725_ADDRESS, 0x80 | 0x00, 0x03);  // Power ON + RGBC enabled
    delay(3);

    // Set integration time (700ms)
    write8(TCS34725_ADDRESS, 0x80 | 0x01, 0x00);

    // Set gain (1x)
    write8(TCS34725_ADDRESS, 0x80 | 0x0F, 0x00);

    colorSensorFound = true;
    return true;
  }

  Serial.println("TCS34725 not found!");
  colorSensorFound = false;
  return false;
}

void write8(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read8(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)1);
  return Wire.read();
}

uint16_t read16(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (uint8_t)2);
  uint16_t value = Wire.read();
  value |= (Wire.read() << 8);
  return value;
}

bool readColorSensor() {
  if (!colorSensorFound) return false;

  // Read color data from TCS34725
  c = read16(TCS34725_ADDRESS, 0x80 | 0x14);
  r = read16(TCS34725_ADDRESS, 0x80 | 0x16);
  g = read16(TCS34725_ADDRESS, 0x80 | 0x18);
  b = read16(TCS34725_ADDRESS, 0x80 | 0x1A);

  // Clear interrupt (if used)
  write8(TCS34725_ADDRESS, 0x66, 0x00);

  return true;
}

Color detectColor() {
  if (!readColorSensor()) {
    return NO_COLOR;
  }

  // Debug output
  Serial.printf("R: %d, G: %d, B: %d, C: %d\n", r, g, b, c);

  // Simple color detection logic
  // Find the dominant color
  uint16_t maxVal = max(max(r, g), b);

  // If all values are very low, it's black/no color
  if (maxVal < 100) {
    return BLACK;
  }

  // Normalize values for comparison
  uint8_t rNorm = (r * 100) / maxVal;
  uint8_t gNorm = (g * 100) / maxVal;
  uint8_t bNorm = (b * 100) / maxVal;

  // Color detection thresholds (adjust based on your markers)
  if (rNorm > 70 && gNorm < 50 && bNorm < 50) {
    return RED;
  } else if (gNorm > 70 && rNorm < 50 && bNorm < 50) {
    return GREEN;
  } else if (bNorm > 70 && rNorm < 50 && gNorm < 50) {
    return BLUE;
  } else if (c > 1000 && r > 500 && g > 500 && b > 500) {
    return WHITE;
  }

  return NO_COLOR;
}

void autoFollowColors() {
  static unsigned long lastDetectionTime = 0;
  const unsigned long DETECTION_INTERVAL = 300;  // Check every 300ms

  if (millis() - lastDetectionTime > DETECTION_INTERVAL) {
    lastDetectionTime = millis();

    Color color = detectColor();

    if (color != detectedColor) {
      detectedColor = color;

      switch (color) {
        case RED:
          Serial.println("Auto: RED detected - STOP");
          stopMotor();
          break;

        case GREEN:
          Serial.println("Auto: GREEN detected - TURN RIGHT");
          stopMotor();
          delay(100);
          turnRight();
          delay(600);
          stopMotor();
          break;

        case BLUE:
          Serial.println("Auto: BLUE detected - TURN LEFT");
          stopMotor();
          delay(100);
          turnLeft();
          delay(600);
          stopMotor();
          break;

        case WHITE:
          Serial.println("Auto: WHITE detected - MOVE FORWARD");
          moveForward();
          break;

        case NO_COLOR:
        case BLACK:
        default:
          Serial.println("Auto: No color - MOVE FORWARD");
          moveForward();
          break;
      }
    }
    else{
       moveForward();
    }

  }
}

float getDistance() {
  // Clear the trig pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // Set the trig pin HIGH for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm)
  float distance = duration * 0.034 / 2;

  // If reading is out of range, return -1
  if (distance > 400 || distance <= 0) {
    return -1;
  }

  return distance;
}

void checkObstacle() {
  if (!manualMode) return;  // Only check in manual mode

  float distance = getDistance();

  if (distance > 0 && distance < OBSTACLE_DISTANCE) {
    if (!obstacleDetected) {
      Serial.println("Obstacle detected at " + String(distance) + "cm! Stopping.");
      obstacleDetected = true;

      // Stop motors if moving forward
      if (currentManualState == MOVING_FORWARD) {
        stopMotor();
        currentManualState = STOPPED;
      }
    }
  } else {
    obstacleDetected = false;
  }
}

void handleManualState() {
  // Check if turn duration has elapsed
  if ((currentManualState == TURNING_LEFT || currentManualState == TURNING_RIGHT) && millis() >= turnEndTime) {
    stopMotor();
    currentManualState = STOPPED;
    Serial.println("Turn completed, returning to stopped state");
  }

  // Check obstacle for forward movement
  if (currentManualState == MOVING_FORWARD && obstacleDetected) {
    stopMotor();
    currentManualState = STOPPED;
    Serial.println("Forward movement stopped due to obstacle");
  }
}

void updateBatteryVoltage() {
  // Read analog value (0-4095 for ESP32 ADC)
  int rawValue = analogRead(BATTERY_PIN);

  // Calculate actual voltage
   Voltage divider: 4.7k + 1k = 5.7k total
   Ratio = 1k/5.7k = 0.1754
  // ESP32 ADC reference voltage = 3.3V
  float adcVoltage = rawValue * (3.3 / 4095.0);
   batteryVoltage = adcVoltage / 0.1754;  // Divide by voltage divider ratio

  

  // Calculate battery percentage (for 3S LiPo: 9.0V - 12.6V)
  if (batteryVoltage >= MAX_BATTERY_VOLTAGE) {
    batteryPercentage = 100;
  } else if (batteryVoltage <= MIN_BATTERY_VOLTAGE) {
    batteryPercentage = 0;
  } else {
    batteryPercentage = map(batteryVoltage * 100, MIN_BATTERY_VOLTAGE * 100, MAX_BATTERY_VOLTAGE * 100, 0, 100);
  }

  // Limit to 0-100%
  batteryPercentage = constrain(batteryPercentage, 0, 100);
}

void updateWaterSensor() {
  // Read water sensor (using PWM_PIN as analog input for water detection)
  waterSensorValue = analogRead(WATER_SENSOR_PIN);
  waterAvailable = (waterSensorValue > WATER_THRESHOLD);
}

bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";

  String contentType = getContentType(path);
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  return "text/plain";
}

void handleControl() {
  if (!manualMode) {
    server.send(200, "text/plain", "AUTO_MODE");
    return;
  }

  String command = server.arg("cmd");
  Serial.println("Manual command: " + command);

  // Update last command time
  lastManualCommand = true;
  lastCommandTime = millis();

  if (command == "forward") {
    // Only move forward if no obstacle detected
    if (!obstacleDetected) {
      moveForward();
      currentManualState = MOVING_FORWARD;
      Serial.println("Moving forward continuously");
    } else {
      Serial.println("Cannot move forward - obstacle detected!");
      server.send(200, "text/plain", "OBSTACLE");
      return;
    }
  } else if (command == "backward") {
    moveBackward();
    currentManualState = MOVING_BACKWARD;
    Serial.println("Moving backward continuously");
  } else if (command == "left") {
    turnLeft();
    currentManualState = TURNING_LEFT;
    turnEndTime = millis() + TURN_DURATION;
    Serial.println("Turning left for 500ms");
  } else if (command == "right") {
    turnRight();
    currentManualState = TURNING_RIGHT;
    turnEndTime = millis() + TURN_DURATION;
    Serial.println("Turning right for 500ms");
  } else if (command == "stop") {
    stopMotor();
    currentManualState = STOPPED;
    Serial.println("Stopped");
  }

  server.send(200, "text/plain", "OK");
}

void handleSpeed() {
  if (server.hasArg("value")) {
    currentSpeed = server.arg("value").toInt();
    analogWrite(PWM_PIN, currentSpeed);
    Serial.println("Speed set to: " + String(currentSpeed));
  }
  server.send(200, "text/plain", "OK");
}

void handlePump() {
  if (server.hasArg("state")) {
    String state = server.arg("state");

    // Check if water is available before turning pump on
    if (state == "on") {
      if (waterAvailable) {
        digitalWrite(PUMP_PIN, HIGH);
        Serial.println("Water pump turned ON");
        server.send(200, "text/plain", "PUMP_ON");
      } else {
        Serial.println("Cannot turn pump ON - No water detected!");
        server.send(200, "text/plain", "NO_WATER");
      }
    } else if (state == "off") {
      digitalWrite(PUMP_PIN, LOW);
      Serial.println("Water pump turned OFF");
      server.send(200, "text/plain", "PUMP_OFF");
    }
  } else {
    server.send(400, "text/plain", "Missing state parameter");
  }
}

void handleMode() {
  if (server.hasArg("set")) {
    String mode = server.arg("set");
    if (mode == "manual") {
      manualMode = true;
      stopMotor();  // Stop when switching to manual
      currentManualState = STOPPED;
      Serial.println("Switched to MANUAL mode");
    } else if (mode == "auto") {
      manualMode = false;
      stopMotor();  // Stop when switching to auto
      currentManualState = STOPPED;
      Serial.println("Switched to AUTO mode");
    }
  }

  // Return current mode
  String response = manualMode ? "manual" : "auto";
  server.send(200, "text/plain", response);
}

void handleStatus() {
  float distance = getDistance();

  String status = "{";
  status += "\"mode\":\"" + String(manualMode ? "manual" : "auto") + "\",";
  status += "\"speed\":" + String(currentSpeed) + ",";
  status += "\"color\":\"" + getColorName(detectedColor) + "\",";
  status += "\"color_sensor\":" + String(colorSensorFound ? "true" : "false") + ",";
  status += "\"battery_voltage\":" + String(batteryVoltage, 2) + ",";
  status += "\"battery_percentage\":" + String(batteryPercentage) + ",";
  status += "\"water_sensor\":" + String(waterSensorValue) + ",";
  status += "\"water_available\":" + String(waterAvailable ? "true" : "false") + ",";
  status += "\"pump_status\":" + String(digitalRead(PUMP_PIN) == HIGH ? "true" : "false") + ",";
  status += "\"distance\":" + String(distance, 1) + ",";
  status += "\"obstacle_detected\":" + String(obstacleDetected ? "true" : "false") + ",";
  status += "\"manual_state\":\"" + getManualStateName(currentManualState) + "\"";
  status += "}";

  server.send(200, "application/json", status);
}

void handleSensorInfo() {
  String info = "{";
  info += "\"r\":" + String(r) + ",";
  info += "\"g\":" + String(g) + ",";
  info += "\"b\":" + String(b) + ",";
  info += "\"c\":" + String(c);
  info += "}";

  server.send(200, "application/json", info);
}

void handleDistance() {
  float distance = getDistance();
  server.send(200, "text/plain", String(distance, 1));
}

String getColorName(Color color) {
  switch (color) {
    case RED: return "red";
    case GREEN: return "green";
    case BLUE: return "blue";
    case WHITE: return "white";
    case BLACK: return "black";
    default: return "none";
  }
}

String getManualStateName(ManualState state) {
  switch (state) {
    case STOPPED: return "stopped";
    case MOVING_FORWARD: return "moving_forward";
    case MOVING_BACKWARD: return "moving_backward";
    case TURNING_LEFT: return "turning_left";
    case TURNING_RIGHT: return "turning_right";
    default: return "unknown";
  }
}

void moveForward() {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void moveBackward() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void turnLeft() {

  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}

void turnRight() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, HIGH);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void stopMotor() {
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
}