#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <time.h>

// ================= OTA & VERSION =================
String currentVersion = "1.0.041"; // Bumped version for WiFi fix
String versionURL = "https://raw.githubusercontent.com/asfandyaralishah112/Traffic_Sensor_src/main/version.json";

// ================= PROTOTYPES =================
void publishStatus(String status);
void publishTelemetry();
void runCalibration();
void initVL53();
void saveCalibration();
void loadCalibration();
void updateAdaptiveBaseline();
void processFlow();
void updateStatusLED();
void setLED(bool r, bool g, bool b);
void syncTime();
void correctBufferedTimestamps();

// ================= DEVICE ID =================
String DEVICE_UID = "UNCONFIGURED";
String ble_name = "SmartCounter-UNCONFIGURED";

// ================= WIFI & PROVISIONING =================
#include "Provisioning.h"
String wifi_ssid = "";
String wifi_pass = "";
String business_name = "";

// ================= MQTT =================
String mqtt_server = "www.cavlineglobal.com";
int mqtt_port = 0;
String mqtt_user = "Traffic_Sensor";
String mqtt_pass = "randompass";
String topic_events, topic_status, topic_telemetry, topic_command;
bool deviceConfigured = false;
bool timeSynced = false;
unsigned long lastTimeSyncMillis = 0; // For daily re-sync
unsigned long dailySyncOffset = 0;    // Random jitter for fleet de-clustering
bool bleStarted = false; // Flag to track BLE initialization state

// ================= UDP TELEMETRY =================
bool udpStreamEnabled = true; // Flag to easily enable/disable UDP stream
const char* udp_server = "192.168.3.10";
const int udp_port = 5005;
WiFiUDP udpClient;
uint32_t udpPacketsSent = 0;

// ================= GPIO =================
#define SDA_PIN 6
#define SCL_PIN 7
#define PIN_LPN 0
#define PIN_RST 8
#define PIN_CALIBRATION 9

#define LED_RED   23
#define LED_GREEN 22
#define LED_BLUE  21

// ================= SYSTEM STATES =================
enum SystemState {
  BOOT,
  WIFI_CONNECT,
  MQTT_CONNECT,
  NORMAL_OPERATION,
  CALIBRATION_MODE,
  OTA_UPDATE,
  PROVISIONING_AP, // New State
  ERROR_STATE
};

SystemState currentState = BOOT;

// ================= CALIBRATION DATA =================
struct CalibrationData {
  uint8_t zone_mask[64];
  uint16_t floor_distance[64];
  uint16_t noise[64]; // v1.0.028: Adaptive noise floor
};

CalibrationData calData;
Preferences preferences; // Calibration preferences
Preferences devicePrefs; // Factory provisioning preferences
bool sensorInitialized = false;

// ================= REGION OCCUPANCY (v1.0.027: Calculated in processFlow) =================
int activePixels = 0;

// ================= TRAJECTORY TRACKING (v1.0.041: Exact Parity with Plot1.py) =================
#define MIN_ACTIVE_PIXELS 1
#define DOOR_LINE 3.5
#define MAX_CONSECUTIVE_MISSES 3

float firstCentroidY = 0;
float lastCentroidY = 0;
float lastVelocityY = 0;       // v1.0.041: For prediction
int trajectoryLength = 0;      // v1.0.041: Replaces motionFrameCount for length check
int consecutiveMisses = 0;     // v1.0.041: Robustness against dropouts

bool reachedEntranceZone = false; 
bool reachedExitZone = false;     
bool trackingActive = false;

volatile bool otaRequested = false;
unsigned long lastTelemetry = 0; // v1.0.027: unified timing

// ================= EVENT BUFFER =================
struct CounterEvent {
  String direction;
  time_t timestamp;
  bool timestampValid;
};

#define MAX_BUFFERED_EVENTS 100
CounterEvent eventBuffer[MAX_BUFFERED_EVENTS];
int eventCount = 0;

// ================= VL53 =================
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;
uint16_t filteredDist[64]; // v1.0.026: Filtered distances based on target status

WiFiClientSecure wifiClientSecure;
WiFiClient wifiClient; 
PubSubClient mqttClient(wifiClient);

uint32_t lastPrint = 0;
uint16_t frameCount = 0;

// =====================================================
// LED CONTROL
// =====================================================
void setLED(bool r, bool g, bool b) {
  // Common Anode: LOW = ON, HIGH = OFF
  digitalWrite(LED_RED, r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
}

void updateStatusLED() {
  if (currentState == CALIBRATION_MODE) {
    // Red blinking
    static bool blink = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      blink = !blink;
      setLED(blink, false, false);
      lastBlink = millis();
    }
  } else if (currentState == PROVISIONING_AP) {
    // Blue Blinking for AP Mode
    static bool blink = false;
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 500) {
      blink = !blink;
      setLED(false, false, blink); // Blue blink
      lastBlink = millis();
    }
  } else if (!sensorInitialized || currentState == ERROR_STATE) {
    setLED(true, false, false); // Red ON
  } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    setLED(false, true, false); // Green ON
  } else if (WiFi.status() == WL_CONNECTED) {
    setLED(true, true, false); // Orange (Red+Green)
  } else {
    setLED(true, false, false); // Red ON
  }
}

// =====================================================
// NVS PERSISTENCE
// =====================================================
void saveCalibration() {
  preferences.begin("counter-cal", false);
  preferences.putBytes("zone_mask", calData.zone_mask, 64);
  preferences.putBytes("floor_dist", calData.floor_distance, 128); // 64 * 2
  preferences.putBytes("noise", calData.noise, 128); // 64 * 2
  preferences.end();
  Serial.println("Calibration saved to NVS");
}

void loadCalibration() {
  preferences.begin("counter-cal", true);
  if (preferences.isKey("zone_mask") && preferences.isKey("noise")) {
    preferences.getBytes("zone_mask", calData.zone_mask, 64);
    preferences.getBytes("floor_dist", calData.floor_distance, 128);
    preferences.getBytes("noise", calData.noise, 128);
    Serial.println("Calibration loaded from NVS");
  } else {
    Serial.println("No calibration found in NVS, using defaults");
    memset(calData.zone_mask, 0, 64); // Default: all zones valid
    for(int i=0; i<64; i++) {
        calData.floor_distance[i] = 4000;
        calData.noise[i] = 50; 
    }
  }
  preferences.end();
}

void updateDynamicNames() {
  // BLE name logic: EXACTLY the business_name if available
  if (business_name.length() > 0) {
    ble_name = business_name;
  } else {
    ble_name = "SmartCounter-UNCONFIGURED";
  }
}

void updateMqttTopics() {
  String baseTopic = "cavline/traffic_sensor/" + DEVICE_UID;
  topic_events    = baseTopic + "/events";
  topic_status    = baseTopic + "/status";
  topic_telemetry = baseTopic + "/telemetry";
  topic_command   = baseTopic + "/command";
  
  Serial.println("MQTT Topics Generated:");
  Serial.println(" - Status: " + topic_status);
  Serial.println(" - Events: " + topic_events);
}

void loadDeviceConfig() {
  devicePrefs.begin("device-config", true);
  deviceConfigured = devicePrefs.getBool("configured", false);
  if (deviceConfigured) {
    DEVICE_UID = devicePrefs.getString("uid", "UNCONFIGURED");
    mqtt_server = devicePrefs.getString("mqtt_server", mqtt_server);
    mqtt_port = devicePrefs.getUInt("mqtt_port", mqtt_port);
    mqtt_user = devicePrefs.getString("mqtt_user", mqtt_user);
    mqtt_pass = devicePrefs.getString("mqtt_pass", mqtt_pass);
    Serial.println("Device configuration loaded from NVS");
  } else {
    Serial.println("No factory device configuration found.");
    DEVICE_UID = "UNCONFIGURED";
  }
  devicePrefs.end();
  
  // Also load WiFi/Business context to form the BLE name
  if (isWiFiConfigured()) {
    business_name = getStoredBusiness();
  }
  
  updateDynamicNames();
  updateMqttTopics();
  Serial.print("Current UID: "); Serial.println(DEVICE_UID);
  Serial.print("Current BLE Name: "); Serial.println(ble_name);
}

// =====================================================
// BLE ADVERTISING (ADV-ONLY)
// =====================================================
void startBLEAdvertising() {
  if (!deviceConfigured || bleStarted) return;

  Serial.println("Initializing BLE Advertising...");
  BLEDevice::init(ble_name.c_str());
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  
  // Requirement 3: 500ms interval (Units of 0.625ms: 500 / 0.625 = 800)
  pAdvertising->setMinInterval(800); 
  pAdvertising->setMaxInterval(800);
  
  // Requirement 4: Advertising-only (Non-connectable)
  // Note: BLE_GAP_CONN_MODE_NON_CONN or similar behavior is default when no services added
  // but we can be explicit if the library supports it directly.
  
  pAdvertising->setScanResponse(false);
  pAdvertising->start();
  
  bleStarted = true;
  Serial.println("BLE Advertising Started: " + ble_name);
}

// =====================================================
// NTP TIME SYNC
// =====================================================
void syncTime() {
  Serial.println("Configuring NTP...");
  // Requirement 2: pool.ntp.org and time.google.com
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  
  unsigned long start = millis();
  while (millis() - start < 10000) {
    time_t now = time(nullptr);
    // Requirement 2: Valid time check (> 1700000000)
    if (now > 1700000000) {
      timeSynced = true;
      lastTimeSyncMillis = millis(); // Track sync time for daily check
      Serial.print("Time synchronized: "); Serial.println(now);
      publishStatus("time_synced");
      correctBufferedTimestamps();
      return;
    }
    delay(500);
    Serial.print(".");
    updateStatusLED();
  }
  Serial.println("\nNTP Timeout. Proceeding without sync.");
}

void correctBufferedTimestamps() {
  if (!timeSynced) return;
  
  time_t currentEpoch = time(nullptr);
  unsigned long currentMillis = millis();
  
  for (int i = 0; i < eventCount; i++) {
    if (!eventBuffer[i].timestampValid) {
      // Requirement 3: millis() Overflow Safety (49-Day Uptime Issue)
      unsigned long eventMillis = (unsigned long)eventBuffer[i].timestamp;
      uint32_t ageMs = (uint32_t)(currentMillis - eventMillis);
      time_t epochAtEvent = currentEpoch - (ageMs / 1000);
      eventBuffer[i].timestamp = epochAtEvent;
      eventBuffer[i].timestampValid = true;
    }
  }
  Serial.println("Buffered events corrected to UTC.");
}

void handleSerialProvisioning() {
  static String inputBuffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, inputBuffer);
      
      if (!error) {
        String cmd = doc["cmd"] | "";
        if (cmd == "write_config") {
          bool force = doc["force"] | false;
          if (deviceConfigured && !force) {
            Serial.println("ERROR: Already configured. Use \"force\":true to overwrite.");
          } else {
            devicePrefs.begin("device-config", false);
            devicePrefs.putString("uid", doc["uid"] | "UNCONFIGURED");
            devicePrefs.putString("mqtt_server", doc["mqtt_server"] | "www.cavlineglobal.com");
            
            uint32_t port = 1883;
            if (doc["mqtt_port"].is<uint32_t>()) {
                port = doc["mqtt_port"].as<uint32_t>();
            }
            devicePrefs.putUInt("mqtt_port", port);

            devicePrefs.putString("mqtt_user", doc["mqtt_user"] | "Traffic_Sensor");
            devicePrefs.putString("mqtt_pass", doc["mqtt_pass"] | "admin");
            devicePrefs.putBool("configured", true);
            devicePrefs.end();
            
            Serial.println("OK"); // MUST BE FIRST LINE FOR PYTHON TOOL
            Serial.println("CONFIG_SAVED");
            delay(500);
            ESP.restart();
          }
        }
      } else if (inputBuffer.length() > 0) {
        Serial.println("ERROR");
      }
      inputBuffer = "";
    } else if (c >= 32) {
      inputBuffer += c;
    }
  }
}
// =====================================================
// CALIBRATION MODE
// =====================================================
void runCalibration() {
  Serial.println("Starting Calibration...");
  publishStatus("calibration_started");
  currentState = CALIBRATION_MODE;
  
  // Step 1: Stabilization (10 seconds)
  Serial.println("Step 1: Stabilization...");
  unsigned long start = millis();
  while (millis() - start < 10000) {
    if (myImager.isDataReady()) myImager.getRangingData(&measurementData);
    updateStatusLED();
    delay(10);
  }

  // Step 2: Zone Analysis (15 seconds)
  Serial.println("Step 2: Zone Analysis...");
  float sum_dist[64] = {0};
  float sum_sq_dist[64] = {0};
  int counts[64] = {0};
  
  start = millis();
  while (millis() - start < 15000) {
    if (myImager.isDataReady()) {
      myImager.getRangingData(&measurementData);
      for (int i = 0; i < 64; i++) {
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
          sum_dist[i] += measurementData.distance_mm[i];
          sum_sq_dist[i] += pow(measurementData.distance_mm[i], 2);
          counts[i]++;
        }
      }
    }
    updateStatusLED();
    delay(10);
  }

  for (int i = 0; i < 64; i++) {
    if (counts[i] > 0) {
      float mean = sum_dist[i] / counts[i];
      float variance = (sum_sq_dist[i] / counts[i]) - pow(mean, 2);
      
      if (variance > 1000) { // arbitrary threshold for noise
        calData.zone_mask[i] = 2; // UNUSABLE
      } else if (mean < 500) { // arbitrary threshold for blocked
        calData.zone_mask[i] = 1; // STATIC_BLOCKED
      } else {
        calData.zone_mask[i] = 0; // VALID_WALK (0 is used as valid here)
        calData.floor_distance[i] = (uint16_t)mean;
      }
    } else {
      calData.zone_mask[i] = 2; // UNUSABLE
    }
  }

  // Step 3: Floor Distance Measurement (15 seconds)
  Serial.println("Step 3: Floor Distance Measurement...");
  start = millis();
  while (millis() - start < 15000) {
    if (myImager.isDataReady()) {
      myImager.getRangingData(&measurementData);
      for (int i = 0; i < 64; i++) {
        if (calData.zone_mask[i] == 0 && (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9)) {
          // Running average for floor distance
          calData.floor_distance[i] = (calData.floor_distance[i] * 0.9) + (measurementData.distance_mm[i] * 0.1);
        }
      }
    }
    updateStatusLED();
    delay(10);
  }

  // Step 4: Noise Calculation
  Serial.println("Step 4: Noise Calculation...");
  for (int i = 0; i < 64; i++) {
    if (calData.zone_mask[i] == 0 && counts[i] > 1) {
      float mean = sum_dist[i] / counts[i];
      float variance = (sum_sq_dist[i] / counts[i]) - pow(mean, 2);
      calData.noise[i] = (uint16_t)sqrt(max(0.0f, variance));
      if (calData.noise[i] < 20) calData.noise[i] = 20; // Floor at 20mm
    } else {
      calData.noise[i] = 50; // Default
    }
  }

  saveCalibration();
  Serial.println("Calibration Complete!");
  publishStatus("calibration_completed");
  currentState = NORMAL_OPERATION;
  publishStatus("returning_to_normal");
}

void checkCalibrationTrigger() {
  static unsigned long lowStart = 0;
  if (digitalRead(PIN_CALIBRATION) == LOW) {
    if (lowStart == 0) lowStart = millis();
    if (millis() - lowStart > 5000) {
      runCalibration();
      lowStart = 0;
    }
  } else {
    lowStart = 0;
  }
}
// =====================================================
// DETECTION LOGIC
// =====================================================

// v1.0.041: Dilation Helper
void dilate(bool input[64], bool output[64]) {
  memset(output, 0, 64 * sizeof(bool));
  for (int i = 0; i < 64; i++) {
    if (input[i]) {
      int y = i / 8;
      int x = i % 8;
      // Mark self and neighbors
      for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
          int ny = y + dy;
          int nx = x + dx;
          if (ny >= 0 && ny < 8 && nx >= 0 && nx < 8) {
            output[ny * 8 + nx] = true;
          }
        }
      }
    }
  }
}

void recordEvent(String direction) {
  time_t t;
  bool valid;
  
  // Requirement 5: Handle synced vs non-synced time
  if (timeSynced) {
    t = time(nullptr);
    valid = true;
  } else {
    t = (time_t)millis();
    valid = false;
  }

  if (eventCount < MAX_BUFFERED_EVENTS) {
    eventBuffer[eventCount].direction = direction;
    eventBuffer[eventCount].timestamp = t;
    eventBuffer[eventCount].timestampValid = valid;
    eventCount++;
    Serial.println("Event Recorded: " + direction + (valid ? " [UTC]" : " [RELATIVE]"));
  } else {
    for (int i = 0; i < MAX_BUFFERED_EVENTS - 1; i++) {
      eventBuffer[i] = eventBuffer[i+1];
    }
    eventBuffer[MAX_BUFFERED_EVENTS-1].direction = direction;
    eventBuffer[MAX_BUFFERED_EVENTS-1].timestamp = t;
    eventBuffer[MAX_BUFFERED_EVENTS-1].timestampValid = valid;
    Serial.println("Event Recorded (Buffer Full): " + direction + (valid ? " [UTC]" : " [RELATIVE]"));
  }
}

void processFlow() {
  bool occupied[64] = {0};
  bool dilated[64] = {0};

  // 1. Initial Thresholding
  for (int i = 0; i < 64; i++) {
    // Skip first two rows (consistency with Plot1.py)
    if (i < 16) continue; 
    
    if (calData.zone_mask[i] == 0) { // VALID_WALK_ZONE
      float diff = (float)calData.floor_distance[i] - (float)filteredDist[i];
      
      float threshold = (float)calData.noise[i] * 2.0f; // v1.0.041: Matched Plot1.py (2.0)

      if (diff > threshold) {
        occupied[i] = true;
      }
    }
  }

  // 2. Dilation
  dilate(occupied, dilated);

  // v1.0.042: Match Python behavior: ignore first two rows after dilation
  for (int i = 0; i < 16; i++) {
    dilated[i] = false;
  }

  // 3. Centroid & Active Count
  float sumY = 0;
  int activeCount = 0;
  for (int i = 0; i < 64; i++) {
    if (dilated[i]) {
      int row = i / 8;
      sumY += (float)row;
      activeCount++;
    }
  }

  // 4. Tracking Logic
  if (activeCount >= MIN_ACTIVE_PIXELS) {
    float centroidY = sumY / (float)activeCount;
    
    if (trackingActive) {
      // Smooth and Velocity
      centroidY = 0.7f * lastCentroidY + 0.3f * centroidY;
      lastVelocityY = centroidY - lastCentroidY;
    } else {
      // Start Tracking
      trackingActive = true;
      firstCentroidY = centroidY;
      lastVelocityY = 0;
      trajectoryLength = 0;
      reachedEntranceZone = false; 
      reachedExitZone = false;
      Serial.println("Motion Started");
    }

    lastCentroidY = centroidY;
    consecutiveMisses = 0;
    trajectoryLength++;

    // Traversal Update
    if (centroidY > DOOR_LINE + 1.0f) reachedEntranceZone = true;
    if (centroidY < DOOR_LINE - 1.0f) reachedExitZone = true;

  } else {
    // Inactive Frame
    consecutiveMisses++;

    if (trackingActive && consecutiveMisses <= MAX_CONSECUTIVE_MISSES) {
      // Prediction Mode
      float predicted = lastCentroidY + lastVelocityY;
      
      // v1.0.042: Clamp to valid VL53L5CX row range
      if (predicted < 0.0f) predicted = 0.0f;
      if (predicted > 7.0f) predicted = 7.0f;

      lastCentroidY = predicted;
      
      // Update trajectory stats based on prediction
      trajectoryLength++;
      if (predicted > DOOR_LINE + 1.0f) reachedEntranceZone = true;
      if (predicted < DOOR_LINE - 1.0f) reachedExitZone = true;
      
      Serial.println("Predicting..."); // Debug
    } 
    else {
      // Stop / Tracking Lost
      if (trackingActive) {
         // Traversal Check
         if (trajectoryLength >= 5) { // Match len(trajectory) > 5
            if (firstCentroidY > DOOR_LINE && lastCentroidY < DOOR_LINE && reachedExitZone) {
               recordEvent("IN");
            } else if (firstCentroidY < DOOR_LINE && lastCentroidY > DOOR_LINE && reachedEntranceZone) {
               recordEvent("OUT");
            }
         }
         Serial.println("Motion Ended");
      }
      trackingActive = false;
      trajectoryLength = 0;
      consecutiveMisses = 0;
      lastVelocityY = 0;
    }
  }
  
  activePixels = activeCount; // Update global for baseline logic
}

// =====================================================
// ADAPTIVE CALIBRATION
// =====================================================
void updateAdaptiveBaseline() {
  if (currentState != NORMAL_OPERATION) return;
  if (trackingActive) return;
  if (activePixels > 0) return;

  const float BASELINE_ALPHA = 0.001;
  const float NOISE_ALPHA = 0.01;

  for (int i = 0; i < 64; i++) {
    if (calData.zone_mask[i] == 0) { // VALID_WALK_ZONE
      uint16_t currentDist = filteredDist[i];
      
      // Plot1.py logic: Update baseline and noise when NOT occupied
      // We already checked activePixels == 0, but we can be more specific per zone
      
      float diff = (float)calData.floor_distance[i] - (float)currentDist;
      float abs_err = abs(diff);
      
      // Update Floor Distance (Baseline)
      float newFloor = ((1.0f - BASELINE_ALPHA) * (float)calData.floor_distance[i]) + (BASELINE_ALPHA * (float)currentDist);
      calData.floor_distance[i] = (uint16_t)newFloor;
      
      // Update Noise Floor
      float newNoise = ((1.0f - NOISE_ALPHA) * (float)calData.noise[i]) + (NOISE_ALPHA * abs_err);
      calData.noise[i] = (uint16_t)newNoise;
      
      // Clamp noise floor to avoid over-sensitivity
      if (calData.noise[i] < 20) calData.noise[i] = 20;
    }
  }
}

// =====================================================
// MQTT FUNCTIONS
// =====================================================
void publishStatus(String status) {
  if (!deviceConfigured || DEVICE_UID == "UNCONFIGURED") return;
  if (!mqttClient.connected()) return;
  
  StaticJsonDocument<256> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["status"] = status;
  doc["version"] = currentVersion;
  doc["udp_sent"] = udpPacketsSent;
  doc["business"] = business_name; 
  
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(topic_status.c_str(), buffer);
  mqttClient.loop(); // Flush status message
  Serial.println("Status Published: " + status + " | UDP Sent: " + String(udpPacketsSent));
}

void publishTelemetry() {
  if (!deviceConfigured || DEVICE_UID == "UNCONFIGURED") return;
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry < 50) return; // 20 FPS max for telemetry
  lastTelemetry = millis();

  StaticJsonDocument<1024> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["state"] = trackingActive ? 1 : 0;
  
  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < 64; i++) {
    zones.add(filteredDist[i]);
  }
  
  // Requirement 8: Add UTC timestamp to telemetry if synced
  if (timeSynced) doc["timestamp"] = (uint32_t)time(nullptr);
  
  char buffer[1024];
  size_t len = serializeJson(doc, buffer);
  
  // Publish to MQTT
  if (mqttClient.connected()) {
    mqttClient.publish(topic_telemetry.c_str(), buffer);
  }

  if (udpStreamEnabled) {
    udpClient.beginPacket(udp_server, udp_port);
    udpClient.write((const uint8_t*)buffer, len);
    if (udpClient.endPacket()) {
      udpPacketsSent++;
    } else {
      Serial.println("UDP Packet Fail");
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error) {
    Serial.println("JSON parse failed");
    return;
  }

  String command = doc["command"].as<String>();
  if (command == "calibrate") {
    if (currentState == CALIBRATION_MODE) {
      Serial.println("Already in calibration mode, ignoring.");
      return;
    }
    runCalibration();
  } else if (command == "update") {
    if (currentState == CALIBRATION_MODE || currentState == OTA_UPDATE) {
      Serial.println("Busy, ignoring update command.");
      return;
    }
    otaRequested = true;
    Serial.println("OTA Update Requested via MQTT");
  }
}

void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  static unsigned long lastConnectAttempt = 0;
  if (millis() - lastConnectAttempt > 5000) {
    lastConnectAttempt = millis();
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(
          DEVICE_UID.c_str(), 
          mqtt_user.c_str(), 
          mqtt_pass.c_str(), 
          topic_status.c_str(), 
          1, 
          true, 
          "{\"status\":\"offline\"}"
        )) {
      Serial.println("connected");
      
      // Publish online status immediately
      publishStatus("online");
      
      // Subscribe to command topic
      mqttClient.subscribe(topic_command.c_str());
      Serial.println("Subscribed to: " + topic_command);
      
      // Requirement 8: Ensure BLE is running after MQTT connection
      startBLEAdvertising();

      currentState = NORMAL_OPERATION;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

void publishBufferedEvents() {
  if (!deviceConfigured || DEVICE_UID == "UNCONFIGURED") return;
  if (!mqttClient.connected()) return;
  
  // Requirement 7: Block publishing until time is synchronized
  if (!timeSynced) return; 

  while (eventCount > 0) {
    CounterEvent ev = eventBuffer[0];
    
    StaticJsonDocument<256> doc;
    doc["device_uid"] = DEVICE_UID;
    doc["ble_name"] = ble_name;
    doc["timestamp"] = ev.timestamp;
    doc["direction"] = ev.direction;
    doc["business"] = business_name; // v1.0.042

    char buffer[256];
    serializeJson(doc, buffer);

    if (mqttClient.publish(topic_events.c_str(), buffer)) {
      Serial.println("Published: " + ev.direction);
      // Remove from buffer (shift)
      for (int i = 0; i < eventCount - 1; i++) {
        eventBuffer[i] = eventBuffer[i+1];
      }
      eventCount--;
    } else {
      Serial.println("Publish failed, keeping in buffer");
      break;
    }
  }
}

// =====================================================
// OTA PROGRESS CALLBACK
// =====================================================
void update_progress(int cur, int total) {
  static bool led_state = false;
  static unsigned long last_blink = 0;
  if (millis() - last_blink > 150) {
    led_state = !led_state;
    // Blink Blue: LOW = ON, HIGH = OFF
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, led_state ? LOW : HIGH);
    last_blink = millis();
  }
}

// =====================================================
// OTA CHECK
// =====================================================
void checkForOTA()
{
  if (currentState == OTA_UPDATE) return; 
  SystemState previousState = currentState;
  currentState = OTA_UPDATE;

  Serial.println("Checking for OTA update...");
  publishStatus("ota_check_started");

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, versionURL);

  int httpCode = http.GET();

  if (httpCode == 200)
  {
    String payload = http.getString();

    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload))
    {
      Serial.println("JSON parse failed");
      publishStatus("returning_to_normal");
      currentState = previousState;
      http.end();
      return;
    }

    String newVersion = doc["version"].as<String>();
    String firmwareURL = doc["firmware"].as<String>();

    Serial.println("Current version: " + currentVersion);
    Serial.println("Available version: " + newVersion);

    if (newVersion != currentVersion)
    {
      Serial.println("New firmware detected. Updating...");
      publishStatus("ota_update_found");
      publishStatus("ota_downloading");

      WiFiClientSecure updateClient;
      updateClient.setInsecure();

      httpUpdate.onProgress(update_progress); // v1.0.032: Blue LED Blink
      t_httpUpdate_return ret =
        httpUpdate.update(updateClient, firmwareURL);

      if (ret != HTTP_UPDATE_OK) {
        Serial.println("Update failed");
        publishStatus("returning_to_normal");
        currentState = previousState;
      } else {
        publishStatus("ota_completed");
        publishStatus("ota_update_finished");
        // Device reboots
      }
    }
    else
    {
      Serial.println("Already latest version.");
      publishStatus("ota_no_update");
      publishStatus("ota_update_finished");
      publishStatus("returning_to_normal");
      currentState = previousState;
    }
  }
  else
  {
    Serial.printf("Version download failed: %d\n", httpCode);
    publishStatus("ota_update_finished");
    publishStatus("returning_to_normal");
    currentState = previousState;
  }

  http.end();
}

// =====================================================
// VL53 INIT
// =====================================================
void initVL53()
{
  pinMode(PIN_LPN, OUTPUT);
  pinMode(PIN_RST, OUTPUT);

  digitalWrite(PIN_LPN, LOW);
  digitalWrite(PIN_RST, HIGH);
  delay(10);

  digitalWrite(PIN_LPN, HIGH);
  delay(10);

  digitalWrite(PIN_RST, LOW);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("VL53L5CX init...");

  if (!myImager.begin())
  {
    Serial.println("Sensor not found");
    sensorInitialized = false;
    currentState = ERROR_STATE;
    publishStatus("error_state_entered");
    return;
  }

  sensorInitialized = true;
  myImager.setResolution(8 * 8); // Reverted to 8x8 v1.0.025
  myImager.setRangingFrequency(15); // Max for 8x8 is 15Hz
  myImager.setIntegrationTime(10); // v1.0.040: Improved sensitivity for shorter/small targets
  
  // Optimization for faster recovery/floor detection (v1.0.022)
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS); 
  myImager.setTargetOrder(SF_VL53L5CX_TARGET_ORDER::STRONGEST);
  myImager.setSharpenerPercent(5);       // Low sharpener to help distinguish targets
  
  myImager.startRanging();

  Serial.println("Sensor ready");
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(100);
  
  // OBJECTIVE 1: 5-second Factory Provisioning Window (SILENT for Python Tool)
  unsigned long factoryStart = millis();
  while (millis() - factoryStart < 5000) {
    handleSerialProvisioning();
    delay(5);
  }

  // OBJECTIVE 2 & 4: Load configurations from NVS
  loadDeviceConfig();

  // Requirement: initialize NTP jitter (0-60 minutes)
  dailySyncOffset = random(0, 3600000);
  
  // Initialize GPIOs
  pinMode(PIN_CALIBRATION, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setLED(false, false, false); // All OFF (High)

  // WiFi Configuration logic
  if (isWiFiConfigured()) {
    Serial.println("WiFi Config Found. Configuring...");
    wifi_ssid = getStoredSSID();
    wifi_pass = getStoredPass();
    business_name = getStoredBusiness();
    
    Serial.print("Connecting to: "); Serial.println(wifi_ssid);
    Serial.print("Business: "); Serial.println(business_name);
    
    Serial.println("Connecting WiFi...");
    currentState = WIFI_CONNECT;
    
    // v1.0.044: Full WiFi Reset for ESP32-C6 (Required after AP Mode)
    WiFi.disconnect(true, true);
    delay(500);

    WiFi.mode(WIFI_OFF);
    delay(500);

    WiFi.mode(WIFI_STA);
    delay(500);

    WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false); 

    // Debug Mode Check
    Serial.println("WiFi Mode After Reset:");
    Serial.println(WiFi.getMode()); // Should be 1 (WIFI_STA)
    
    unsigned long startConnect = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      updateStatusLED();
      
      // v1.0.042: Fallback to AP if connection fails > 60s
      if (millis() - startConnect > 60000) {
        Serial.println("\nConnection Failed. Falling back to AP Mode.");
        currentState = PROVISIONING_AP;
        setupProvisioning(DEVICE_UID);
        return; // Exit setup loop for WiFi
      }
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi connected");
      Serial.println(WiFi.localIP());
      
      // Requirement 9: WiFi -> NTP -> MQTT
      syncTime();

      // OTA check first
      checkForOTA();
      
      // Load calibration from NVS
      loadCalibration();
      
      // Start sensor after OTA
      initVL53();
      
      // Setup MQTT
      mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
      mqttClient.setCallback(mqttCallback);
      mqttClient.setBufferSize(1024);
      
      // Initialize UDP
      udpClient.begin(udp_port);
      Serial.printf("UDP Initialized on port %d\n", udp_port);
      
      // Start BLE Advertising
      startBLEAdvertising();

      currentState = NORMAL_OPERATION;
    }
  } else {
    Serial.println("No Config Found. Starting Provisioning Mode...");
    currentState = PROVISIONING_AP;
    setupProvisioning(DEVICE_UID);
  }
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleSerialProvisioning();
  // v1.0.042: PROVISIONING LOOP
  if (currentState == PROVISIONING_AP) {
    loopProvisioning();
    updateStatusLED();
    delay(5);
    return; // Block other logic
  }

  // Handle Networking
  static bool wasConnected = false;
  bool nowConnected = (WiFi.status() == WL_CONNECTED);

  if (nowConnected && !wasConnected) {
    Serial.println("WiFi reconnected, syncing time...");
    syncTime();
  }
  wasConnected = nowConnected;

  const unsigned long DAILY_SYNC_INTERVAL = 86400000UL; // 24 hours
  // Requirement: Add random offset to prevent sync clustering in large fleets
  if (nowConnected && (millis() - lastTimeSyncMillis > (DAILY_SYNC_INTERVAL + dailySyncOffset))) {
    Serial.println("Daily time sync...");
    syncTime();
  }

  if (!nowConnected) {
    currentState = WIFI_CONNECT;
    // We could add reconnection fallback logic here too if needed, 
    // but typically we stay in loop trying to reconnect.
    // The requirement was "If ESP cannot connect to stored WiFi within 60 seconds... return to AP mode".
    // This is handled in Setup. If it loses connection mid-operation, it usually tries to reconnect.
    // Adding 60s fallback here would be complex as it might trigger on transient loss.
    // User requirement specifically mentioned "Boot Logic" and "Connection Failure Handling" under constraints.
    // I implemented it in Setup.
  } else if (!mqttClient.connected()) {
    currentState = MQTT_CONNECT;
    mqttReconnect();
  } else {
    mqttClient.loop();
    publishBufferedEvents();
    publishTelemetry();
  }

  updateStatusLED();
  checkCalibrationTrigger();

  // Async OTA trigger
  if (otaRequested && currentState != CALIBRATION_MODE && currentState != OTA_UPDATE) {
    otaRequested = false;
    checkForOTA();
  }

  if (currentState != CALIBRATION_MODE && currentState != PROVISIONING_AP) {
    if (myImager.isDataReady())
    {
      myImager.getRangingData(&measurementData);
      
      // v1.0.026: Populating filtered distance layer
      for (int i = 0; i < 64; i++) {
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
          filteredDist[i] = measurementData.distance_mm[i];
        } else {
          filteredDist[i] = 4000; // Treat as FAR distance
        }
      }

      processFlow();
      updateAdaptiveBaseline();
      
      publishTelemetry(); // Share filtered grid over UDP

      frameCount++;
      if (millis() - lastPrint > 1000)
      {
        Serial.print("FPS:");
        Serial.println(frameCount);
        frameCount = 0;
        lastPrint = millis();
      }
    }
  }
  
  delay(5); // Small delay to prevent watchdog issues
}
