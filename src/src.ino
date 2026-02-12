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

// ================= OTA & VERSION =================
String currentVersion = "1.0.033";
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

// ================= DEVICE ID =================
const char* DEVICE_UID = "ESP32C6_COUNTER_001";
const char* BLE_BROADCAST_NAME = "SmartCounter_001";

// ================= WIFI =================
const char* ssid = "EncryptedAir";
const char* password = "BlueSky@786";

// ================= MQTT =================
const char* mqtt_server = "192.168.3.10";
const int mqtt_port = 1883;
const char* mqtt_user = "Traffic_Sensor";
const char* mqtt_pass = "admin";
const char* mqtt_topic = "door/counter/events";

// ================= UDP TELEMETRY =================
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
Preferences preferences;
bool sensorInitialized = false;

// ================= REGION OCCUPANCY (v1.0.027: Calculated in processFlow) =================
int activePixels = 0;

// ================= TRAJECTORY TRACKING (v1.0.032: Exact Parity with Plot1.py) =================
#define MIN_ACTIVE_PIXELS 3
#define DOOR_LINE 3.5
#define MIN_MOTION_FRAMES 5

float firstCentroidY = 0;
float lastCentroidY = 0;
int motionFrameCount = 0;
bool reachedEntranceZone = false; // v1.0.032: Traversal confirmation
bool reachedExitZone = false;     // v1.0.032: Traversal confirmation
bool trackingActive = false;

volatile bool otaRequested = false;
unsigned long lastTelemetry = 0; // v1.0.027: unified timing

// ================= EVENT BUFFER =================
struct CounterEvent {
  String direction;
  unsigned long timestamp;
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
// updateRegionOccupancy is removed in v1.0.027 as centroid logic handles occupancy

void recordEvent(String direction) {
  if (eventCount < MAX_BUFFERED_EVENTS) {
    eventBuffer[eventCount].direction = direction;
    eventBuffer[eventCount].timestamp = millis();
    eventCount++;
    Serial.println("Event Recorded: " + direction);
  } else {
    for (int i = 0; i < MAX_BUFFERED_EVENTS - 1; i++) {
      eventBuffer[i] = eventBuffer[i+1];
    }
    eventBuffer[MAX_BUFFERED_EVENTS-1].direction = direction;
    eventBuffer[MAX_BUFFERED_EVENTS-1].timestamp = millis();
    Serial.println("Event Recorded (Buffer Full): " + direction);
  }
}

void processFlow() {
  int activeCount = 0;
  float sumY = 0;
  
  for (int i = 0; i < 64; i++) {
    // Skip first two rows (v1.0.027: consistency with Plot1.py)
    if (i < 16) continue; 
    
    if (calData.zone_mask[i] == 0) { // VALID_WALK_ZONE
      // Noise-Adaptive Thresholding (v1.0.028: Match Plot1.py)
      float diff = (float)calData.floor_distance[i] - (float)filteredDist[i];
      float threshold = (float)calData.noise[i] * 3.0f; // v1.0.033: Reduced from 4.0 for sensitivity

      if (diff > threshold) {
        int row = i / 8;
        sumY += (float)row;
        activeCount++;
      }
    }
  }

  if (activeCount >= MIN_ACTIVE_PIXELS) {
    float centroidY = sumY / (float)activeCount;
    
    if (!trackingActive) {
      trackingActive = true;
      firstCentroidY = centroidY;
      motionFrameCount = 0;
      reachedEntranceZone = false; // Reset flags (v1.0.032)
      reachedExitZone = false;
      Serial.println("Motion Started");
    }

    // Traversal Confirmation (Hysteresis v1.0.032)
    if (centroidY > DOOR_LINE + 1.0f) reachedEntranceZone = true;
    if (centroidY < DOOR_LINE - 1.0f) reachedExitZone = true;
    
    lastCentroidY = centroidY;
    motionFrameCount++;

  } else {
    if (trackingActive) {
      if (motionFrameCount >= MIN_MOTION_FRAMES) {
        // Evaluate crossing with Traversal Confirmation (v1.0.032)
        if (firstCentroidY > DOOR_LINE && lastCentroidY < DOOR_LINE && reachedExitZone) {
          recordEvent("IN");
        } else if (firstCentroidY < DOOR_LINE && lastCentroidY > DOOR_LINE && reachedEntranceZone) {
          recordEvent("OUT");
        }
      }
      
      trackingActive = false;
      motionFrameCount = 0;
      Serial.println("Motion Ended");
    }
  }
  
  activePixels = activeCount; // Update global for baseline logic v1.0.027
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
  if (!mqttClient.connected()) return;
  
  String topic = String("door/counter/status/") + DEVICE_UID;
  StaticJsonDocument<256> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["status"] = status;
  doc["version"] = currentVersion;
  doc["udp_sent"] = udpPacketsSent;
  
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(topic.c_str(), buffer);
  mqttClient.loop(); // Flush status message
  Serial.println("Status Published: " + status + " | UDP Sent: " + String(udpPacketsSent));
}

void publishTelemetry() {
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
  
  char buffer[1024];
  size_t len = serializeJson(doc, buffer);
  
  udpClient.beginPacket(udp_server, udp_port);
  udpClient.write((const uint8_t*)buffer, len);
  if (udpClient.endPacket()) {
    udpPacketsSent++;
  } else {
    Serial.println("UDP Packet Fail");
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
    if (mqttClient.connect(DEVICE_UID, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      
      // Subscribe to command topic
      String commandTopic = String("door/counter/commands/") + DEVICE_UID;
      mqttClient.subscribe(commandTopic.c_str());
      Serial.println("Subscribed to: " + commandTopic);
      
      currentState = NORMAL_OPERATION;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
    }
  }
}

void publishBufferedEvents() {
  if (!mqttClient.connected()) return;

  while (eventCount > 0) {
    CounterEvent ev = eventBuffer[0];
    
    StaticJsonDocument<256> doc;
    doc["device_uid"] = DEVICE_UID;
    doc["ble_name"] = BLE_BROADCAST_NAME;
    doc["timestamp"] = ev.timestamp;
    doc["direction"] = ev.direction;

    char buffer[256];
    serializeJson(doc, buffer);

    if (mqttClient.publish(mqtt_topic, buffer)) {
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
  myImager.setIntegrationTime(10); // v1.0.033: Improved sensitivity for shorter/small targets
  
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
  delay(1000);

  // Initialize GPIOs
  pinMode(PIN_CALIBRATION, INPUT_PULLUP);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setLED(false, false, false); // All OFF (High)

  Serial.println("Connecting WiFi...");
  currentState = WIFI_CONNECT;
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    updateStatusLED();
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // OTA check first
  checkForOTA();

  // Load calibration from NVS
  loadCalibration();

  // Start sensor after OTA
  initVL53();
  
  // Setup MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  
  // Initialize UDP
  udpClient.begin(udp_port);
  Serial.printf("UDP Initialized on port %d\n", udp_port);
  
  currentState = NORMAL_OPERATION;
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  // Handle Networking
  if (WiFi.status() != WL_CONNECTED) {
    currentState = WIFI_CONNECT;
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

  if (currentState != CALIBRATION_MODE) {
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
