#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>

// ================= OTA & VERSION =================
String currentVersion = "1.0.013";
String versionURL = "https://raw.githubusercontent.com/asfandyaralishah112/Traffic_Sensor_src/main/version.json";

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
  uint16_t threshold[64];
};

CalibrationData calData;
Preferences preferences;
bool sensorInitialized = false;

// ================= REGION OCCUPANCY =================
bool A_active = false;
bool M_active = false;
bool B_active = false;

enum FlowState {
  FLOW_IDLE,
  FLOW_A,           // IN started (A only)
  FLOW_AM,          // IN progressing (A and M)
  FLOW_MB,          // IN progressing (M and B)
  FLOW_B_AFTER_IN,  // IN almost done (B only)
  
  FLOW_B,           // OUT started (B only)
  FLOW_BM,          // OUT progressing (B and M)
  FLOW_MA,          // OUT progressing (M and A)
  FLOW_A_AFTER_OUT, // OUT almost done (A only)
  
  FLOW_CLEARING     // Waiting for all regions to clear
};

FlowState flowState = FLOW_IDLE;
volatile bool otaRequested = false;
unsigned long stateStartTime = 0;
int clearFrames = 0;
#define MIN_STATE_FRAMES 2
#define CLEAR_FRAMES_REQ 5

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
  preferences.putBytes("threshold", calData.threshold, 128); // 64 * 2
  preferences.end();
  Serial.println("Calibration saved to NVS");
}

void loadCalibration() {
  preferences.begin("counter-cal", true);
  if (preferences.isKey("zone_mask")) {
    preferences.getBytes("zone_mask", calData.zone_mask, 64);
    preferences.getBytes("floor_dist", calData.floor_distance, 128);
    preferences.getBytes("threshold", calData.threshold, 128);
    Serial.println("Calibration loaded from NVS");
  } else {
    Serial.println("No calibration found in NVS, using defaults");
    memset(calData.zone_mask, 1, 64); // Default: all zones valid (simplified)
    for(int i=0; i<64; i++) calData.threshold[i] = 1000; // Default threshold
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

  // Step 4: Threshold Calculation
  Serial.println("Step 4: Threshold Calculation...");
  for (int i = 0; i < 64; i++) {
    if (calData.zone_mask[i] == 0) {
      calData.threshold[i] = calData.floor_distance[i] / 2;
    } else {
      calData.threshold[i] = 0;
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
void updateRegionOccupancy() {
  int aCount = 0;
  int mCount = 0;
  int bCount = 0;

  for (int i = 0; i < 64; i++) {
    if (calData.zone_mask[i] == 0) { // VALID_WALK_ZONE
      if (measurementData.distance_mm[i] > 0 && 
          measurementData.distance_mm[i] < calData.threshold[i] &&
          (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9)) {
        
        int col = i % 8;
        if (col <= 2) aCount++;
        else if (col <= 5) mCount++;
        else bCount++;
      }
    }
  }

  A_active = (aCount >= 2);
  M_active = (mCount >= 2);
  B_active = (bCount >= 2);
}

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
  updateRegionOccupancy();
  
  static FlowState candidateState = FLOW_IDLE;
  static int candidateFrames = 0;

  // Determine which state the current occupancy points towards
  FlowState nextStep = flowState; 

  switch (flowState) {
    case FLOW_IDLE:
      if (A_active && !M_active && !B_active) nextStep = FLOW_A;
      else if (B_active && !M_active && !A_active) nextStep = FLOW_B;
      break;

    case FLOW_A:
      if (A_active && M_active) nextStep = FLOW_AM;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_AM:
      if (M_active && B_active) nextStep = FLOW_MB;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_MB:
      if (!A_active && !M_active && B_active) nextStep = FLOW_B_AFTER_IN;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_B_AFTER_IN:
      if (!A_active && !M_active && !B_active) nextStep = FLOW_CLEARING;
      break;

    case FLOW_B:
      if (B_active && M_active) nextStep = FLOW_BM;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_BM:
      if (M_active && A_active) nextStep = FLOW_MA;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_MA:
      if (!B_active && !M_active && A_active) nextStep = FLOW_A_AFTER_OUT;
      else if (!A_active && !M_active && !B_active) nextStep = FLOW_IDLE;
      break;

    case FLOW_A_AFTER_OUT:
      if (!A_active && !M_active && !B_active) nextStep = FLOW_CLEARING;
      break;

    case FLOW_CLEARING:
      // Clearing logic is slightly different: we wait for N frames of silence.
      if (!A_active && !M_active && !B_active) {
        clearFrames++;
        if (clearFrames >= CLEAR_FRAMES_REQ) {
          flowState = FLOW_IDLE;
          clearFrames = 0;
          candidateFrames = 0;
          candidateState = FLOW_IDLE;
        }
      } else {
        clearFrames = 0;
      }
      return; // Skip the transition logic below for clearing
  }

  // Handle Stability for all other states
  if (nextStep != flowState) {
    if (nextStep == candidateState) {
      candidateFrames++;
      if (candidateFrames >= MIN_STATE_FRAMES) {
        // Condition met for required frames, transition now
        if (flowState == FLOW_B_AFTER_IN && nextStep == FLOW_CLEARING) recordEvent("IN");
        if (flowState == FLOW_A_AFTER_OUT && nextStep == FLOW_CLEARING) recordEvent("OUT");
        
        flowState = nextStep;
        candidateFrames = 0;
      }
    } else {
      // Condition changed, reset stability counter
      candidateState = nextStep;
      candidateFrames = 0;
    }
  } else {
    // Condition matches current state, reset candidate
    candidateFrames = 0;
    candidateState = flowState;
  }
}

// =====================================================
// ADAPTIVE CALIBRATION
// =====================================================
void updateAdaptiveBaseline() {
  if (currentState != NORMAL_OPERATION) return;
  if (flowState != FLOW_IDLE) return;
  if (A_active || M_active || B_active) return;

  const float alpha = 0.003;

  for (int i = 0; i < 64; i++) {
    if (calData.zone_mask[i] == 0) { // VALID_WALK_ZONE
      if ((measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) &&
          measurementData.distance_mm[i] > 0) {
        
        uint16_t currentDist = measurementData.distance_mm[i];
        uint16_t baselineDist = calData.floor_distance[i];

        // Rule 6: Safety Clamp
        if (abs((int)currentDist - (int)baselineDist) <= 300) {
          // Rule 4: Exponential Moving Average
          float newFloor = ((1.0f - alpha) * (float)baselineDist) + (alpha * (float)currentDist);
          calData.floor_distance[i] = (uint16_t)newFloor;
          
          // Rule 5: Threshold update
          calData.threshold[i] = calData.floor_distance[i] / 2;
        }
      }
    }
  }
}

// =====================================================
// MQTT FUNCTIONS
// =====================================================
void publishStatus(String status) {
  if (!mqttClient.connected()) return;
  
  String topic = String("door/counter/status/") + DEVICE_UID;
  StaticJsonDocument<128> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["status"] = status;
  
  char buffer[128];
  serializeJson(doc, buffer);
  mqttClient.publish(topic.c_str(), buffer);
  mqttClient.loop(); // Flush status message
  Serial.println("Status Published: " + status);
}

void publishTelemetry() {
  if (!mqttClient.connected()) return;
  
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry < 100) return;
  lastTelemetry = millis();

  String topic = String("door/counter/telemetry/") + DEVICE_UID;
  StaticJsonDocument<1024> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["state"] = (int)flowState;
  
  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < 64; i++) {
    zones.add(measurementData.distance_mm[i]);
  }
  
  char buffer[1024];
  serializeJson(doc, buffer);
  mqttClient.publish(topic.c_str(), buffer);
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
  myImager.setResolution(8 * 8);
  myImager.setRangingFrequency(15);
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
      processFlow();
      updateAdaptiveBaseline();

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
