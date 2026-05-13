#include <WiFi.h>
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
#include "esp_sleep.h"
#include "driver/gpio.h"

// ================= WIFI & PROVISIONING =================
#include "Provisioning.h"


// ================= OTA & VERSION =================
String currentVersion = "1.1.057";
String versionURL = "https://raw.githubusercontent.com/asfandyaralishah112/Traffic_Sensor_src/main/version.json";

// ================= PROTOTYPES =================
void publishStatus(String status);
void publishTelemetry();
void processFlow();
void updateStatusLED();
void setLED(bool r, bool g, bool b);
void syncTime();
void correctBufferedTimestamps();

// ================= DEVICE ID =================
String DEVICE_UID = "UNCONFIGURED";
String ble_name = "SmartCounter-UNCONFIGURED";



String wifi_ssid = "";
String wifi_pass = "";
String business_name = "";
String screen_id = "";
int customer_count = 100;

// ================= MQTT =================
String mqtt_server = "af6e6b1eb2344e0f8f248e053a117476.s1.eu.hivemq.cloud";
int mqtt_port = 8883;
String mqtt_user = "Cavline_Sensors";
String mqtt_pass = "Cav@364!";
String topic_events, topic_status, topic_telemetry, topic_command;
bool mqttTelemetryEnabled = false; // Flag to easily enable/disable MQTT telemetry stream
bool deviceConfigured = false;
bool timeSynced = false;
bool bootStatusSent = false; // Flag for one-time power-on status
unsigned long lastTimeSyncMillis = 0; // For daily re-sync
unsigned long dailySyncOffset = 0;    // Random jitter for fleet de-clustering
bool bleStarted = false; // Flag to track BLE initialization state




// ================= GPIO =================
// TX_PIN removed - Only detection mode active

#define ADC1_PIN 6   // Sensor 1
#define ADC2_PIN 7   // Sensor 2


#define LED_RED   23
#define LED_GREEN 22
#define LED_BLUE  21

#define PIR_PIN   5
#define SENSOR_PWR_PIN 1 // Powers the phototransistors (High = ON)
#define SLEEP_DELAY_MS 30000 // 30 seconds of inactivity before deep sleep

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

Preferences preferences; // System preferences
Preferences devicePrefs; // Factory provisioning preferences
bool sensorInitialized = true; // Set true for phototransistor as it starts immediately




// ================= BEAM SENSING VARIABLES =================
float vref = 3.3;
int adc_max = 4095;
float TH_HIGH = 1.5;
float TH_LOW  = 0.8;

bool beam1_active = false;
bool beam2_active = false;

// State Machine for Direction
enum DetectionState { IDLE, B1_HIT, BOTH_HIT, B2_HIT };
DetectionState detectionState = IDLE;

volatile bool otaRequested = false;
unsigned long lastTelemetry = 0; // v1.0.027: unified timing

// ================= EVENT BUFFER =================
struct CounterEvent {
  char direction[8];
  time_t timestamp;
  bool timestampValid;
};

#define MAX_BUFFERED_EVENTS 100
RTC_DATA_ATTR CounterEvent eventBuffer[MAX_BUFFERED_EVENTS];
RTC_DATA_ATTR int eventCount = 0;

// beamFreq1 and beamFreq2 removed as they are no longer applicable for static beams


WiFiClientSecure wifiClientSecure;
WiFiClient wifiClient; 
PubSubClient mqttClient; // Dynamically assigned in setup/reconnect

uint32_t lastPrint = 0;
uint16_t frameCount = 0;
unsigned long lastActivityTime = 0;
bool wokeFromSleep = false;
bool networkingRequested = false;
int connectionRetries = 0;
bool otaChecked = false;

// Power Saving Definitions
#define BEAM_CLEAR_LEVEL HIGH // Assuming HIGH means beam is hitting the sensor

// =====================================================
// LED CONTROL
// =====================================================
void setLED(bool r, bool g, bool b) {
  // Common Anode: LOW = ON, HIGH = OFF
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
}

void updateStatusLED() {
  if (currentState == CALIBRATION_MODE) {
    // Blue Heartbeat Blink
    unsigned long m = millis() % 1000;
    bool blink = (m < 100) || (m > 250 && m < 350);
    setLED(false, false, blink);
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
    setLED(false, false, true); // Blue ON instead of Red
  } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    setLED(false, true, false); // Green ON
  } else if (WiFi.status() == WL_CONNECTED) {
    setLED(false, true, true); // Cyan (Green+Blue) instead of Orange
  } else {
    setLED(false, true, true); // Cyan (Green+Blue) instead of Blue/Red
  }
}

// =====================================================
// FACTORY RESET
// =====================================================
void factoryReset() {
  Serial.println("FACTORY RESET TRIGGERED!");
  
  // Clear WiFi configuration
  deviceConfigured = false; // Internal flag
  Preferences wifiPrefs;
  wifiPrefs.begin("wifi-config", false);
  wifiPrefs.clear();
  wifiPrefs.end();
  
  Serial.println("WiFi Configuration Cleared.");
  
  // Visual feedback: Fast blue blinking for 2 seconds
  for (int i = 0; i < 10; i++) {
    setLED(false, false, true); // Blue ON
    delay(100);
    setLED(false, false, false); // Blue OFF
    delay(100);
  }
  
  setLED(false, false, false); // All OFF
  Serial.println("Rebooting...");
  delay(500);
  ESP.restart();
}

// =====================================================
// NVS PERSISTENCE
// =====================================================
// Removed calibration NVS functions as per migration to phototransistor


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
    // MQTT configuration is now hardcoded for fleet scalability
    Serial.println("Device UID loaded from NVS. MQTT using hardcoded cluster config.");
  } else {
    Serial.println("No factory device configuration found.");
    DEVICE_UID = "UNCONFIGURED";
  }
  devicePrefs.end();
  
  // Also load WiFi/Business context to form the BLE name
  if (isWiFiConfigured()) {
    business_name = getStoredBusiness();
    screen_id = getStoredScreenId();
    customer_count = getStoredCustomerCount();
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
// Removed runCalibration and trigger logic

// =====================================================
// DETECTION LOGIC
// =====================================================

// v1.0.041: Dilation Helper
// Removed dilation helper


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
    strncpy(eventBuffer[eventCount].direction, direction.c_str(), sizeof(eventBuffer[eventCount].direction) - 1);
    eventBuffer[eventCount].direction[sizeof(eventBuffer[eventCount].direction) - 1] = '\0';
    eventBuffer[eventCount].timestamp = t;
    eventBuffer[eventCount].timestampValid = valid;
    eventCount++;
    networkingRequested = true; // Trigger WiFi/MQTT connection on activity
    Serial.println("Event Recorded: " + direction + (valid ? " [UTC]" : " [RELATIVE]"));
  } else {
    for (int i = 0; i < MAX_BUFFERED_EVENTS - 1; i++) {
      eventBuffer[i] = eventBuffer[i+1];
    }
    strncpy(eventBuffer[MAX_BUFFERED_EVENTS-1].direction, direction.c_str(), sizeof(eventBuffer[MAX_BUFFERED_EVENTS-1].direction) - 1);
    eventBuffer[MAX_BUFFERED_EVENTS-1].direction[sizeof(eventBuffer[MAX_BUFFERED_EVENTS-1].direction) - 1] = '\0';
    eventBuffer[MAX_BUFFERED_EVENTS-1].timestamp = t;
    eventBuffer[MAX_BUFFERED_EVENTS-1].timestampValid = valid;
    networkingRequested = true; // Trigger WiFi/MQTT connection on activity
    Serial.println("Event Recorded (Buffer Full): " + direction + (valid ? " [UTC]" : " [RELATIVE]"));
  }
}

// Old processFlow removed as part of phototransistor migration


// =====================================================
// ADAPTIVE CALIBRATION
// =====================================================
// Removed adaptive baseline logic


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
  doc["business"] = business_name; 
  doc["screen_id"] = screen_id;
  doc["customer_count"] = customer_count;
  
  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(topic_status.c_str(), buffer);
  mqttClient.loop(); // Flush status message
  Serial.println("Status Published: " + status);
}

void publishTelemetry() {
  if (!deviceConfigured || DEVICE_UID == "UNCONFIGURED") return;
  static unsigned long lastTelemetryTime = 0;
  if (millis() - lastTelemetryTime < 1000) return; // 1 FPS for beam telemetry
  lastTelemetryTime = millis();

  StaticJsonDocument<512> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["state"] = (int)detectionState;
  
  JsonArray beams = doc.createNestedArray("beams");
  // Report 1 for clear, 0 for blocked instead of frequency
  beams.add(beam1_active ? 1 : 0);
  beams.add(beam2_active ? 1 : 0);
  
  JsonArray states = doc.createNestedArray("states");
  states.add(beam1_active ? 1 : 0);
  states.add(beam2_active ? 1 : 0);
  
  if (timeSynced) doc["timestamp"] = (uint32_t)time(nullptr);
  
  char buffer[512];
  serializeJson(doc, buffer);
  
  // Publish to MQTT
  if (mqttTelemetryEnabled && mqttClient.connected()) {
    mqttClient.publish(topic_telemetry.c_str(), buffer);
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
  if (command == "update") {
    if (currentState == OTA_UPDATE) {
      Serial.println("Busy, ignoring update command.");
      return;
    }
    otaRequested = true;
    Serial.println("OTA Update Requested via MQTT");
  } else if (command == "telemetry") {

    String state = doc["state"].as<String>();
    if (state == "ON") {
      mqttTelemetryEnabled = true;
      Serial.println("MQTT Telemetry Enabled");
    } else if (state == "OFF") {
      mqttTelemetryEnabled = false;
      Serial.println("MQTT Telemetry Disabled");
    }
    // Optional: publish status back to confirm
    publishStatus(mqttTelemetryEnabled ? "telemetry_on" : "telemetry_off");
  } else if (command == "reset") {
    Serial.println("Remote Factory Reset Requested via MQTT");
    factoryReset();
  } else if (command == "get_customers") {
    Serial.println("Customer count report requested via MQTT");
    publishStatus("customer_count_report");
  }
}

void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  
  static unsigned long lastConnectAttempt = 0;
  if (millis() - lastConnectAttempt > 5000) {
    lastConnectAttempt = millis();
    Serial.print("Attempting MQTT connection...");
    
    // Choose client based on port
    if (mqtt_port == 8883) {
      wifiClientSecure.setInsecure(); // Standard for many IoT setups without managed CA
      mqttClient.setClient(wifiClientSecure);
      Serial.print(" (Secure) ");
    } else {
      mqttClient.setClient(wifiClient);
    }
    
    if (mqttClient.connect(
          DEVICE_UID.c_str(), 
          mqtt_user.c_str(), 
          mqtt_pass.c_str(), 
          topic_status.c_str(), 
          1, 
          true, 
          "{\"status\":\"online\"}"
        )) {
      Serial.println("connected");
      
      // Publish online status immediately
      publishStatus("online");

      // Requirement: Power-cycle one-time status (UID, Business, Screen Id)
      if (!bootStatusSent) {
          StaticJsonDocument<256> bootDoc;
          bootDoc["device_uid"] = DEVICE_UID;
          bootDoc["business"] = business_name;
          bootDoc["screen_id"] = screen_id;
          bootDoc["event"] = "power_on";
          
          char bootBuffer[256];
          serializeJson(bootDoc, bootBuffer);
          mqttClient.publish(topic_status.c_str(), bootBuffer);
          bootStatusSent = true;
          Serial.println("Power-on status published.");
      }
      
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
    doc["screen_id"] = screen_id;

    char buffer[256];
    serializeJson(doc, buffer);

    if (mqttClient.publish(topic_events.c_str(), buffer)) {
      Serial.print("Published: "); Serial.println(ev.direction);
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
// Removed initVL53


void goToSleep() {
  Serial.println("\n--- Entering Deep Sleep ---");
  // Turn off sensor power and LEDs
  digitalWrite(SENSOR_PWR_PIN, LOW);
  setLED(false, false, false);
  
  // Flush serial to ensure message is sent
  Serial.flush();
  delay(100); // Stabilization delay
  
  // Configure wakeup: GPIO 5 (PIR) High
  uint64_t wakeup_mask = (1ULL << PIR_PIN);
  esp_sleep_enable_ext1_wakeup(wakeup_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  
  esp_deep_sleep_start();
}

void enterLightSleep() {
  // Configure GPIO wakeup for light sleep
  esp_sleep_enable_gpio_wakeup();
  
  // Wake on Beam 1 change
  gpio_wakeup_enable((gpio_num_t)ADC1_PIN, digitalRead(ADC1_PIN) == HIGH ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  // Wake on Beam 2 change
  gpio_wakeup_enable((gpio_num_t)ADC2_PIN, digitalRead(ADC2_PIN) == HIGH ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  // Wake on PIR going LOW (to trigger deep sleep transition)
  gpio_wakeup_enable((gpio_num_t)PIR_PIN, GPIO_INTR_LOW_LEVEL);
  
  // Enter light sleep
  esp_light_sleep_start();
}


// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(115200);
  delay(100);

  // Initialize Sensor Power as early as possible
  pinMode(SENSOR_PWR_PIN, OUTPUT);
  digitalWrite(SENSOR_PWR_PIN, HIGH);

  // Initialize PIR Pin
  pinMode(PIR_PIN, INPUT); 
  
  // Log wakeup reason
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  Serial.print("Wakeup Reason (Raw): "); Serial.println(wakeup_reason);

  if (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    Serial.println("Wakeup Source: Sleep Wakeup (PIR or GPIO)");
    wokeFromSleep = true;
    networkingRequested = (eventCount > 0); // Upload only if we have events
  } else {
    Serial.println("Wakeup Source: Power-on or Hardware Reset");
    wokeFromSleep = false;
    networkingRequested = true; // Connect immediately for OTA/Status on cold boot
  }
  lastActivityTime = millis();

  // If we have buffered events from a previous session, trigger networking
  if (eventCount > 0) {
    Serial.printf("Found %d buffered events in RTC memory. Triggering upload...\n", eventCount);
    networkingRequested = true;
  }
  
  // OBJECTIVE 1: 5-second Factory Provisioning Window (Skip on Wakeup for instant detection)
  if (!wokeFromSleep) {
    unsigned long factoryStart = millis();
    while (millis() - factoryStart < 5000) {
      handleSerialProvisioning();
      delay(5);
    }
  }

  // OBJECTIVE 2 & 4: Load configurations from NVS
  loadDeviceConfig();

  // Requirement: initialize NTP jitter (0-60 minutes)
  dailySyncOffset = random(0, 3600000);
  
  // Initialize GPIOs
  // pinMode(TX_PIN, OUTPUT) removed - Only detection mode active

  pinMode(ADC1_PIN, INPUT);
  pinMode(ADC2_PIN, INPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setLED(false, true, true); // Blue + Green ON (Cyan)

  // Start 1kHz tone for IR Emitters
  // tone(TX_PIN, 1000) removed - Only detection mode active

  analogReadResolution(12);
  Serial.println("Dual Beam ADC Detection Mode Initialized");

  // v1.0.043: Init Success Signal - Cyan (Blue+Green ON)
  if (sensorInitialized) {
    setLED(false, true, true);
  }


  // Removed auto-calibration trigger


  // WiFi Configuration logic
  if (isWiFiConfigured()) {
    wifi_ssid = getStoredSSID();
    wifi_pass = getStoredPass();
    business_name = getStoredBusiness();
    screen_id = getStoredScreenId();

    // Configure MQTT Server once
    mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
    if (mqtt_port == 8883) {
      wifiClientSecure.setInsecure();
      mqttClient.setClient(wifiClientSecure);
    } else {
      mqttClient.setClient(wifiClient);
    }
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(1024);

    if (networkingRequested && !wokeFromSleep) {
      Serial.println("First Boot: Connecting WiFi for OTA Check...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
      
      unsigned long start = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        Serial.print(".");
        updateStatusLED();
      }
      
      if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Connected");
        syncTime();
        checkForOTA();
        otaChecked = true;
        startBLEAdvertising();
      } else {
        connectionRetries++; // Count as a failed attempt
      }
    }
    currentState = NORMAL_OPERATION;
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
  if (networkingRequested) {
    if (WiFi.status() != WL_CONNECTED && connectionRetries < 3) {
      static unsigned long lastConnectAttempt = 0;
      if (millis() - lastConnectAttempt > 15000) { // Try every 15s
        Serial.printf("\nAttempting WiFi Connection (%d/3)...\n", connectionRetries + 1);
        WiFi.begin(wifi_ssid.c_str(), wifi_pass.c_str());
        lastConnectAttempt = millis();
        
        // Brief wait to see if it connects immediately
        unsigned long waitStart = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - waitStart < 5000) {
          delay(100);
          updateStatusLED();
        }
        
        if (WiFi.status() != WL_CONNECTED) {
          connectionRetries++;
          if (connectionRetries >= 3) Serial.println("WiFi Connection Failed after 3 attempts.");
        } else {
          Serial.println("WiFi Connected");
          syncTime();
          if (!otaChecked) { checkForOTA(); otaChecked = true; }
          startBLEAdvertising();
        }
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      if (!mqttClient.connected()) {
        currentState = MQTT_CONNECT;
        mqttReconnect();
      } else {
        mqttClient.loop();
        publishBufferedEvents();
        publishTelemetry();
      }
    }
  }

  updateStatusLED();

  // Async OTA trigger
  if (otaRequested && currentState != OTA_UPDATE) {
    otaRequested = false;
    checkForOTA();
  }

  // =====================================================
  // BEAM SAMPLING (Static Digital Level)
  // =====================================================
  if (currentState != CALIBRATION_MODE && currentState != PROVISIONING_AP) {
    // Update beam states from digital levels
    beam1_active = (digitalRead(ADC1_PIN) == BEAM_CLEAR_LEVEL);
    beam2_active = (digitalRead(ADC2_PIN) == BEAM_CLEAR_LEVEL);

    processFlow();
    publishTelemetry();

    if (millis() - lastPrint > 1000) {
      Serial.print("S1: "); Serial.print(beam1_active ? "OK " : "BLK");
      Serial.print(" | S2: "); Serial.println(beam2_active ? "OK " : "BLK");
      lastPrint = millis();
    }
  }

  // =====================================================
  // SLEEP MANAGEMENT (Aggressive Optimization)
  // =====================================================
  bool motionDetected = (digitalRead(PIR_PIN) == HIGH);
  bool beamsBlocked = (!beam1_active || !beam2_active);
  
  // Stay awake if motion, beams are blocked, or a sequence is in progress
  bool shouldStayAwake = motionDetected || beamsBlocked || (detectionState != IDLE);

  // Instant return to deep sleep ONLY if no activity at all
  if (!shouldStayAwake && currentState == NORMAL_OPERATION) {
    goToSleep();
  }

  // Only stay fully awake if we have networking work or other operations
  bool pendingWork = networkingRequested && (eventCount > 0) && (connectionRetries < 3 || (WiFi.status() == WL_CONNECTED && mqttClient.connected()));
  
  if (!pendingWork && !shouldStayAwake && currentState == NORMAL_OPERATION) {
    // This case should be covered by deep sleep, but for safety in other states:
    enterLightSleep();
  } else if (!pendingWork && shouldStayAwake && currentState == NORMAL_OPERATION) {
    // If we should stay awake but have no networking, use light sleep to wait for changes
    enterLightSleep();
  }
  
  delay(1); 
}

// =====================================================
// DIRECTIONAL STATE MACHINE
// =====================================================
void processFlow() {
  // Beams are active (TRUE) when clear, and inactive (FALSE) when blocked.
  bool b1 = !beam1_active; // b1 is true if blocked
  bool b2 = !beam2_active; // b2 is true if blocked
  
  static bool last_b1 = false;
  static bool last_b2 = false;
  static unsigned long stateEntryTime = 0;
  static int sequence = 0; // 0: None, 1: B1 hit first, 2: B2 hit first, 3: Both hit simultaneously
  static bool bothHitFlag = false;

  // Update stateEntryTime on any state change to prevent timeout while actively blocked
  if (b1 != last_b1 || b2 != last_b2) {
    stateEntryTime = millis();
  }

  switch (detectionState) {
    case IDLE:
      if (b1 || b2) {
        stateEntryTime = millis();
        bothHitFlag = false;
        if (b1 && !b2) {
          detectionState = B1_HIT;
          sequence = 1;
          Serial.println("Sequence Start: B1");
        } else if (b2 && !b1) {
          detectionState = B2_HIT;
          sequence = 2;
          Serial.println("Sequence Start: B2");
        } else {
          // Simultaneous hit
          detectionState = BOTH_HIT;
          sequence = 3;
          bothHitFlag = true;
          Serial.println("Sequence Start: BOTH (Ambiguous)");
        }
      }
      break;

    case B1_HIT:
      if (b1 && b2) {
        detectionState = BOTH_HIT;
        bothHitFlag = true;
      } else if (!b1 && !b2) {
        // Completion Check for IN (B2 -> BOTH -> B1 -> CLEAR)
        // or Ambiguous resolved to IN
        if (sequence == 2 && bothHitFlag) {
          recordEvent("IN");
        }
        detectionState = IDLE;
        sequence = 0;
      }
      break;

    case B2_HIT:
      if (b1 && b2) {
        detectionState = BOTH_HIT;
        bothHitFlag = true;
      } else if (!b1 && !b2) {
        // Completion Check for OUT (B1 -> BOTH -> B2 -> CLEAR)
        // or Ambiguous resolved to OUT
        if (sequence == 1 && bothHitFlag) {
          recordEvent("OUT");
        }
        detectionState = IDLE;
        sequence = 0;
      }
      break;

    case BOTH_HIT:
      if (!b1 && b2) {
        // B1 cleared first - moving towards B2 (OUT)
        if (sequence == 3) {
          sequence = 1; 
          Serial.println("Ambiguity Resolved: OUT");
        }
        detectionState = B2_HIT;
      } else if (b1 && !b2) {
        // B2 cleared first - moving towards B1 (IN)
        if (sequence == 3) {
          sequence = 2;
          Serial.println("Ambiguity Resolved: IN");
        }
        detectionState = B1_HIT;
      } else if (!b1 && !b2) {
        // Both cleared at exact same time
        if (sequence == 1) recordEvent("OUT");
        else if (sequence == 2) recordEvent("IN");
        detectionState = IDLE;
        sequence = 0;
      }
      break;
  }

  // Timeout stuck states (300 seconds / 5 minutes)
  // This allows people to stand in the door for a long time.
  if (detectionState != IDLE && (millis() - stateEntryTime > 300000)) {
    Serial.println("Sequence Timeout (5 min) - Resetting");
    detectionState = IDLE;
    sequence = 0;
    bothHitFlag = false;
  }

  last_b1 = b1;
  last_b2 = b2;
}
