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
String currentVersion = "1.0.011";
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

#define LED_RED   23
#define LED_GREEN 22
#define LED_BLUE  21

// ================= SYSTEM STATES =================
enum SystemState {
  BOOT,
  WIFI_CONNECT,
  MQTT_CONNECT,
  NORMAL_OPERATION,
  OTA_UPDATE,
  ERROR_STATE
};

SystemState currentState = BOOT;

// ================= DETECTION CONSTANTS =================
#define PERSON_DELTA_MM 400
#define MIN_CLUSTER_SIZE 2
#define MAX_TRACKS 5
#define TRACK_TIMEOUT_MS 1000
#define TRACK_MATCH_MAX_DIST 3.0f
#define REVERSAL_MARGIN 1.5f

// ================= DATA STRUCTURES =================
struct Cluster {
    float x, y;
    int size;
    bool used = false;
};

struct BaselineData {
    uint16_t baseline_distance[64];
};

struct Track {
    uint16_t id;
    bool active = false;
    float start_x;
    float current_x;
    float current_y;
    float max_x_reached;
    float min_x_reached;
    unsigned long last_seen;
    bool entered_side_left; // true if started in LEFT, false if RIGHT
    bool reached_center = false;
    bool entered_exit_zone = false;
};

BaselineData baseData;
bool baselineInitialized = false;
Track tracks[MAX_TRACKS];
uint16_t nextTrackId = 1;

// ================= VL53 =================
// ... (rest of the declarations same as before)
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;

WiFiClientSecure wifiClientSecure;
WiFiClient wifiClient; 
PubSubClient mqttClient(wifiClient);

uint32_t lastPrint = 0;
uint16_t frameCount = 0;
volatile bool otaRequested = false;

// ================= EVENT BUFFER =================
struct CounterEvent {
  String direction;
  unsigned long timestamp;
};

#define MAX_BUFFERED_EVENTS 100
CounterEvent eventBuffer[MAX_BUFFERED_EVENTS];
int eventCount = 0;

// =====================================================
// LED CONTROL
// =====================================================
void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_RED, r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
}

void updateStatusLED() {
  if (currentState == ERROR_STATE) {
    setLED(true, false, false);
  } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    setLED(false, true, false); 
  } else if (WiFi.status() == WL_CONNECTED) {
    setLED(true, true, false);
  } else {
    setLED(true, false, false);
  }
}

// =====================================================
// BASELINE LOGIC
// =====================================================
void updateBaseline() {
    if (!baselineInitialized) {
        int valid_count = 0;
        for (int i = 0; i < 64; i++) {
            if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
                baseData.baseline_distance[i] = measurementData.distance_mm[i];
                valid_count++;
            }
        }
        // Initialize if at least 80% (51 zones) are valid
        if (valid_count >= 51) {
            baselineInitialized = true;
            Serial.printf("Baseline Initialized (%d zones valid)\n", valid_count);
        }
        return;
    }

    // Adapt only if no tracks are active
    bool tracksActive = false;
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (tracks[i].active) {
            tracksActive = true;
            break;
        }
    }
    if (tracksActive) return;

    const float alpha = 0.002f;
    for (int i = 0; i < 64; i++) {
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            uint16_t current = measurementData.distance_mm[i];
            int delta = (int)baseData.baseline_distance[i] - (int)current;
            
            // Only update if it's a small change (prevents adapting to people)
            if (abs(delta) < 300) {
                baseData.baseline_distance[i] = (uint16_t)((1.0f - alpha) * baseData.baseline_distance[i] + alpha * current);
            }
        }
    }
}

// =====================================================
// CLUSTER & TRACKING LOGIC
// =====================================================

int getCentroids(Cluster* centroids) {
    bool occupied[64] = {0};
    int labels[64];
    for (int i = 0; i < 64; i++) {
        labels[i] = -1;
        if (measurementData.target_status[i] == 5 || measurementData.target_status[i] == 9) {
            int delta = (int)baseData.baseline_distance[i] - (int)measurementData.distance_mm[i];
            if (delta > PERSON_DELTA_MM) occupied[i] = true;
        }
    }

    // Simple CCL (Connected Component Labeling) for 8x8 grid
    int clusterCount = 0;
    for (int i = 0; i < 64; i++) {
        if (occupied[i] && labels[i] == -1) {
            // Start new cluster
            int stack[64];
            int top = 0;
            stack[top++] = i;
            labels[i] = clusterCount;
            
            float sumX = 0, sumY = 0;
            int size = 0;
            
            while (top > 0) {
                int curr = stack[--top];
                int r = curr / 8;
                int c = curr % 8;
                sumX += c;
                sumY += r;
                size++;
                
                // Neighbors (4-connectivity)
                int dr[] = {-1, 1, 0, 0};
                int dc[] = {0, 0, -1, 1};
                for (int n = 0; n < 4; n++) {
                    int nr = r + dr[n];
                    int nc = c + dc[n];
                    if (nr >= 0 && nr < 8 && nc >= 0 && nc < 8) {
                        int ni = nr * 8 + nc;
                        if (occupied[ni] && labels[ni] == -1) {
                            labels[ni] = clusterCount;
                            stack[top++] = ni;
                        }
                    }
                }
            }
            
            if (size >= MIN_CLUSTER_SIZE && clusterCount < 10) {
                centroids[clusterCount].x = sumX / size;
                centroids[clusterCount].y = sumY / size;
                centroids[clusterCount].size = size;
                clusterCount++;
            }
        }
    }
    return clusterCount;
}

void recordEvent(String direction) {
  if (eventCount < MAX_BUFFERED_EVENTS) {
    eventBuffer[eventCount].direction = direction;
    eventBuffer[eventCount].timestamp = millis();
    eventCount++;
    Serial.println("Event Recorded: " + direction);
  }
}

void processTracking() {
    Cluster clusters[10];
    int nClusters = getCentroids(clusters);
    unsigned long now = millis();

    // 1. Match clusters to existing tracks
    for (int i = 0; i < MAX_TRACKS; i++) {
        if (!tracks[i].active) continue;
        
        float minDist = TRACK_MATCH_MAX_DIST;
        int bestIdx = -1;
        
        for (int j = 0; j < nClusters; j++) {
            if (clusters[j].used) continue;
            
            // Motion direction check: reject clusters moving against entry direction
            if (tracks[i].entered_side_left && clusters[j].x < tracks[i].current_x - 0.2) continue;
            if (!tracks[i].entered_side_left && clusters[j].x > tracks[i].current_x + 0.2) continue;

            float d = sqrt(pow(tracks[i].current_x - clusters[j].x, 2) + pow(tracks[i].current_y - clusters[j].y, 2));
            if (d < minDist) {
                minDist = d;
                bestIdx = j;
            }
        }
        
        if (bestIdx != -1) {
            // Update tracking extremes
            tracks[i].current_x = clusters[bestIdx].x;
            tracks[i].current_y = clusters[bestIdx].y;
            tracks[i].last_seen = now;
            clusters[bestIdx].used = true;
            
            if (tracks[i].current_x > tracks[i].max_x_reached) tracks[i].max_x_reached = tracks[i].current_x;
            if (tracks[i].current_x < tracks[i].min_x_reached) tracks[i].min_x_reached = tracks[i].current_x;

            // Reversal detection based on displacement margin
            bool reversed = false;
            if (tracks[i].entered_side_left && tracks[i].current_x < (tracks[i].max_x_reached - REVERSAL_MARGIN)) reversed = true;
            if (!tracks[i].entered_side_left && tracks[i].current_x > (tracks[i].min_x_reached + REVERSAL_MARGIN)) reversed = true;

            if (reversed) {
                Serial.printf("Track %d reversed (displacement), discarding\n", tracks[i].id);
                tracks[i].active = false;
                continue;
            }
            
            // Progress validation
            if (tracks[i].current_x >= 2.0 && tracks[i].current_x <= 5.0) {
                tracks[i].reached_center = true;
            }
            
            // Exit zone validation
            if (tracks[i].entered_side_left && tracks[i].current_x >= 6.0) tracks[i].entered_exit_zone = true;
            if (!tracks[i].entered_side_left && tracks[i].current_x <= 1.0) tracks[i].entered_exit_zone = true;

        } else if (now - tracks[i].last_seen > TRACK_TIMEOUT_MS) {
            // Track expired - check if it finished crossing
            if (tracks[i].reached_center && tracks[i].entered_exit_zone) {
                if (tracks[i].entered_side_left && tracks[i].current_x >= 6.0) {
                    recordEvent("IN");
                } else if (!tracks[i].entered_side_left && tracks[i].current_x <= 1.0) {
                    recordEvent("OUT");
                } else {
                    Serial.printf("Track %d expired near edge but not in exit zone, discarding\n", tracks[i].id);
                }
            } else {
                Serial.printf("Track %d expired without full crossing\n", tracks[i].id);
            }
            tracks[i].active = false;
        }
    }

    // 2. Create new tracks for unused clusters in entry zones
    for (int j = 0; j < nClusters; j++) {
        if (clusters[j].used) continue;
        
        bool left = (clusters[j].x <= 1.5);
        bool right = (clusters[j].x >= 5.5);
        
        if (left || right) {
            for (int i = 0; i < MAX_TRACKS; i++) {
                if (!tracks[i].active) {
                    tracks[i].id = nextTrackId++;
                    tracks[i].active = true;
                    tracks[i].start_x = clusters[j].x;
                    tracks[i].current_x = clusters[j].x;
                    tracks[i].current_y = clusters[j].y;
                    tracks[i].max_x_reached = clusters[j].x;
                    tracks[i].min_x_reached = clusters[j].x;
                    tracks[i].last_seen = now;
                    tracks[i].entered_side_left = left;
                    tracks[i].reached_center = false;
                    tracks[i].entered_exit_zone = false;
                    Serial.printf("New Track %d started from %s\n", tracks[i].id, left ? "LEFT" : "RIGHT");
                    break;
                }
            }
        }
    }
}

// =====================================================
// MQTT FUNCTIONS
// =====================================================
void publishBufferedEvents() {
  if (!mqttClient.connected()) return;
  while (eventCount > 0) {
    CounterEvent ev = eventBuffer[0];
    StaticJsonDocument<256> doc;
    doc["device_uid"] = DEVICE_UID;
    doc["timestamp"] = ev.timestamp;
    doc["direction"] = ev.direction;
    char buffer[256];
    serializeJson(doc, buffer);
    if (mqttClient.publish(mqtt_topic, buffer)) {
      for (int i = 0; i < eventCount - 1; i++) eventBuffer[i] = eventBuffer[i+1];
      eventCount--;
    } else break;
  }
}

void publishTelemetry() {
  if (!mqttClient.connected()) return;
  static unsigned long lastTelemetry = 0;
  if (millis() - lastTelemetry < 100) return;
  lastTelemetry = millis();

  StaticJsonDocument<1024> doc;
  doc["device_uid"] = DEVICE_UID;
  doc["state"] = 0; // Legacy placeholder
  JsonArray zones = doc.createNestedArray("zones");
  for (int i = 0; i < 64; i++) zones.add(measurementData.distance_mm[i]);
  
  char buffer[1024];
  serializeJson(doc, buffer);
  mqttClient.publish("door/counter/telemetry/ESP32C6_COUNTER_001", buffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;
  String command = doc["command"].as<String>();
  if (command == "update") otaRequested = true;
}

void mqttReconnect() {
  if (WiFi.status() != WL_CONNECTED) return;
  static unsigned long lastConnectAttempt = 0;
  if (millis() - lastConnectAttempt > 5000) {
    lastConnectAttempt = millis();
    if (mqttClient.connect(DEVICE_UID, mqtt_user, mqtt_pass)) {
      String commandTopic = String("door/counter/commands/") + DEVICE_UID;
      mqttClient.subscribe(commandTopic.c_str());
      currentState = NORMAL_OPERATION;
    }
  }
}

// =====================================================
// OTA CHECK
// =====================================================
void checkForOTA() {
  if (currentState == OTA_UPDATE) return;
  SystemState previousState = currentState;
  currentState = OTA_UPDATE;
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.begin(client, versionURL);
  if (http.GET() == 200) {
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, http.getString())) {
      String newVersion = doc["version"].as<String>();
      if (newVersion != currentVersion) {
        WiFiClientSecure updateClient;
        updateClient.setInsecure();
        httpUpdate.update(updateClient, doc["firmware"].as<String>());
      }
    }
  }
  http.end();
  currentState = previousState;
}

// =====================================================
// SETUP & LOOP
// =====================================================
void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  setLED(false, false, false);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    updateStatusLED();
  }

  checkForOTA();

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
  if (myImager.begin()) {
    myImager.setResolution(8 * 8);
    myImager.setRangingFrequency(15);
    myImager.startRanging();
  } else {
    currentState = ERROR_STATE;
  }

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(1024);
  currentState = NORMAL_OPERATION;
}

void loop() {
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

  if (otaRequested) {
    otaRequested = false;
    checkForOTA();
  }

  if (myImager.isDataReady()) {
    myImager.getRangingData(&measurementData);
    updateBaseline();
    if (baselineInitialized) {
        processTracking();
    }

    frameCount++;
    if (millis() - lastPrint > 1000) {
      Serial.print("FPS:");
      Serial.println(frameCount);
      frameCount = 0;
      lastPrint = millis();
    }
  }
  delay(5);
}
