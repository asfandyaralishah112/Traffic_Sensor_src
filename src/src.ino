#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

// ================= WIFI =================
const char* ssid = "EncryptedAir";
const char* password = "BlueSky@786";

// Current firmware version
String currentVersion = "1.0.005";

String versionURL =
"https://raw.githubusercontent.com/asfandyaralishah112/Traffic_Sensor_src/main/version.json";

// ================= VL53 =================
#define SDA_PIN 6
#define SCL_PIN 7
#define PIN_LPN 0
#define PIN_RST 8

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;

uint32_t lastPrint = 0;
uint16_t frameCount = 0;

// =====================================================
// OTA CHECK
// =====================================================
void checkForOTA()
{
  Serial.println("Checking for OTA update...");

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

      WiFiClientSecure updateClient;
      updateClient.setInsecure();

      t_httpUpdate_return ret =
        httpUpdate.update(updateClient, firmwareURL);

      if (ret != HTTP_UPDATE_OK)
        Serial.println("Update failed");
    }
    else
    {
      Serial.println("Already latest version.");
    }
  }
  else
  {
    Serial.printf("Version download failed: %d\n", httpCode);
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
    while (1);
  }

  myImager.setResolution(8 * 8);
  myImager.setRangingFrequency(5);
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

  Serial.println("Connecting WiFi...");

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  // OTA check first
  checkForOTA();

  // Start sensor after OTA
  initVL53();
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  if (myImager.isDataReady())
  {
    myImager.getRangingData(&measurementData);

    for (int i = 0; i < 64; i++)
    {
      Serial.print(measurementData.distance_mm[i]);
      if (i < 63) Serial.print(",");
    }
    Serial.println();

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
