#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>
#define LED_PIN 8

const char* ssid = "air fiber 2.4Ghz_EXT";
const char* password = "lachi@8182";

// Current firmware version on device
String currentVersion = "1.0.004";

// URL to version.json on GitHub
String versionURL =
"https://raw.githubusercontent.com/asfandyaralishah112/Traffic_Sensor_src/main/version.json";

void checkForOTA() {

  Serial.println("Checking for OTA update...");

  WiFiClientSecure client;
  client.setInsecure();   // required for GitHub HTTPS

  HTTPClient http;
  http.begin(client, versionURL);

  int httpCode = http.GET();

  if (httpCode == 200) {

    String payload = http.getString();
    Serial.println("Version file received:");
    Serial.println(payload);

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println("JSON parse failed");
      http.end();
      return;
    }

    String newVersion = doc["version"].as<String>();
    String firmwareURL = doc["firmware"].as<String>();

    Serial.println("Current version: " + currentVersion);
    Serial.println("Available version: " + newVersion);

    if (newVersion != currentVersion) {

      Serial.println("New firmware detected. Starting update...");

      WiFiClientSecure updateClient;
      updateClient.setInsecure();

      t_httpUpdate_return ret =
          httpUpdate.update(updateClient, firmwareURL);

      switch (ret) {

        case HTTP_UPDATE_FAILED:
          Serial.printf("Update failed. Error (%d): %s\n",
                        httpUpdate.getLastError(),
                        httpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("No updates available.");
          break;

        case HTTP_UPDATE_OK:
          Serial.println("Update successful. Rebooting...");
          break;
      }
    }
    else {
      Serial.println("Device already on latest version.");
    }
  }
  else {
    Serial.printf("Failed to download version file. HTTP code: %d\n", httpCode);
  }

  http.end();
}

void setup() {

  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  Serial.println("\nConnecting to WiFi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  checkForOTA();
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(5000);
}
