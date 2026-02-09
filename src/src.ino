#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <ArduinoJson.h>

const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";

String currentVersion = "1.0";
String versionURL = "https://raw.githubusercontent.com/USERNAME/REPO/main/version.json";

void checkForOTA() {

  HTTPClient http;
  http.begin(versionURL);

  int httpCode = http.GET();

  if (httpCode == 200) {

    String payload = http.getString();

    StaticJsonDocument<200> doc;
    deserializeJson(doc, payload);

    String newVersion = doc["version"];
    String firmwareURL = doc["firmware"];

    if (newVersion != currentVersion) {

      Serial.println("Updating firmware...");

      t_httpUpdate_return ret =
        httpUpdate.update(firmwareURL);

      switch (ret) {
        case HTTP_UPDATE_FAILED:
          Serial.println("Update Failed");
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("No Update");
          break;

        case HTTP_UPDATE_OK:
          Serial.println("Update Success");
          break;
      }
    }
  }

  http.end();
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  checkForOTA();
}

void loop() {}
