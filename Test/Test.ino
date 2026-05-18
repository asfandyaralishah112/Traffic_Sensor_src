#define SENSOR1_PIN 2
#define SENSOR2_PIN 3
#define PIR_PIN     5
#define PIR_PIN1     6

void setup() {

    Serial.begin(115200);

    analogReadResolution(12); // 0 - 4095

    pinMode(PIR_PIN, INPUT);
    pinMode(PIR_PIN1, INPUT);

    Serial.println("ADC + PIR Monitor Started");
}

void loop() {

    int sensor1 = analogRead(SENSOR1_PIN);
    int sensor2 = analogRead(SENSOR2_PIN);

    int pirState = digitalRead(PIR_PIN);
    int pir1State = digitalRead(PIR_PIN1);

    // Serial.print("GPIO2: ");
    // Serial.print(sensor1);

    // Serial.print("    GPIO3: ");
    // Serial.print(sensor2);

    Serial.print("    PIR(GPIO5): ");

    if (pirState) {
        Serial.println("MOTION");
    } else {
        Serial.println("NO MOTION");
    }
    Serial.print("    PIR(GPIO6): ");

    if (pirState) {
        Serial.println("MOTION");
    } else {
        Serial.println("NO MOTION");
    }

    delay(1);
}