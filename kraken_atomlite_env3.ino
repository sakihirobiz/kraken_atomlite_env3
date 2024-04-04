#include <M5Atom.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "Kraken.h"

// Sensor
#include <Adafruit_SHT31.h>
#include "QMP6988.h"

// LED colors
#define LED_GREEN 0x00f000
#define LED_RED 0xf00000
#define LED_BLUE 0x0000f0
#define LED_PURPLE 0xf000f0
#define LED_WHITE 0xf0f0f0

// Device Type
#define DEVICE_TYPE "atomenv3"
#define RESET_INTERVAL_MSEC 60 * 60 * 1000   // Device will reset each 60min.

// WiFi
#define WIFI_SSID "[WIFI-SSID]"
#define WIFI_PASS "[WIFI-PASSWORD]"

// MQTT
#define MQTT_PORT 1883
#define MQTT_TOPIC "kraken"
#define MQTT_HOST "[MQTT HOST]"
// Other Settings
#define DEBUG_MODE 0                // 0: Release 1: Debug (WiFi/MQTT do not establish)

// Setup clients
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
// MQTT Client ID
String clientId = "KRKNATOMENV3_" + String(random(0xffff), HEX) + String(random(0xffff), HEX);
// Kraken
bool DEVICE_STARTED = false;
Kraken kraken;
// ENV3
Adafruit_SHT31 sht3x;
QMP6988 qmp6988;

float tmp      = 0.0;
float hum      = 0.0;
float pressure = 0.0;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message arrived [%s]\n", topic);
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    M5.dis.drawpix(0, LED_RED);
    Serial.printf("MQTT disconnected. Retry to connect...\n");
    connectMqtt();
  }
  mqttClient.loop();
}

void connectMqtt() {
    // 
    int retryCount = 0;
    Serial.println("connecting mqtt\n");
    while (!mqttClient.connected()) {
        if (mqttClient.connect(clientId.c_str())) {
            M5.dis.drawpix(0, LED_PURPLE);
            Serial.println("Connected to Server\n");
            Serial.println(WiFi.localIP());
        } else {
            M5.dis.drawpix(0, LED_GREEN);
            Serial.println("Connection error\n");
            Serial.println(mqttClient.state());
            // Wait 5 seconds before retrying
            delay(5000);
            if (retryCount > 4) esp_restart(); // reboot device.
                ++retryCount;
        }
    }
}

void connectWifi() {
  int retryCount = 0;
  Serial.println("Connecting to");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    M5.dis.drawpix(0, LED_RED);
    delay(500);
    Serial.print(".");
    ++retryCount;
    if (retryCount > 30) {
      Serial.println("Can not connect to WiFi. Reboot device.\n");
      esp_restart(); // reboot device.
    }
  }
  M5.dis.drawpix(0, LED_WHITE);
  Serial.println("Connected!\n");
}

void publish(float tmp, float hum, float pressure, char *topic) {
    DynamicJsonDocument doc(1024);
    doc["type"] = "env3";
    doc["tmp"] = tmp;
    doc["hum"] = hum;
    doc["pressure"] = pressure;
    doc["device"] = DEVICE_TYPE;
    String json;
    serializeJson(doc, json);
    Serial.println("pub_json");
    Serial.println(json);
  // Publish a message to MQTT server
    if (DEBUG_MODE == 0) {
    if (mqttClient.publish(topic, json.c_str())) {
        M5.dis.drawpix(0, LED_GREEN);
        Serial.println("MQTT Message published!\n");
    } else {
        M5.dis.drawpix(0, LED_PURPLE);
        Serial.println("ERROR: MQTT Message was not published...\n");
    }
    delay(1000);
    M5.dis.drawpix(0, LED_WHITE);
    }
    doc.clear();
    json.clear();
}

void setup() {
    //upload speed
    Serial.begin(115200);

    //device start
    M5.begin(true, false, true);
    
    //serial communication start
    Wire.begin(26, 32);

    //Sensor start
    Serial.println(F("ENVIII Unit(SHT30 and QMP6988)"));
    qmp6988.init();
    sht3x.begin();

    if (DEBUG_MODE == 0) {
        // WiFi
        connectWifi();
        // Serial.printf("IP %s\n", WiFi.localIP().toString().c_str());
        // MQTT
        mqttClient.setServer(MQTT_HOST, MQTT_PORT);
        mqttClient.setCallback(mqttCallback);
        connectMqtt();
        kraken.Init(&Serial, &mqttClient, MQTT_TOPIC, DEVICE_TYPE, DEBUG_MODE);
    } else {
        Serial.println("DEBUG: NO MQTT\n");
    }
}

void loop() {

    //
    if (DEBUG_MODE == 0) {
        mqttLoop();
        if (!DEVICE_STARTED) {
            kraken.sendStatus(String("device_started"), 1, MQTT_TOPIC);
            DEVICE_STARTED = true;
        }
        if (millis() > RESET_INTERVAL_MSEC) {
            kraken.sendStatus(String("device_reset"), 2, MQTT_TOPIC);
            delay(100);
            esp_restart();
        }
    }
    
    // get env value
    float tmp = sht3x.readTemperature();
    float hum = sht3x.readHumidity();
    float pressure = qmp6988.calcPressure();

    publish(tmp, hum, pressure, MQTT_TOPIC);
    Serial.println("Temp, Humi ,Pressure");
    Serial.println(tmp);
    Serial.println(hum);
    Serial.println(pressure);
    M5.dis.drawpix(0, LED_WHITE);
    delay(10000);
}