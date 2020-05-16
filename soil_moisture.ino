extern "C" {
  #include "user_interface.h"
}
#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80);

#define RTC_USER_MEMORY_START 65;
#define RTC_USER_MEMORY_LEN 127;
#define HOSTNAME_PREFIX "smartgarden-"

const int MOISTURE_MIN_DEFAULT = 250;
const int MOISTURE_MAX_DEFAULT = 632;

// Deep Sleep duration
const int SLEEP_TIME_MIN = 30;

// Send data every x * SLEEP_TIME_MIN
const int SEND_FREQUENCY = 4;

struct Config {
  char hostname[100];
  char wifiSsid[100];
  char wifiPass[100];
  char mqttServer[16];
  int mqttPort;
  char mqttTopic[100];
  int moistureMin;
  int moistureMax;
};

struct Config conf;

typedef struct {
  int battery;
  int wakeUpCounter;
} rtcStore;

rtcStore rtcMem;

/**
 * Write to RTC user memory
 *
 * @param inVal
 * @param offset
 */
void writeRtc(rtcStore *inVal, int offset = 0) {
  offset += RTC_USER_MEMORY_START;
  int buckets = sizeof(*inVal) / 4;
  if (buckets == 0) buckets = 1;

  system_rtc_mem_write(offset, inVal, buckets * 4);
}

/**
 * Read from RTC user memory
 *
 * @param inVal
 * @param offset
 */
void readRtc(rtcStore *inVal, int offset = 0) {
  offset += RTC_USER_MEMORY_START;

  system_rtc_mem_read(offset, inVal, sizeof(*inVal));
}

/**
 * Setup
 */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Setup");

  // Load config
  readConfig();

  // Load wake up counter
  readRtc(&rtcMem);

  // Sensor not set up
  if (!conf.wifiSsid || strlen(conf.wifiSsid) == 0) {
    // Setup WiFi
    setupWifi();

    // Setup webserver
    setupWebserver();

    return;
  }
  // Sensor set up and send cycle
  else if (strlen(conf.mqttServer) > 0 && rtcMem.wakeUpCounter % SEND_FREQUENCY == 0) {
    // Prevent integer overflow
    rtcMem.wakeUpCounter = 0;

    // Setup WiFi
    setupWifi();

    // Connect to MQTT broker
    client.setServer(conf.mqttServer, conf.mqttPort);

    // Read sensor
    float moistureLevel = readSensor();

    // Publish sensor data
    mqttPublish(moistureLevel);

    // Precaution
    delay(100);
  }

  // Increase wake up counter
  rtcMem.wakeUpCounter++;
  writeRtc(&rtcMem);

  // Enter deep sleep
  Serial.println("Going to sleep...");
  ESP.deepSleep(SLEEP_TIME_MIN * 60 * 1000 * 1000);
}

/**
 * Setup webserver
 */
void setupWebserver() {
  // Statics
  server.on("/public/css/design.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/public/css/design.css", "text/css");
  });

  server.on("/public/lib/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/public/lib/bootstrap.min.css", "text/css");
  });

  server.on("/public/js/setup.js", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/public/js/setup.js", "text/javascript");
  });

  // Root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!conf.wifiSsid || strlen(conf.wifiSsid) == 0) {
      request->redirect("/setup");
    }
    else {
      request->send(SPIFFS, "/pages/setup.html", "text/html");
    }
  });

  // Setup
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/pages/setup.html", "text/html");
  });

  // Settings
  server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true)) {
      request->getParam("ssid", true)->value().toCharArray(conf.wifiSsid, 100);
    }

    if (request->hasParam("pass", true)) {
      request->getParam("pass", true)->value().toCharArray(conf.wifiPass, 100);
    }

    if (request->hasParam("mqtt-server", true)) {
      request->getParam("mqtt-server", true)->value().toCharArray(conf.mqttServer, 100);
    }

    if (request->hasParam("mqtt-port", true)) {
      conf.mqttPort = request->getParam("mqtt-port", true)->value().toInt();
    }

    if (request->hasParam("mqtt-topic", true)) {
      request->getParam("mqtt-topic", true)->value().toCharArray(conf.mqttTopic, 100);
    }

    // Save config
    writeConfig();

    // Reconnect WiFi
    setupWifi();

    request->send(200, "text/plain", "message received");
  });

  // Factory Reset
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Reset config
    resetConfig();

    // Reconnect WiFi
    setupWifi();

    request->send(200, "text/plain", "Config reset.");
  });

  // Start webserver
  server.begin();
}

/**
 * Loop
 */
void loop() {}

/**
 * Load config from SPIFFS
 */
void readConfig() {
  char theHostname[20];
  setHostname(theHostname);

  if (SPIFFS.begin()) {
    if (SPIFFS.exists("/config.json")) {
      File configFile = SPIFFS.open("/config.json", "r");

      if (configFile) {
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);

        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        Serial.println();

        strcpy(conf.mqttServer, json["mqtt_server"] | "");
        conf.mqttPort = json.get<int>("mqtt_port") | 1883;
        strcpy(conf.mqttTopic, json["mqtt_topic"] | theHostname);
        strcpy(conf.hostname, json["hostname"] | theHostname);
        strcpy(conf.wifiSsid, json["wifi_ssid"] | "");
        strcpy(conf.wifiPass, json["wifi_pass"] | "");
        conf.moistureMin = json.get<int>("moisture_min") | MOISTURE_MIN_DEFAULT;
        conf.moistureMax = json.get<int>("moisture_max") | MOISTURE_MAX_DEFAULT;

        if (json.success()) {
          Serial.println("Parsed JSON");
          return;
        }
      }
    }
    else {
      Serial.println("[Error] No config found");
    }
  }
  else {
    Serial.println("[ERROR] Failed to mount file system");
  }

  strcpy(conf.mqttServer, "");
  strcpy(conf.mqttTopic, theHostname);
  strcpy(conf.hostname, theHostname);
}

/**
 * Save config to SPIFFS
 */
void writeConfig() {
  Serial.println("Saving config...");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();

  json["mqtt_server"] = conf.mqttServer;
  json["mqtt_port"] = conf.mqttPort;
  json["mqtt_topic"] = conf.mqttTopic;
  json["hostname"] = conf.hostname;
  json["wifi_ssid"] = conf.wifiSsid;
  json["wifi_pass"] = conf.wifiPass;
  json["moisture_min"] = conf.moistureMin;
  json["moisture_max"] = conf.moistureMax;

  File configFile = SPIFFS.open("/config.json", "w");

  if (!configFile) {
    Serial.println("[Error] Failed to open config file for writing");
    return;
  }

  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

/**
 * Delete all data from config in SPIFFS
 */
void resetConfig() {
  Serial.println("Resetting config...");

  strcpy(conf.mqttServer, "");
  conf.mqttPort = 1883;
  strcpy(conf.mqttTopic, "");
  strcpy(conf.hostname, "");
  strcpy(conf.wifiSsid, "");
  strcpy(conf.wifiPass, "");

  writeConfig();
}

/**
 * Generate random hostname
 *
 * @return char[]
 */
void setHostname(char* hostname) {
  srand(time(NULL));
  strcpy(hostname, HOSTNAME_PREFIX);

  int random = rand() % (9999 + 1 - 1000) + 1000;
  char randomBuffer[4];
  sprintf(randomBuffer, "%d", random);

  strcat(hostname, randomBuffer);
}

/**
 * Setup WiFi
 */
void setupWifi() {
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);

  if (strlen(conf.wifiSsid) > 0) {
    WiFi.hostname(conf.hostname);
    WiFi.mode(WIFI_STA);
    WiFi.begin(conf.wifiSsid, conf.wifiPass);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
      if ((unsigned long)(millis() - startTime) >= 5000) break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    char hostname[20];
    setHostname(hostname);
    Serial.println("Setting up softAP");
    Serial.println(hostname);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(hostname);
  }

  WiFi.printDiag(Serial);
}

/**
 * Reconnect to MQTT server
 */
void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(conf.hostname)) {
      Serial.println(" connected.");
    }
    else {
      Serial.print(" failed for hostname ");
      Serial.print(conf.hostname);
      Serial.print(", rc=");
      Serial.print(client.state());
      Serial.println(" - Try again in 2 seconds");

      // Wait 5 seconds before retrying
      delay(2000);
    }
  }
}

/**
 * Publish MQTT message
 *
 * @param moistureLevel
 */
void mqttPublish(float moistureLevel) {
  if (!conf.mqttServer || strlen(conf.mqttServer) == 0) {
    return;
  }

  if (!client.connected()) {
    mqttReconnect();
  }

  client.loop();

  DynamicJsonBuffer jsonBuffer;
  JsonObject& msg = jsonBuffer.createObject();

  msg["moisture"] = moistureLevel;
  msg["battery"] = 100;

  char msgBuffer[100];
  msg.printTo(msgBuffer, sizeof(msgBuffer));
  Serial.println(msgBuffer);

  client.publish(conf.mqttTopic, msgBuffer);
}

/**
 * Read moisture sensor
 *
 * @return float
 */
float readSensor() {
  int moisture = analogRead(0);

  int moistureMin = min(moistureMin, moisture);
  int moistureMax = max(moistureMax, moisture);

  // Update config if necessary
  if (moistureMin < conf.moistureMin) {
    conf.moistureMin = moistureMin;

    writeConfig();
  }

  if (moistureMax > conf.moistureMax) {
    conf.moistureMax = moistureMax;

    writeConfig();
  }

  int G = moistureMax - moistureMin;
  int W = moisture - moistureMin;
  float p = (100.0 * W) / G;

  return 100.0 - p;
}
