#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80);

char* getHostname();

int moi = 0;
int moistMin = 250;
int moistMax = 632;

struct Config {
  char hostname[100];
  char wifiSsid[100];
  char wifiPass[100];
  char mqttServer[16];
  int mqttPort;
  char mqttTopic[100];
};

struct Config conf;

void setup() {
  delay(1000);
  Serial.begin(115200);

  // Load config
  loadConfig(conf);

  // Setup WiFi
  setupWifi(conf);

  // Setup webserver
  setupWebserver();

  if (strlen(conf.mqttServer) > 0) {
    Serial.println("Connecting to MQTT:");
    Serial.println(conf.mqttServer);
    Serial.println(strlen(conf.mqttServer));
    // Connect to MQTT broker
    client.setServer(conf.mqttServer, conf.mqttPort);
  }
}

void setupWebserver() {
  // Statics
  server.on("/public/css/design.css", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[GET] /public/css/design.css");
    request->send(SPIFFS, "/public/css/design.css", "text/css");
  });

  server.on("/public/lib/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[GET] /public/lib/bootstrap.min.css");

    if (SPIFFS.exists("/public/lib/bootstrap.min.css")) {
      Serial.println("Bootstrap exists!");
    }
    else {
      Serial.println("Bootstrap does not exist?!");
    }

    request->send(SPIFFS, "/public/lib/bootstrap.min.css", "text/css");
  });

  server.on("/public/js/setup.js", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[GET] /public/js/setup.js");
    request->send(SPIFFS, "/public/js/setup.js", "text/javascript");
  });

  // Root
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("[GET] /");
    if (strlen(conf.wifiSsid) == 0) {
      request->redirect("/setup");
    }
    else {
      request->send(SPIFFS, "/pages/setup.html", "text/html");
    }
  });

  // Setup
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("[GET] /setup");
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
      request->getParam("mqtt-topic", true)->value().toCharArray(conf.hostname, 100);
    }

    // Save config
    saveConfig(conf);

    // Reconnect WiFi
    setupWifi(conf);

    request->send(200, "text/plain", "message received");
  });

  // Factory Reset
  server.on("/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Reset config
    resetConfig(conf);

    // Reconnect WiFi
    setupWifi(conf);

    request->send(200, "text/plain", "Config reset.");
  });

  // Start webserver
  server.begin();
}

void loop() {
  moi = readSensor();

  //Serial.println(String(moi) + "%");

  mqttPublish(moi);

  delay(1000);
}

void loadConfig(Config &conf) {
  char theHostname[100];
  setHostname(theHostname);
  
  //const char* theHostname = "smartgarden-1234"; //getHostname();

  Serial.print("char* hostname = ");
  Serial.println(theHostname);

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

        Serial.print("Hostname: ");
        Serial.println(theHostname);

        strcpy(conf.mqttServer, json["mqtt_server"] | "");
        conf.mqttPort = json.get<int>("mqtt_port") | 1883;
        strcpy(conf.mqttTopic, json["mqtt_topic"] | theHostname);
        strcpy(conf.hostname, json["hostname"] | theHostname);
        strcpy(conf.wifiSsid, json["wifiSsid"] | "");
        strcpy(conf.wifiPass, json["wifiPass"] | "");

        if (json.success()) {
          Serial.println("\n Parsed JSON");
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

  Serial.print("Did set hostname to: ");
  Serial.println(theHostname);

  strcpy(conf.mqttServer, "");
  strcpy(conf.mqttTopic, theHostname);
  strcpy(conf.hostname, theHostname);
}

void saveConfig(const Config &conf) {
  Serial.println("Saving config...");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();

  json["mqtt_server"] = conf.mqttServer;
  json["mqtt_port"] = conf.mqttPort;
  json["mqtt_topic"] = conf.mqttTopic;
  json["hostname"] = conf.hostname;
  json["wifiSsid"] = conf.wifiSsid;
  json["wifiPass"] = conf.wifiPass;

  File configFile = SPIFFS.open("/config.json", "w");

  if (!configFile) {
    Serial.println("[Error] Failed to open config file for writing");
    return;
  }

  json.prettyPrintTo(Serial);
  json.printTo(configFile);
  configFile.close();
}

void resetConfig(Config &conf) {
  Serial.println("Resetting config...");

  strcpy(conf.mqttServer, "");
  conf.mqttPort = 1883;
  strcpy(conf.mqttTopic, "");
  strcpy(conf.hostname, "");
  strcpy(conf.wifiSsid, "");
  strcpy(conf.wifiPass, "");

  saveConfig(conf);
}

char* getHostname() {
  char hostname[100];
  int id = (rand() % 9999 + 1000);
  sprintf(hostname, "%s%d", "smartgarden-", id);

  Serial.print("getHostname: ");
  Serial.println(hostname);

  return hostname;
}

void setHostname(char* hostname) {
        srand(time(NULL));
        strcpy(hostname, "smartgarden-");

        int random = rand() % (9999 + 1 - 1000) + 1000;
        char randomBuffer[4];
        sprintf(randomBuffer, "%d", random);
        
        strcat(hostname, randomBuffer);
}

void setupWifi(Config &conf) {
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(1000);

  if (strlen(conf.wifiSsid) > 0) {
    WiFi.hostname(conf.hostname);
    WiFi.mode(WIFI_STA);
    WiFi.begin(conf.wifiSsid, conf.wifiPass);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
      if ((unsigned long)(millis() - startTime) >= 5000) break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Setting up softAP");
    Serial.println(conf.hostname);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(conf.hostname);
  }

  WiFi.printDiag(Serial);
}

void mqttReconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(conf.hostname)) {
      Serial.println(" connected.");
    }
    else {
      Serial.print(" failed, rc=");
      Serial.print(client.state());
      Serial.println(" - Try again in 5 seconds");

      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttPublish(int moi) {
  if (!conf.mqttServer || strlen(conf.mqttServer) == 0) {
    return;
  }

  if (!client.connected()) {
    mqttReconnect();
  }

  client.loop();

  DynamicJsonBuffer jsonBuffer;
  JsonObject& msg = jsonBuffer.createObject();

  msg["moisture"] = moi;
  msg["battery"] = 100;

  char msgBuffer[100];
  msg.printTo(msgBuffer, sizeof(msgBuffer));
  Serial.println(msgBuffer);

  // msg = String(moi).c_str();

  client.publish(conf.mqttTopic, msgBuffer);
}

int readSensor() {
  moi = analogRead(0);

  moistMin = min(moistMin, moi);
  moistMax = max(moistMax, moi);

  Serial.println(String(moistMin) + " | " + String(moi) + " | " + String(moistMax));

  int G = moistMax - moistMin;
  int W = moi - moistMin;
  float p = (100.0 * W) / G;

  return 100.0 - p;
}
