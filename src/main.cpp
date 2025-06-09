#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <FS.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

// ---------- Button Handling ----------
const int upLeft = 5;  // Adjust to your actual pin
int pump1Out = 13;
int p1prox1In = 39;
int p1prox2In = 17;
bool upLeftPressed = false;
bool upLeftLongPressed = false;
bool upLeftReleased = false;
bool checkedProx1Already = false;
bool checkedProx2Already = false;
unsigned long upLeftStartTime = 0;
const unsigned long updnLongPress = 1000;
const unsigned long debounce = 200;
unsigned long lastMQTTReconnectAttempt = 0;

// --------------- Global Settings and Declarations ---------------
struct Settings {
  String ssid;
  String password;
  String htmlVersion;
  String cssVersion;
  String mqttServerAddress;
  int    mqttPort;
  String updateTopic;
  String baseUpdateUrl;
};

#define SETTINGS_FILE "/settings.json"
Settings settings;

String serialNumber = "000001";
String VER_STRING = "v1.0.0";

// ------------------ HiveMQ Cloud MQTT Setup ------------------
// Replace with your HiveMQ Cloud details:
const char* HIVEMQ_SERVER = "bd58f8878eef4f5eb73ac65312b10130.s1.eu.hivemq.cloud"; // <-- Change this!
const int   HIVEMQ_PORT   = 8883;
const char* HIVEMQ_USER   = "a3Admin"; // <-- Change this!
const char* HIVEMQ_PASS   = "DontLetMeIn247"; // <-- Change this!

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
AsyncWebServer server(80);
DNSServer dnsServer;

// ------------------ Save Settings Function ------------------
bool saveSettings() {
  DynamicJsonDocument doc(2048);
  doc["ssid"]              = settings.ssid;
  doc["password"]          = settings.password;
  doc["htmlVersion"]       = settings.htmlVersion;
  doc["cssVersion"]        = settings.cssVersion;
  doc["mqttServerAddress"] = settings.mqttServerAddress;
  doc["mqttPort"]          = settings.mqttPort;
  doc["updateTopic"]       = settings.updateTopic;
  doc["baseUpdateUrl"]     = settings.baseUpdateUrl;
  
  File file = SPIFFS.open(SETTINGS_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open settings file for writing.");
    return false;
  }
  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write settings to file.");
    file.close();
    return false;
  }
  file.close();
  Serial.println("Settings saved to SPIFFS.");
  return true;
}

// ------------------ Load default settings -------------------
void loadDefaults(){
  Serial.println("Settings file not found. Using and saving default settings.");
  settings.ssid              = "R&D Wifi";
  settings.password          = "DontLetMeIn247";
  settings.htmlVersion       = "0";
  settings.cssVersion        = "0";
  settings.mqttServerAddress = HIVEMQ_SERVER;
  settings.mqttPort          = HIVEMQ_PORT;
  settings.updateTopic       = "a3/updates";
  settings.baseUpdateUrl     = "http://mydomain/webupdates";
}
// ------------------ Load Settings Function ------------------
bool loadSettings() {
  if (!SPIFFS.exists(SETTINGS_FILE)) {
    loadDefaults();
    saveSettings();
    return false;
  }
  
  File file = SPIFFS.open(SETTINGS_FILE, FILE_READ);
  if (!file) {
    Serial.println("Failed to open settings file for reading.");
    return false;
  }
  size_t size = file.size();
  DynamicJsonDocument doc(size + 200);
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  if (error) {
    Serial.print("Failed to parse settings: ");
    Serial.println(error.c_str());
    return false;
  }
  settings.ssid              = doc["ssid"]              | "R&D Wifi";
  settings.password          = doc["password"]          | "DontLetMeIn247";
  settings.htmlVersion       = doc["htmlVersion"]       | "0";
  settings.cssVersion        = doc["cssVersion"]        | "0";
  settings.mqttServerAddress = doc["mqttServerAddress"] | HIVEMQ_SERVER;
  settings.mqttPort          = doc["mqttPort"]          | HIVEMQ_PORT;
  settings.updateTopic       = doc["updateTopic"]       | "a3/updates";
  settings.baseUpdateUrl     = doc["baseUpdateUrl"]     | "http://mydomain/webupdates";
  
  Serial.println("Settings loaded from SPIFFS:");
  Serial.println("SSID: " + settings.ssid);
  Serial.println("Password: " + settings.password);
  Serial.println("HTML Version: " + settings.htmlVersion);
  Serial.println("CSS Version: " + settings.cssVersion);
  Serial.println("MQTT Server Address: " + settings.mqttServerAddress);
  Serial.println("MQTT Port: " + String(settings.mqttPort));
  Serial.println("Update Topic: " + settings.updateTopic);
  Serial.println("Base Update URL: " + settings.baseUpdateUrl);
  
  return true;
}

// ------------------ WiFi and Mode Functions ------------------
void startStationMode(const String &wifiSSID, const String &wifiPassword) {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  Serial.print("Connecting to STA (up to 15 sec)...");
  unsigned long startTime = millis();
  bool connected = false;
  while (millis() - startTime < 15000) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (connected) {
    Serial.println("STA connected! IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("STA connection failed.");
  }
}

// ------------------ MQTT Messaging Functions ------------------
void sendMQTTMessage(const String &topic, const String &payload) {
  if (mqttClient.connected()) {
    if(mqttClient.publish(topic.c_str(), payload.c_str())) {
      Serial.println("Published to " + topic + ": " + payload);
    } else {
      Serial.println("Failed to publish to " + topic);
    }
  } else {
    Serial.println("MQTT client not connected, cannot publish to " + topic);
  }
}

void subscribeMQTTTopic(const String &topic) {
  if (mqttClient.connected()) {
    mqttClient.subscribe(topic.c_str());
    Serial.println("Subscribed to " + topic);
  } else {
    Serial.println("MQTT client not connected, cannot subscribe to " + topic);
  }
}

void checkMQTTConnection() {
  if (!mqttClient.connected() && (millis() - lastMQTTReconnectAttempt > 5000)) {
    lastMQTTReconnectAttempt = millis();
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    // Connect using HiveMQ username/password
    if (mqttClient.connect(clientId.c_str(), HIVEMQ_USER, HIVEMQ_PASS)) {
      Serial.println(" connected.");
      subscribeMQTTTopic("a3/" + serialNumber + "/update");
      subscribeMQTTTopic("a3/" + serialNumber + "/querySerial");
      subscribeMQTTTopic("a3/identifyYourself");
      subscribeMQTTTopic("a3/test/pump1"); // Subscribe for test mode pump1
      subscribeMQTTTopic("a3/test/proxy1");
      subscribeMQTTTopic("a3/test/proxy2"); 
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - will retry in 5 seconds");
    }
  }
}

void publishState() {
    DynamicJsonDocument doc(256);
    doc["serial"] = serialNumber;
    doc["version"] = VER_STRING;
    String payload;
    serializeJson(doc, payload);
    String topic = "a3/" + serialNumber + "/state";
    sendMQTTMessage(topic, payload);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("MQTT message received on topic: ");
  Serial.println(topic);
  Serial.println("Message: " + msg);

  String expectedUpdateTopic = "a3/" + serialNumber + "/update";
  String expectedQuerySerialTopic = "a3/" + serialNumber + "/querySerial";
  String identifyYourselfTopic = "a3/identifyYourself";

  if (String(topic) == expectedUpdateTopic) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("MQTT JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    String newSSID = doc["ssid"] | "";
    String newPassword = doc["password"] | "";
    if(newSSID.length() > 0 && newPassword.length() > 0) {
      Serial.println("Received WiFi credentials via MQTT:");
      Serial.println("SSID: " + newSSID);
      settings.ssid = newSSID;
      settings.password = newPassword;
      saveSettings();
      startStationMode(newSSID, newPassword);
    }
  }
  else if (String(topic) == expectedQuerySerialTopic) {
    Serial.println("Received serial number query via MQTT.");
    DynamicJsonDocument responseDoc(256);
    responseDoc["serial"] = serialNumber;
    responseDoc["version"] = VER_STRING;
    String responsePayload;
    serializeJson(responseDoc, responsePayload);
    sendMQTTMessage("a3/" + serialNumber + "/serialResponse", responsePayload);
  }
  else if (String(topic) == identifyYourselfTopic) {
    Serial.println("Received identify request via MQTT.");
    // Respond with serial only (for device discovery)
    DynamicJsonDocument responseDoc(128);
    responseDoc["serial"] = serialNumber;
    String responsePayload;
    serializeJson(responseDoc, responsePayload);
    sendMQTTMessage("a3/identifyResponse", responsePayload);
  }
  // ----------- Test Mode Pump1 MQTT Logic -----------
  else if (String(topic) == "a3/test/pump1") {
    if (msg == "on") {
      digitalWrite(pump1Out, HIGH);
      sendMQTTMessage("a3/test/pump1", "running");
      Serial.println("Pump1 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump1Out, LOW);
      sendMQTTMessage("a3/test/pump1", "stopped");
      Serial.println("Pump1 turned OFF (test mode)");
    }
  }
}

// ------------------ OTA and Web Server Endpoints ------------------
void setupServerEndpoints() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });
  server.on("/main.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/main.html", "text/html");
  });
  server.on("/savewifi", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      String newSSID = request->getParam("ssid", true)->value();
      String newPassword = request->getParam("password", true)->value();
      settings.ssid = newSSID;
      settings.password = newPassword;
      saveSettings();
      request->send(200, "text/plain", "Settings saved. Reboot device to apply new WiFi configuration.");
    } else {
      request->send(400, "text/plain", "Missing parameters.");
    }
  });
  server.on("/ota", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "OTA endpoint placeholder");
  });
  server.begin();
  Serial.println("Web server started.");
}

void readPins() {
  if (digitalRead(upLeft) == LOW && !upLeftPressed && !upLeftLongPressed) {
      Serial.println("Up/Left pressed");
      upLeftStartTime = millis();
      upLeftPressed = true;
  }
  if (upLeftPressed && millis() - upLeftStartTime >= updnLongPress && digitalRead(upLeft) == LOW && !upLeftLongPressed) {
      Serial.println("Up/Left Long Pressed");
      upLeftLongPressed = true;
      upLeftReleased = false;
  }
  if (upLeftPressed && millis() - upLeftStartTime >= debounce && digitalRead(upLeft) == HIGH) {
      upLeftReleased = true;
      Serial.println("Up/Left released");
      upLeftPressed = false;
      upLeftLongPressed = false;
  }
  if (digitalRead(p1prox1In) == LOW) {
      Serial.println("Pump1 Prox1 Triggered");
      checkedProx1Already = false; // Reset checked state on trigger
      sendMQTTMessage("a3/test/proxy1", "on");
  }
  if (digitalRead(p1prox2In) == LOW) {
      Serial.println("Pump1 Prox2 Triggered");
      checkedProx2Already = false; // Reset checked state on trigger
      sendMQTTMessage("a3/test/proxy2", "on");
  }
  if (digitalRead(p1prox1In) == HIGH and checkedProx1Already == false) {
      Serial.println("Pump1 Prox1 Released");
      checkedProx1Already = true;
      sendMQTTMessage("a3/test/proxy1", "off");
  }
  if (digitalRead(p1prox2In) == HIGH and checkedProx2Already == false) {
      Serial.println("Pump1 Prox2 Released");
      checkedProx2Already = true;
      sendMQTTMessage("a3/test/proxy2", "off");
  }
}

// ------------------ Setup and Loop ------------------
void setup() {
  Serial.begin(115200);
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed!");
    return;
  }
  loadSettings();
  if (settings.ssid.length() > 0 && settings.password.length() > 0) {
    startStationMode(settings.ssid, settings.password);
  } 
  else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Setup-A3");
    Serial.println("Started captive portal mode");
  }

  // Insecure connection for MQTT (no certificate validation)
  wifiClient.setInsecure();

  Serial.println("MQTT Server: " + settings.mqttServerAddress);
  mqttClient.setServer(settings.mqttServerAddress.c_str(), settings.mqttPort);
  mqttClient.setCallback(mqttCallback);
  setupServerEndpoints();
  
  pinMode(upLeft, INPUT_PULLUP);
  pinMode(pump1Out, OUTPUT); // Setup Pump1 pin for test mode
  digitalWrite(pump1Out, LOW);
  pinMode(p1prox1In, INPUT_PULLUP);
  pinMode(p1prox2In, INPUT_PULLUP);
}

unsigned long lastStatePublish = 0;

void loop() {
  readPins();
  if (upLeftReleased == true){
    upLeftReleased = false;
    Serial.println("Up/Left button released");
    loadDefaults();
    saveSettings();
    esp_restart();
  }

  if (millis() - lastStatePublish > 500) {
    checkMQTTConnection();
    mqttClient.loop();
    lastStatePublish = millis();
  }
}