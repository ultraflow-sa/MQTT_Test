#include "defs.h"
#include "subs.h"
#include <queue>

// ----------- Global Variable Definitions -----------
bool p1prox1On = false;
bool p1prox2On = false;
bool p2prox1On = false;
bool p2prox2On = false;
unsigned long lastMQTTReconnectAttempt = 0;

wifiSettings_t wifiSettings;
Settings settings;

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
AsyncWebServer server(80);
DNSServer dnsServer;

struct MQTTMessage {
  String topic;
  String payload;
  int priority; // Lower number = higher priority
  
  bool operator<(const MQTTMessage& other) const {
    return priority > other.priority; // Reverse for min-heap
  }
};

std::priority_queue<MQTTMessage> mqttQueue;
unsigned long lastMQTTProcess = 0;

void queueMQTTMessage(String topic, String payload, int priority = 5) {
  MQTTMessage msg;
  msg.topic = topic;
  msg.payload = payload;
  msg.priority = priority;
  mqttQueue.push(msg);
}

void processMQTTQueue() {
  if (!mqttQueue.empty() && (millis() - lastMQTTProcess > 50)) { // Process every 50ms
    MQTTMessage msg = mqttQueue.top();
    mqttQueue.pop();
    
    mqttClient.publish(msg.topic.c_str(), msg.payload.c_str(), true);
    Serial.println("Processed priority message: " + msg.topic + " = " + msg.payload);
    
    lastMQTTProcess = millis();
  }
}

// ------------------ Save wifiSettings Function ------------------
bool saveWiFiSettings() {
  DynamicJsonDocument doc(2048);
  doc["ssid"]              = wifiSettings.ssid;
  doc["password"]          = wifiSettings.password;
  doc["htmlVersion"]       = wifiSettings.htmlVersion;
  doc["cssVersion"]        = wifiSettings.cssVersion;
  doc["mqttServerAddress"] = wifiSettings.mqttServerAddress;
  doc["mqttPort"]          = wifiSettings.mqttPort;
  doc["updateTopic"]       = wifiSettings.updateTopic;
  doc["baseUpdateUrl"]     = wifiSettings.baseUpdateUrl;
  
  File file = LittleFS.open(WIFI_SETTINGS_FILE, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open wifiSettings file for writing.");
    return false;
  }
  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write wifiSettings to file.");
    file.close();
    return false;
  }
  file.close();
  Serial.println("wifiSettings saved to LittleFS.");
  return true;
}

// ------------------ Load default wifiSettings -------------------
void loadWiFiDefaults(){
  Serial.println("wifiSettings file not found. Using and saving default wifiSettings.");
  wifiSettings.ssid              = "R&D Wifi";
  wifiSettings.password          = "DontLetMeIn247";
  wifiSettings.htmlVersion       = "0";
  wifiSettings.cssVersion        = "0";
  wifiSettings.mqttServerAddress = HIVEMQ_SERVER;
  wifiSettings.mqttPort          = HIVEMQ_PORT;
  wifiSettings.updateTopic       = "a3/updates";
  wifiSettings.baseUpdateUrl     = "http://mydomain/webupdates";
}
// ------------------ Load Settings Function ------------------
bool loadWiFiSettings() {
  if (!LittleFS.exists(WIFI_SETTINGS_FILE)) { // Changed SPIFFS to LittleFS
    loadWiFiDefaults();
    saveWiFiSettings();
    return false;
  }
  
  File file = LittleFS.open(WIFI_SETTINGS_FILE, FILE_READ); // Changed SPIFFS to LittleFS
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
  wifiSettings.ssid              = doc["ssid"]              | "R&D Wifi";
  wifiSettings.password          = doc["password"]          | "DontLetMeIn247";
  wifiSettings.htmlVersion       = doc["htmlVersion"]       | "0";
  wifiSettings.cssVersion        = doc["cssVersion"]        | "0";
  wifiSettings.mqttServerAddress = doc["mqttServerAddress"] | HIVEMQ_SERVER;
  wifiSettings.mqttPort          = doc["mqttPort"]          | HIVEMQ_PORT;
  wifiSettings.updateTopic       = doc["updateTopic"]       | "a3/updates";
  wifiSettings.baseUpdateUrl     = doc["baseUpdateUrl"]     | "http://mydomain/webupdates";
  
  Serial.println("wifiSettings loaded from LittleFS:");
  Serial.println("SSID: " + wifiSettings.ssid);
  Serial.println("Password: " + wifiSettings.password);
  Serial.println("HTML Version: " + wifiSettings.htmlVersion);
  Serial.println("CSS Version: " + wifiSettings.cssVersion);
  Serial.println("MQTT Server Address: " + wifiSettings.mqttServerAddress);
  Serial.println("MQTT Port: " + String(wifiSettings.mqttPort));
  Serial.println("Update Topic: " + wifiSettings.updateTopic);
  Serial.println("Base Update URL: " + wifiSettings.baseUpdateUrl);
  
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

void publishState() {
    DynamicJsonDocument doc(256);
    doc["serial"] = serialNumber;
    doc["version"] = VER_STRING;
    String payload;
    serializeJson(doc, payload);
    String topic = "a3/" + serialNumber + "/state";
    sendMQTTMessage(topic, payload);
}

String getP1SettingsJsonStr() {
  DynamicJsonDocument doc(1024);
  // --- PUMP 1 ---
  doc["modeP1"] = String(settings.P1_MAIN_MODE); //Main mode of pump1, PLS or SLS
  doc["runModeP1"] = String(settings.P1_RUN_MODE); //Run mode of pump1, TIME or CYCLES
  doc["pauseTimeP1"] = settings.P1_PAUSE_TIME; //Pause time of pump1 in seconds, used for both time and cycle mode
  doc["timeCyclesP1"] = String(settings.P1_RUN_TIME_CYC); // Run time for pump1 in seconds, converts to minutes and seconds in TIME run mode or cycles in CYCLE mode, pump on
  doc["proxy1P1"] = static_cast<bool>(settings.P1_PROX1); // Pump1 proxy1 in use, YES or NO, automatic if Cycle mode is used
  doc["dwellTimeP1Px1"] = settings.P1_PROX1_DWELL;  // Pump1 proxy1 dwell time in seconds, convert to hours and minutes
  doc["proxy2P1"] = static_cast<bool>(settings.P1_PROX2); // Pump1 proxy2 in use, YES or NO 
  doc["dwellTimeP1Px2"] = settings.P1_PROX2_DWELL;  // Pump1 proxy2 dwell time in seconds, convert to hours and minutes
  doc["levelP1"] = static_cast<bool>(settings.P1_LVL);  // Use level detection for pump1, YES or NO
  doc["levelTypeP1"] = String(settings.P1_LVL_TYPE); // Level type for pump1, Low Level only, LLO, Pulsed full, PULF, Pulsed empty, PULE, or Hall sensor level, HEF
  doc["levelNoncP1"] = String(settings.P1_LVL_NONC); // If LLO level type for pump1, level NO or NC

  String out;
  serializeJson(doc, out);

  // Debug: print the generated JSON string
  Serial.print("getP1SettingsJsonStr() JSON: ");
  Serial.println(out);

  return out;
}

String getP2SettingsJsonStr(){
  DynamicJsonDocument doc(1024);
  // --- PUMP 2 ---
  doc["pump2InUse"] = static_cast<bool>(settings.PUMP2_IN_USE); // Pump2 in use, YES or NO
  doc["modeP2"] = String(settings.P2_MAIN_MODE); //Main mode of pump2, PLS or SLS
  doc["runModeP2"] = String(settings.P2_RUN_MODE); //Run mode of pump2, TIME or CYCLES
  doc["pauseTimeP2"] = settings.P2_PAUSE_TIME; //Pause time of pump2 in seconds, used for both time and cycle mode
  doc["timeCyclesP2"] = String(settings.P2_RUN_TIME_CYC); // Run time for pump2 in seconds, converts to minutes and seconds in TIME run mode or cycles in CYCLE mode, pump on
  doc["proxy1P2"] = static_cast<bool>(settings.P2_PROX1); // Pump2 proxy1 in use, YES or NO, automatic if Cycle mode is used
  doc["dwellTimeP2Px1"] = settings.P2_PROX1_DWELL;  // Pump2 proxy1 dwell time in seconds, convert to hours and minutes
  doc["proxy2P2"] = static_cast<bool>(settings.P2_PROX2); // Pump2 proxy2 in use, YES or NO
  doc["dwellTimeP2Px2"] = settings.P2_PROX2_DWELL; // Pump2 proxy2 dwell time in seconds, convert to hours and minutes
  doc["levelP2"] = static_cast<bool>(settings.P2_LVL); // Use level detection for pump2, YES or NO
  doc["levelTypeP2"] = String(settings.P2_LVL_TYPE);  // Level type for pump2, Low Level only, LLO, Pulsed full, PULF, Pulsed empty, PULE, or Hall sensor level, HEF
  doc["levelNoncP2"] = String(settings.P2_LVL_NONC); // If LLO level type for pump2, level NO or NC

  String out;
  serializeJson(doc, out);

  // Debug: print the generated JSON string
  Serial.print("getP2SettingsJsonStr() JSON: ");
  Serial.println(out);

  return out;
}

String getExtraSettingsJsonStr() {
  DynamicJsonDocument doc(512);
  // --- EXT LAMP ---
  doc["extLampInUse"] = static_cast<bool>(settings.EXT_LAMP);
  doc["extLampType"] = String(settings.LAMP_TYP);

  // --- BLOCKAGE CURRENT ---
  doc["blockCurrentP1"] = settings.P1_BLOCK_CURRENT;
  doc["blockCurrentP2"] = settings.P2_BLOCK_CURRENT;

  String out;
  serializeJson(doc, out);

  // Debug: print the generated JSON string
  Serial.print("getExtraSettingsJsonStr() JSON: ");
  Serial.println(out);

  return out;
}

// ------------------ MQTT Callback ------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("MQTT message received on topic: ");
  Serial.println(topic);
  Serial.println("Message: " + msg);

  String updateTopic = "a3/" + serialNumber + "/update";
  String querySerialTopic = "a3/" + serialNumber + "/querySerial";
  String identifyYourselfTopic = "a3/identifyYourself";
  String queryP1SettingsTopic = "a3/" + serialNumber + "/queryP1Settings";
  String queryP2SettingsTopic = "a3/" + serialNumber + "/queryP2Settings";
  String queryXtraSettingsTopic = "a3/" + serialNumber + "/queryXtraSettings";
  String switchPump1Topic = "a3/" + serialNumber + "/test/pump1";
  String switchPump2Topic = "a3/" + serialNumber + "/test/pump2";
  String p1proxy1Topic = "a3/" + serialNumber + "/test/p1proxy1";
  String p1proxy2Topic = "a3/" + serialNumber + "/test/p1proxy2";
  String p2proxy1Topic = "a3/" + serialNumber + "/test/p2proxy1";
  String p2proxy2Topic = "a3/" + serialNumber + "/test/p2proxy2";
  String P1settingsReplyTopic = "a3/" + serialNumber + "/P1settingsReply";
  String P2settingsReplyTopic = "a3/" + serialNumber + "/P2settingsReply";
  String xtraSettingsReplyTopic = "a3/" + serialNumber + "/xtraSettingsReply";

  if (String(topic) == updateTopic) {
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
      wifiSettings.ssid = newSSID;
      wifiSettings.password = newPassword;
      saveWiFiSettings();
      startStationMode(newSSID, newPassword);
    }
  }
  else if (String(topic) == querySerialTopic) {
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
  else if (String(topic) == queryP1SettingsTopic) {
    // Send settings as JSON string
    String settingsJson = getP1SettingsJsonStr();
    mqttClient.publish(P1settingsReplyTopic.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent settings JSON via MQTT.");
    return;
  }
  else if (String(topic) == queryP2SettingsTopic) {
    // Send settings as JSON string
    String settingsJson = getP2SettingsJsonStr();
    mqttClient.publish(P2settingsReplyTopic.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent P2 settings JSON via MQTT.");
    return;
  }
  else if (String(topic) == queryXtraSettingsTopic) {
    // Send extra settings as JSON string
    String settingsJson = getExtraSettingsJsonStr();
    mqttClient.publish(xtraSettingsReplyTopic.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent extra settings JSON via MQTT.");
    return;
  }
  // ----------- Test Mode Pump1 MQTT Logic -----------
  else if (String(topic) == switchPump1Topic) {
    if (msg == "on") {
      digitalWrite(pump1Out, HIGH);
      queueMQTTMessage(switchPump1Topic, "running", 1); // High priority response
      Serial.println("Pump1 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump1Out, LOW);
      queueMQTTMessage(switchPump1Topic, "stopped", 1); // High priority response
      Serial.println("Pump1 turned OFF (test mode)");
    } else if (msg == "currentAutoCalibrate") {
      // Set boolean to trigger auto calibration in loop()
      pump1AutoCalibrate = true;
      Serial.println("P1 auto calibration requested");
    }
  }
  // ----------- Test Mode Pump2 MQTT Logic -----------
  else if (String(topic) == switchPump2Topic) {
    if (msg == "on") {
      digitalWrite(pump2Out, HIGH);
      sendMQTTMessage(switchPump2Topic, "running");
      Serial.println("Pump2 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump2Out, LOW);
      sendMQTTMessage(switchPump2Topic, "stopped");
      Serial.println("Pump2 turned OFF (test mode)");
    } else if (msg == "currentAutoCalibrate") {
      // Set boolean to trigger auto calibration in loop()
      pump2AutoCalibrate = true;
      Serial.println("P2 auto calibration requested");
    }
  }
}

// ------------------ OTA and Web Server Endpoints ------------------
void setupServerEndpoints() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html"); // Changed SPIFFS to LittleFS
  });
  server.on("/main.html", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/main.html", "text/html"); // Changed SPIFFS to LittleFS
  });
  server.on("/savewifi", HTTP_POST, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid", true) && request->hasParam("password", true)) {
      String newSSID = request->getParam("ssid", true)->value();
      String newPassword = request->getParam("password", true)->value();
      wifiSettings.ssid = newSSID;
      wifiSettings.password = newPassword;
      saveWiFiSettings();
      request->send(200, "text/plain", "wifiSettings saved. Reboot device to apply new WiFi configuration.");
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

// ------------------ Setup and Loop ------------------
void setup() {
  Serial.begin(115200);
  if (!LittleFS.begin(true)) { // Changed SPIFFS to LittleFS
    Serial.println("LittleFS initialisation failed!");
    return;
  }
  settings = readSettings();
  loadWiFiSettings();
  if (wifiSettings.ssid.length() > 0 && wifiSettings.password.length() > 0) {
    startStationMode(wifiSettings.ssid, wifiSettings.password);
  } 
  else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Setup-A3");
    Serial.println("Started captive portal mode");
  }

  // Insecure connection for MQTT (no certificate validation)
  wifiClient.setInsecure();

  Serial.println("MQTT Server: " + wifiSettings.mqttServerAddress);
  mqttClient.setServer(wifiSettings.mqttServerAddress.c_str(), wifiSettings.mqttPort);
  mqttClient.setCallback(mqttCallback);
  setupServerEndpoints();
  
  pinMode(upLeft, INPUT_PULLUP);
  pinMode(dnRight, INPUT_PULLUP);
  pinMode(enter, INPUT_PULLUP);
  pinMode(pump1Out, OUTPUT); // Setup Pump1 pin for test mode
  digitalWrite(pump1Out, LOW);
  pinMode(pump2Out, OUTPUT); // Setup Pump2 pin for test mode
  digitalWrite(pump2Out, LOW);
  pinMode(p1prox1In, INPUT_PULLUP);
  pinMode(p1prox2In, INPUT_PULLUP);
  pinMode(p2prox1In, INPUT_PULLUP);
  pinMode(p2prox2In, INPUT_PULLUP);
}

unsigned long lastStatePublish = 0;

void loop() {
  // Process high-priority MQTT messages first
  processMQTTQueue();
  
  readPins();
  if (upLeftReleased == true){
    upLeftReleased = false;
    Serial.println("Up/Left button released");
    loadWiFiDefaults();
    saveWiFiSettings();
    esp_restart();
  }

  if (millis() - lastStatePublish > 500) {
    checkMQTTConnection();
    mqttClient.loop();
    lastStatePublish = millis();
  }

/* void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    String message;
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    
    String topicStr = String(topic);
    Serial.println("Received message: " + message + " on topic: " + topicStr);
  }
*/
  // Handle auto calibration requests
  if (pump1AutoCalibrate == true and pump1Running == false) {
    Serial.println("Starting P1 auto calibration");
    // Send acknowledgment that test is starting
    queueMQTTMessage(switchPump1Topic, "startingTest", 1); // High priority
    // Start pump for calibration
    digitalWrite(pump1Out, HIGH);
    pump1Running = true; // Set running state for pump1
    pump1StartTime = millis();
  }

  if (pump1AutoCalibrate == true and pump1Running == true and (millis() - pump1StartTime >= 10000)) {
    // Stop pump
    digitalWrite(pump1Out, LOW);
    pump1Running = false; // Reset running state for pump1
    pump1AutoCalibrate = false; // Reset auto calibration flag    
    // Send calibrated current value (replace "1" with actual measured value)
    String calibratedCurrent = "1"; // This should be your actual measured current
    sendMQTTMessage(switchPump1Topic, calibratedCurrent);
    Serial.println("P1 auto calibration complete: " + calibratedCurrent);
  }

  if (pump2AutoCalibrate == true and pump2Running == false) {
    Serial.println("Starting P2 auto calibration");
    // Send acknowledgment that test is starting
    sendMQTTMessage(switchPump2Topic, "startingTest");
    // Start pump for calibration
    digitalWrite(pump2Out, HIGH);
    pump2Running = true; // Set running state for pump2
    pump2StartTime = millis();
  }

  if (pump2AutoCalibrate == true and pump2Running == true and (millis() - pump2StartTime >= 10000)) {
    // Stop pump
    digitalWrite(pump2Out, LOW);
    pump2Running = false; // Reset running state for pump2
    pump2AutoCalibrate = false; // Reset auto calibration flag    
    // Send calibrated current value (replace "1" with actual measured value)
    String calibratedCurrent = "1"; // This should be your actual measured current
    sendMQTTMessage(switchPump2Topic, calibratedCurrent);
    Serial.println("P2 auto calibration complete: " + calibratedCurrent);
  }
}