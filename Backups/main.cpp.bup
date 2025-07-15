#include "defs.h"
#include "subs.h"

// Forward declarations - add these lines
void mqttCallback(char* topic, byte* payload, unsigned int length);
void startEmbeddedBroker();
void stopEmbeddedBroker();
void switchToAPMode();
void attemptSTAReconnection();
void checkWiFiConnection();
bool saveWiFiSettings();
void loadWiFiDefaults();
bool loadWiFiSettings();
void startStationMode(const String &wifiSSID, const String &wifiPassword);
void publishState();
String getP1SettingsJsonStr();
String getP2SettingsJsonStr();
String getExtraSettingsJsonStr();
void setupServerEndpoints();
void attemptWiFiReconnection();
void printWiFiStatus();

// Simple MQTT broker class for AP mode only
class SimpleMQTTBroker {
private:
  WiFiServer server;
  std::vector<WiFiClient> clients;
  bool isRunning = false;
  
public:
  SimpleMQTTBroker(int port = 1883) : server(port) {}
  
  bool begin() {
    server.begin();
    isRunning = true;
    Serial.println("Embedded MQTT Broker started on port 1883");
    return true;
  }
  
  void loop() {
    if (!isRunning) return;
    
    WiFiClient newClient = server.available();
    if (newClient) {
      clients.push_back(newClient);
      Serial.println("New MQTT client connected to embedded broker");
    }
    
    for (auto it = clients.begin(); it != clients.end();) {
      if (!it->connected()) {
        it = clients.erase(it);
      } else {
        if (it->available()) {
          String data = it->readString();
          // Basic MQTT handling for web interface
        }
        ++it;
      }
    }
  }
  
  void stop() {
    isRunning = false;
    for (auto& client : clients) {
      client.stop();
    }
    clients.clear();
    server.close();
    Serial.println("Embedded MQTT Broker stopped");
  }
  
  bool hasClients() {
    return !clients.empty();
  }
};

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

SimpleMQTTBroker* embeddedBroker = nullptr;
bool isEmbeddedBrokerActive = false;
WiFiClient localMqttClient;
PubSubClient localClient(localMqttClient);

void startEmbeddedBroker() {
  if (isEmbeddedBrokerActive) return;
  
  Serial.println("Starting embedded MQTT broker for AP mode...");
  embeddedBroker = new SimpleMQTTBroker(1883);
  
  if (embeddedBroker->begin()) {
    isEmbeddedBrokerActive = true;
    Serial.println("Embedded MQTT broker running at " + WiFi.softAPIP().toString() + ":1883");
    
    // Connect local client to embedded broker
    delay(1000);
    localClient.setServer(WiFi.softAPIP().toString().c_str(), 1883);
    localClient.setCallback(mqttCallback);
    
    String clientId = "ESP32LocalClient_" + serialNumber;
    if (localClient.connect(clientId.c_str())) {
      Serial.println("Local client connected to embedded broker");
      // Subscribe to all topics via your existing function in subs.h
      subscribeMQTTTopic("a3/" + serialNumber + "/update");
      subscribeMQTTTopic("a3/" + serialNumber + "/querySerial");
      subscribeMQTTTopic("a3/identifyYourself");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/pump1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/pump2");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p1proxy1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p1proxy2");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p2proxy1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p2proxy2");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p1level");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p2level");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/extlamp");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/pump1");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/pump2");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryP1Settings");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryP2Settings");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryXtraSettings");
      subscribeMQTTTopic("a3/" + serialNumber + "/P1settingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/P2settingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/xtraSettingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/webConnect");
      subscribeMQTTTopic("a3/" + serialNumber + "/webDisconnect");
    }
  } else {
    Serial.println("Failed to start embedded broker");
    delete embeddedBroker;
    embeddedBroker = nullptr;
  }
}

void stopEmbeddedBroker() {
  if (!isEmbeddedBrokerActive) return;
  
  Serial.println("Stopping embedded MQTT broker...");
  if (localClient.connected()) {
    localClient.disconnect();
  }
  if (embeddedBroker) {
    embeddedBroker->stop();
    delete embeddedBroker;
    embeddedBroker = nullptr;
  }
  isEmbeddedBrokerActive = false;
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
  doc["modeP1"] = String(settings.P1_MAIN_MODE);
  doc["runModeP1"] = String(settings.P1_RUN_MODE);
  doc["pauseTimeP1"] = settings.P1_PAUSE_TIME;
  doc["timeCyclesP1"] = String(settings.P1_RUN_TIME_CYC);
  doc["proxy1P1"] = String(settings.P1_PROX1);
  doc["dwellTimeP1Px1"] = settings.P1_PROX1_DWELL;
  doc["proxy2P1"] = String(settings.P1_PROX2);
  doc["dwellTimeP1Px2"] = settings.P1_PROX2_DWELL;
  doc["levelP1"] = String(settings.P1_LVL);
  doc["levelTypeP1"] = String(settings.P1_LVL_TYPE);
  doc["levelNoncP1"] = String(settings.P1_LVL_NONC);

  String out;
  serializeJson(doc, out);
  Serial.print("getP1SettingsJsonStr() JSON: ");
  Serial.println(out);
  return out;
}

String getP2SettingsJsonStr(){
  DynamicJsonDocument doc(1024);
  // --- PUMP 2 ---
  doc["pump2InUse"] = String(settings.PUMP2_IN_USE);
  doc["modeP2"] = String(settings.P2_MAIN_MODE);
  doc["runModeP2"] = String(settings.P2_RUN_MODE);
  doc["pauseTimeP2"] = settings.P2_PAUSE_TIME;
  doc["timeCyclesP2"] = String(settings.P2_RUN_TIME_CYC);
  doc["proxy1P2"] = String(settings.P2_PROX1);
  doc["dwellTimeP2Px1"] = settings.P2_PROX1_DWELL;
  doc["proxy2P2"] = String(settings.P2_PROX2);
  doc["dwellTimeP2Px2"] = settings.P2_PROX2_DWELL;
  doc["levelP2"] = String(settings.P2_LVL);
  doc["levelTypeP2"] = String(settings.P2_LVL_TYPE);
  doc["levelNoncP2"] = String(settings.P2_LVL_NONC);

  String out;
  serializeJson(doc, out);
  Serial.print("getP2SettingsJsonStr() JSON: ");
  Serial.println(out);
  return out;
}

String getExtraSettingsJsonStr() {
  DynamicJsonDocument doc(512);
  // --- EXT LAMP ---
  doc["extLampInUse"] = String(settings.EXT_LAMP);
  doc["extLampType"] = String(settings.LAMP_TYP);

  // --- BLOCKAGE CURRENT ---
  doc["blockCurrentP1"] = settings.P1_BLOCK_CURRENT;
  doc["blockCurrentP2"] = settings.P2_BLOCK_CURRENT;

  String out;
  serializeJson(doc, out);
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
  String p1levelTopic = "a3/" + serialNumber + "/test/p1level";
  String p2levelTopic = "a3/" + serialNumber + "/test/p2level";
  String extlampTopic = "a3/" + serialNumber + "/test/extlamp";
  String P1settingsReplyTopic = "a3/" + serialNumber + "/P1settingsReply";
  String P2settingsReplyTopic = "a3/" + serialNumber + "/P2settingsReply";
  String xtraSettingsReplyTopic = "a3/" + serialNumber + "/xtraSettingsReply";
  String P1SaveSettingsTopic = "a3/" + serialNumber + "/P1settingsSave";
  String P2SaveSettingsTopic = "a3/" + serialNumber + "/P2settingsSave";
  String xtraSettingsSaveTopic = "a3/" + serialNumber + "/xtraSettingsSave";

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
    DynamicJsonDocument responseDoc(128);
    responseDoc["serial"] = serialNumber;
    String responsePayload;
    serializeJson(responseDoc, responsePayload);
    sendMQTTMessage("a3/identifyResponse", responsePayload);
  }
  else if (String(topic) == queryP1SettingsTopic) {
    String settingsJson = getP1SettingsJsonStr();
    String P1settingsReplyTopicSend = "a3/" + serialNumber + "/P1settingsReply";
    mqttClient.publish(P1settingsReplyTopicSend.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent P1 settings JSON via MQTT.");
    return;
  }
  else if (String(topic) == queryP2SettingsTopic) {
    String settingsJson = getP2SettingsJsonStr();
    String P2settingsReplyTopicSend = "a3/" + serialNumber + "/P2settingsReply";
    mqttClient.publish(P2settingsReplyTopicSend.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent P2 settings JSON via MQTT.");
    return;
  }
  else if (String(topic) == queryXtraSettingsTopic) {
    String settingsJson = getExtraSettingsJsonStr();
    String xtraSettingsReplyTopicSend = "a3/" + serialNumber + "/xtraSettingsReply";
    mqttClient.publish(xtraSettingsReplyTopicSend.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent extra settings JSON via MQTT.");
    return;
  }
  // ----------- Test Mode Pump1 MQTT Logic -----------
  else if (String(topic) == switchPump1Topic) {
    if (msg == "on") {
      digitalWrite(pump1Out, HIGH);
      sendMQTTMessage(switchPump1Topic, "running");
      Serial.println("Pump1 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump1Out, LOW);
      sendMQTTMessage(switchPump1Topic, "stopped");
      Serial.println("Pump1 turned OFF (test mode)");
    } else if (msg == "currentAutoCalibrate") {
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
      pump2AutoCalibrate = true;
      Serial.println("P2 auto calibration requested");
    }
  }
  // ----------- Settings Update Handlers - UPDATED TO MATCH SEND FIELD NAMES -----------
  else if (String(topic) == P1SaveSettingsTopic) {
    Serial.println("Received P1 settings update via MQTT");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("P1 settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply P1 settings using same field names as getP1SettingsJsonStr() sends
    if (doc.containsKey("modeP1")) {
      settings.P1_MAIN_MODE = doc["modeP1"].as<String>();
      Serial.println("Updated P1 Main Mode: " + settings.P1_MAIN_MODE);
    }
    
    if (doc.containsKey("pauseTimeP1")) {
      settings.P1_PAUSE_TIME = doc["pauseTimeP1"].as<uint32_t>();
      Serial.println("Updated P1 Pause Time: " + String(settings.P1_PAUSE_TIME));
    }
    
    if (doc.containsKey("runModeP1")) {
      settings.P1_RUN_MODE = doc["runModeP1"].as<String>();
      Serial.println("Updated P1 Run Mode: " + settings.P1_RUN_MODE);
    }
    
    if (doc.containsKey("timeCyclesP1")) {
      settings.P1_RUN_TIME_CYC = doc["timeCyclesP1"].as<uint32_t>();
      Serial.println("Updated P1 Time/Cycles: " + String(settings.P1_RUN_TIME_CYC));
    }
    
    if (doc.containsKey("proxy1P1")) {
      settings.P1_PROX1 = doc["proxy1P1"].as<String>();
      Serial.println("Updated P1 Proxy1: " + settings.P1_PROX1);
    }
    
    if (doc.containsKey("dwellTimeP1Px1")) {
      settings.P1_PROX1_DWELL = doc["dwellTimeP1Px1"].as<uint32_t>();
      Serial.println("Updated P1 Proxy1 Dwell: " + String(settings.P1_PROX1_DWELL));
    }
    
    if (doc.containsKey("proxy2P1")) {
      settings.P1_PROX2 = doc["proxy2P1"].as<String>();
      Serial.println("Updated P1 Proxy2: " + settings.P1_PROX2);
    }
    
    if (doc.containsKey("dwellTimeP1Px2")) {
      settings.P1_PROX2_DWELL = doc["dwellTimeP1Px2"].as<uint32_t>();
      Serial.println("Updated P1 Proxy2 Dwell: " + String(settings.P1_PROX2_DWELL));
    }
    
    if (doc.containsKey("levelP1")) {
      settings.P1_LVL = doc["levelP1"].as<String>();
      Serial.println("Updated P1 Level: " + settings.P1_LVL);
    }
    
    if (doc.containsKey("levelTypeP1")) {
      settings.P1_LVL_TYPE = doc["levelTypeP1"].as<String>();
      Serial.println("Updated P1 Level Type: " + settings.P1_LVL_TYPE);
    }
    
    if (doc.containsKey("levelNoncP1")) {
      settings.P1_LVL_NONC = doc["levelNoncP1"].as<String>();
      Serial.println("Updated P1 Level N/O-N/C: " + settings.P1_LVL_NONC);
    }
    
    writeSettings(settings);
    Serial.println("P1 settings saved to flash");
  }
  
  else if (String(topic) == P2SaveSettingsTopic) {
    Serial.println("Received P2 settings update via MQTT");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("P2 settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply P2 settings using same field names as getP2SettingsJsonStr() sends
    if (doc.containsKey("pump2InUse")) {
      settings.PUMP2_IN_USE = doc["pump2InUse"].as<String>();
      Serial.println("Updated Pump2 In Use: " + settings.PUMP2_IN_USE);
    }
    
    if (doc.containsKey("modeP2")) {
      settings.P2_MAIN_MODE = doc["modeP2"].as<String>();
      Serial.println("Updated P2 Main Mode: " + settings.P2_MAIN_MODE);
    }
    
    if (doc.containsKey("pauseTimeP2")) {
      settings.P2_PAUSE_TIME = doc["pauseTimeP2"].as<uint32_t>();
      Serial.println("Updated P2 Pause Time: " + String(settings.P2_PAUSE_TIME));
    }
    
    if (doc.containsKey("runModeP2")) {
      settings.P2_RUN_MODE = doc["runModeP2"].as<String>();
      Serial.println("Updated P2 Run Mode: " + settings.P2_RUN_MODE);
    }
    
    if (doc.containsKey("timeCyclesP2")) {
      settings.P2_RUN_TIME_CYC = doc["timeCyclesP2"].as<uint32_t>();
      Serial.println("Updated P2 Time/Cycles: " + String(settings.P2_RUN_TIME_CYC));
    }
    
    if (doc.containsKey("proxy1P2")) {
      settings.P2_PROX1 = doc["proxy1P2"].as<String>();
      Serial.println("Updated P2 Proxy1: " + settings.P2_PROX1);
    }
    
    if (doc.containsKey("dwellTimeP2Px1")) {
      settings.P2_PROX1_DWELL = doc["dwellTimeP2Px1"].as<uint32_t>();
      Serial.println("Updated P2 Proxy1 Dwell: " + String(settings.P2_PROX1_DWELL));
    }
    
    if (doc.containsKey("proxy2P2")) {
      settings.P2_PROX2 = doc["proxy2P2"].as<String>();
      Serial.println("Updated P2 Proxy2: " + settings.P2_PROX2);
    }
    
    if (doc.containsKey("dwellTimeP2Px2")) {
      settings.P2_PROX2_DWELL = doc["dwellTimeP2Px2"].as<uint32_t>();
      Serial.println("Updated P2 Proxy2 Dwell: " + String(settings.P2_PROX2_DWELL));
    }
    
    if (doc.containsKey("levelP2")) {
      settings.P2_LVL = doc["levelP2"].as<String>();
      Serial.println("Updated P2 Level: " + settings.P2_LVL);
    }
    
    if (doc.containsKey("levelTypeP2")) {
      settings.P2_LVL_TYPE = doc["levelTypeP2"].as<String>();
      Serial.println("Updated P2 Level Type: " + settings.P2_LVL_TYPE);
    }
    
    if (doc.containsKey("levelNoncP2")) {
      settings.P2_LVL_NONC = doc["levelNoncP2"].as<String>();
      Serial.println("Updated P2 Level N/O-N/C: " + settings.P2_LVL_NONC);
    }
    
    writeSettings(settings);
    Serial.println("P2 settings saved to flash");
  }
  
  else if (String(topic) == xtraSettingsSaveTopic) {
    Serial.println("Received Extra settings update via MQTT");
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("Extra settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply Extra settings using same field names as getExtraSettingsJsonStr() sends
    if (doc.containsKey("extLampInUse")) {
      settings.EXT_LAMP = doc["extLampInUse"].as<String>();
      Serial.println("Updated External Lamp: " + settings.EXT_LAMP);
    }
    
    if (doc.containsKey("extLampType")) {
      settings.LAMP_TYP = doc["extLampType"].as<String>();
      Serial.println("Updated Lamp Type: " + settings.LAMP_TYP);
    }
    
    if (doc.containsKey("blockCurrentP1")) {
      settings.P1_BLOCK_CURRENT = doc["blockCurrentP1"].as<uint32_t>();
      Serial.println("Updated P1 Block Current: " + String(settings.P1_BLOCK_CURRENT));
    }
    
    if (doc.containsKey("blockCurrentP2")) {
      settings.P2_BLOCK_CURRENT = doc["blockCurrentP2"].as<uint32_t>();
      Serial.println("Updated P2 Block Current: " + String(settings.P2_BLOCK_CURRENT));
    }
    
    writeSettings(settings);
    Serial.println("Extra settings saved to flash");
  }
  else if (String(topic) == "a3/" + serialNumber + "/live/pump1") {
    if (msg == "start") {
      // Start pump1 in live mode
      digitalWrite(pump1Out, HIGH);
      sendMQTTMessage("a3/" + serialNumber + "/live/pump1", "running");
      Serial.println("Pump1 started in live mode");
      
      // Start publishing live status updates
    } else if (msg == "stop") {
      // Stop pump1 in live mode
      digitalWrite(pump1Out, LOW);
      sendMQTTMessage("a3/" + serialNumber + "/live/pump1", "stopped");
      Serial.println("Pump1 stopped in live mode");
    }
  }
  
  else if (String(topic) == "a3/" + serialNumber + "/live/pump2") {
    if (msg == "start") {
      // Start pump2 in live mode
      digitalWrite(pump2Out, HIGH);
      sendMQTTMessage("a3/" + serialNumber + "/live/pump2", "running");
      Serial.println("Pump2 started in live mode");
      
      // Start publishing live status updates
    } else if (msg == "stop") {
      // Stop pump2 in live mode
      digitalWrite(pump2Out, LOW);
      sendMQTTMessage("a3/" + serialNumber + "/live/pump2", "stopped");
      Serial.println("Pump2 stopped in live mode");
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

void switchToAPMode() {
  Serial.println("Switching to AP mode...");
  
  // Disconnect from WiFi
  WiFi.disconnect(true);
  delay(1000);
  
  // Start AP mode
  WiFi.mode(WIFI_AP);
  delay(500);
  
  String apSSID = "A3_Setup_" + serialNumber;
  String apPassword = "12345678";
  
  if (WiFi.softAP(apSSID.c_str(), apPassword.c_str())) {
    Serial.println("AP mode started successfully");
    Serial.print("AP SSID: ");
    Serial.println(apSSID);
    Serial.print("AP Password: ");
    Serial.println(apPassword);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    
    isAPMode = true;
    lastSTARetry = millis(); // Start the STA retry timer
    
    // Disconnect MQTT since we're no longer connected to internet
    if (mqttClient.connected()) {
      mqttClient.disconnect();
      Serial.println("MQTT disconnected due to AP mode switch");
    }

    startEmbeddedBroker();

  } else {
    Serial.println("Failed to start AP mode!");
  }
}

void attemptWiFiReconnection() {
  wifiRetryCount++;
  Serial.printf("WiFi reconnection attempt %d/%d\n", wifiRetryCount, MAX_WIFI_RETRIES);
  
  // Try to reconnect
  WiFi.disconnect();
  delay(1000);
  WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  
  // Wait up to 10 seconds for connection
  int connectionTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && connectionTimeout < 10) {
    delay(1000);
    connectionTimeout++;
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi reconnection successful!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    wifiRetryCount = 0; // Reset counter on successful connection
    
    // Reconnect MQTT if it was connected before
    if (!mqttClient.connected()) {
      Serial.println("Attempting to reconnect MQTT...");
      // Your existing MQTT connection code here
    }
  } else {
    Serial.printf("WiFi reconnection attempt %d failed.\n", wifiRetryCount);
    
    if (wifiRetryCount >= MAX_WIFI_RETRIES) {
      Serial.println("Max WiFi retries reached. Switching to AP mode...");
      switchToAPMode();
    }
  }
}

void attemptSTAReconnection() {
  Serial.println("Attempting to switch back to STA mode...");
  
  stopEmbeddedBroker();

  // Stop AP mode temporarily to try STA
  WiFi.softAPdisconnect(true);
  delay(500);
  
  // Try to connect to STA
  WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  
  // Wait up to 10 seconds for connection
  int connectionTimeout = 0;
  while (WiFi.status() != WL_CONNECTED && connectionTimeout < 10) {
    delay(1000);
    connectionTimeout++;
    Serial.print(".");
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Successfully reconnected to STA mode!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    isAPMode = false;
    wifiRetryCount = 0;
    
    // Reconnect MQTT
    if (!mqttClient.connected()) {
      Serial.println("Attempting to reconnect MQTT...");
      // Your existing MQTT connection code here
    }
  } else {
    Serial.println("STA reconnection failed. Returning to AP mode...");
    // Restart AP mode
    WiFi.softAP("A3_Setup_" + serialNumber, "12345678");
    Serial.println("AP mode restarted");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
    startEmbeddedBroker();
  }
}

// ------------------ WiFi Monitoring Functions ------------------
void checkWiFiConnection() {
  // Only check if enough time has passed
  if (millis() - lastWiFiCheck < WIFI_CHECK_INTERVAL) {
    return;
  }
  
  lastWiFiCheck = millis();
  
  // If we're in AP mode, try to reconnect to STA periodically
  if (isAPMode) {
    if (millis() - lastSTARetry > STA_RETRY_INTERVAL) {
      Serial.println("AP Mode: Attempting to reconnect to STA mode...");
      attemptSTAReconnection();
      lastSTARetry = millis();
    }
    return;
  }
  
  // Check if WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Attempting to reconnect...");
    attemptWiFiReconnection();
  } else {
    // WiFi is connected, reset retry counter
    if (wifiRetryCount > 0) {
      Serial.println("WiFi connection restored. Resetting retry counter.");
      wifiRetryCount = 0;
    }
  }
}

void printWiFiStatus() {
  if (isAPMode) {
    Serial.print("WiFi Status: AP Mode - ");
    Serial.print("SSID: A3_Setup_" + serialNumber);
    Serial.print(", IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.print("WiFi Status: STA Mode - ");
    Serial.print("SSID: ");
    Serial.print(WiFi.SSID());
    Serial.print(", IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(", RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  }
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
  pinMode(p1lvlIn, INPUT_PULLUP);
  pinMode(p2lvlIn, INPUT_PULLUP); 
}

unsigned long lastStatePublish = 0;

void loop() {
  readPins();
  if (upLeftReleased == true){
    upLeftReleased = false;
    Serial.println("Up/Left button released");
    loadWiFiDefaults();
    saveWiFiSettings();
    esp_restart();
  }

  if (millis() - lastStatePublish > 500) {
    if (isAPMode && isEmbeddedBrokerActive) {
      // In AP mode, use embedded broker
      embeddedBroker->loop();
      if (localClient.connected()) {
        localClient.loop();
      }
    } else {
      // In STA mode, use external broker (your existing code)
      checkMQTTConnection();
      mqttClient.loop();
    }
    lastStatePublish = millis();
  }

  // Handle auto calibration requests
  if (pump1AutoCalibrate == true and pump1Running == false) {
    Serial.println("Starting P1 auto calibration");
    // Send acknowledgment that test is starting
    sendMQTTMessage(switchPump1Topic, "startingTest");
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

  // Check WiFi connection and reconnect if necessary
  if (millis() - lastWiFiCheck > WIFI_CHECK_INTERVAL) {
    lastWiFiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi not connected, attempting to reconnect...");
      WiFi.reconnect();
      wifiRetryCount++;
      if (wifiRetryCount > MAX_WIFI_RETRIES) {
        Serial.println("Max WiFi retry attempts reached. Switching to AP mode.");
        isAPMode = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAP("Setup-A3");
        Serial.println("Started AP mode");
      }
    } else {
      wifiRetryCount = 0; // Reset retry count if connected
    }
  }

  // Retry STA mode if in AP mode and interval has passed
  if (isAPMode && millis() - lastSTARetry > STA_RETRY_INTERVAL) {
    lastSTARetry = millis();
    Serial.println("Retrying STA mode...");
    isAPMode = false;
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  }
  
  checkWiFiConnection();
}
