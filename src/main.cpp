#include "defs.h"
#include "subs.h"

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

// WiFi connection monitoring variables
unsigned long lastWiFiCheck = 0;
const unsigned long WIFI_CHECK_INTERVAL = 10000; // Check every 10 seconds
int wifiRetryCount = 0;
const int MAX_WIFI_RETRIES = 5;
bool isAPMode = false;
unsigned long lastSTARetry = 0;
const unsigned long STA_RETRY_INTERVAL = 30000; // Retry STA mode every 30 seconds when in AP mode

// Add these variables for web interface control
bool webInterfaceActive = false;
unsigned long webInterfaceLastSeen = 0;
String controllerStateBeforeWeb = "";

// Add pump mode tracking variables
String pump1Mode = "STOPPED";  // Current pump1 operating mode
String pump2Mode = "STOPPED";  // Current pump2 operating mode

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

// ------------------ Load Settings Function ------------------
bool loadWiFiSettings() {
  if (!LittleFS.exists(WIFI_SETTINGS_FILE)) {
    loadWiFiDefaults();
    saveWiFiSettings();
    return false;
  }
  
  File file = LittleFS.open(WIFI_SETTINGS_FILE, FILE_READ);
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
      pump1Mode = "TEST_RUNNING";
      sendMQTTMessage(switchPump1Topic, "running");
      Serial.println("Pump1 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump1Out, LOW);
      pump1Mode = "STOPPED";
      sendMQTTMessage(switchPump1Topic, "stopped");
      Serial.println("Pump1 turned OFF (test mode)");
    } else if (msg == "currentAutoCalibrate") {
      pump1AutoCalibrate = true;
      pump1Mode = "CALIBRATING";
      Serial.println("P1 auto calibration requested");
    }
  }
  // ----------- Test Mode Pump2 MQTT Logic -----------
  else if (String(topic) == switchPump2Topic) {
    if (msg == "on") {
      digitalWrite(pump2Out, HIGH);
      pump2Mode = "TEST_RUNNING";
      sendMQTTMessage(switchPump2Topic, "running");
      Serial.println("Pump2 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump2Out, LOW);
      pump2Mode = "STOPPED";
      sendMQTTMessage(switchPump2Topic, "stopped");
      Serial.println("Pump2 turned OFF (test mode)");
    } else if (msg == "currentAutoCalibrate") {
      pump2AutoCalibrate = true;
      pump2Mode = "CALIBRATING";
      Serial.println("P2 auto calibration requested");
    }
  }
  // ----------- Live Mode Pump MQTT Logic -----------
  else if (String(topic) == "a3/" + serialNumber + "/live/pump1") {
    if (msg == "start") {
      digitalWrite(pump1Out, HIGH);
      pump1Mode = "LIVE_RUNNING";
      sendMQTTMessage("a3/" + serialNumber + "/live/pump1", "running");
      Serial.println("Pump1 started in live mode");
    } else if (msg == "stop") {
      digitalWrite(pump1Out, LOW);
      pump1Mode = "STOPPED";
      sendMQTTMessage("a3/" + serialNumber + "/live/pump1", "stopped");
      Serial.println("Pump1 stopped in live mode");
    }
  }
  else if (String(topic) == "a3/" + serialNumber + "/live/pump2") {
    if (msg == "start") {
      digitalWrite(pump2Out, HIGH);
      pump2Mode = "LIVE_RUNNING";
      sendMQTTMessage("a3/" + serialNumber + "/live/pump2", "running");
      Serial.println("Pump2 started in live mode");
    } else if (msg == "stop") {
      digitalWrite(pump2Out, LOW);
      pump2Mode = "STOPPED";
      sendMQTTMessage("a3/" + serialNumber + "/live/pump2", "stopped");
      Serial.println("Pump2 stopped in live mode");
    }
  }
  // ----------- Web Interface Control Handlers -----------
  else if (String(topic) == "a3/" + serialNumber + "/webInterfaceActive") {
    webInterfaceActive = true;
    webInterfaceLastSeen = millis();
    Serial.println("Web interface has taken control");
  }
  else if (String(topic) == "a3/" + serialNumber + "/webHeartbeat") {
    webInterfaceLastSeen = millis();
    webInterfaceActive = true;
  }
  else if (String(topic) == "a3/" + serialNumber + "/queryControllerState") {
    String currentState = "{\"pump1Mode\":\"" + pump1Mode + 
                         "\",\"pump2Mode\":\"" + pump2Mode + 
                         "\",\"wasAutonomous\":true}";
    controllerStateBeforeWeb = currentState;
    sendMQTTMessage("a3/" + serialNumber + "/controllerStateResponse", currentState);
    Serial.println("Sent controller state to web interface");
  }
  else if (String(topic) == "a3/" + serialNumber + "/exitWebInterface" || 
           String(topic) == "a3/" + serialNumber + "/restoreControllerState") {
    webInterfaceActive = false;
    
    // Stop all pumps first
    digitalWrite(pump1Out, LOW);
    digitalWrite(pump2Out, LOW);
    pump1Running = false;
    pump2Running = false;
    
    // Update pump modes
    pump1Mode = "STOPPED";
    pump2Mode = "STOPPED";
    
    Serial.println("Web interface disconnected - returning to normal operation");
    
    if (String(topic) == "a3/" + serialNumber + "/restoreControllerState") {
      Serial.println("Restoring original controller state: " + msg);
    }
  }
  // ----------- Settings Update Handlers -----------
  else if (String(topic) == P1SaveSettingsTopic) {
    Serial.println("Received P1 settings update via MQTT");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("P1 settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply P1 settings
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
}

// Add the setup() and loop() functions at the end of the file
void setup() {
  Serial.begin(115200);
  
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }
  
  // Load settings
  settings = readSettings();
  loadWiFiSettings();
  
  // Initialize pins
  pinMode(pump1Out, OUTPUT);
  pinMode(pump2Out, OUTPUT);
  digitalWrite(pump1Out, LOW);
  digitalWrite(pump2Out, LOW);
  
  // Initialize MQTT
  mqttClient.setCallback(mqttCallback);
  
  // Start WiFi
  startStationMode(wifiSettings.ssid, wifiSettings.password);
}

void loop() {
  // Check for web interface heartbeat timeout
  if (webInterfaceActive && (millis() - webInterfaceLastSeen > 60000)) {
    Serial.println("Web interface timeout - returning to normal operation");
    webInterfaceActive = false;
    
    // Stop all pumps and return to autonomous operation
    digitalWrite(pump1Out, LOW);
    digitalWrite(pump2Out, LOW);
    pump1Running = false;
    pump2Running = false;
    pump1Mode = "STOPPED";
    pump2Mode = "STOPPED";
  }
  
  // Handle auto calibration requests
  if (pump1AutoCalibrate == true && pump1Running == false) {
    Serial.println("Starting P1 auto calibration");
    sendMQTTMessage("a3/" + serialNumber + "/test/pump1", "startingTest");
    digitalWrite(pump1Out, HIGH);
    pump1Running = true;
    pump1Mode = "CALIBRATING";
    pump1StartTime = millis();
  }

  if (pump1AutoCalibrate == true && pump1Running == true && (millis() - pump1StartTime >= 10000)) {
    digitalWrite(pump1Out, LOW);
    pump1Running = false;
    pump1AutoCalibrate = false;
    pump1Mode = "STOPPED";
    String calibratedCurrent = "1";
    sendMQTTMessage("a3/" + serialNumber + "/test/pump1", calibratedCurrent);
    Serial.println("P1 auto calibration complete: " + calibratedCurrent);
  }

  if (pump2AutoCalibrate == true && pump2Running == false) {
    Serial.println("Starting P2 auto calibration");
    sendMQTTMessage("a3/" + serialNumber + "/test/pump2", "startingTest");
    digitalWrite(pump2Out, HIGH);
    pump2Running = true;
    pump2Mode = "CALIBRATING";
    pump2StartTime = millis();
  }

  if (pump2AutoCalibrate == true && pump2Running == true && (millis() - pump2StartTime >= 10000)) {
    digitalWrite(pump2Out, LOW);
    pump2Running = false;
    pump2AutoCalibrate = false;
    pump2Mode = "STOPPED";
    String calibratedCurrent = "1";
    sendMQTTMessage("a3/" + serialNumber + "/test/pump2", calibratedCurrent);
    Serial.println("P2 auto calibration complete: " + calibratedCurrent);
  }

  // Check WiFi connection
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
      wifiRetryCount = 0;
    }
  }

  // Retry STA mode if in AP mode
  if (isAPMode && millis() - lastSTARetry > STA_RETRY_INTERVAL) {
    lastSTARetry = millis();
    Serial.println("Retrying STA mode...");
    isAPMode = false;
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  }
  
  checkMQTTConnection();
}
