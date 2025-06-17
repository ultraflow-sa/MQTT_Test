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
      sendMQTTMessage(switchPump1Topic, "running");
      Serial.println("Pump1 turned ON (test mode)");
    } else if (msg == "off") {
      digitalWrite(pump1Out, LOW);
      sendMQTTMessage(switchPump1Topic, "stopped");
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
  // ----------- Settings Update Handlers -----------
  else if (String(topic) == P1settingsReplyTopic) {
    Serial.println("Received P1 settings update via MQTT");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("P1 settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply P1 settings - direct assignment based on Settings struct
    if (doc.containsKey("mainModeP1")) {
      settings.P1_MAIN_MODE = doc["mainModeP1"].as<String>();
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
      settings.P1_PROX1 = doc["proxy1P1"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated P1 Proxy1: " + settings.P1_PROX1);
    }
    
    if (doc.containsKey("dwellTimeP1Px1")) {
      settings.P1_PROX1_DWELL = doc["dwellTimeP1Px1"].as<uint32_t>();
      Serial.println("Updated P1 Proxy1 Dwell: " + String(settings.P1_PROX1_DWELL));
    }
    
    if (doc.containsKey("proxy2P1")) {
      settings.P1_PROX2 = doc["proxy2P1"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated P1 Proxy2: " + settings.P1_PROX2);
    }
    
    if (doc.containsKey("dwellTimeP1Px2")) {
      settings.P1_PROX2_DWELL = doc["dwellTimeP1Px2"].as<uint32_t>();
      Serial.println("Updated P1 Proxy2 Dwell: " + String(settings.P1_PROX2_DWELL));
    }
    
    if (doc.containsKey("levelP1")) {
      settings.P1_LVL = doc["levelP1"].as<String>(); // Direct assignment - already "YES" or "NO"
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
    
    // Save settings to flash
    writeSettings(settings);
    Serial.println("P1 settings saved to flash");
  }
  
  else if (String(topic) == P2settingsReplyTopic) {
    Serial.println("Received P2 settings update via MQTT");
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("P2 settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply P2 settings - direct assignment based on Settings struct
    if (doc.containsKey("pump2InUse")) {
      settings.PUMP2_IN_USE = doc["pump2InUse"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated Pump2 In Use: " + settings.PUMP2_IN_USE);
    }
    
    if (doc.containsKey("mainModeP2")) {
      settings.P2_MAIN_MODE = doc["mainModeP2"].as<String>();
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
      settings.P2_PROX1 = doc["proxy1P2"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated P2 Proxy1: " + settings.P2_PROX1);
    }
    
    if (doc.containsKey("dwellTimeP2Px1")) {
      settings.P2_PROX1_DWELL = doc["dwellTimeP2Px1"].as<uint32_t>();
      Serial.println("Updated P2 Proxy1 Dwell: " + String(settings.P2_PROX1_DWELL));
    }
    
    if (doc.containsKey("proxy2P2")) {
      settings.P2_PROX2 = doc["proxy2P2"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated P2 Proxy2: " + settings.P2_PROX2);
    }
    
    if (doc.containsKey("dwellTimeP2Px2")) {
      settings.P2_PROX2_DWELL = doc["dwellTimeP2Px2"].as<uint32_t>();
      Serial.println("Updated P2 Proxy2 Dwell: " + String(settings.P2_PROX2_DWELL));
    }
    
    if (doc.containsKey("levelP2")) {
      settings.P2_LVL = doc["levelP2"].as<String>(); // Direct assignment - already "YES" or "NO"
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
    
    // Save settings to flash
    writeSettings(settings);
    Serial.println("P2 settings saved to flash");
  }
  
  else if (String(topic) == xtraSettingsReplyTopic) {
    Serial.println("Received Extra settings update via MQTT");
    DynamicJsonDocument doc(512);
    DeserializationError error = deserializeJson(doc, msg);
    if (error) {
      Serial.print("Extra settings JSON parse error: ");
      Serial.println(error.c_str());
      return;
    }
    
    // Parse and apply Extra settings - direct assignment based on Settings struct
    if (doc.containsKey("externalLampXtra")) {
      settings.EXT_LAMP = doc["externalLampXtra"].as<String>(); // Direct assignment - already "YES" or "NO"
      Serial.println("Updated External Lamp: " + settings.EXT_LAMP);
    }
    
    if (doc.containsKey("lampTypeXtra")) {
      settings.LAMP_TYP = doc["lampTypeXtra"].as<String>();
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
    
    // Save settings to flash
    writeSettings(settings);
    Serial.println("Extra settings saved to flash");
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
    checkMQTTConnection();
    mqttClient.loop();
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
