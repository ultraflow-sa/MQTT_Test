#include "defs.h"
#include "subs.h"

// ----------- Global Variable Definitions -----------
bool p1prox1On = false;
bool p1prox2On = false;
unsigned long lastMQTTReconnectAttempt = 0;

wifiSettings_t wifiSettings;
Settings settings;

WiFiClientSecure wifiClient;
PubSubClient mqttClient(wifiClient);
AsyncWebServer server(80);
DNSServer dnsServer;

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

String getSettingsJsonString() {
  DynamicJsonDocument doc(1024);

  // --- PUMP 1 ---
  doc["modeP1"] = String(settings.P1_MAIN_MODE);
  doc["runModeP1"] = String(settings.P1_RUN_MODE);
  doc["pauseTimeP1"] = settings.P1_PAUSE_TIME;
  doc["timeCyclesP1"] = String(settings.P1_RUN_TIME_CYC);
  doc["timeCyclesValueP1"] = settings.P1_CYC_TIMEOUT;
  doc["proxy1P1"] = static_cast<bool>(settings.P1_PROX1);
  doc["dwellTimeP1Px1"] = settings.P1_PROX1_DWELL;
  doc["proxy2P1"] = static_cast<bool>(settings.P1_PROX2); 
  doc["dwellTimeP1Px2"] = settings.P1_PROX2_DWELL;
  doc["levelP1"] = static_cast<bool>(settings.P1_LVL);
  doc["levelTypeP1"] = String(settings.P1_LVL_TYPE);
  doc["levelNoncP1"] = String(settings.P1_LVL_NONC);

  // --- PUMP 2 ---
  doc["pump2InUse"] = static_cast<bool>(settings.PUMP2_IN_USE);
  doc["modeP2"] = String(settings.P2_MAIN_MODE);
  doc["runModeP2"] = String(settings.P2_RUN_MODE);
  doc["pauseTimeP2"] = settings.P2_PAUSE_TIME;
  doc["timeCyclesP2"] = String(settings.P2_RUN_TIME_CYC);
  doc["timeCyclesValueP2"] = settings.P2_CYC_TIMEOUT;
  doc["proxy1P2"] = static_cast<bool>(settings.P2_PROX1);
  doc["dwellTimeP2Px1"] = settings.P2_PROX1_DWELL;
  doc["proxy2P2"] = static_cast<bool>(settings.P2_PROX2);
  doc["dwellTimeP2Px2"] = settings.P2_PROX2_DWELL;
  doc["levelP2"] = static_cast<bool>(settings.P2_LVL);
  doc["levelTypeP2"] = String(settings.P2_LVL_TYPE);
  doc["levelNoncP2"] = String(settings.P2_LVL_NONC);

  // --- EXT LAMP ---
  doc["extLampInUse"] = static_cast<bool>(settings.EXT_LAMP);
  doc["extLampType"] = String(settings.LAMP_TYP);

  // --- BLOCKAGE CURRENT ---
  doc["blockCurrentP1"] = settings.P1_BLOCK_CURRENT;
  doc["blockCurrentP2"] = settings.P2_BLOCK_CURRENT;

  String out;
  serializeJson(doc, out);

  // Debug: print the generated JSON string
  Serial.print("getSettingsJsonString() JSON: ");
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
  String querySettingsTopic = "a3/" + serialNumber + "/querySettings";
  String switchPump1Topic = "a3/" + serialNumber + "/test/pump1";
  String proxy1Topic = "a3/" + serialNumber + "/test/proxy1";
  String proxy2Topic = "a3/" + serialNumber + "/test/proxy2";
  String settingsReplyTopic = "a3/" + serialNumber + "/settingsReply";

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
  else if (String(topic) == querySettingsTopic) {
    // Send settings as JSON string
    String settingsJson = getSettingsJsonString();
    mqttClient.publish(settingsReplyTopic.c_str(), settingsJson.c_str(), true);
    Serial.println("Sent settings JSON via MQTT.");
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
    loadWiFiDefaults();
    saveWiFiSettings();
    esp_restart();
  }

  if (millis() - lastStatePublish > 500) {
    checkMQTTConnection();
    mqttClient.loop();
    lastStatePublish = millis();
  }
}