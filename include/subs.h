#ifndef SUBS_H
#define SUBS_H

#include "defs.h"

extern void mqttCallback(char* topic, byte* payload, unsigned int length);
extern void setupServerEndpoints();
extern void startEmbeddedBroker();

// ------------------ MQTT Topic strings ------------------
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
String p1levelTopic = "a3/" + serialNumber + "/live/p1level";
String p2levelTopic = "a3/" + serialNumber + "/live/p2level";
String extlampTopic = "a3/" + serialNumber + "/live/extlamp";
String P1settingsReplyTopic = "a3/" + serialNumber + "/P1settingsReply";
String P2settingsReplyTopic = "a3/" + serialNumber + "/P2settingsReply";
String xtraSettingsReplyTopic = "a3/" + serialNumber + "/xtraSettingsReply";
String P1SaveSettingsTopic = "a3/" + serialNumber + "/P1settingsSave";
String P2SaveSettingsTopic = "a3/" + serialNumber + "/P2settingsSave";
String xtraSettingsSaveTopic = "a3/" + serialNumber + "/xtraSettingsSave";
String webHeartbeatTopic = "a3/" + serialNumber + "/webHeartbeat";

void processMQTTViaBluetooth(String topic, String payload) {
  // Reuse your existing MQTT handling logic
  char topicChar[topic.length() + 1];
  char payloadChar[payload.length() + 1];
  
  topic.toCharArray(topicChar, topic.length() + 1);
  payload.toCharArray(payloadChar, payload.length() + 1);
  
  // Call your existing MQTT callback
  mqttCallback(topicChar, (byte*)payloadChar, payload.length());
}

class MyBLEServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    bleDeviceConnected = true;
    Serial.println("=== BLE CLIENT CONNECTED (NO SECURITY) ===");
    lastBluetoothHeartbeat = millis();
    if (!webClientActive) {
      webClientActive = true;
    }
    
    // Stop advertising immediately when connected
    BLEDevice::getAdvertising()->stop();
    Serial.println("BLE advertising stopped (client connected)");
  };

  void onDisconnect(BLEServer* pServer) {
    bleDeviceConnected = false;
    Serial.println("=== BLE CLIENT DISCONNECTED ===");
    webClientActive = false;
    lastBluetoothHeartbeat = 0;
    
    // Stop pumps for safety
    digitalWrite(pump1Out, LOW);
    digitalWrite(pump2Out, LOW);
    Serial.println("*** PUMPS STOPPED DUE TO BLE DISCONNECT ***");
    
    // Restart advertising with delay to prevent rapid reconnection issues
    delay(1000);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted after disconnect");
  }
};

class MyBLECallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    
    if (rxValue.length() > 0) {
      Serial.println("BLE received: " + rxValue);
      
      DynamicJsonDocument doc(512);
      DeserializationError error = deserializeJson(doc, rxValue);
      
      if (!error) {
        String topic = doc["topic"] | "";
        String payload = doc["payload"] | "";
        
        if (topic.length() > 0) {
          processMQTTViaBluetooth(topic, payload);
        }
      }
    }
  }
};

// ------------------ MQTT Messaging Functions ------------------
void sendBluetoothMQTT(String topic, String payload) {
  if (bluetoothActive && bleDeviceConnected && pTxCharacteristic) {
    DynamicJsonDocument doc(512);
    doc["topic"] = topic;
    doc["payload"] = payload;
    
    String message;
    serializeJson(doc, message);
    
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
  }
}

void sendMQTTMessage(const String &topic, const String &payload) {
  // Try WiFi MQTT first
  if (!isAPMode && mqttClient.connected()) {
    mqttClient.publish(topic.c_str(), payload.c_str());
    Serial.println("Sent via WiFi MQTT: " + topic + " -> " + payload);
  }
  // Fallback to Bluetooth
  else if (bluetoothActive && bleDeviceConnected) {
    sendBluetoothMQTT(topic, payload);
  }
  // Last resort: log only
  else {
    Serial.println("No connection available for: " + topic + " -> " + payload);
  }
}

void subscribeMQTTTopic(const String &topic) {
  if (!isAPMode && mqttClient.connected()) {
    // Use external MQTT in STA mode only
    if (mqttClient.subscribe(topic.c_str())) {
      Serial.println("Subscribed to topic: " + topic);
    } else {
      Serial.println("Failed to subscribe to topic: " + topic);
    }
  } else if (isAPMode) {
    // In AP mode: Just log the subscription request (web interface handles communication)
    Serial.println("AP Mode - Subscription logged: " + topic);
  } else {
    Serial.println("No MQTT connection available for subscription: " + topic);
  }
}

void checkMQTTConnection() {
  if (isAPMode) return; // No MQTT in AP mode
  
  // In STA mode: Use external MQTT only
  if (!mqttClient.connected() && (millis() - lastMQTTReconnectAttempt > 5000)) {
    lastMQTTReconnectAttempt = millis();
    Serial.print("Attempting MQTT connection...");
    String clientId = "A3_Device_" + serialNumber;
    
    // Connect using HiveMQ username/password
    if (mqttClient.connect(clientId.c_str(), HIVEMQ_USER, HIVEMQ_PASS)) {
      Serial.println(" connected.");
      
      // Subscribe to all necessary topics
      subscribeMQTTTopic("a3/" + serialNumber + "/update");
      subscribeMQTTTopic("a3/" + serialNumber + "/querySerial");
      subscribeMQTTTopic("a3/identifyYourself");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/pump1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/pump2");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p1proxy1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p1proxy2");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p2proxy1");
      subscribeMQTTTopic("a3/" + serialNumber + "/test/p2proxy2");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/p1level");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/p2level");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/extlamp");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/pump1");
      subscribeMQTTTopic("a3/" + serialNumber + "/live/pump2");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryP1Settings");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryP2Settings");
      subscribeMQTTTopic("a3/" + serialNumber + "/queryXtraSettings");
      subscribeMQTTTopic("a3/" + serialNumber + "/P1settingsReply");
      subscribeMQTTTopic("a3/" + serialNumber + "/P2settingsReply");
      subscribeMQTTTopic("a3/" + serialNumber + "/xtraSettingsReply");
      subscribeMQTTTopic("a3/" + serialNumber + "/P1settingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/P2settingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/xtraSettingsSave");
      subscribeMQTTTopic("a3/" + serialNumber + "/webConnect");
      subscribeMQTTTopic("a3/" + serialNumber + "/webDisconnect");
      subscribeMQTTTopic("a3/" + serialNumber + "/webHeartbeat");
      
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" - will retry in 5 seconds");
    }
  }
}

void handleBluetoothMQTT() {
  if (!bluetoothActive) return;
  
  // Handle connection state changes
  if (bleOldDeviceConnected != bleDeviceConnected) {
    
    if (bleDeviceConnected) {
      // Device just connected
      Serial.println("BLE device connected - stopping advertising");
      BLEDevice::getAdvertising()->stop();
    } else {
      // Device just disconnected
      Serial.println("BLE device disconnected - restarting advertising");
      delay(100); // Short delay to prevent rapid reconnection issues
      BLEDevice::getAdvertising()->start();
    }
    
    bleOldDeviceConnected = bleDeviceConnected;
  }
  
  // Monitor for stuck connections
  static unsigned long lastConnectionCheck = 0;
  if (millis() - lastConnectionCheck > 30000) { // Check every 30 seconds
    if (bleDeviceConnected && (millis() - lastBluetoothHeartbeat > 60000)) {
      Serial.println("BLE connection appears stuck - forcing restart");
      
      // Force disconnect and restart
      if (pBLEServer) {
        pBLEServer->disconnect(pBLEServer->getConnId());
      }
      
      delay(1000);
      BLEDevice::getAdvertising()->start();
      Serial.println("BLE advertising restarted after stuck connection");
    }
    lastConnectionCheck = millis();
  }
}

bool isBluetoothClientConnected() {
  return bleDeviceConnected;
}

void setupBluetoothFallback() {
  String bleName = "A3_Setup_" + serialNumber;
  
  Serial.println("Starting BLE setup: " + bleName);
  
  // Initialize BLE with no security
  BLEDevice::init(bleName.c_str());
  
  // Use the correct BLE security settings
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
  BLEDevice::setSecurityCallbacks(nullptr);
  
  // Set BLE security parameters using the correct API
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint32_t passkey = 0;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));

  // Create BLE Server
  pBLEServer = BLEDevice::createServer();
  pBLEServer->setCallbacks(new MyBLEServerCallbacks());

  // Create BLE Service
  BLEService *pService = pBLEServer->createService(BLE_SERVICE_UUID);

  // Create TX characteristic (ESP32 sends data to client)
  pTxCharacteristic = pService->createCharacteristic(
                      BLE_CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // Disable security on characteristic level
  pTxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);

  // Create RX characteristic (ESP32 receives data from client)
  pRxCharacteristic = pService->createCharacteristic(
                       BLE_CHARACTERISTIC_UUID_RX,
                       BLECharacteristic::PROPERTY_WRITE
                     );
  pRxCharacteristic->setCallbacks(new MyBLECallbacks());

  // Disable security on characteristic level
  pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);

  // Start the service
  pService->start();

  // Configure advertising with no security requirements
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x20);  // Increased intervals for stability
  pAdvertising->setMaxPreferred(0x40);
  
  // Start advertising
  BLEDevice::startAdvertising();
  
  bluetoothActive = true;
  Serial.println("BLE MQTT service started (no security)");
  Serial.println("BLE Name: " + bleName);
  Serial.println("Service UUID: " + String(BLE_SERVICE_UUID));
}

void startBluetoothFallback() {
  bluetoothFallbackActive = true;
  isAPMode = true; // Use AP mode logic for local serving
  
  // Start BLE
  setupBluetoothFallback();
  
  // Start local web server for serving pages
  setupServerEndpoints();
  server.begin();
  
  Serial.println("=== BLE FALLBACK MODE ACTIVE ===");
  Serial.println("Serve main.html with WiFi tab for credential setup");
}

//Routine to write to LittleFs for settings and .CSV storage/retrieval
// Write settings to file
void writeSettings(const Settings &settings) {
  File file = LittleFS.open(SETTINGS_FILE, "w");
  if (!file) {
    Serial.println("Failed to open settings file for writing");
    return;
  }

  // Create a JSON document
  DynamicJsonDocument doc(2048);

  // Populate the JSON document with settings
  doc["P1_MAIN_MODE"] = settings.P1_MAIN_MODE;
  doc["P1_PAUSE_TIME"] = settings.P1_PAUSE_TIME;
  doc["P1_RUN_MODE"] = settings.P1_RUN_MODE;
  doc["P1_RUN_TIME_CYC"] = settings.P1_RUN_TIME_CYC;
  doc["P1_CYC_TIMEOUT"] = settings.P1_CYC_TIMEOUT;
  doc["P1_PROX1"] = settings.P1_PROX1;
  doc["P1_PROX1_DWELL"] = settings.P1_PROX1_DWELL;
  doc["P1_PROX2"] = settings.P1_PROX2;
  doc["P1_PROX2_DWELL"] = settings.P1_PROX2_DWELL;
  doc["P1_PROX2_CYC"] = settings.P1_PROX2_CYC;
  doc["P1_VENT_TIME"] = settings.P1_VENT_TIME;
  doc["P1_LEVEL"] = settings.P1_LVL;
  doc["P1_LVL_TYPE"] = settings.P1_LVL_TYPE;
  doc["P1_LVL_NONC"] = settings.P1_LVL_NONC;
  doc["PUMP2_IN_USE"] = settings.PUMP2_IN_USE;
  doc["P2_MAIN_MODE"] = settings.P2_MAIN_MODE;
  doc["P2_PAUSE_TIME"] = settings.P2_PAUSE_TIME;
  doc["P2_RUN_MODE"] = settings.P2_RUN_MODE;
  doc["P2_RUN_TIME_CYC"] = settings.P2_RUN_TIME_CYC;
  doc["P2_CYC_TIMEOUT"] = settings.P2_CYC_TIMEOUT;
  doc["P2_PROX1"] = settings.P2_PROX1;
  doc["P2_PROX1_DWELL"] = settings.P2_PROX1_DWELL;
  doc["P2_PROX2"] = settings.P2_PROX2;
  doc["P2_PROX2_DWELL"] = settings.P2_PROX2_DWELL;
  doc["P2_PROX2_CYC"] = settings.P2_PROX2_CYC;
  doc["P2_VENT_TIME"] = settings.P2_VENT_TIME;
  doc["P2_LEVEL"] = settings.P2_LVL;
  doc["P2_LVL_TYPE"] = settings.P2_LVL_TYPE;
  doc["P2_LVL_NONC"] = settings.P2_LVL_NONC;
  doc["EXT_LAMP"] = settings.EXT_LAMP;
  doc["LAMP_TYP"] = settings.LAMP_TYP;
  doc["P1_CURR_STATE"] = settings.P1_CURR_STATE;
  doc["P1_CURR_TIME"] = settings.P1_CURR_TIME;
  doc["P2_CURR_STATE"] = settings.P2_CURR_STATE;
  doc["P2_CURR_TIME"] = settings.P2_CURR_TIME;
  doc["ACTIVE_STATE"] = settings.ACTIVE_STATE;
  doc["P1_BLOCK_CURRENT"] = settings.P1_BLOCK_CURRENT;
  doc["P1_PROX1_FLT_CNT"] = settings.P1_PROX1_FLT_CNT; // Fault counter for Pump 1 Proxy 1
  doc["P1_PROX2_FLT_CNT"] = settings.P1_PROX2_FLT_CNT; // Fault counter for Pump 1 Proxy 2
  doc["P1_LVL_FLT_CNT"] = settings.P1_LVL_FLT_CNT; // Fault counter for Pump 1 Level
  doc["P1_BLK_FLT_CNT"] = settings.P1_BLK_FLT_CNT; //Fault counter for Pump 1 blockage events
  doc["P1_OC_FLT_CNT"] = settings.P1_OC_FLT_CNT;  //Fault counter for Pump 1 open circuit faults
  doc["P1_SHRT_FLT_CNT"] = settings.P1_SHRT_FLT_CNT; //Fault counter for Pump 1 motor short circuit
  doc["P2_BLOCK_CURRENT"] = settings.P2_BLOCK_CURRENT;
  doc["P2_PROX1_FLT_CNT"] = settings.P2_PROX1_FLT_CNT; // Fault counter for Pump 2 Proxy 1
  doc["P2_PROX2_FLT_CNT"] = settings.P2_PROX2_FLT_CNT; // Fault counter for Pump 2 Proxy 2
  doc["P2_LVL_FLT_CNT"] = settings.P2_LVL_FLT_CNT; // Fault counter for Pump 2 Level
  doc["P2_BLK_FLT_CNT"] = settings.P2_BLK_FLT_CNT; //Fault counter for Pump 2 blockage events
  doc["P2_OC_FLT_CNT"] = settings.P2_OC_FLT_CNT;  //Fault counter for Pump 2 open circuit faults
  doc["P2_SHRT_FLT_CNT"] = settings.P2_SHRT_FLT_CNT; //Fault counter for Pump 2 motor short circuit
  doc["TOT_ON_TIME"] = settings.TOT_ON_TIME;//Total on time accumulator
  doc["TOT_P1_RUN_TIME"] = settings.TOT_P1_RUN_TIME; //Total Pump1 run time accumulator
  doc["TOT_P1_PSE_TIME"] = settings.TOT_P1_PSE_TIME;//Total Pump 1 pause time accumulator
  doc["TOT_P2_RUN_TIME"] = settings.TOT_P2_RUN_TIME; //Total Pump2 run time accumulator
  doc["TOT_P2_PSE_TIME"] = settings.TOT_P2_PSE_TIME;//Total Pump 2 pause time accumulator
  doc["TOT_ERR_TIME"] = settings.TOT_ERR_TIME;//Total error time accumulator
  doc["TOT_P1_PROX1_ERR_TIME"] = settings.TOT_P1_PROX1_ERR_TIME;//Total Pump1 proxy 1 error time accumulator
  doc["TOT_P1_PROX2_ERR_TIME"] = settings.TOT_P1_PROX2_ERR_TIME;//Total Pump1 proxy 2 error time accumulator
  doc["TOT_P1_LVL_ERR_TIME"] = settings.TOT_P1_LVL_ERR_TIME;//Total Pump1 level error time accumulator
  doc["TOT_P1_SC_TIME"] = settings.TOT_P1_SC_TIME;//Total Pump1 short curcuit time accumulator
  doc["TOT_P1_OC_TIME"] = settings.TOT_P1_OC_TIME;//Total Pump1 open circuit time accumulator
  doc["TOT_P1_BLK_TIME"] = settings.TOT_P1_BLK_TIME;//Total Pump1 overpressure time accumulator
  doc["TOT_P2_PROX1_ERR_TIME"] = settings.TOT_P2_PROX1_ERR_TIME;//Total Pump1 proxy 1 error time accumulator
  doc["TOT_P2_PROX2_ERR_TIME"] = settings.TOT_P2_PROX2_ERR_TIME;//Total Pump1 proxy 2 error time accumulator
  doc["TOT_P2_LVL_ERR_TIME"] = settings.TOT_P2_LVL_ERR_TIME;//Total Pump1 level error time accumulator
  doc["TOT_P2_SC_TIME"] = settings.TOT_P2_SC_TIME;//Total Pump1 short curcuit time accumulator
  doc["TOT_P2_OC_TIME"] = settings.TOT_P2_OC_TIME;//Total Pump1 open circuit time accumulator
  doc["TOT_P2_BLK_TIME"] = settings.TOT_P2_BLK_TIME;//Total Pump1 overpressure time accumulator
  doc["TOT_LMP_SC_TIME"] = settings.TOT_LMP_SC_TIME;//Total error lamp short circuit time
  doc["TOT_SEQ_NO"] = settings.TOT_SEQ_NO;//Maximum sequence number for logging
  doc["VER_STRING"] = settings.VER_STRING;//Version string for display
  doc["WEB_URL"] = settings.WEB_URL; //Web URL for the controller

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println("Failed to write settings to file");
  }
  else {
    Serial.println("Wrote settings to file");
    #ifdef DEBUG
    Serial.println("Settings written to file:");
    serializeJsonPretty(doc, Serial);
    #endif
  }
  file.close();
}

// Reads the ADC on supplyVoltPin, smooths the result and then scales the value.
// The measured voltage at the ADC is: V_measured = V_in * (R2/(R1+R2))
// Therefore, V_in = V_measured * ((R1+R2)/R2)
// For R1 = 15K and R2 = 2.2K, (R1+R2)/R2 is ~7.818.
float readSupplyVoltage() {
  // Parameters for exponential smoothing
  const float alpha = 0.35;    // Smoothing factor: 0 = no update, 1 = no smoothing
  static float smoothedADC = 0; // Persist smoothed ADC value
  // Read raw ADC value from pin (ESP32 ADC typically 12-bit: 0...4095)
  int raw = analogRead(supplyVoltPin);
  // Initialize the smoothed value at first run
  if (smoothedADC == 0) {
    smoothedADC = raw;
  }
  // Update the smoothed value exponentially
  smoothedADC = alpha * raw + (1 - alpha) * smoothedADC;
  // Convert to measured voltage (assuming a 3.3V ADC reference)
  float vMeasured = (smoothedADC / 4095.0) * 3.3;
  // Scale the measured voltage using the voltage divider ratio (15K to input, 2.2K to ground)
  float voltageDividerRatio = (15000.0 + 2200.0) / 2200.0; // ≈7.818
  float vSupply = vMeasured * voltageDividerRatio;
  // Apply linear correction to compensate for non-linearity
  // Using coefficients derived from calibration:
  //    v_corrected = 0.878*vSupply + 1.2
  float vCorrected = 0.878 * vSupply + 1.5;
  
  // Round the corrected value to two decimal places
  vCorrected = round(vCorrected * 100.0) / 100.0;
  
  return vCorrected;
}

// Get main supply voltage here
String getSupplyVoltage(void){
  // Read the supply voltage from the ESP ADC pin4
  supplyVoltage = (float)readSupplyVoltage();
  Serial.print("Supply voltage: ");
  Serial.println(supplyVoltage); // Print the supply voltage
  return String(supplyVoltage);
}


bool fileExists(String checkname) {
  // Try to open the file in read mode
  File file = LittleFS.open("/" + checkname, "r");
  // If the file exists, file is valid
  if (file) {
    file.close();  // Always close the file after opening
    return true;   // File exists
  } 
  else {
    return false;  // File doesn't exist
  }
}

// Read settings from file
Settings readSettings() {
  Serial.println("Reading settings");
  Settings settings = defaultSettings; // Default settings in case of failure
  if (!LittleFS.exists(SETTINGS_FILE)) {
      Serial.println("Settings file does not exist, writing defaults.");
      writeSettings(defaultSettings); // Write your default Settings struct
      return defaultSettings;
  }
  File file = LittleFS.open(SETTINGS_FILE, "r");
  if (!file) {
    Serial.println("Failed to open settings file for reading");
    return settings;
  }

  // Create a JSON document to deserialize
  DynamicJsonDocument doc(4096);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.print("Failed to read settings from file: ");
    Serial.println(error.f_str());
    Serial.println("Loading default settings");
    file.close();
    // Write default settings to the file
    writeSettings(defaultSettings);
    return settings;
  }

  // Read the settings from the JSON document
  settings.P1_MAIN_MODE = doc["P1_MAIN_MODE"].as<String>();
  settings.P1_PAUSE_TIME = doc["P1_PAUSE_TIME"];
  settings.P1_RUN_MODE = doc["P1_RUN_MODE"].as<String>();
  settings.P1_RUN_TIME_CYC = doc["P1_RUN_TIME_CYC"];
  settings.P1_CYC_TIMEOUT = doc["P1_CYC_TIMEOUT"];
  settings.P1_PROX1 = doc["P1_PROX1"].as<String>();
  settings.P1_PROX1_DWELL = doc["P1_PROX1_DWELL"];
  settings.P1_PROX2 = doc["P1_PROX2"].as<String>();
  settings.P1_PROX2_DWELL = doc["P1_PROX2_DWELL"];
  settings.P1_PROX2_CYC = doc["P1_PROX2_CYC"];
  settings.P1_VENT_TIME = doc["P1_VENT_TIME"];
  settings.P1_LVL = doc["P1_LEVEL"].as<String>();
  settings.P1_LVL_TYPE = doc["P1_LVL_TYPE"].as<String>();
  settings.P1_LVL_NONC = doc["P1_LVL_NONC"].as<String>();
  settings.PUMP2_IN_USE = doc["PUMP2_IN_USE"].as<String>();
  settings.P2_MAIN_MODE = doc["P2_MAIN_MODE"].as<String>();
  settings.P2_PAUSE_TIME = doc["P2_PAUSE_TIME"];
  settings.P2_RUN_MODE = doc["P2_RUN_MODE"].as<String>();
  settings.P2_RUN_TIME_CYC = doc["P2_RUN_TIME_CYC"];
  settings.P2_CYC_TIMEOUT = doc["P2_CYC_TIMEOUT"];
  settings.P2_PROX1 = doc["P2_PROX1"].as<String>();
  settings.P2_PROX1_DWELL = doc["P2_PROX1_DWELL"];
  settings.P2_PROX2 = doc["P2_PROX2"].as<String>();
  settings.P2_PROX2_DWELL = doc["P2_PROX2_DWELL"];
  settings.P2_PROX2_CYC = doc["P2_PROX2_CYC"];
  settings.P2_VENT_TIME = doc["P2_VENT_TIME"];
  settings.P2_LVL = doc["P2_LEVEL"].as<String>();
  settings.P2_LVL_TYPE = doc["P2_LVL_TYPE"].as<String>();
  settings.P2_LVL_NONC = doc["P2_LVL_NONC"].as<String>();
  settings.EXT_LAMP = doc["EXT_LAMP"].as<String>();
  settings.LAMP_TYP = doc["LAMP_TYP"].as<String>();
  settings.P1_CURR_STATE = doc["P1_CURR_STATE"].as<String>();
  settings.P1_CURR_TIME = doc["P1_CURR_TIME"];
  settings.P2_CURR_STATE = doc["P2_CURR_STATE"].as<String>();
  settings.P2_CURR_TIME = doc["P2_CURR_TIME"];
  settings.ACTIVE_STATE = doc["ACTIVE_STATE"].as<String>();
  settings.P1_BLOCK_CURRENT = doc["P1_BLOCK_CURRENT"];
  settings.P1_PROX1_FLT_CNT = doc["P1_PROX1_FLT_CNT"];
  settings.P1_PROX2_FLT_CNT = doc["P1_PROX2_FLT_CNT"];
  settings.P1_LVL_FLT_CNT = doc["P1_LVL_FLT_CNT"];
  settings.P1_BLK_FLT_CNT = doc["P1_BLK_FLT_CNT"];
  settings.P1_OC_FLT_CNT = doc["P1_OC_FLT_CNT"];
  settings.P1_SHRT_FLT_CNT = doc["P1_SHRT_FLT_CNT"];
  settings.P2_BLOCK_CURRENT = doc["P2_BLOCK_CURRENT"];
  settings.P2_PROX1_FLT_CNT = doc["P2_PROX1_FLT_CNT"];
  settings.P2_PROX2_FLT_CNT = doc["P2_PROX2_FLT_CNT"];
  settings.P2_LVL_FLT_CNT = doc["P2_LVL_FLT_CNT"];
  settings.P2_BLK_FLT_CNT = doc["P2_BLK_FLT_CNT"];
  settings.P2_OC_FLT_CNT = doc["P2_OC_FLT_CNT"];
  settings.P2_SHRT_FLT_CNT = doc["P2_SHRT_FLT_CNT"];
  settings.TOT_ON_TIME = doc["TOT_ON_TIME"];
  settings.TOT_P1_RUN_TIME = doc["TOT_P1_RUN_TIME"];
  settings.TOT_P1_PSE_TIME = doc["TOT_P1_PSE_TIME"];
  settings.TOT_P2_RUN_TIME = doc["TOT_P2_RUN_TIME"];
  settings.TOT_P2_PSE_TIME = doc["TOT_P2_PSE_TIME"];
  settings.TOT_ERR_TIME = doc["TOT_ERR_TIME"];
  settings.TOT_P1_PROX1_ERR_TIME = doc["TOT_P1_PROX1_ERR_TIME"];
  settings.TOT_P1_PROX2_ERR_TIME = doc["TOT_P1_PROX2_ERR_TIME"];
  settings.TOT_P1_LVL_ERR_TIME = doc["TOT_P1_LVL_ERR_TIME"];
  settings.TOT_P1_SC_TIME = doc["TOT_P1_SC_TIME"];
  settings.TOT_P1_OC_TIME = doc["TOT_P1_OC_TIME"];
  settings.TOT_P1_BLK_TIME = doc["TOT_P1_BLK_TIME"];
  settings.TOT_P2_PROX1_ERR_TIME = doc["TOT_P2_PROX1_ERR_TIME"];
  settings.TOT_P2_PROX2_ERR_TIME = doc["TOT_P2_PROX2_ERR_TIME"];
  settings.TOT_P2_LVL_ERR_TIME = doc["TOT_P2_LVL_ERR_TIME"];
  settings.TOT_P2_SC_TIME = doc["TOT_P2_SC_TIME"];
  settings.TOT_P2_OC_TIME = doc["TOT_P2_OC_TIME"];
  settings.TOT_P2_BLK_TIME = doc["TOT_P2_BLK_TIME"];
  settings.TOT_LMP_SC_TIME = doc["TOT_LMP_SC_TIME"];
  settings.TOT_SEQ_NO = doc["TOT_SEQ_NO"];
  settings.VER_STRING = doc["VER_STRING"].as<String>();
  settings.WEB_URL = doc["WEB_URL"].as<String>();

  file.close();

  // Debug: print the loaded settings as JSON
  //String debugJson;
  //serializeJson(doc, debugJson);
  //Serial.print("Loaded settings JSON: ");
  //Serial.println(debugJson);

  Serial.println("Settings successfully loaded from file");
  return settings;
}

// Function to read a single setting
String readSetting(const char* key) {
  File file = LittleFS.open(SETTINGS_FILE, "r");
  if (!file) {
    Serial.println("Failed to open settings file for reading");
    return "";
  }

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, file);
  if (error) {
    Serial.print("Failed to read settings from file: ");
    Serial.println(error.f_str());
    file.close();
    return "";
  }

  String value = doc[key].as<String>(); // Retrieve the value as a string
  file.close();
  return value;
}

// Calculate day, hours and minutes
String secondsToDhm(uint32_t total_seconds) {
  int days = total_seconds / 86400;
  int remainder = total_seconds % 86400;
  int hours = remainder / 3600;
  remainder = remainder % 3600; // Adjust remainder to exclude hours
  int minutes = remainder / 60;

  // Format the time string with leading zeros for each unit
  String dispTimeStr = "";
  if (days < 10) dispTimeStr += "0";
  dispTimeStr += String(days) + ":";

  if (hours < 10) dispTimeStr += "0";
  dispTimeStr += String(hours) + ":";

  if (minutes < 10) dispTimeStr += "0";
  dispTimeStr += String(minutes);

  return dispTimeStr;
}

// Calculate hours, minutes, and seconds
String secondsToHms(uint32_t total_seconds) {
  int hours = total_seconds / 3600;
  int remainder = total_seconds % 3600;
  int minutes = remainder / 60;
  int seconds = remainder % 60;

  // Format the time string with leading zeros for each unit
  String dispTimeStr = "";
  if (hours < 10) dispTimeStr += "0";
  dispTimeStr += String(hours) + ":";

  if (minutes < 10) dispTimeStr += "0";
  dispTimeStr += String(minutes) + ":";

  if (seconds < 10) dispTimeStr += "0";
  dispTimeStr += String(seconds);

  return dispTimeStr;
}

String secondsToHm(uint32_t total_seconds) {
  // Calculate hours, minutes, and seconds
  int hours = total_seconds / 3600;
  int remainder = total_seconds % 3600;
  int minutes = remainder / 60;

  // Format the time string as "HH:MM"
  char dispTimeStr[6]; // HH:MM format needs 6 characters
  snprintf(dispTimeStr, sizeof(dispTimeStr), "%02d:%02d", hours, minutes);

  return String(dispTimeStr);
}

String secondsToMs(uint32_t total_seconds) {
  // Calculate hours, minutes, and seconds
  int minutes = total_seconds / 60;
  int remainder = total_seconds % 60;
  int seconds = remainder;

  // Format the time string as "HH:MM"
  char dispTimeStr[6]; // MM:SS format needs 6 characters
  snprintf(dispTimeStr, sizeof(dispTimeStr), "%02d:%02d", minutes, seconds);

  return String(dispTimeStr);
}

// Exactly what it says
void annoyingBeep(void){
  digitalWrite(buzzOut, HIGH);
  delay(15);
  digitalWrite(buzzOut, LOW);
}

// Check the buttons for presses, long presses and releases.
void readPins() {
  if (digitalRead(upLeft) == LOW && !upLeftPressed && !upLeftLongPressed) {
      Serial.println("Up/Left pressed");
      annoyingBeep();
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

  if (digitalRead(dnRight) == LOW && !dnRightPressed && !dnRightLongPressed) {
      Serial.println("Down/Right pressed");
      annoyingBeep();
      dnRightStartTime = millis();
      dnRightPressed = true;
  }
  if (dnRightPressed && millis() - dnRightStartTime >= updnLongPress && digitalRead(dnRight) == LOW && !dnRightLongPressed) {
      Serial.println("Down/Right Long Pressed");
      dnRightLongPressed = true;
      dnRightReleased = false;
  }
  if (dnRightPressed && millis() - dnRightStartTime >= debounce && digitalRead(dnRight) == HIGH) {
      dnRightReleased = true;
      Serial.println("Down/Right released");
      dnRightPressed = false;
      dnRightLongPressed = false;
  }

  if (digitalRead(enter) == LOW && !enterPressed && !enterLongPressed) {
      Serial.println("Enter pressed");
      annoyingBeep();
      enterStartTime = millis();
      enterPressed = true;
  }
  if (enterPressed && millis() - enterStartTime >= longPress && digitalRead(enter) == LOW && !enterLongPressed) {
      Serial.println("Enter Long Pressed");
      enterLongPressed = true;
      enterReleased = false;
  }
  if (enterPressed && millis() - enterStartTime >= debounce && digitalRead(enter) == HIGH) {
      enterReleased = true;
      Serial.println("Enter released");
      enterPressed = false;
      enterLongPressed = false;
  }
  if (digitalRead(p1prox1In) == LOW and p1prox1On == false) {
      Serial.println("Pump1 Prox1 Triggered");
      p1prox1On = true; // Reset checked state on trigger
      sendMQTTMessage(p1proxy1Topic, "on");
  }
  if (digitalRead(p1prox2In) == LOW and p1prox2On == false) {
      Serial.println("Pump1 Prox2 Triggered");
      p1prox2On = true; // Reset checked state on trigger
      sendMQTTMessage(p1proxy2Topic, "on");
  }
  if (digitalRead(p1prox1In) == HIGH and p1prox1On == true) {
      Serial.println("Pump1 Prox1 Released");
      p1prox1On = false;
      sendMQTTMessage(p1proxy1Topic, "off");
  }
  if (digitalRead(p1prox2In) == HIGH and p1prox2On == true) {
      Serial.println("Pump1 Prox2 Released");
      p1prox2On = false;
      sendMQTTMessage(p1proxy2Topic, "off");
  }
  if (digitalRead(p2prox1In) == LOW and p2prox1On == false) {
      Serial.println("Pump2 Prox1 Triggered");
      p2prox1On = true; // Reset checked state on trigger
      sendMQTTMessage(p2proxy1Topic, "on");
  }
  if (digitalRead(p2prox2In) == LOW and p2prox2On == false) {
      Serial.println("Pump2 Prox2 Triggered");
      p2prox2On = true; // Reset checked state on trigger
      sendMQTTMessage(p2proxy2Topic, "on");
  }
  if (digitalRead(p2prox1In) == HIGH and p2prox1On == true) {
      Serial.println("Pump2 Prox1 Released");
      p2prox1On = false;
      sendMQTTMessage(p2proxy1Topic, "off");
  }
  if (digitalRead(p2prox2In) == HIGH and p2prox2On == true) {
      Serial.println("Pump2 Prox2 Released");
      p2prox2On = false;
      sendMQTTMessage(p2proxy2Topic, "off");
  }
  // P1 Level checking
  if (settings.P1_LVL == "YES") {
    if (settings.P1_LVL_TYPE == "HEF") {
      // Hall Effect sensor - send percentage (50% for now)
      // In future, replace 50 with actual sensor reading
      static unsigned long lastP1LevelUpdate = 0;
      if (millis() - lastP1LevelUpdate > 10000) { // Update every 10 seconds
        sendMQTTMessage(p1levelTopic, "50");
        lastP1LevelUpdate = millis();
      }
    } else {
      // Digital level sensor
      if (digitalRead(p1lvlIn) == LOW && p1lvlOn == false) {
        Serial.println("Pump1 Level Triggered");
        p1lvlOn = true;
        sendMQTTMessage(p1levelTopic, "on");
      }
      if (digitalRead(p1lvlIn) == HIGH && p1lvlOn == true) {
        Serial.println("Pump1 Level Released");
        p1lvlOn = false;
        sendMQTTMessage(p1levelTopic, "off");
      }
    }
  }

  // P2 Level checking
  if (settings.P2_LVL == "YES") {
    if (settings.P2_LVL_TYPE == "HEF") {
      // Hall Effect sensor - send percentage (50% for now)
      static unsigned long lastP2LevelUpdate = 0;
      if (millis() - lastP2LevelUpdate > 10000) { // Update every 10 seconds
        sendMQTTMessage(p2levelTopic, "50");
        lastP2LevelUpdate = millis();
      }
    } else {
      // Digital level sensor
      if (digitalRead(p2lvlIn) == LOW && p2lvlOn == false) {
        Serial.println("Pump2 Level Triggered");
        p2lvlOn = true;
        sendMQTTMessage(p2levelTopic, "on");
      }
      if (digitalRead(p2lvlIn) == HIGH && p2lvlOn == true) {
        Serial.println("Pump2 Level Released");
        p2lvlOn = false;
        sendMQTTMessage(p2levelTopic, "off");
      }
    }
  }
}

// Get the current variable values for the setup function so no default values accidentally get saved if the user chooses not to save changes.
void getOriginalValues(){
  oldP1MainMode = settings.P1_MAIN_MODE;
  oldP2MainMode = settings.P2_MAIN_MODE;
  oldP1PauseTime = settings.P1_PAUSE_TIME;
  oldP2PauseTime = settings.P2_PAUSE_TIME;
  oldP1RunMode = settings.P1_RUN_MODE;
  oldP2RunMode = settings.P2_RUN_MODE;
  oldP1Prox1 = settings.P1_PROX1;
  oldP1Prox2 = settings.P1_PROX2;
  oldP2Prox1 = settings.P2_PROX1;
  oldP2Prox2 = settings.P2_PROX2;
  oldP1RunTimeCyc = settings.P1_RUN_TIME_CYC;
  oldP2RunTimeCyc = settings.P2_RUN_TIME_CYC;
  oldP1Prox1Dwell = settings.P1_PROX1_DWELL;
  oldP1Prox2Dwell = settings.P1_PROX2_DWELL;
  oldP2Prox1Dwell = settings.P2_PROX1_DWELL;
  oldP2Prox2Dwell = settings.P2_PROX2_DWELL;
  oldP1Lvl = settings.P1_LVL;
  oldP2Lvl = settings.P2_LVL;
  oldP1LvlType = settings.P1_LVL_TYPE;
  oldP2LvlType = settings.P2_LVL_TYPE;
  oldP1LvlNonC = settings.P1_LVL_NONC;
  oldP2LvlNonC = settings.P2_LVL_NONC;
  oldLmpYN = settings.EXT_LAMP;
  oldLmpType = settings.LAMP_TYP;
  oldP1BlockCurrent = settings.P1_BLOCK_CURRENT;
  oldP2BlockCurrent = settings.P2_BLOCK_CURRENT;
  oldP2InUse = settings.PUMP2_IN_USE;

  Serial.println("Original values loaded for setup menu");
}

// If the user chooses not to save changes made in the setup manu, this restores the original values.
// Each setup function keeps the old variables in a global variable, so they can be restored here.
void restoreOriginalValues(){
  settings.P1_MAIN_MODE = oldP1MainMode;
  settings.P2_MAIN_MODE = oldP2MainMode;
  settings.P1_PAUSE_TIME = oldP1PauseTime;
  settings.P2_PAUSE_TIME = oldP2PauseTime;
  settings.P1_RUN_MODE = oldP1RunMode;
  settings.P2_RUN_MODE = oldP2RunMode;
  settings.P1_PROX1 = oldP1Prox1;
  settings.P1_PROX2 = oldP1Prox2;
  settings.P2_PROX1 = oldP2Prox1;
  settings.P2_PROX2 = oldP2Prox2;
  settings.P1_RUN_TIME_CYC = oldP1RunTimeCyc;
  settings.P2_RUN_TIME_CYC = oldP2RunTimeCyc;
  settings.P1_PROX1_DWELL = oldP1Prox1Dwell;
  settings.P1_PROX2_DWELL = oldP1Prox2Dwell;
  settings.P2_PROX1_DWELL = oldP2Prox1Dwell;
  settings.P2_PROX2_DWELL = oldP2Prox2Dwell;
  settings.P1_LVL = oldP1Lvl;
  settings.P2_LVL = oldP2Lvl;
  settings.P1_LVL_TYPE = oldP1LvlType;
  settings.P2_LVL_TYPE = oldP2LvlType;
  settings.P1_LVL_NONC = oldP1LvlNonC;
  settings.P2_LVL_NONC = oldP2LvlNonC;
  settings.EXT_LAMP = oldLmpYN;
  settings.LAMP_TYP = oldLmpType;
  settings.P1_BLOCK_CURRENT = oldP1BlockCurrent;
  settings.P2_BLOCK_CURRENT = oldP2BlockCurrent;
  settings.PUMP2_IN_USE = oldP2InUse;

  Serial.println("Restored original values");
}


void triggerFault(int faultIndex){
  if (faultIndex >= 0 && faultIndex < numFaults){
    faultStatus[faultIndex]++;
  }
}

void clearFault(int faultIndex){
  if (faultIndex >= 0 and faultIndex < numFaults){
    faultStatus[faultIndex] = 0;
  }
}

void transitionToWiFiMode() {
  bluetoothFallbackActive = false;
  isAPMode = false;
  
  // Stop BLE advertising and cleanup
  if (bluetoothActive) {
    BLEDevice::getAdvertising()->stop();
    if (pBLEServer) {
      pBLEServer->removeService(pBLEServer->getServiceByUUID(BLE_SERVICE_UUID));
    }
    Serial.println("BLE advertising stopped");
  }
  
  // Configure MQTT client for WiFi mode
  wifiClient.setInsecure();
  mqttClient.setServer(wifiSettings.mqttServerAddress.c_str(), wifiSettings.mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(10);
  
  Serial.println("=== TRANSITIONED TO WIFI MODE ===");
  Serial.println("IP address: " + WiFi.localIP().toString());
  
  // Notify any connected clients about the transition
  if (webClientActive) {
    sendMQTTMessage("a3/" + serialNumber + "/status", "wifi_connected");
  }
}

void transitionToBluetoothMode() {
  bluetoothFallbackActive = true;
  isAPMode = true;
  
  // Disconnect MQTT
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  
  // Start BLE fallback
  setupBluetoothFallback();
  
  Serial.println("=== TRANSITIONED TO BLE MODE ===");
}


#endif
