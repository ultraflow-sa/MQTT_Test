#ifndef SUBS_H
#define SUBS_H

#include "defs.h"

extern void mqttCallback(char* topic, byte* payload, unsigned int length);
extern void setupServerEndpoints();
extern void startEmbeddedBroker();
void sendBLEMessage(const String &topic, const String &payload);
void cleanupBLE();
void startBLEMode();
void setupBluetoothFallback();

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
    Serial.println("=== BLE CLIENT CONNECTED ===");
    lastBluetoothHeartbeat = millis();
    
    // Stop advertising when connected
    BLEDevice::getAdvertising()->stop();
    Serial.println("BLE advertising stopped (client connected)");
  };

  void onDisconnect(BLEServer* pServer) {
    bleDeviceConnected = false;
    Serial.println("=== BLE CLIENT DISCONNECTED ===");
    
    // Safety: Stop pumps when client disconnects
    digitalWrite(pump1Out, LOW);
    digitalWrite(pump2Out, LOW);
    Serial.println("Pumps stopped due to BLE disconnect");
    
    // Reset session flags
    bleSessionActive = false;
    webClientActive = false;
    lastBluetoothHeartbeat = 0;

    // Restart advertising with Android compatibility
    delay(1000);
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(true);  // Re-enable for Android
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising restarted with Android compatibility");
  }
};

class MyBLECharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    
    if (rxValue.length() > 0) {
      Serial.println("BLE received: " + rxValue);
      lastBluetoothHeartbeat = millis();
      
      DynamicJsonDocument doc(512);
      DeserializationError error = deserializeJson(doc, rxValue);
      
      if (!error) {
        String topic = doc["topic"] | "";
        String payload = doc["payload"] | "";
        
        if (topic.length() > 0) {
          // Process like MQTT message
          mqttCallback((char*)topic.c_str(), (byte*)payload.c_str(), payload.length());
          
          // Also bridge to any WebSocket clients
          bridgeBLEToWebSocket(topic, payload);
        }
      } else {
        Serial.println("BLE message JSON parse error: " + String(error.c_str()));
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
  if (!isAPMode && mqttClient.connected()) {
    // WiFi mode - use MQTT
    mqttClient.publish(topic.c_str(), payload.c_str());
    Serial.println("MQTT message sent - Topic: " + topic + ", Payload: " + payload);
  } else if (bluetoothActive && bleDeviceConnected) {
    // BLE mode - send via BLE characteristic
    sendBLEMessage(topic, payload);
    
    // Also bridge to WebSocket clients (for web interface)
    bridgeBLEToWebSocket(topic, payload);
  } else if (isAPMode && ws.count() > 0) {
    // AP mode with WebSocket clients
    bridgeBLEToWebSocket(topic, payload);
  } else {
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

void sendInitialDeviceInfo() {
  if (!pTxCharacteristic) return;
  
  // Send serial number
  DynamicJsonDocument doc(256);
  doc["topic"] = "a3/" + serialNumber + "/serialResponse";
  doc["payload"] = "{\"serial\":\"" + serialNumber + "\",\"version\":\"" + settings.VER_STRING + "\"}";
  doc["timestamp"] = millis();
  
  String message;
  serializeJson(doc, message);
  
  pTxCharacteristic->setValue(message.c_str());
  pTxCharacteristic->notify();
  
  Serial.println("Sent initial device info via BLE");
}

void setupBluetoothFallback() {
  Serial.println("=== SETUPBLUETOOTHFALLBACK CALLED ===");
  
  String bleName = "A3_Setup_" + serialNumber;
  Serial.println("BLE Name will be: " + bleName);
  
  // Check if already active
  if (bluetoothActive) {
    Serial.println("BLE already active - cleaning up first");
    BLEDevice::deinit(true);
    delay(3000);
    bluetoothActive = false;
  }
  
  Serial.println("Step 1: Initializing BLE device...");
  try {
    BLEDevice::init(bleName.c_str());
    Serial.println("Step 1: SUCCESS - BLE device initialized");
  } catch (...) {
    Serial.println("Step 1: FAILED - BLE device initialization failed");
    return;
  }
  
Serial.println("Step 2: Setting BLE power and security...");
try {
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  
  // Disable all security/pairing requirements
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_NO_MITM);
  BLEDevice::setSecurityCallbacks(nullptr);
  BLEDevice::setMTU(512);
  
  // Explicitly disable pairing/bonding
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  
  Serial.println("Step 2: SUCCESS - BLE power and security set (no pairing required)");
} catch (...) {
  Serial.println("Step 2: FAILED - BLE power/security configuration failed");
  return;
}
  
  Serial.println("Step 3: Creating BLE server...");
  try {
    pBLEServer = BLEDevice::createServer();
    if (pBLEServer == nullptr) {
      Serial.println("Step 3: FAILED - pBLEServer is NULL");
      return;
    }
    pBLEServer->setCallbacks(new MyBLEServerCallbacks());
    Serial.println("Step 3: SUCCESS - BLE server created");
  } catch (...) {
    Serial.println("Step 3: FAILED - BLE server creation failed");
    return;
  }

  Serial.println("Step 4: Creating BLE service...");
  try {
    BLEService *pService = pBLEServer->createService(BLE_SERVICE_UUID);
    if (pService == nullptr) {
      Serial.println("Step 4: FAILED - pService is NULL");
      return;
    }
    Serial.println("Step 4: SUCCESS - BLE service created with UUID: " + String(BLE_SERVICE_UUID));
  } catch (...) {
    Serial.println("Step 4: FAILED - BLE service creation failed");
    return;
  }

  Serial.println("Step 5: Creating TX characteristic...");
  try {
    BLEService *pService = pBLEServer->getServiceByUUID(BLE_SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(
                        BLE_CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
                      );
    if (pTxCharacteristic == nullptr) {
      Serial.println("Step 5: FAILED - pTxCharacteristic is NULL");
      return;
    }
    pTxCharacteristic->addDescriptor(new BLE2902());
    Serial.println("Step 5: SUCCESS - TX characteristic created");
  } catch (...) {
    Serial.println("Step 5: FAILED - TX characteristic creation failed");
    return;
  }

  Serial.println("Step 6: Creating RX characteristic...");
  try {
    BLEService *pService = pBLEServer->getServiceByUUID(BLE_SERVICE_UUID);
    pRxCharacteristic = pService->createCharacteristic(
                        BLE_CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                      );
    if (pRxCharacteristic == nullptr) {
      Serial.println("Step 6: FAILED - pRxCharacteristic is NULL");
      return;
    }
    pRxCharacteristic->setCallbacks(new MyBLECharacteristicCallbacks());
    Serial.println("Step 6: SUCCESS - RX characteristic created");
  } catch (...) {
    Serial.println("Step 6: FAILED - RX characteristic creation failed");
    return;
  }

  Serial.println("Step 7: Starting BLE service...");
  try {
    BLEService *pService = pBLEServer->getServiceByUUID(BLE_SERVICE_UUID);
    pService->start();
    Serial.println("Step 7: SUCCESS - BLE service started");
  } catch (...) {
    Serial.println("Step 7: FAILED - BLE service start failed");
    return;
  }

  Serial.println("Step 8: Configuring advertising for Android visibility...");
  try {
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    if (pAdvertising == nullptr) {
      Serial.println("Step 8: FAILED - pAdvertising is NULL");
      return;
    }
    
    // Clear any existing advertising data
    pAdvertising->stop();
    delay(1000);
    
    // Set advertising data manually for maximum Android compatibility
    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;           // Include device name
    adv_data.include_txpower = true;        // Include TX power for Android
    adv_data.min_interval = 0x20;           // 20ms intervals
    adv_data.max_interval = 0x40;           // 40ms intervals
    adv_data.appearance = 0x00;
    adv_data.manufacturer_len = 0;
    adv_data.p_manufacturer_data = NULL;
    adv_data.service_data_len = 0;
    adv_data.p_service_data = NULL;
    adv_data.service_uuid_len = 16;         // 128-bit UUID
    
    // Set the Nordic UART service UUID
    static uint8_t service_uuid[16] = {
      0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
      0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
    };
    adv_data.p_service_uuid = service_uuid;
    
    adv_data.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    
    esp_ble_gap_config_adv_data(&adv_data);
    
    // Set advertising parameters for maximum visibility
    esp_ble_adv_params_t adv_params = {};
    adv_params.adv_int_min = 0x20;          // 20ms
    adv_params.adv_int_max = 0x40;          // 40ms  
    adv_params.adv_type = ADV_TYPE_IND;     // Connectable, scannable
    adv_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.peer_addr_type = BLE_ADDR_TYPE_PUBLIC;
    adv_params.channel_map = ADV_CHNL_ALL;  // Advertise on all 3 channels
    adv_params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;
    
    esp_ble_gap_start_advertising(&adv_params);
    
    Serial.println("Step 8: SUCCESS - Android-optimized advertising configured");
  } catch (...) {
    Serial.println("Step 8: FAILED - Android advertising configuration failed");
    return;
  }

  Serial.println("Step 9: Starting advertising...");
  try {
    BLEDevice::startAdvertising();
    Serial.println("Step 9: SUCCESS - Advertising started");
  } catch (...) {
    Serial.println("Step 9: FAILED - Advertising start failed");
    return;
  }

  bluetoothActive = true;
  
  Serial.println("=== BLE SETUP COMPLETE ===");
  Serial.println("Device name: " + bleName);
  Serial.println("Service UUID: " + String(BLE_SERVICE_UUID));
  Serial.println("bluetoothActive: " + String(bluetoothActive));
  Serial.println("pBLEServer: " + String(pBLEServer != nullptr ? "Valid" : "NULL"));
  Serial.println("pTxCharacteristic: " + String(pTxCharacteristic != nullptr ? "Valid" : "NULL"));
  Serial.println("pRxCharacteristic: " + String(pRxCharacteristic != nullptr ? "Valid" : "NULL"));
  
  // Final verification
  Serial.println("=== ADVERTISING VERIFICATION ===");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  if (pAdvertising != nullptr) {
    Serial.println("Advertising object exists");
    // Add any other verification checks here
  } else {
    Serial.println("Advertising object is NULL!");
  }
}

void startBluetoothFallback() {
  Serial.println("Starting Bluetooth fallback mode...");
  
  // Stop any existing WiFi connections
  WiFi.disconnect(true);
  delay(1000);
  
  // Switch to AP mode for BLE
  WiFi.mode(WIFI_AP);
  delay(1000);
  
  // Start AP
  String apSSID = "A3_Setup_" + serialNumber;
  String apPassword = "12345678";
  
  if (WiFi.softAP(apSSID.c_str(), apPassword.c_str())) {
    Serial.println("AP started for BLE mode: " + apSSID);
    Serial.println("AP IP: " + WiFi.softAPIP().toString());
    
    // Start DNS server properly
    if (!dnsServer.start(53, "*", WiFi.softAPIP())) {
      Serial.println("Failed to start DNS server");
    } else {
      Serial.println("DNS server started for captive portal");
    }
    
    // Start BLE
    setupBluetoothFallback();
    
    bluetoothFallbackActive = true;
    isAPMode = true;
    
    Serial.println("=== BLE MODE ACTIVE ===");
  } else {
    Serial.println("Failed to start AP for BLE mode");
  }
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

// ------------------ MQTT Setup Function ------------------
void setupMQTT(){
  wifiClient.setInsecure();
  Serial.println("MQTT Server: " + wifiSettings.mqttServerAddress);
  mqttClient.setServer(wifiSettings.mqttServerAddress.c_str(), wifiSettings.mqttPort);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  mqttClient.setSocketTimeout(10);
  Serial.println("MQTT client configured - connection will be established in checkMQTTConnection()");  
}

void transitionToBluetoothMode() {
  Serial.println("Transitioning to Bluetooth mode...");
  
  // Stop MQTT
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  
  // Disconnect WiFi
  WiFi.disconnect(true);
  delay(1000);
  
  // Start BLE mode
  startBLEMode();
  
  Serial.println("Bluetooth mode active");
}

// ------------------ Transition to WiFi Mode Function ------------------
void transitionToWiFiMode() {
  Serial.println("Transitioning to WiFi mode...");
  
  // Stop BLE
  if (bluetoothActive) {
    Serial.println("Stopping BLE service");
    BLEDevice::deinit(false);
    bluetoothActive = false;
    bluetoothFallbackActive = false;
    bleDeviceConnected = false;
    delay(2000);
    Serial.println("BLE stopped");
  }
  
  // Stop DNS server
  dnsServer.stop();
  
  // Switch to STA mode
  WiFi.softAPdisconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  
  // Connect to WiFi
  WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  
  // Wait for connection
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi mode active");
    Serial.println("IP address: " + WiFi.localIP().toString());
    
    isAPMode = false;
    bluetoothFallbackActive = false;
    currentState = STATE_WIFI_CONNECTED;
    setupMQTT();
    Serial.println("MQTT setup complete");
  } else {
    Serial.println("\nWiFi connection failed - returning to BLE mode");
    startBLEMode();
  }
}

void attemptWiFiConnection() {
  if (currentState == STATE_BLE_CONNECTED && bleSessionActive) {
    Serial.println("BLE session active - deferring WiFi connection");
    return;
  }
  
  Serial.println("Attempting WiFi connection to: " + wifiSettings.ssid);
  currentState = STATE_WIFI_CONNECTING;
  
  // Clean shutdown of BLE if active
  if (bluetoothActive) {
    cleanupBLE();
  }
  
  WiFi.mode(WIFI_STA);
  delay(1000);
  WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.password.c_str());
  
  // Try for 8 seconds
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 8000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== WiFi Connected Successfully ===");
    Serial.println("IP address: " + WiFi.localIP().toString());
    
    currentState = STATE_WIFI_CONNECTED;
    isAPMode = false;
    bluetoothFallbackActive = false;
    setupMQTT();
    
    if (webURL.length() > 0) {
      Serial.println("Web URL configured: " + webURL);
    }
    
  } else {
    Serial.println("\n=== WiFi Connection Failed ===");
    if (initialSetupMode) {
      startBLEMode();
    } else {
      currentState = STATE_SEARCHING;
      lastConnectionAttempt = millis();
    }
  }
}

void startBLEMode() {
  Serial.println("=== STARTBLEMODE CALLED ===");
  
  currentState = STATE_BLE_ADVERTISING;
  bleAdvertisingStartTime = millis();
  
  // Switch to AP mode for web server
  Serial.println("Setting WiFi to AP mode...");
  WiFi.mode(WIFI_AP);
  delay(1000);
  
  String apSSID = "A3_Setup_" + serialNumber;
  Serial.println("Starting WiFi AP: " + apSSID);
  WiFi.softAP(apSSID.c_str(), "12345678");
  
  // Start DNS server for captive portal
  Serial.println("Starting DNS server...");
  if (!dnsServer.start(53, "*", WiFi.softAPIP())) {
    Serial.println("Failed to start DNS server");
  } else {
    Serial.println("DNS server started successfully");
  }
  
  // Start BLE advertising - THIS IS THE CRITICAL CALL
  Serial.println("About to call setupBluetoothFallback()...");
  setupBluetoothFallback();
  Serial.println("setupBluetoothFallback() completed");
  
  bluetoothFallbackActive = true;
  isAPMode = true;
  
  Serial.println("=== STARTBLEMODE COMPLETE ===");
  Serial.println("currentState: " + String(currentState));
  Serial.println("bluetoothActive: " + String(bluetoothActive));
  Serial.println("bluetoothFallbackActive: " + String(bluetoothFallbackActive));
  Serial.println("isAPMode: " + String(isAPMode));
}

void cleanupBLE() {
  if (bluetoothActive) {
    Serial.println("Cleaning up BLE connection");
    BLEDevice::deinit(false);
    bluetoothActive = false;
    bleDeviceConnected = false;
    bluetoothFallbackActive = false;
    delay(1000);
  }
  
  if (isAPMode) {
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    isAPMode = false;
  }
}

void handleConnectionStateMachine() {
  switch (currentState) {
    case STATE_WIFI_CONNECTING:
      // Wait for WiFi connection result (handled in attemptWiFiConnection)
      break;
      
    case STATE_WIFI_CONNECTED:
      // Monitor WiFi connection
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("=== WiFi Connection Lost ===");
        currentState = STATE_SEARCHING;
        lastConnectionAttempt = millis();
      }
      break;
      
    case STATE_BLE_ADVERTISING:
      // Check for BLE connection
      if (bleDeviceConnected && !bleSessionActive) {
        Serial.println("=== BLE Client Connected ===");
        currentState = STATE_BLE_CONNECTED;
        bleSessionActive = true;
        webClientActive = true;
        lastBluetoothHeartbeat = millis();
      }
      
      // In initial setup mode, keep advertising indefinitely
      if (!initialSetupMode && (millis() - bleAdvertisingStartTime > BLE_ADVERTISING_TIMEOUT)) {
        if (wifiCredentialsAvailable) {
          Serial.println("BLE advertising timeout (30s) - retrying WiFi connection");
          attemptWiFiConnection();
        }
      }
      break;
      
    case STATE_BLE_CONNECTED:
      // Monitor BLE session
      if (!bleDeviceConnected) {
        Serial.println("=== BLE Client Disconnected ===");
        bleSessionActive = false;
        webClientActive = false;
        lastBluetoothHeartbeat = 0;
        
        // Session ended - can now try WiFi if available
        if (wifiCredentialsAvailable && !initialSetupMode) {
          Serial.println("BLE session ended - attempting WiFi connection");
          attemptWiFiConnection();
        } else {
          // Return to advertising
          currentState = STATE_BLE_ADVERTISING;
          bleAdvertisingStartTime = millis();  // ✓ Reset advertising timer
          Serial.println("Returning to BLE advertising");
        }
      }
      break;
      
    case STATE_SEARCHING:
      // Periodic connection attempts
      if (millis() - lastConnectionAttempt > CONNECTION_RETRY_INTERVAL) {
        if (wifiCredentialsAvailable) {
          Serial.println("Retrying WiFi connection");
          attemptWiFiConnection();
          
          // If WiFi fails repeatedly, fall back to BLE
          static int wifiFailCount = 0;
          if (currentState == STATE_SEARCHING) {  // If we're still searching after attempt
            wifiFailCount++;
            if (wifiFailCount >= 2) {  // After 2 failed attempts (10 seconds total)
              Serial.println("WiFi failed multiple times - starting BLE fallback (30s advertising)");
              wifiFailCount = 0;
              startBLEMode();
            }
          } else {
            wifiFailCount = 0;  // Reset on success
          }
        } else {
          Serial.println("No WiFi credentials - starting BLE mode");
          startBLEMode();
        }
        lastConnectionAttempt = millis();
      }
      break;
  }
}

void handleClientSessionMonitoring() {
  // Only check timeouts when we have an active session
  if (webClientActive && (lastWebHeartbeat > 0 || lastBluetoothHeartbeat > 0)) {
    unsigned long lastHeartbeat = max(lastWebHeartbeat, lastBluetoothHeartbeat);
    
    if (millis() - lastHeartbeat > HEARTBEAT_TIMEOUT) {
      Serial.println("=== CLIENT SESSION TIMEOUT ===");
      
      // Safety: Stop pumps
      digitalWrite(pump1Out, LOW);
      digitalWrite(pump2Out, LOW);
      Serial.println("Pumps stopped due to session timeout");
      
      // Reset session state
      webClientActive = false;
      lastWebHeartbeat = 0;
      lastBluetoothHeartbeat = 0;
      
      // In BLE mode, this might trigger a state change
      if (currentState == STATE_BLE_CONNECTED) {
        bleSessionActive = false;
      }
    }
  }
}

void handleAutoCalibration() {
  // P1 Auto Calibration
  if (pump1AutoCalibrate && !pump1Running) {
    Serial.println("Starting P1 auto calibration");
    sendMQTTMessage("a3/" + serialNumber + "/test/pump1", "startingTest");
    digitalWrite(pump1Out, HIGH);
    pump1Running = true;
    pump1StartTime = millis();
  }

  if (pump1AutoCalibrate && pump1Running && (millis() - pump1StartTime >= 10000)) {
    digitalWrite(pump1Out, LOW);
    pump1Running = false;
    pump1AutoCalibrate = false;
    String calibratedCurrent = "1"; // Replace with actual measurement
    sendMQTTMessage("a3/" + serialNumber + "/test/pump1", calibratedCurrent);
    Serial.println("P1 auto calibration complete: " + calibratedCurrent);
  }

  // P2 Auto Calibration (similar logic)
  if (pump2AutoCalibrate && !pump2Running) {
    Serial.println("Starting P2 auto calibration");
    sendMQTTMessage("a3/" + serialNumber + "/test/pump2", "startingTest");
    digitalWrite(pump2Out, HIGH);
    pump2Running = true;
    pump2StartTime = millis();
  }

  if (pump2AutoCalibrate && pump2Running && (millis() - pump2StartTime >= 10000)) {
    digitalWrite(pump2Out, LOW);
    pump2Running = false;
    pump2AutoCalibrate = false;
    String calibratedCurrent = "1"; // Replace with actual measurement
    sendMQTTMessage("a3/" + serialNumber + "/test/pump2", calibratedCurrent);
    Serial.println("P2 auto calibration complete: " + calibratedCurrent);
  }
}

void sendBLEMessage(const String &topic, const String &payload) {
  if (bluetoothActive && bleDeviceConnected && pTxCharacteristic) {
    DynamicJsonDocument doc(512);
    doc["topic"] = topic;
    doc["payload"] = payload;
    doc["timestamp"] = millis();
    
    String message;
    serializeJson(doc, message);
    
    Serial.println("Sending BLE message: " + message);
    
    // Send via BLE characteristic
    pTxCharacteristic->setValue(message.c_str());
    pTxCharacteristic->notify();
    
    // Update heartbeat
    lastBluetoothHeartbeat = millis();
    
  } else {
    Serial.println("Cannot send BLE message - not connected or characteristic not available");
    Serial.println("bluetoothActive: " + String(bluetoothActive));
    Serial.println("bleDeviceConnected: " + String(bleDeviceConnected));
    Serial.println("pTxCharacteristic: " + String(pTxCharacteristic != nullptr ? "OK" : "NULL"));
  }
}

#endif
