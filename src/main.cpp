#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>

// Uncomment to force clearing stored credentials on startup
//#define CLEAR_CREDENTIALS

// ------------------ Local Settings Structure ------------------
// Now all configuration variables are stored here.
struct Settings {
  String ssid;
  String password;
  String htmlVersion;
  String cssVersion;
  
  String mqttServerAddress;
  int    mqttPort;
  String updateTopic;
  
  // Base URL for all web updates (e.g., "http://mydomain/webupdates")
  String baseUpdateUrl;
};

#define SETTINGS_FILE "/settings.json"
Settings settings;  // Global settings instance

// ------------------ Load/Save Settings Functions ------------------
bool loadSettings() {
  if (!SPIFFS.exists(SETTINGS_FILE)) {
    Serial.println("Settings file not found. Using default settings.");
    // Default values.
    settings.ssid              = "";
    settings.password          = "";
    settings.htmlVersion       = "0";
    settings.cssVersion        = "0";
    
    settings.mqttServerAddress = "your.mqtt.server.com";
    settings.mqttPort          = 1883;
    settings.updateTopic       = "device/updates";
    
    // Set the base update folder URL.
    settings.baseUpdateUrl     = "http://mydomain/webupdates";
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
  settings.ssid              = doc["ssid"]              | "";
  settings.password          = doc["password"]          | "";
  settings.htmlVersion       = doc["htmlVersion"]       | "0";
  settings.cssVersion        = doc["cssVersion"]        | "0";
  
  settings.mqttServerAddress = doc["mqttServerAddress"] | "your.mqtt.server.com";
  settings.mqttPort          = doc["mqttPort"]          | 1883;
  settings.updateTopic       = doc["updateTopic"]       | "device/updates";
  
  settings.baseUpdateUrl     = doc["baseUpdateUrl"]     | "http://mydomain/webupdates";
  
  Serial.println("Settings loaded from SPIFFS.");
  return true;
}

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

// ------------------ Update File Function ------------------
// This function downloads the file from the provided URL via HTTP,
// then writes it to SPIFFS at the provided file path.
bool updateFile(const char* url, const char* spiffsPath) {
  HTTPClient http;
  Serial.println("Downloading from: " + String(url));
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    File file = SPIFFS.open(spiffsPath, FILE_WRITE);
    if (!file) {
      Serial.println("Failed to open file for writing: " + String(spiffsPath));
      http.end();
      return false;
    }
    file.print(payload);
    file.close();
    Serial.println("File updated: " + String(spiffsPath));
    http.end();
    return true;
  } else {
    Serial.println("HTTP GET failed, code: " + String(httpCode));
    http.end();
    return false;
  }
}

// ------------------ Global Objects ------------------ 
AsyncWebServer server(80);
DNSServer dnsServer;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String serialNumber = "000001";
String VER_STRING = "v1.0.0";

const char* AP_SSID = "Setup-A3";

// ------------------ LED and Button Setup ------------------
#define LED_PIN 48
#define LED_COUNT 1
Adafruit_NeoPixel rgbLamp(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int upLeft = 5;
bool upLeftPressed = false;
bool upLeftLongPressed = false;
bool upLeftReleased = false;
unsigned long upLeftStartTime = 0;
unsigned long updnLongPress = 1500;  // ms for long press
unsigned long debounce = 150;        // ms debounce

// Global flags
bool credentialsSaved = false;
bool apFinalized = false;

// ------------------ Helper Functions ------------------
// Template processor for HTML
String processor(const String &var) {
  if (var == "SERIAL_NUMBER")
    return serialNumber;
  if (var == "VER_STRING")
    return VER_STRING;
  return String();
}

// Clear saved credentials
void clearCredentials() {
  settings.ssid = "";
  settings.password = "";
  saveSettings();
  Serial.println("Cleared saved WiFi credentials in settings");
}

// ------------------ Refactored RGB LED Function ------------------
void showRGBleds(int red, int grn, int blue) {
  for (int pixel = 0; pixel < LED_COUNT; pixel++) {
    rgbLamp.setPixelColor(pixel, red, grn, blue);
    rgbLamp.show();
    delay(1);
  }
}

// ------------------ Button Reading ------------------
void readPins() {
  if (digitalRead(upLeft) == LOW && !upLeftPressed && !upLeftLongPressed) {
    Serial.println("Up/Left pressed");
    upLeftStartTime = millis();
    upLeftPressed = true;
  }
  if (upLeftPressed && millis() - upLeftStartTime >= updnLongPress &&
      digitalRead(upLeft) == LOW && !upLeftLongPressed) {
    Serial.println("Up/Left Long Pressed");
    upLeftLongPressed = true;
    upLeftReleased = false;
  }
  if (upLeftPressed && millis() - upLeftStartTime >= debounce &&
      digitalRead(upLeft) == HIGH) {
    upLeftReleased = true;
    Serial.println("Up/Left released");
    upLeftPressed = false;
    upLeftLongPressed = false;
  }
}

// ------------------ OTA Initialization ------------------
void initOTA() {
  ArduinoOTA.setHostname(("A3_" + serialNumber).c_str());
  ArduinoOTA.onStart([]() {
    Serial.println("Start OTA update...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA update complete.");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Initialized");
}

// ------------------ MQTT Callback and Reconnect ------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message received on topic: ");
  Serial.println(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.println("Message: " + msg);
  
  // Expect JSON message like:
  // {"htmlVersion": "2025-06-02T12:00:00Z", "cssVersion": "2025-06-02T12:00:00Z"}
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("MQTT JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }
  
  String remoteHtmlVersion = doc["htmlVersion"] | "";
  String remoteCssVersion  = doc["cssVersion"]  | "";
  
  Serial.println("Remote HTML version: " + remoteHtmlVersion);
  Serial.println("Remote CSS version: " + remoteCssVersion);
  
  // Build full URLs based on the base update URL.
  String fullHtmlUrl = settings.baseUpdateUrl + "/main.html";
  String fullCssUrl  = settings.baseUpdateUrl + "/style.css";
  
  // Compare and update HTML file if necessary.
  if (remoteHtmlVersion != settings.htmlVersion) {
    Serial.println("New HTML version available. Attempting update...");
    if (updateFile(fullHtmlUrl.c_str(), "/main.html")) {
      settings.htmlVersion = remoteHtmlVersion;
      saveSettings();
      Serial.println("HTML file updated.");
    }
  }
  
  // Compare and update CSS file if necessary.
  if (remoteCssVersion != settings.cssVersion) {
    Serial.println("New CSS version available. Attempting update...");
    if (updateFile(fullCssUrl.c_str(), "/style.css")) {
      settings.cssVersion = remoteCssVersion;
      saveSettings();
      Serial.println("CSS file updated.");
    }
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" connected.");
      mqttClient.subscribe(settings.updateTopic.c_str());
    } else {
      Serial.print(" failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" – retrying in 5 seconds");
      delay(5000);
    }
  }
}

// ------------------ WiFi and Mode Management ------------------
void startStationMode(const String &wifiSSID, const String &wifiPassword) {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  Serial.print("Connecting to STA (up to 15 sec)");
  unsigned long startTime = millis();
  bool connected = false;
  while (millis() - startTime < 15000) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    yield();
    Serial.print(".");
  }
  Serial.println();
  if (connected) {
    Serial.println("STA connected!");
  } else {
    Serial.println("STA connection failed; remaining in AP mode.");
  }
}

void finalizeAPMode() {
  Serial.println("Finalizing AP mode: disabling SoftAP and switching to STA-only mode...");
  WiFi.softAPdisconnect(true);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  WiFi.mode(WIFI_STA);
  Serial.println("Finalization complete: now running as STA only.");
}

void startCaptivePortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID);
  dnsServer.start(53, "*", WiFi.softAPIP());
  Serial.println("Started captive portal (AP mode).");
}

// ------------------ Server Endpoints ------------------
void setupServerEndpoints() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Endpoint / requested");
    if (settings.ssid.length() == 0) {
      Serial.println("No credentials saved; serving basic.html");
      request->send(SPIFFS, "/basic.html", "text/html", false, processor);
    } else if (WiFi.status() == WL_CONNECTED && settings.ssid.length() > 0 && !apFinalized) {
      Serial.println("STA connected with credentials – redirecting to /finalizemain");
      request->redirect("/finalizemain");
    } else {
      Serial.println("AP finalized; serving main page");
      request->send(SPIFFS, "/main.html", "text/html", false);
    }
  });
  
  server.on("/basic.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Endpoint /basic.html requested");
    request->send(SPIFFS, "/basic.html", "text/html", false, processor);
  });
  
  server.on("/notify.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Serving notify.html");
    request->send(SPIFFS, "/notify.html", "text/html");
  });
  
  server.on("/main.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Endpoint /main.html requested");
    request->send(SPIFFS, "/main.html", "text/html", false);
  });
  
  server.on("/getip", HTTP_GET, [](AsyncWebServerRequest *request) {
    String ip = WiFi.localIP().toString();
    Serial.println("GETIP requested, sending IP: " + ip);
    request->send(200, "application/json", "{\"ip\":\"" + ip + "\"}");
  });
  
  server.on("/savewifi", HTTP_POST, [](AsyncWebServerRequest *request) {
    Serial.println("Endpoint /savewifi requested");
    String usrSSID, usrPassword;
    if (request->hasParam("ssid", true)) {
      usrSSID = request->getParam("ssid", true)->value();
      Serial.println("Received SSID: " + usrSSID);
    }
    if (request->hasParam("password", true)) {
      usrPassword = request->getParam("password", true)->value();
      Serial.println("Received password: " + usrPassword);
    }
    if (usrSSID.length() > 0 && usrPassword.length() > 0) {
      settings.ssid = usrSSID;
      settings.password = usrPassword;
      saveSettings();
      Serial.println("Stored credentials. Attempting STA connection...");
      startStationMode(usrSSID, usrPassword);
      if (WiFi.status() == WL_CONNECTED) {
        credentialsSaved = true;
        String staIP = WiFi.localIP().toString();
        Serial.println("STA IP Address: " + staIP);
        request->redirect("http://" + WiFi.softAPIP().toString() + "/notify.html?sta=" + staIP);
      } else {
        Serial.println("Failed to connect to STA network");
        request->send(500, "text/html", "<h2>Failed to connect to STA network</h2>");
      }
    } else {
      Serial.println("Missing SSID or Password in /savewifi");
      request->send(400, "text/html", "<h2>Missing SSID or Password</h2>");
    }
  });
  
  server.on("/finalizemain", HTTP_GET, [](AsyncWebServerRequest *request) {
    String staIP = WiFi.localIP().toString();
    Serial.println("Endpoint /finalizemain requested; current STA IP: " + staIP);
    request->send(SPIFFS, "/main.html", "text/html", false);
    xTaskCreate([](void *param) {
      vTaskDelay(1500 / portTICK_PERIOD_MS);
      finalizeAPMode();
      apFinalized = true;
      Serial.println("AP mode finalized from /finalizemain task");
      vTaskDelete(NULL);
    }, "FinalizeTask", 4096, NULL, 1, NULL);
  });
  
  server.on("/finalize", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.println("Endpoint /finalize requested");
    finalizeAPMode();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "AP mode disabled. Now in STA-only mode.");
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
  });
  
  server.onNotFound([](AsyncWebServerRequest *request) {
    Serial.println("Endpoint not found: " + request->url());
    request->redirect("/");
  });
  
  server.serveStatic("/style.css", SPIFFS, "/style.css", "max-age=86400");
  server.begin();
  Serial.println("Server started.");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started.");

  // LED initialization using the refactored RGB function.
  rgbLamp.begin();
  showRGBleds(128, 0, 0);    // Turn on RED for start-up indication.
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  showRGBleds(0, 0, 0);      // Turn LEDs off.
  
  pinMode(upLeft, INPUT_PULLUP);

  #ifdef CLEAR_CREDENTIALS
    clearCredentials();
  #endif

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }
  
  // Debug: List SPIFFS files.
  {
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
      Serial.print("File: ");
      Serial.print(file.name());
      Serial.print(" (");
      Serial.print(file.size());
      Serial.println(" bytes)");
      file = root.openNextFile();
    }
  }
  
  loadSettings();

  // Setup MQTT using settings from the structure
  mqttClient.setServer(settings.mqttServerAddress.c_str(), settings.mqttPort);
  mqttClient.setCallback(mqttCallback);

  // If WiFi credentials exist, attempt STA connection.
  if (settings.ssid.length() > 0 && settings.password.length() > 0) {
    credentialsSaved = true;
    apFinalized = true; // Assume STA-only mode if credentials exist.
    Serial.println("Credentials found: " + settings.ssid);
    startStationMode(settings.ssid, settings.password);
    if (WiFi.status() == WL_CONNECTED) {
      showRGBleds(0, 128, 0);  // Indicate success with GREEN.
      Serial.println("Starting in dual mode (STA+AP) with SSID: " + settings.ssid);
    } else {
      Serial.println("STA connection failed; remaining in AP mode.");
    }
  } else {
    Serial.println("No credentials found. Starting in Captive Portal Mode.");
    startCaptivePortal();
    showRGBleds(0, 0, 126); // Indicate AP mode with BLUE.
  }

  setupServerEndpoints();

  if (WiFi.status() == WL_CONNECTED) {
    initOTA();
  }

  Serial.println("Setup complete.");
}

void loop() {
  if (WiFi.getMode() == WIFI_AP) {
    dnsServer.processNextRequest();
  }
  if (WiFi.getMode() == WIFI_STA || WiFi.getMode() == WIFI_AP_STA) {
    ArduinoOTA.handle();
  }
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  readPins();
  
  if (upLeftReleased) {
    upLeftReleased = false;
    clearCredentials();
    Serial.println("Restarting ESP...");
    esp_restart();
  }
}