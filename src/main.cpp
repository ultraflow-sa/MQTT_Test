#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <FS.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
//#include "mbedtls/version.h"

// ---------- Assumed Definitions for Subs Functions ----------
const int upLeft = 5;  // (adjust to your actual pin)
bool upLeftPressed = false;
bool upLeftLongPressed = false;
bool upLeftReleased = false;
unsigned long upLeftStartTime = 0;
const unsigned long updnLongPress = 1000; // e.g., the long press threshold in ms
const unsigned long debounce = 200;       // e.g., debounce time in ms
unsigned long lastMQTTReconnectAttempt = 0;

// ---------- ORIGINAL readPins() from subs.h ----------
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
}

// --------------- Global Settings and Declarations ---------------
struct Settings {
  String ssid;
  String password;
  String htmlVersion;
  String cssVersion;
  
  String mqttServerAddress;
  int    mqttPort;
  String updateTopic;
  
  // Base URL for webupdates (if used)
  String baseUpdateUrl;
};

#define SETTINGS_FILE "/settings.json"
Settings settings;  // Global settings instance

// Global device identity (change defaults as needed)
String serialNumber = "000001";
String VER_STRING = "v1.0.0";

// ------------------ TLS Setup ------------------
// Replace the following with your full Amazon Root CA certificate.
// Download it from: https://www.amazontrust.com/repository/AmazonRootCA1.pem
const char AWS_ROOT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

const char DEVICE_CERT[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDWTCCAkGgAwIBAgIUc5UkSTk6sbT2oudq0KcvwYRSSd4wDQYJKoZIhvcNAQEL
BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g
SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTI1MDYwMzA3Mzkz
OFoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0
ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALYAdoAOcvsBrsBN3MZ+
BMfCrEww08P4ZPa5djCvu7dfFb97Y0cPHIYkoHnIiWfeyxblVb3I8WiX0DOv2zQA
4rEgQ0WHBq638utxzmIy1xLJwkL0jBIcvSmLI+lPXGKSi6QAZ45hHy3TRT/2xsk1
m/PL/MDizQeVt8MwvYM3rTCSz2Xh01lND882FiPMcO8ZOuLFUdHjTbL2P+oQA3uF
hZW6xicYjL+qhmQK36k6AzyL05T3f8oah/imbHBASwQu4HZusrZMWVlb3Zb5ooKL
xOWahC41gIxQ8PScUmv8sFVzPV4dOUN3CWLCERS5FVHcJ2E0q7ciXcT4T0IFHMqS
V+cCAwEAAaNgMF4wHwYDVR0jBBgwFoAUP8pwcWKuHjIwvQo8xi1bMiyJlBowHQYD
VR0OBBYEFPfu9KTMTXYZO4ilozviXQy0zspJMAwGA1UdEwEB/wQCMAAwDgYDVR0P
AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAYKQNf2W2CP5iNn8+en7lq6LZ7
vtycxSVGziHtC9F2BOSVaDFg8ma9mb6u977c5fbnEMGWMGPyNwJID06W5Rij8DZj
lm664Lw0iXKCKsMu0otAZTjZbxwY5tE3WuaZv49HLnnWL4+v8U7qs4bGZs490M4A
ukWMVHba3OkT2vnmzLc0XeLykqnpU1LPsreRlb+k/TCwa1zS/LTRVFE1cYF3EelC
S7ZtWkkVZcGo9y9d8O3yZYi4RhaY8aii7mAAUNHGLsrli6Bl2Kpdn3LQrfe2rqhp
yD7jR/ys/cr++nwxu4yHnnEeRzovlY/bzh2BFCAttMRBaNKWz+i5O7GfKfis
-----END CERTIFICATE-----
)EOF";

const char DEVICE_PVT_KEY[] PROGMEM = R"EOF(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEAtgB2gA5y+wGuwE3cxn4Ex8KsTDDTw/hk9rl2MK+7t18Vv3tj
Rw8chiSgeciJZ97LFuVVvcjxaJfQM6/bNADisSBDRYcGrrfy63HOYjLXEsnCQvSM
Ehy9KYsj6U9cYpKLpABnjmEfLdNFP/bGyTWb88v8wOLNB5W3wzC9gzetMJLPZeHT
WU0PzzYWI8xw7xk64sVR0eNNsvY/6hADe4WFlbrGJxiMv6qGZArfqToDPIvTlPd/
yhqH+KZscEBLBC7gdm6ytkxZWVvdlvmigovE5ZqELjWAjFDw9JxSa/ywVXM9Xh05
Q3cJYsIRFLkVUdwnYTSrtyJdxPhPQgUcypJX5wIDAQABAoIBAGJ5taRsLQJrUqZy
erZbkTKUvq0q7inmyJpGlxCYxTTemeHVXU4hewmJ39qFCvPMtI5a4B2kEBrLqbeN
u0lUAVRdZIjGGnOGzEVgeo7fe0eLKzUXJILYUfGce33Nlusu56eBIIcFPd2JprqJ
R92uyAcNpGCpVs53z8opISFSir7who009W9F1/CNr/S3doeQL/cwcC+X2gBRzl/L
ecsor39y4rAmpTvplEukTrmPNl+SXOhE0jaK7c/Oq4t+Fwwn+Yjq1ffde7MJ1UVE
/RkJ8elK54s1DI3wL84poQJgHVo72sd/ovbQyDnP59XjpDWVSqxhh6SfvE4lMTk9
eEicjoECgYEA5Tr2CgIZiQ3V5+5wGveG5j+y/SE354IF6SeZjAGABrF5NmCaPlAB
N3HHPvZgPib3/KFcYty3LdjjwkmjwXxeWvhMfMrCdfZUv9mh3CVf+Wdx2jVwvbFj
7SVaNbifIu8D31N0lqu0xEk8HEVRIEdZttj/XpQAsO/hactUCUVjI3ECgYEAy0GQ
iwVlbr2jhPfia4RNLBGGifCw39nuljiWc53tZH8cCMAo2Pj0Poj42LUs/uCNinb1
KhI6SRhsCVg9gvWHEtAzfMuL04AYs02gwqcplDKb7/SxTKM3MFnNez7zoZj2ThsZ
/aezfdNSI1GtP5C4yQ8JSqrGLyS5Gz68gDNW1NcCgYEAtJvEBxaob+fKxbaD9XtZ
ekhuCisGFQ3JAm8E996mtJ9YVZO/c6EtlDW9Osp8AfxKH24zkGE+oozkxsumjmgj
H6HLAffvR2oSVCZw9TctayAqADdi4NLHXK21aeZQ3AFeF7N1hNE81/qrtcqXpMsS
0JIggrAx6zbVb7mYOMXG2sECgYBuk77VQxxJwPXi8xDFOXbakLZG+SDgx6lbJHIF
eFecIuLXh+MxF0+Zbd8j1n0iNHwMZmc+eigneBfehZkBVB4mCAray8nISJonggYB
n/uo2lenldGXRfxSW6jEch0NPzBQkcH0E3Y87+hxuo+sU0mUaI+/hWTIZQwnXFwk
yslkhQKBgGq/tfcCh/eXa8/jW3lWe9s7UqkbajcZ3ZYbVOE6H5bHa3Oq1bXgfDyQ
hnQytFdBapyhDpQgVjaHh4bfCEd5hdhM/AeD7NSriIeWhr6KBkgnWun0ToB6NxF+
s+Z4xNtAdF0b14rK3rrRJARS3iRMKA0BcaG+apqnzvTivLVzDR6i
-----END RSA PRIVATE KEY-----
)EOF";


// Use WiFiClientSecure for TLS connections.
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
  settings.mqttServerAddress = "a21m848drovnra-ats.iot.eu-north-1.amazonaws.com";
  settings.mqttPort          = 8883;
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
  settings.mqttServerAddress = doc["mqttServerAddress"] | "a21m848drovnra-ats.iot.eu-north-1.amazonaws.com";
  settings.mqttPort          = doc["mqttPort"]          | 8883;
  settings.updateTopic       = doc["updateTopic"]       | "a3/updates";
  settings.baseUpdateUrl     = doc["baseUpdateUrl"]     | "http://mydomain/webupdates";
  
  // Print loaded settings for debugging.
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

// Generic function to publish an MQTT message.
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

// Generic function to subscribe to an MQTT topic.
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
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" connected.");
      subscribeMQTTTopic("a3/" + serialNumber + "/update");
      subscribeMQTTTopic("a3/" + serialNumber + "/querySerial");
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
  else if (String(topic) == "a3/" + serialNumber + "/querySerial") {
    Serial.println("Received serial number query via MQTT.");
    DynamicJsonDocument responseDoc(256);
    responseDoc["serial"] = serialNumber;
    String responsePayload;
    serializeJson(responseDoc, responsePayload);
    sendMQTTMessage("a3/" + serialNumber + "/serialResponse", responsePayload);
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

//  Serial.print("mbedTLS version: ");
//  Serial.println(MBEDTLS_VERSION_STRING);

  wifiClient.setCACert(AWS_ROOT_CA);
  wifiClient.setCertificate(DEVICE_CERT);
  wifiClient.setPrivateKey(DEVICE_PVT_KEY);  

  Serial.println("MQTT Server: " + settings.mqttServerAddress);
  mqttClient.setServer(settings.mqttServerAddress.c_str(), settings.mqttPort);
  mqttClient.setCallback(mqttCallback);
  setupServerEndpoints();
  
  // Configure pin for readPins() (using upLeft as defined)
  pinMode(upLeft, INPUT_PULLUP);
  
  publishState();
}

unsigned long lastStatePublish = 0;

void loop() {
  readPins();
  if (upLeftReleased == true){
    upLeftReleased = false;
    Serial.println("Up/Left button released");
    loadDefaults(); // Load defaults
    saveSettings(); // Save settings
    esp_restart(); // Restart the device to apply changes
  }

  if (millis() - lastStatePublish > 5000) {
    checkMQTTConnection();
    mqttClient.loop();
    lastStatePublish = millis();
  }
}