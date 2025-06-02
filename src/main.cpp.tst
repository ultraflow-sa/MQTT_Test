#include <DNSServer.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"

DNSServer dnsServer;
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("esp-captive");
  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP: "); Serial.println(apIP);

  dnsServer.start(53, "*", apIP);

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(200, "text/html", "<h1>Captive Portal</h1><p>You are connected!</p>");
  });
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
}