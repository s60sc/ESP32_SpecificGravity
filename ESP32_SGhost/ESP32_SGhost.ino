
// Simple example of ESP32/Arduino host to receive updates 
// from ESP32_SpecificGravity (SG) app and display on web page
// In ESP32_SpecificGravity app, under Edit Config / Wifi tab, set Remote Client value to be this app's IP address

// s60sc 2021, 2026

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include "SGhostData.h"

WebServer server(80);
String json = "";

// user definable constants
static const char* ssid = ""; // user router ssid
static const char* password = ""; // user router password
static const IPAddress ipaddr(192, 168, 1, 122); // this app static IP address, required for ESP32_SpecificGravity app access
static const IPAddress gateway(192, 168, 1, 1); // router
static const IPAddress subnet(255, 255, 255, 0);
static const IPAddress DNS(192, 168, 1, 1); // router

#define LED_PIN 2 // blink led, actual pin depends on ESP32 module used

static void processSGappInput() {
  // http://192.168.1.122/update?data=%7B%22tilt%22%3A%2248.5%22%2C%22SG%22%3A%221.024%22%2C%22temp%22%3A%220.2%22%2C%22batt%22%3A%224.1%22%2C%22getTime%22%3A%2213:10:41%22%7D
  json = server.arg("plain");  // raw JSON text
  server.send(200); 
}

static void notFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "Not found: " + server.uri());
}

static void startWebServer() {
  server.on("/", []() {server.send(200, "text/html", index_html);});
  server.on("/refresh", []() {server.send(200, "application/json", json);}); // update for browser
  server.onNotFound(notFound);
  server.on("/update", processSGappInput); // update from SG app
  server.begin();
}

void setup() {
  Serial.begin(115200);
  // wifi connection for web
  WiFi.config(ipaddr, gateway, subnet, DNS);
  Serial.printf("Connect to %s\n", ssid);
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    printf(".");
  }
  puts("");
  startWebServer();
  printf("HTTP server on WiFi network with IP: %s\n", WiFi.localIP().toString().c_str());
  puts("");
}

void loop() {
  server.handleClient();
  static bool blinking = true;
  static uint32_t checkTime = millis();
  if (millis() - checkTime > 1000) {
    // blink once a second
    digitalWrite(LED_PIN, blinking);
    blinking = !blinking;
    checkTime = millis();
  } else delay(2);
}
