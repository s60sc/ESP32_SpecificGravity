
// Simple example of ESP32/Arduino host to receive updates 
// from ESP32_SpecificGravity and display on web page

// s60sc 2021

#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "time.h"
#include "SGhostData.h"

// user definable constants
static const char* ssid = "";
static const char* password = "";
static const IPAddress ipaddr(192, 168, 1, 127); // device static IP address
static const IPAddress gateway(192, 168, 1, 1); // router
static const IPAddress subnet(255, 255, 255, 0);
static const IPAddress DNS(192, 168, 1, 1); // router
static char timezone[64] = "GMT0BST,M3.5.0/01,M10.5.0/02"; // UK

static AsyncWebServer server(80); 

#define LED_PIN 5 // blink led, actual pin depends on ESP32 module used
#define JSONLEN 100
static char jsonMessage[JSONLEN+1];

static inline time_t getEpoch() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec;
}

// return formatted current time
static void getTOD(char* timestring, size_t strSize) {
  // get formatted current date and time
  char timeFormat[] = "%Y-%m-%d %H:%M:%S"; 
  time_t currEpoch = getEpoch();
  strftime(timestring, strSize, timeFormat, localtime(&currEpoch));
}

static void getLocalNTP() {
  // get current time from NTP server and apply to ESP32
  const char* ntpServer = "pool.ntp.org";
  int i = 0;
  do {
    configTzTime(timezone, ntpServer);
    delay(1000);
  } while (getEpoch() < 10000 && i++ < 5); // try up to 5 times
  if (getEpoch() > 10000) {
    char timeFormat[20];
    getTOD(timeFormat, sizeof(timeFormat));
    printf("Got current time from NTP: %s\n", timeFormat);
  }
  else puts("Unable to sync with NTP");
}

static void processNodeInputs(AsyncWebServerRequest *request, JsonVariant json) {
  char timeString[30];
  getTOD(timeString, sizeof(timeString));
  
  int jsonPtr = snprintf(jsonMessage, JSONLEN, "{\"0\":\"%s\"", timeString);
  // key / val pair is item index into array / new value
  JsonObject jsonObj = json.as<JsonObject>();
  for (JsonPair kv : jsonObj) {
    // iterate thru each json pair
    jsonPtr += snprintf(jsonMessage + jsonPtr, JSONLEN - jsonPtr, ",\"%s\":\"%s\"", kv.key().c_str(), kv.value().as<const char*>());
    snprintf(jsonMessage + jsonPtr, JSONLEN - jsonPtr, "}");
  }
  request->send(200); 
}
static void startWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {request->send(200, "text/html", index_html);});
  server.on("/refresh", HTTP_GET, [](AsyncWebServerRequest* request) {request->send(200, "application/json", jsonMessage);});
  server.onNotFound([](AsyncWebServerRequest* request) {request->send(404, "text/plain", "Not found: " + request->url());}); 
  AsyncCallbackJsonWebHandler* handleAppInput = new AsyncCallbackJsonWebHandler("/appinput", processNodeInputs);
  server.addHandler(handleAppInput);
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*"); // prevent CORS error
  server.begin();
}

void setup() {
  Serial.begin(115200);
  // wifi connection for web
  WiFi.config(ipaddr, gateway, subnet, DNS);
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    printf(".");
  }
  puts("");
  startWebServer();
  printf("Connected to WiFi network with web server IP: %s\n", WiFi.localIP().toString().c_str());
  getLocalNTP(); // get time from NTP
  puts("");
}

void loop() {
  static bool blinking = true;
  digitalWrite(LED_PIN, blinking);
  blinking = !blinking;
  delay(1000);
}
