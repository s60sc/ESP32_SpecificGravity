
// Calculates fermenting wort Specific Gravity from tilt angle of floating sensor,
// where tilt angle varies with wort density.
// Tilt Angle is calculated assuming device only has rotational motion.
// Specific Gravity is calculated from the tilt angle with reference to a nth order polynomial 
// derived from a table of user supplied readings.
// 
// Uses the hardware design approach from http://www.ispindel.de/docs/README_en.html
// but simplified to use an ESP32 with an integral 18650 battery holder and battery management IC TPS5400
// and obtain G force readings and temperature from MCU6050 accelerometer on GY-521 module.
// The code base has been independently developed.
//
// The generated data is packaged into a json string, this data can be viewed:
// - whilst the ESP32 is awake by using its own web server accessed from a browser using the 'ipaddr' address below
// - on a remote hub listening on the address given by 'wsServerHost' below, which could be another ESP32.
// 
// On power up, ESP32 will remain awake for calibration purposes until user presses
// the Start button on web page.
// The ESP32 will then periodically wakeup to collect data and send this to the remote hub
// then return to deep sleep after 'TIME_AWAKE' for the period of time in 'TIME_TO_SLEEP'
// 
// The hardware module shown in the images fits snugly in a 33mm width * 120mm height PETling.
// Helps to move the on/off switch from a horizontal to vertical position.
//
// To calibrate device before first use:
// - Open ESP32 web page to get readings with periodic refresh
// - Balance device in PETling so reads c. 25 deg angle in plain water @ 20C
// - Use hydrometer to measure SG for different sugar solutions to above your maximum wort original gravity
// - Record each device tilt angle and hydrometer reading in 'angle_gravity' in 'SGdata.h'.
// More data points gives improved accuracy, order does not need to be sorted
// For subsequent use, adjust position of device in PETling so that it reads the same 
// tilt angle in plain water @ 20C as originally calibrated. 
//
// Connections:
// MPU6050  ESP32
// ADO      GND
// SDA‎ ‎     GPIO 21
// SCL‎      ‎GPIO 22
// VCC‎      3V3
// GND      GND
//
// Check accelerometer works properly (can be faulty), by checking that 
// gXYZ is close to 1g, and gX varies between to 0g (horizontal) to +/- 1g (vertical)
//
// The battery voltage to be measured needs to go thru a voltage divider comprising 
// two equal high value resistors to keep voltage below 3V3 at the ADC pin.
//
// s60sc 2020

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "esp_adc_cal.h"
#include <regex>
#include <Update.h>
#include <SPIFFS.h>
#include "SGdata.h"

// user definable constants
static const char* ssid = "****";
static const char* password = "****";
static const IPAddress ipaddr(192, 168, 1, 126); // device static IP address
static const IPAddress gateway(192, 168, 1, 1); // router
static const IPAddress subnet(255, 255, 255, 0);
static const IPAddress DNS(192, 168, 1, 1); // router
static const char* wsServerHost = "192.168.1.123"; // hub address
static AsyncWebServer server(80); 
static AsyncWebSocket ws("/ws"); // used by ota
static AsyncEventSource events("/events"); // used by ota
static size_t content_len;

#define LED_PIN 5 // blink led, actual pin depends on ESP32 module used
#define BATT_PIN ADC1_GPIO34_CHANNEL // ADC pin for monitoring battery voltage
#define PD 3 // degrees of polynomial, 3 is good
#define TIME_ASLEEP (60*30)  // Time ESP32 will be in deep sleep (in seconds) 
#define TIME_AWAKE 30 // Time ESP32 will be awake (in seconds) between sleeps
#define NO_OF_SAMPLES 16 // ADC multisampling

extern const char* angle_gravity;
extern const char* index_html;

// polynomial data
static int N = 0; // no. of data-points
static const int pdMax = 6; // maximum degrees of polynomial
static double coEff[pdMax]; // polynomial coefficients
static double B[pdMax+1][pdMax+2]; // the normal augmented matrix

static char hubURL[30];
static char jsonMessage[100];
static bool stayAwake = true;
static uint32_t awakeTime;
static esp_adc_cal_characteristics_t *adc_chars; // holds ADC characteristics
static const adc_atten_t ADCatten = ADC_ATTEN_DB_11; // attenuation level
static const adc_unit_t ADCunit = ADC_UNIT_1; // using ADC1
static const adc_bits_width_t ADCbits = ADC_WIDTH_BIT_11; // ADC bit resolution
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds 
#define DEFAULT_VREF 1100 // if eFuse or two point not available on old ESPs

// MPU6050 definitions
#define MPU6050_ADDR 0x68 // MPU6050 I2C address if AD0 grounded, 0x69 if pulled high
#define SENS_2G (32768.0/2.0) // divider for 2G sensitivity reading
#define ACCEL_BYTES 6 // 2 bytes per axis
#define CONFIG 0x1A
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B
#define NO_VAL 0xFF

byte writeMPU6050(byte regAddr, byte regVal, bool busRelease = true){
  Wire.beginTransmission(MPU6050_ADDR);
  if (regAddr != NO_VAL) Wire.write(regAddr);
  if (regVal != NO_VAL) Wire.write(regVal);
  return Wire.endTransmission(busRelease);
}

bool setupMPU6050() {
  // check MPU6050 is available
  Wire.begin(); 
  if (byte err = writeMPU6050(NO_VAL, NO_VAL)) {
    printf("MPU6050 not found at %#04x with error %u\n", MPU6050_ADDR, err);
    return false;
  } else {  
    writeMPU6050(CONFIG, 0x00); // set full range
    writeMPU6050(PWR_MGMT_1, 0x01); // wakeup the sensor
    return true;
  }
}

void calculateSG() {
  // get data from MPU6050
  writeMPU6050(ACCEL_XOUT_H, NO_VAL, false); 
  Wire.requestFrom(MPU6050_ADDR, ACCEL_BYTES+2); // read 3 axis accelerometer & temperature
  int16_t rawX = Wire.read() << 8 | Wire.read();
  int16_t rawY = Wire.read() << 8 | Wire.read();
  int16_t rawZ = Wire.read() << 8 | Wire.read();
  int16_t rawTemp = Wire.read() << 8 | Wire.read();
  // each axis G force value, straight down is 1.0 if stationary
  double gX = (double)(rawX / SENS_2G); 
  double gY = (double)(rawY / SENS_2G);
  double gZ = (double)(rawZ / SENS_2G);
  printf("gX : %0.2f\n", gX); // pitch
  // calculate gravity from all 3 axes to eliminate roll
  double gXYZ = sqrt(pow(gX,2)+pow(gY,2)+pow(gZ,2));
  printf("gXYZ : %0.2f\n", gXYZ);

  // axis used for pitch is whichever is linear to PETling length
  // generally this will be the X axis
  // get tilt angle (pitch) wrt to horizontal and convert to degrees
  double ratio = gX/gXYZ;
  float tiltAngle = (float)((ratio < 0.5) ? 90-fabs(asin(ratio)*RAD_TO_DEG) : fabs(acos(ratio)*RAD_TO_DEG));
  printf("tiltAngle: %0.1f\n", tiltAngle); 

  // calculate SG using polynomial
  float specificGravity = coEff[0];
  for (int i=1; i<=PD; i++) specificGravity += coEff[i]*pow(tiltAngle,i);
  printf("specificGravity : %0.4f\n", specificGravity);
  
  float temp = (rawTemp / 340.0) + 36.53; // degrees celsius
  printf("Temp : %0.1fC\n", temp);

  // build json string to send to remote hub and local web server
  sprintf(jsonMessage, "{\"1\":\"%0.1f\",\"2\":\"%0.4f\",\"3\":\"%0.1f\",", tiltAngle, specificGravity, temp);
}

float battVoltage() {
  // get multiple readings of battery voltage from ADC pin and average
  // input battery voltage is halved by resistor divider to keep it below 3V3.
  uint32_t ADCsample = 0;
  for (int j = 0; j < NO_OF_SAMPLES; j++) ADCsample += adc1_get_raw(BATT_PIN); 
  ADCsample /= NO_OF_SAMPLES;
  // convert ADC averaged pin value to curve adjusted voltage in mV
  if (ADCsample > 0) ADCsample = esp_adc_cal_raw_to_voltage(ADCsample, adc_chars);
  return (float)ADCsample/500.0; // as input voltage was halved
}

size_t extractPairVals(const char* pairValStr, const char* itemSep, const char* pairSep, std::vector<double> &pairVals) {
  // populate numeric array from paired numeric values in string 
  char* pairStart = strdup(pairValStr); 
  char* pairEnd = strchr(pairStart, pairSep[0]); 
  int i = 0;
  while (pairEnd != NULL) {
    *pairEnd++ = '\0'; // replace pair separator with string terminator
    if (strlen(pairStart)) { 
      char* pairItem = strtok(pairStart, itemSep); // get pair
      while (pairItem != NULL) {
        pairVals.push_back (atof(pairItem));
        pairItem = strtok(NULL, itemSep); // next item in pair
      }
      i++;
    }
    pairStart = pairEnd; // point to start of next pair
    pairEnd = strchr(pairStart, pairSep[0]); // point to end of next pair
  }
  return i; // number of pairs
}

static void start_handler(AsyncWebServerRequest *request) {
  puts("User requested deep sleep");
  request->send(200, "text/html", " ");
  delay(1000);
  esp_deep_sleep_start();
}

static void reset_handler(AsyncWebServerRequest *request) {
  puts("User requested reset");
  request->send(200, "text/html", " ");
  delay(1000);
  ESP.restart();
}

static void update_handler(AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index){
    puts("OTA Update");
    request->redirect("/");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("spiffs") > -1) ? U_SPIFFS : U_FLASH;
    char type[30] = "Update Start - ";
    if (cmd == U_FLASH) strcat(type,"Sketch");
    else {
      strcat(type,"Files");
      SPIFFS.end();
    }
    events.send(type, "ota");
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, cmd)) Update.printError(Serial);
  }
  if (Update.write(data, len) != len) {
    Update.printError(Serial);
    Serial.printf("Progress: %d%%\n", (Update.progress()*100)/Update.size());
  }

  if (final) {
    events.send("Update End", "ota");
    if (!Update.end(true)) Update.printError(Serial);
    else {
      puts("OTA Update complete");
      delay(1000);
      ESP.restart();
    }
  }
}

static void printProgress(size_t prg, size_t sz) {
  static int oldProg = 0;
  char p[32];
  int currProg = prg*100/(content_len*5); // display each 5%
  if (currProg > oldProg) {
    sprintf(p, "Progress: %u%%\n", currProg*5);
    events.send(p, "ota");
    oldProg = currProg;   
  }
}

void startWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {request->send(200, "text/html", index_html);});
  server.on("/refresh", HTTP_GET, [](AsyncWebServerRequest* request) {request->send(200, "application/json", jsonMessage);});
  server.on("/start", HTTP_GET, start_handler);
  server.on("/reset", HTTP_GET, reset_handler); 
  server.on("/update", HTTP_POST, [](AsyncWebServerRequest* request) {}, update_handler);
  server.on("/ota",  HTTP_GET, [](AsyncWebServerRequest* request) {request->send(200, "text/html", otaStatus);}); // OTA monitoring page
  server.on("/doOta", HTTP_POST, [](AsyncWebServerRequest* request) {}, update_handler);
  server.onNotFound([](AsyncWebServerRequest* request) {request->send(404, "text/plain", "Not found: " + request->url());}); 

  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*"); // prevent CORS error
  server.addHandler(&ws); // web socket url for ota
  server.addHandler(&events); // web socket events for ota
  events.onConnect([](AsyncEventSourceClient *client){});
  server.begin();
  Update.onProgress(printProgress);
}

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : puts("Wakeup by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : puts("Wakeup by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : 
      // expected wakeup reason from deep sleep
      puts("Wakeup by internal timer"); 
      stayAwake = false;
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : puts("Wakeup by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : puts("Wakeup by ULP program"); break;
    case ESP_SLEEP_WAKEUP_GPIO: puts("Wakeup by GPIO"); break;    
    case ESP_SLEEP_WAKEUP_UART: puts("Wakeup by UART"); break; 
    default : puts("Wakeup by reset"); break;
  }
}

void sendHubTask(void *pvParameter) {
  // periodically connect to remote hub and send Http request with status data in json
  delay(5000);
  while (true) {
    WiFiClient wclient;
    HTTPClient http;
    http.begin(wclient, hubURL); 
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(jsonMessage);
    if (httpCode != HTTP_CODE_OK) printf("Hub (%s) failure: %d:%s, sent %s", hubURL, httpCode, 
      http.errorToString(httpCode).c_str(), jsonMessage);
    http.end();
    delay(5000);
  }
  vTaskDelete(NULL);
}

void generatePolynomial() {
  // Derived from https://www.bragitoff.com/2018/06/polynomial-fitting-c-program
  // Fit a polynomial curve to a given set of data points using the Least Squares Approximation Method
  // and return the polynomial coefficients
  int i,j,k;
  // read in data point pairs
  static std::vector<double> dp;
  N = extractPairVals(angle_gravity, " ", "\n", dp);
  // an array of size 2*PD+1 for storing N, Sig xi, Sig xi^2, etc. which are the independent components of the normal matrix
  double X[2*PD+1] = {0.0};  
  for (i=0; i<=2*PD; i++) for (j=0; j<N; j++) X[i] += pow(dp[j*2],i);

  // rhs
  double Y[PD+1] = {0.0};      
  for (i=0; i<=PD; i++) for (j=0; j<N; j++) Y[i] += pow(dp[j*2],i)*dp[(j*2)+1];
  for (i=0; i<=PD; i++) for (j=0; j<=PD; j++) B[i][j] = X[i+j]; 
  for (i=0; i<=PD; i++) B[i][PD+1] = Y[i];
  
  // gauss elimination 
  for (i=0; i<PD; i++) {
    // Partial Pivoting
    for (k=i+1; k<PD+1; k++) {
      // If diagonal element(absolute value) is smaller than any of the terms below it
      if (fabs(B[i][i]) < fabs(B[k][i])) {
        // Swap the rows
        for (j=0; j<PD+2; j++) {                
          double temp = B[i][j];
          B[i][j] = B[k][j];
          B[k][j] = temp;
        }
      }
    }
    for (k=i+1; k<PD+1; k++) {
      double term = B[k][i]/B[i][i];
      for(j=0; j<PD+2; j++) B[k][j] = B[k][j]-term*B[i][j];
    }     
  }
  
  // Back-substitution
  for (i=PD; i>=0; i--) {
    coEff[i] = B[i][PD+1];
    for (j=i+1; j<PD+1; j++) coEff[i] -= B[i][j]*coEff[j];
    coEff[i] /= B[i][i];
  } 
    
  printf("\nPolynomial coefficients: ");
  for (i=0; i<=PD; i++) printf("%u:%0.9f ",i, coEff[i]);
  puts("");
}

void setup() {
  // Characterise ADC to generate voltage curve for battery monitoring
  adc1_config_width(ADCbits);
  adc1_config_channel_atten(BATT_PIN, ADCatten);
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADCunit, ADCatten, ADCbits, DEFAULT_VREF, adc_chars);
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) puts("ADC characterised using eFuse Two Point Value");
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) puts("ADC characterised using eFuse Vref");
  else puts("ADC characterised using Default Vref");

  float voltage = battVoltage();
  printf("Battery voltage: %0.1f\n", voltage);
  if (voltage < 3.0) {
    // battery voltage too low, shut down to avoid over discharge
    writeMPU6050(PWR_MGMT_1, 0x40); // PWR_MGMT_1 register set to sleep
    delay(500);
    esp_deep_sleep_start();
  }

  pinMode(LED_PIN, OUTPUT);
  setupMPU6050();
  // deep sleep setup
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(TIME_ASLEEP * uS_TO_S_FACTOR);
  awakeTime = millis();

  // wifi connection for web
  WiFi.config(ipaddr, gateway, subnet, DNS);
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    printf(".");
  }
  printf("\nConnected to WiFi network with web server IP: %s\n", WiFi.localIP().toString().c_str());

  // generate polynomial coefficients for tile angle / specific gravity correlation
  generatePolynomial();
  
  // start task to send data to remote hub
  snprintf(hubURL, 30, "http://%s/appinput", wsServerHost); //  hub url
  xTaskCreate(&sendHubTask, "sendHubTaskTask", 2048, NULL, 2, NULL);
  startWebServer();
}

void loop() {
  static bool blinking = true;
  digitalWrite(LED_PIN, blinking);
  blinking = !blinking;
  delay(1000);
  calculateSG();
  float voltage = battVoltage();
  printf("Batt voltage : %0.1f\n", voltage);
  // finalise json string
  sprintf(jsonMessage+strlen(jsonMessage), "\"4\":\"%0.2f\"}", voltage);
  if (!stayAwake && (millis()-awakeTime)/1000 > TIME_AWAKE) {
    // time to sleep
    writeMPU6050(PWR_MGMT_1, 0x40); // PWR_MGMT_1 register set to sleep
    puts("Scheduled deep sleep");
    delay(1000);
    esp_deep_sleep_start();
  }
  puts("");
}
