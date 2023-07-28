// SpecificGravity web & prefs functions
//
// s60sc 2023

#include "appGlobals.h"

#define PD 1 // degrees of polynomial, for this implementation, only first order is used 

static char* p;
static char wsServerHost[16]; // host address (eg ESP32_SGhost)
static int hostInterval = 10; // interval in seconds to notify Host when awake
static int LED_PIN = 5;
static int timeAsleep = 30;
static int timeAwake = 60;

// polynomial data
static const int N = 2; // no. of data-points
static double dp[N*2]; // data point pairs
// dp[0] is water tilt angle, dp[1] is SG of water, dp[2] is OG tilt angle, dp[3] is OG value
static const int pdMax = 3; // maximum degrees of polynomial
static double coEff[pdMax]; // polynomial coefficients
static double B[pdMax+1][pdMax+2]; // the normal augmented matrix

float tiltAngle = 0;
float specificGravity = 0;
float temp = 0;
static char hostURL[50];
static bool startMon = false;
static uint32_t awakeTime;
#define WAKE_PIN 0 // boot button used to force wake and reset monitoring
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds 
char SGdata[150];

static void calculateSG() {
  // get data from MPU6050
  double* Gforce = readMPU6050();
  // calculate gravity from all 3 axes to eliminate roll
  double gXYZ = sqrt(pow(Gforce[0],2)+pow(Gforce[1],2)+pow(Gforce[2],2));
  LOG_DBG("gXYZ should be close to 1, is: %0.2f", gXYZ);
  
  // axis used for pitch is whichever is linear to PETling length
  // generally this will be the X axis
  // get tilt angle (pitch) wrt to horizontal and convert to degrees
  double ratio = Gforce[0]/gXYZ;
  tiltAngle = (float)((ratio < 0.5) ? 90-fabs(asin(ratio)*RAD_TO_DEG) : fabs(acos(ratio)*RAD_TO_DEG));

  // calculate SG using polynomial
  specificGravity = coEff[0];
  for (int i=1; i<=PD; i++) specificGravity += coEff[i]*pow(tiltAngle,i);
  
  temp = Gforce[3]; // degrees celsius
}

static void generatePolynomial() {
  // Derived from https://www.bragitoff.com/2018/06/polynomial-fitting-c-program
  // Fit a polynomial curve to a given set of data points using the Least Squares Approximation Method
  // and return the polynomial coefficients
  int i,j,k;
  // read in data point pairs 
  //  N = getPairVals(angle_gravity, " ", "\n", dp); // input from data file - not used
  // an array of size 2*PD+1 for storing N, Sig xi, Sig xi^2, etc. which are the independent components of the normal matrix
  double X[2*PD+1] = {0.0};  
  dp[1] = 1.0; // SG of water
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

  LOG_INF("Polynomial coefficients: ");
  for (i=0; i<=PD; i++) logPrint("%u:%0.9f ",i, coEff[i]);
  logPrint("\n");
}

static void sendHost() {
  // periodically connect to remote host and send Http request with status data in json
  WiFiClient wclient;
  HTTPClient http;
  http.setConnectTimeout(1000);
  http.begin(wclient, hostURL); 
  http.addHeader("Content-Type", "application/json");
  http.POST(SGdata);
////  int httpCode = http.POST(SGdata);
////  if (httpCode != HTTP_CODE_OK) LOG_WRN("Host (%s) failure: %d:%s", hostURL, httpCode, 
////    http.errorToString(httpCode).c_str());
  http.end();
}

void doDeepSleep() {
  digitalWrite(LED_PIN, 0);
  esp_sleep_enable_timer_wakeup(timeAsleep * 60 * uS_TO_S_FACTOR); // in minutes
  sleepMPU6050();
  goToSleep(WAKE_PIN, true);
}

void SGsetup() {
  if (wakeupResetReason() == ESP_SLEEP_WAKEUP_EXT0) {
    updateStatus("startMon", "0"); // boot button pressed
    updateStatus("save", "1");
  }
  prepPeripherals();
  startI2C();
  if (!checkMPU6050()) LOG_WRN("MPU6050 not available");
  // setup battery monitoring 
  float voltage = readVoltage();
  LOG_INF("Battery voltage: %0.1f", voltage);
  if (voltage < voltLow && voltLow > 0) {
     // battery voltage too low, shut down to avoid over discharge
    LOG_WRN("Shut down as battery voltage too low: %dV", voltage);
    delay(2000); 
    doDeepSleep();
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 0);
  
  // deep sleep setup
  awakeTime = millis();

  snprintf(hostURL, 50, "http://%s/update", wsServerHost); 
}

void SGloop() {
  static bool blinking = true;
  static int sendCntr = 0;
  digitalWrite(LED_PIN, blinking);
  blinking = !blinking;
  delay(1000);
  float voltage = readVoltage();
  calculateSG();
  // build json string to send to remote host and local web server
  snprintf(SGdata, sizeof(SGdata)-1, "{\"tilt\":\"%0.1f\",\"SG\":\"%0.4f\",\"temp\":\"%0.1f\",\"batt\":\"%0.2f\",\"water\":\"%0.1f\",\"getTime\":\"1\",\"sg_rssi\":\"%d dbm\"}", tiltAngle, specificGravity, temp, voltage, dp[0], WiFi.RSSI());
  // check if time to sleep
  if (startMon && (millis()-awakeTime)/1000 > timeAwake) doDeepSleep();

  // send data to host at given interval
  if (++sendCntr > hostInterval) {
    sendCntr = 0;
    sendHost();
  }
}

void displayValuesOled(int inDispIndex, bool dispChanged) {}

bool externalPeripheral(byte pinNum, uint32_t outputData) {
  return false;
}

void prepUart() {}


/************************ webServer callbacks *************************/

bool updateAppStatus(const char* variable, const char* value) {
  // update vars from browser input
  bool res = true; 
  int intVal = atoi(value);
  float fltVal = atof(value);
  if (!strcmp(variable, "custom")) return res;
  else if (!strcmp(variable, "waterAngle")) dp[0] = fltVal; // water tilt angle
  else if (!strcmp(variable, "OGval")) dp[3] = fltVal; // OG input value
  else if (!strcmp(variable, "OGangle")) {
    // generate polynomial coefficients for tilt angle / specific gravity correlation
    dp[2] = fltVal; // OG tilt angle
    generatePolynomial(); 
  }
  else if (!strcmp(variable, "startMon")) startMon = bool(intVal); // start monitoring
  else if (!strcmp(variable, "voltUse")) voltUse = (bool)intVal;
  else if (!strcmp(variable, "voltPin")) voltPin = intVal;
  else if (!strcmp(variable, "voltDivider")) voltDivider = intVal;
  else if (!strcmp(variable, "voltLow")) voltLow = fltVal;
  else if (!strcmp(variable, "voltInterval")) voltInterval = intVal;
  else if (!strcmp(variable, "hostInterval")) hostInterval = intVal;
  else if (!strcmp(variable, "wsServerHost")) strcpy(wsServerHost, value);
  else if (!strcmp(variable, "I2C_SDA")) I2C_SDA = intVal;
  else if (!strcmp(variable, "I2C_SCL")) I2C_SCL = intVal;
  else if (!strcmp(variable, "MPU6050addr")) MPU6050addr = strtoul(value, NULL, 16);
  else if (!strcmp(variable, "timeAwake")) timeAwake = intVal;
  else if (!strcmp(variable, "timeAsleep")) timeAsleep = intVal;
  else if (!strcmp(variable, "LED_PIN")) LED_PIN = intVal;
  return res;
}

void wsAppSpecificHandler(const char* wsMsg) {
  // message from web socket
  int wsLen = strlen(wsMsg) - 1;
  switch ((char)wsMsg[0]) {
    case 'H': 
      // keepalive heartbeat, return status
    break;
    case 'S': 
      // status request
      buildJsonString(wsLen); // required config number 
      logPrint("%s\n", jsonBuff);
    break;   
    case 'U': 
      // update or control request
      memcpy(jsonBuff, wsMsg + 1, wsLen); // remove 'U'
      parseJson(wsLen);
    break;
    case 'K': 
      // kill websocket connection
      killWebSocket();
    break;
    default:
      LOG_WRN("unknown command %c", (char)wsMsg[0]);
    break;
  }
}

void buildAppJsonString(bool filter) {
  // build app specific part of json string
  p = jsonBuff + 1;
  p += sprintf(p, "\"startMon\":\"%s\",", startMon ? "1" : "0");
  // output SG monitoring data 
  p += snprintf(p, strlen(SGdata), "%s", SGdata+1);
  *(--p) = ',';
}

esp_err_t webAppSpecificHandler(httpd_req_t *req, const char* variable, const char* value) {
  return ESP_OK;
}

bool appDataFiles() {
  // callback from setupAssist.cpp, for any app specific files 
  return true;
}

void doAppPing() {}

void OTAprereq() {}
