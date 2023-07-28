
// additional code used for my personal copy of published apps
// NOT to be stored on github
// Supplies my specific wifi SSIDs and passwords
//
// Camera uses timers 0 & 1
// mjpeg2sd uses timer 3 for frame rate
// Timer 2 used for polling timer
//
// my SMTP PWD: "vdvwcwfttbuovtvh" (need to regen if account password changed)
// my FTP PWD: "a1zYetanother1"
// my default AP Password: "1234567890"  (WPA_PSK passwords must be minimum 8 chars)
//
// To obtain local CA certificate, open powershell, run WSL, enter:
// C:\Utilities\openssl s_client -showcerts -connect www.howsmyssl.com:443
// use 3rd certificate output, labelled i:O = Digital Signature Trust Co
//
// To get certificate for required server, eg: raw.githubusercontent.com
// C:\Utilities\openssl-3\x64\bin\openssl s_client -showcerts -verify 5 -connect raw.githubusercontent.com:443
// Required cerificate is 2nd one (parent) and 1st (child) doesnt work - see: https://www.esp32.com/viewtopic.php?t=12083
//
// APP NOTES:
// 
// ESP32-CAM_MJPEG2SD:
// - uncomment INCLUDE_SMTP, INCLUDE_FTP, INCLUDE_SD 
// - set STORAGE to SD_MMC
// - https://kandi.openweaver.com/c++/s60sc/ESP32-CAM_MJPEG2SD
//
// TuyaDevice on ESP32-C3 (inverse applies if testing on ESP32 with SD):
// - Comment out INCLUDE_SD, INCLUDE_SMTP, INCLUDE_FTP
// - set UART0 to true 
// - set STORAGE to LittleFS

#include "appGlobals.h" 

// **************** my wifi config ****************/ 

static int findWifi() {
  // find suitable wifi access point with strongest signal
  int ssidIndex = -1;
  int bestSignal = -999;  
  int numNetworks = WiFi.scanNetworks();
  for (int i=0; i < numNetworks; i++) {
    if (strstr(WiFi.SSID(i).c_str(), "bisk") != NULL) {
      // dont use bisk0ts as cam web page cant be accessed - reason unknown
      if (strcmp(WiFi.SSID(i).c_str(), "bisk0ts") != 0) { 
        int sigStrength = WiFi.RSSI(i);
        if (sigStrength > bestSignal) {
          bestSignal = sigStrength;
          ssidIndex = i;
        }
        LOG_INF("Network: %s; signal strength: %d dBm; Encryption: %s; channel: %u", WiFi.SSID(i).c_str(), sigStrength, getEncType(i), WiFi.channel(i));
      }
    }
    yield();
  }
  return ssidIndex;
}

// setup wifi for personal environment
static bool prepWifi() {
  // set up wifi
  if (WiFi.status() != WL_CONNECTED) {
    int ssidIndex = findWifi();
    if (ssidIndex >= 0) {
      updateStatus("ST_SSID", WiFi.SSID(ssidIndex).c_str());
      updateStatus("ST_Pass", "lr15next"); 
      updateStatus("ST_ip", "192.168.1." STATIC_IP_OCTAL);
      updateStatus("ST_sn", "255.255.255.0");
      updateStatus("ST_gw", "192.168.1.1");
      updateStatus("ST_ns1", "192.168.1.1");
    } else {
      LOG_WRN("No suitable WiFi access point found");
      return false;
    }
  }
  return true; // already connnected
}

#ifdef SIDE_ALARM
static void ledTask(void *arg) {
  // flash external led on yale alarm
  // need to set lampUse on and value for lampLevel 
  int fullDark = 2; // lowest lightLevel value
  int fullLevel = 15; // max lampLevel
  delay(10000);
  int tempInterval = 0;
  while (true) {
    int onSecs = voltInterval * 1000;
    int offSecs = voltDivider * 1000;
    int offLevel = (int)voltLow;
    // requested light level at night
    int onLevel = lampLevel; 
    // max poss brightness during day
    if (lightLevel >= nightSwitch) onLevel = fullLevel;
    // reduce light level from full to requested during dusk
    else if (lightLevel > fullDark) onLevel = lampLevel + ((fullLevel - lampLevel) 
      * (lightLevel - fullDark)  / (nightSwitch - fullDark));
    setLamp(onLevel);
    delay(onSecs);
    setLamp(offLevel);
    delay(offSecs);
    tempInterval += onSecs + offSecs;
    if (tempInterval >= 5 * 60 * 1000) {
      // once per 5 minutes
      LOG_WRN("Cam temp: %0.1f", readTemperature(true));
      tempInterval = 0;
    }
    delay(100); // in case onSecs + offSecs = 0
  }
}
#endif

/****************** initial setup ****************/

void devSetup() {
  LOG_WRN("***** Using devSetup *****");
  prepWifi();
#ifdef SIDE_ALARM
  // ledTask only used for mjpeg2sd sideAlarm
  xTaskCreate(ledTask, "ledTask", 2048, NULL, 1, NULL);
#endif
  debugMemory("devSetup");
}
