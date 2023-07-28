// Global SG declarations
//
// s60sc 2023

#pragma once
#include "globals.h"

#define ALLOW_SPACES false // set true to allow whitespace in configs.txt key values

// web server ports
#define WEB_PORT 80 // app control
#define OTA_PORT (WEB_PORT + 1) // OTA update

/*********************** Fixed defines leave as is ***********************/ 
/** Do not change anything below here unless you know what you are doing **/

//#define DEV_ONLY // leave commented out
#define STATIC_IP_OCTAL "123" // dev only
#define CHECK_MEM false // leave as false
#define FLUSH_DELAY 200 // for debugging crashes
 
#define APP_NAME "ESP32_SG" // max 15 chars
#define APP_VER "1.2"

#define MAX_CLIENTS 2 // allowing too many concurrent web clients can cause errors
#define INDEX_PAGE_PATH DATA_DIR "/SG" HTML_EXT
#define FILE_NAME_LEN 64
#define JSON_BUFF_LEN (1024 * 4) 
#define MAX_CONFIGS 100 // > number of entries in configs.txt
#define GITHUB_URL "https://raw.githubusercontent.com/s60sc/ESP32_SpecificGravity/master"
#define STORAGE LittleFS // One of LittleFS or SD_MMC
#define RAMSIZE (1024 * 8) 
#define CHUNKSIZE (1024 * 4)
#define RAM_LOG_LEN 5000 // size of ram stored system message log in bytes
//#define INCLUDE_FTP 
//#define INCLUDE_SMTP
//#define INCLUDE_SD
//#define INCLUDE_MQTT

#define IS_IO_EXTENDER false // must be false except for IO_Extender
#define EXTPIN 100

// to determine if newer data files need to be loaded
#define HTM_VER "1"
#define JS_VER "0"
#define CFG_VER "1"

#define NULL_TEMP -127.0
#define FILE_EXT ""

// I2C devices requiring separate libraries
#define USE_BMP280 false
#define USE DS3231 false
#define USE_SSD1306 false
#define USE_MPU6050 true
#define USE_DS18B20 false

// LCD 1602
enum onoffType {OFF, ON};
enum lfType {LEFT, RIGHT};
enum customChar {CELSIUS, CC1, CC2, CC3, CC4, CC5, CC6, CC7};

/******************** Function declarations *******************/

// global app specific functions
void SGsetup();
void SGloop();
bool checkMPU6050();
double* readMPU6050();
bool sleepMPU6050(bool doSleep = true);
void checkI2C();
bool getI2Cdata (uint8_t clientAddr, uint8_t controlByte, uint8_t numBytes);
bool sendI2Cdata(int clientAddr, uint8_t controlByte, uint8_t numBytes); 
bool startI2C();
void displayValuesOled(int inDispIndex, bool dispChanged);

/******************** Global app declarations *******************/

// status & control fields 

// batt monitoring 
extern int voltPin; 
extern bool voltUse; // true to report on ADC pin eg for for battery
extern int voltDivider;
extern float voltLow;
extern int voltInterval;

extern int I2C_SDA;
extern int I2C_SCL;
extern int MPU6050addr;
