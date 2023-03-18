// Global SG declarations
//
// s60sc 2023

#pragma once
#include "globals.h"

// web server ports
#define WEB_PORT 80 // app control
#define OTA_PORT 82 // OTA update

/********************* fixed defines leave as is *******************/ 
/** Do not change anything below here unless you know what you are doing **/

//#define DEV_ONLY // leave commented out
#define STATIC_IP_OCTAL "123" // dev only
#define CHECK_MEM false // leave as false
#define FLUSH_DELAY 0 // for debugging crashes
 
#define APP_NAME "ESP32_SG" // max 15 chars
#define APP_VER "1.1"

#define MAX_CLIENTS 2 // allowing too many concurrent web clients can cause errors
#define DATA_DIR "/data"
#define HTML_EXT ".htm"
#define TEXT_EXT ".txt"
#define JS_EXT ".js"
#define CSS_EXT ".css"
#define ICO_EXT ".ico"
#define SVG_EXT ".svg"
#define INDEX_PAGE_PATH DATA_DIR "/SG" HTML_EXT
#define CONFIG_FILE_PATH DATA_DIR "/configs" TEXT_EXT
#define LOG_FILE_PATH DATA_DIR "/log" TEXT_EXT
#define OTA_FILE_PATH DATA_DIR "/OTA" HTML_EXT 
#define FILE_NAME_LEN 64
#define ONEMEG (1024 * 1024)
#define MAX_PWD_LEN 64
#define JSON_BUFF_LEN (1024 * 4) 
#define MAX_CONFIGS 100 // > number of entries in configs.txt
#define GITHUB_URL "https://raw.githubusercontent.com/s60sc/ESP32_SpecificGravity/master"

#define FILLSTAR "****************************************************************"
#define DELIM '~'
#define STORAGE LittleFS // use of LIttleFS or SD_MMC
#define RAMSIZE (1024 * 8) 
#define CHUNKSIZE (1024 * 4)
#define RAM_LOG_LEN 5000 // size of ram stored system message log in bytes
//#define INCLUDE_FTP 
//#define INCLUDE_SMTP
//#define INCLUDE_SD

#define IS_IO_EXTENDER false // must be true for IO_Extender
#define EXTPIN 100
#define NULL_TEMP -127.0
#define BOUNDARY_VAL "123456789000000000000987654321"

// I2C devices requiring separate libraries
#define USE_BMP280 false
#define USE DS3231 false
#define USE_SSD1306 false

/******************** Function declarations *******************/

// global app specific functions

void SGsetup();
void SGloop();
void startI2C();
double* readMPU6050();
bool sleepMPU6050(bool doSleep = true);
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
