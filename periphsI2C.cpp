
// I2C driver and devices
//
// OLED SSD1306 display 128*64
// PCF8591 ADC
// BMP280 temperature & pressure
// DS3231 RTC
// LCD 1602 display 2*16 
// MPU6050 6 axis accel & gyro
//
// s60sc 2023

#include "appGlobals.h"
#include <Wire.h>

// I2C device address constants
#define SSD1306_BIaddr 0x3d // built in oled 
#define SSD1306_Extaddr 0x3c // external oled
#define PCF8591addr 0x48 // PCF8591 ADC
#define BMP280_Def 0x76 // BMP280 default address
#define BMP280_Alt 0x77 // BMP280 alternative address
#define DS3231_RTC 0x68 // real time clock (address may conflict with MPU6050
#define LCD1602 0x27 // 16 chars by 2 lines LCD
#define BADDATA 0xFFFFFFFF // used to indicate bad data value

#if USE_SSD1306
#include "SSD1306Wire.h" 
SSD1306Wire oled(40); // oled i2c display, dummy address
#endif
#if USE_BMP280
#include <BMx280I2C.h>
// To get BMx280I2C library to compile, removed static from
// tempHumToDewPoint() and tempDewToHumidity() in BMx280MI.h
BMx280I2C bmp(BMP280_Def);
#endif
#if USE_DS3231
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);
#endif
#include "driver/rtc_io.h"

// global constants
int I2C_SDA;
int I2C_SCL;
const bool flipOled = false; // true if oled pins oriented above display
const uint8_t dispMax = 3;  // number of different oled display frames
float sensorsVals[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
static volatile bool RTCalarmFlag = false;
int MPU6050addr = 0x69; // MPU6050 I2C address if AD0 pulled high, 0x68 if AD0 grounded

static byte I2CDATA[10]; // store I2C data received or to be sent 
// I2C device names, indexed by address
static bool deviceStatus[128] = {false}; // whether device present
static const char* clientName[128] = {
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  "", "", "", "", "", "", "", "LCD1602", "", "", "", "", "", "", "", "",
  "", "", "", "", "", "", "", "", "", "", "", "", "SSD1306", "SSD1306", "", "",
  "", "", "", "", "", "", "", "", "PCF8591", "", "", "", "", "", "", "",
  "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", "",
  "", "", "", "", "", "", "", "", "DS3231/MPU6050", "MPU6050", "", "", "", "", "", "",
  "", "", "", "", "", "", "BMP280", "BMP280", "", "", "", "", "", "", "", ""};
  
struct tempHumid { // used by BMP280 & DHT11
  float temp;
  float humid; // pressure for BMP280
  float heatidx; // apparent temp based on humidity
  float dewpoint; // temp at which condensation occurs
};

static bool oledOK = false;
static bool MPU6050ok = false;

/***************************************** OLED Display *************************************/

void twinkleLed(uint8_t ledPin, uint16_t interval, uint8_t blinks) {
  // twinkle external (not built in) led, for given number of blinks, 
  //  with given interval in ms between blinks
  bool ledState = true;
  for (int i=0; i<blinks*2; i++) {
    digitalWrite(ledPin, ledState);
    delay(interval);
    ledState = !ledState;
  }
}

// OLED SSD1306 display 128*64
void oledLine(const char* msg, int hpos, int vpos, int msgwidth, int fontsize) {
#if (USE_SSD1306) 
  // display text message on OLED SSD1306 display 
  // to avoid flicker, only call periodically
  // args: message string, horizontal pixel start, vertical pixel start, width to clear, font type
  // clear original line
  if (oledOK) {
    oled.setTextAlignment(TEXT_ALIGN_LEFT);
    oled.setColor(BLACK);
    oled.fillRect(hpos, vpos, msgwidth, fontsize*5/4); // allow for tails on fonts
    // display given text, fontsizes are 10, 16, 24, starting at horiz pixel hpos & vertical pixel vpos
    oled.setFont(ArialMT_Plain_10);
    if (fontsize == 16) oled.setFont(ArialMT_Plain_16);
    if (fontsize == 24) oled.setFont(ArialMT_Plain_24);
    oled.setColor(WHITE);
    oled.drawString(hpos, vpos, msg);
  }
#endif
}

static void tellTale() {
#if (USE_SSD1306) 
  static bool ledState = false;
  ledState = !ledState;
  static const char* tellTaleStr[] = {"*", ""}; // shows that oled (& I2C) are running
  oledLine(tellTaleStr[ledState],124,60,4,10); 
#endif
}

void oledDisplay() {
#if (USE_SSD1306) 
  if (oledOK) {
////    tellTale();   // oled telltale
    oled.display();
  }
#endif
}

static bool oledInit() {
#if (USE_SSD1306) 
  /* s60sc added setAddress() method to local SSD1306Wire.h library
   under public:
    void setAddress(uint8_t _address, int _sda, int _scl) {
      this->_address = _address;
      this->_sda = _sda;
      this->_scl = _scl;
    }
  */
  if (!deviceStatus[SSD1306_BIaddr]) return false;
  oled.end();
  oled.setAddress(SSD1306_BIaddr, I2C_SDA, I2C_SCL);
  if (oled.init()) {
    if (flipOled) oled.flipScreenVertically();
    return true;
  }
#endif
  return false;
}

void finalMsg(const char* finalTxt) {
#if (USE_SSD1306) 
  // display message on persistent oled screen before esp32 goes to sleep
  oled.resetDisplay();
  oledLine(finalTxt,0,0,128,16);
  oled.display();
  delay(2000);
#endif
}

void changeDisplay() {
#if (USE_SSD1306) 
  // switch between different screens
  oledOK = oledInit();  // in case oled display has been plugged in
  static int dispIndex = 0; 
  dispIndex++;
  if (dispIndex >= dispMax) dispIndex = 0;
  displayValuesOled(dispIndex, true);
#endif
}

void updateDisplay() {
  displayValuesOled(-1, false); // refresh currently displayed values
}

/*********************** PCF8591 ************************/

uint32_t getPCF8591() { // analog channels
/*   
NOTE: change to return a struct of 4 bytes
   YL-40 module
   return the 4 ADC channel 8 bit values, using auto increment control instruction
   PC8591 commands:
   bits 0-1: channel 0 (00) -> 3 (11)
   bit 3: autoincrement
   bits 4-5: input programming, separate inputs (00), etc
   bit 6: analog out enable
  */
  if (getI2Cdata(PCF8591addr, 0x44, 5)) {
    // need to read 5 bytes, but first gets ignored as is previous 0 channel
    // order high->low channels 3 2 1 0
    for (int i=1; i<5; i++) sensorsVals[5+i] = smoothAnalog(I2CDATA[i]);
    return (byte)sensorsVals[9] << 24 | (byte)sensorsVals[8] << 16 | (byte)sensorsVals[7] << 8 | (byte)sensorsVals[6]; 
  } 
  return BADDATA;
}

/********************************** BMP280 ************************************/

tempHumid getBMP280() { // temp & pressure
#if USE_BMP280
//  (bmp.readPressure() * 0.000145);   // pascals converted to PSI 
//  (bmp.readPressure() * 0.01);  // pascals converted to mB
  if (bmp.hasValue()) sensorsVals[7] = smoothSensor(bmp.getPressure(), sensorsVals[7], 0.5);  // pascals 
  // ambient temperature (but affected by chip heating)
  if (bmp.hasValue()) sensorsVals[6] = smoothSensor(bmp.getTemperature(), sensorsVals[6], 0.5); // celsius 
#endif
  return {sensorsVals[6], sensorsVals[7], 0, 0};
}

void checkBMP() {
#if USE_BMP280
  // check if BMP280 that is supposed to be present have become available
  static bool haveBMP = false;
  if (!haveBMP) {
    haveBMP = bmp.begin();
    if (!haveBMP) LOG_ERR("BMP280 not registered");
    else {
      bmp.resetToDefaults();
      bmp.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
      bmp.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);
      bmp.measure();
    }
  }
#endif
}

/********************************** MPU6050 ************************************/

// MPU6050 definitions
#define SENS_2G (32768.0/2.0) // divider for 2G sensitivity reading
#define ACCEL_BYTES 6 // 2 bytes per axis
#define CONFIG 0x1A
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B

double* readMPU6050() {
  // get data from MPU6050 
  static double Gforce[4] = {0};
  if (MPU6050ok) {
    if (getI2Cdata(MPU6050addr, ACCEL_XOUT_H, ACCEL_BYTES+2)) { 
      // read 3 axis accelerometer & temperature
      int16_t raw[4]; // X, Y, Z, Temp
      for (int i=0; i<4; i++) raw[i] = I2CDATA[i*2] << 8 | I2CDATA[(i*2)+1]; 
      // each axis G force value, straight down is 1.0 if stationary
      for (int i=0; i<3; i++) Gforce[i] = raw[i] / SENS_2G;
      Gforce[3] = ((double)raw[3] / 340.0) + 36.53; // degrees celsius
    }
  }
  return Gforce;
}

bool sleepMPU6050(bool doSleep) {
  // power down or wake up MPU6050 
  I2CDATA[0] = doSleep ? 0x40 : 0x01;
  // PWR_MGMT_1 register set to sleep
  return sendI2Cdata(MPU6050addr, PWR_MGMT_1, 1);
}

bool checkMPU6050() {
#if USE_MPU6050
  if (deviceStatus[MPU6050addr]) {
    // set full range
    I2CDATA[0] = 0x00; 
    MPU6050ok = sendI2Cdata(MPU6050addr, CONFIG, 1);
    // wakeup the sensor 
    if (MPU6050ok ) sleepMPU6050(false);
  } else MPU6050ok = false;
#endif
  return MPU6050ok;
}

/********************************* DS3231 RTC ************************************/

static void IRAM_ATTR RTCalarmISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  RTCalarmFlag = true;
  if (xHigherPriorityTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}

void setupRTC(int RtcSquareWaveInterrupt) {
#if USE_DS3231
  // CONNECTIONS:
  // DS3231 SDA --> SDA
  // DS3231 SCL --> SCL
  // DS3231 VCC --> 3.3v or 5v
  // DS3231 GND --> GND
  // DS3231 SQW --> Alarm Interrupt Pin - needs pullup

  // set the interupt pin to input mode with pullup
  pinMode(RtcSquareWaveInterrupt, INPUT_PULLUP);

  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__); // compilation time
  if (!Rtc.IsDateTimeValid()) {
    Serial.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (!Rtc.GetIsRunning()) {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial.println("RTC is older than compile time, Updating DateTime");
    Rtc.SetDateTime(compiled);
  }
  
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmBoth); // set to be alarm output
  Rtc.LatchAlarmsTriggeredFlags();  // throw away any old alarm state before we ran
  // setup alarm interrupt 
  attachInterrupt(digitalPinToInterrupt(RtcSquareWaveInterrupt), RTCalarmISR, FALLING);
#endif
}

int cycleRange(int currVal, int minVal, int maxVal) {
  // cycle round values
  if (currVal < minVal) return maxVal;
  if (currVal > maxVal) return minVal;
  return currVal;
}

void setRTCintervalAlarm(int alarmHour, int alarmMin) {
#if USE_DS3231
  // Alarm 1 can be once per second, or at given time - seconds accuracy
  // Used here for repeated interval time (hours & minutes of interval) - so set multiple times
  // occurs on 30 secs mark to avoid clash with setRTCrolloverAlarm()
  // args are hours and mins to occur after current time
  int nextHour = cycleRange(Rtc.GetDateTime().Hour()+alarmHour, 0, 23);
  int nextMin = cycleRange(Rtc.GetDateTime().Minute()+alarmMin, 0, 59);
  DS3231AlarmOne alarm1(0, nextHour, nextMin, 30, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);
#endif
}

void setRTCspecificAlarm(int alarmHour, int alarmMin) {
#if USE_DS3231
  // Alarm 1 can be once per second, or at given time - seconds accuracy
  // Used here for specific time (hours & minutes of day) - so can be set multiple times
  // occurs on 30 secs mark to avoid clash with setRTCrolloverAlarm()
  // args are specific hour and minute of day to occur
  DS3231AlarmOne alarm1(0, alarmHour, alarmMin, 30, DS3231AlarmOneControl_HoursMinutesSecondsMatch);
  Rtc.SetAlarmOne(alarm1);
#endif
}

void setRTCrolloverAlarm(int alarmHour, int alarmMin) {
#if USE_DS3231
  // Alarm 2 can be once per minute, or at a given time - minute accuracy
  // Used here for daily rollover alarm - set once
  DS3231AlarmTwo alarm2(0, alarmHour, alarmMin, DS3231AlarmTwoControl_HoursMinutesMatch);
  Rtc.SetAlarmTwo(alarm2);
#endif
}

uint32_t getRTCtime() {
#if USE_DS3231
  // get current RTC time as epoch
  if (!Rtc.IsDateTimeValid()) LOG_WRN("RTC lost confidence in the DateTime!");
//  return (uint32_t) Rtc.GetDateTime();
#endif
return 0;
}

int RTCalarmed() {
  // check if RTC alarm occurred and return alarm number
  int wasAlarmed = 0;
#if USE_DS3231
  if (RTCalarmFlag) { 
    RTCalarmFlag = false; // reset the flag
    DS3231AlarmFlag flag = Rtc.LatchAlarmsTriggeredFlags(); // which alarms triggered and reset for next
    if (flag & DS3231AlarmFlag_Alarm1) wasAlarmed = 1; 
    if (flag & DS3231AlarmFlag_Alarm2) wasAlarmed = 2;
  }
#endif
  return wasAlarmed;
}

float RTCtemperature() {
#if USE_DS3231
  // internal temperature of DS3231
  RtcTemperature temp = Rtc.GetTemperature();
  return temp.AsFloatDegC();
#endif
  return 0;
}

void RTCdatetime(char* datestring, int datestringLen) {
#if USE_DS3231
  // return RTC formatted date time string
  if (!Rtc.IsDateTimeValid()) Serial.println("RTC lost confidence in the DateTime!");
  RtcDateTime dt = Rtc.GetDateTime(); // seconds since jan 1 2000
  snprintf(datestring, datestringLen, "%02u/%02u/%04u %02u:%02u:%02u",
    dt.Day(), dt.Month(), dt.Year(), dt.Hour(), dt.Minute(), dt.Second());
#endif
}


/**************************** LCD1602 ******************************/
// I2C LCD display: 2 lines, 16 cols 
// Derived from https://github.com/arduino-libraries/LiquidCrystal

// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En B00000100  // Enable bit
#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

#define NUM_ROWS 2
#define NUM_COLS 16

static uint8_t displaycontrol;
static uint8_t displaymode;
static uint8_t backlightval;

static void lcdWrite(uint8_t data) {
  I2CDATA[0] = data | backlightval;
  sendI2Cdata(LCD1602, 0, 1); 
}

static void writeNibble(uint8_t value) {
	lcdWrite(value);
	lcdWrite(value | En);	  // En high
	delayMicroseconds(1);		// pulse
	lcdWrite(value & ~En);	// En low
	delayMicroseconds(50);	// commands need > 37us to settle
}

static void lcdSend(uint8_t value, uint8_t mode = 0) {
  // write either command (mode = 0) or data, as two 4 bit values
  writeNibble((value & 0xf0) | mode);
	writeNibble(((value << 4 ) & 0xf0) | mode); 
}

void lcdBacklight(onoffType lightOn) {
  // Turn the backlight on / off
  backlightval = (lightOn == ON) ? LCD_BACKLIGHT : LCD_NOBACKLIGHT;
  lcdWrite(backlightval);
}

void lcdClear() {
  // clear display, set cursor position to zero
  lcdSend(LCD_CLEARDISPLAY);
  delayMicroseconds(2000);  
}

void lcdDisplay(onoffType setDisplay) {
  // Turn the display on / off (not backlight)
  if (setDisplay == ON) displaycontrol |= LCD_DISPLAYON;
  else displaycontrol &= ~LCD_DISPLAYON;
  lcdSend(LCD_DISPLAYCONTROL | displaycontrol);
}

bool lcdInit() {  
  if (!deviceStatus[LCD1602]) return false;
	uint8_t displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

 	// can only use 4 bit mode with PCF8574 as not enough pins for HD44780 8 bit.
  // use magic sequence to set it  
  writeNibble(0x03 << 4);
  delayMicroseconds(4500); 
  writeNibble(0x03 << 4);
  delayMicroseconds(4500); 
  writeNibble(0x03 << 4); 
  delayMicroseconds(150);
  writeNibble(0x02 << 4); 

	// set initial display format
	lcdSend(LCD_FUNCTIONSET | displayfunction);  
	displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;               
  lcdBacklight(ON);  
	lcdDisplay(ON);
	lcdClear();
	displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
	lcdSend(LCD_ENTRYMODESET | displaymode);  
  return true;
}

void lcdPrint(const char* str) {
  // write string to lcd
	for (int i=0; i<strlen(str); i++) lcdSend((uint8_t)str[i], Rs);
}

void lcdHome() {
  // set cursor position to zero
	lcdSend(LCD_RETURNHOME);  
	delayMicroseconds(2000); 
}

void lcdSetCursorPos(uint8_t row, uint8_t col) {
  // set row and col of cursor position
	int row_offsets[] = {0x00, 0x40, 0x14, 0x54}; 
  if (row > NUM_ROWS) row = NUM_ROWS - 1;
  if (col > NUM_COLS) col = NUM_COLS - 1;
	lcdSend(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void lcdLineCursor(onoffType showLine) {
  // Turn the underline cursor on / off
  if (showLine == ON) displaycontrol |= LCD_CURSORON;
  else displaycontrol &= ~LCD_CURSORON;
	lcdSend(LCD_DISPLAYCONTROL | displaycontrol);
}

void lcdBlinkCursor(onoffType showBlink) {
  // Turn the blinking cursor on / off
  if (showBlink == ON) displaycontrol |= LCD_BLINKON;
  else displaycontrol &= ~LCD_BLINKON;
	lcdSend(LCD_DISPLAYCONTROL | displaycontrol);
}

void lcdScrollText(lfType scrolling) {
  // scroll the current display left or right one position (no wrapping)
  uint8_t moveDir = (scrolling == RIGHT) ? LCD_MOVERIGHT : LCD_MOVELEFT;
  lcdSend(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | moveDir);
}

void lcdTextDirection(lfType textDir) {
  // write text forward or backward from cursor 
  if (textDir == RIGHT) displaymode |= LCD_ENTRYLEFT;
  else displaymode &= ~LCD_ENTRYLEFT;
	lcdSend(LCD_ENTRYMODESET | displaymode);
}

void lcdAutoScroll(onoffType autoScroll) {
  // As each character entered at cursor, scroll previous text left
	if (autoScroll == ON) displaymode |= LCD_ENTRYSHIFTINCREMENT; 
  else displaymode &= ~LCD_ENTRYSHIFTINCREMENT; 
	lcdSend(LCD_ENTRYMODESET | displaymode);
}

void lcdLoadCustom(customChar charLoc, uint8_t charmap[]) {
  // Load custom character
  // To create, see https://maxpromer.github.io/LCD-Character-Creator/
  // array of 8 lines of 5 bits, where bits represent pixel on / off
  // eg define & load custom char (degrees celsius symbol)
  // uint8_t celsius[] = {B01000, B10100, B01011, B00100, B00100, B00100, B00011, B00000};
  // lcdLoadCustom(CELSIUS, celsius);
  uint8_t _charLoc = (uint8_t)charLoc;
	_charLoc &= 0x7; // CGRAM location to load 0 - 7
	lcdSend(LCD_SETCGRAMADDR | (_charLoc << 3));
	for (int i=0; i<8; i++)	lcdSend(charmap[i], Rs);
}

void lcdWriteCustom(customChar charLoc) {
  // write one of 8 custom chars 
  lcdSend((uint8_t)charLoc, Rs);
}


/********************* Generic I2C Utilities ***********************/

static bool sendTransmission(int clientAddr, bool scanning) {
  // common function used to send request to I2C device and determine outcome
  byte result = Wire.endTransmission(true);
    /*1: data too long to fit in transmit buffer
      2: received NACK on transmit of address
      3: received NACK on transmit of data
      4: other error, e.g. switched off 
      5: i2c busy 
      8: unknown pcf8591 status */
      
  if (!scanning && result > 0) LOG_ERR("Client %s at 0x%x with connection error: %d", clientName[clientAddr], clientAddr, result);
  return (result == 0) ? true : false;
}

static bool scanI2C() {
  // find details of any active I2C devices
  byte address;
  int nDevices;
  LOG_INF("I2C client scanning");
  nDevices = 0;
  for (address = 0; address < 127; address++) {
    Wire.beginTransmission(address);
    // only report error if client device meant to be present
    if (sendTransmission(address, true)) {
      LOG_INF("I2C device %s present at address: 0x%x", clientName[address], address);
      nDevices++;
      deviceStatus[address] = true;
    }
  }
  if (nDevices == 0) {
    LOG_INF("No I2C devices found");
    return false;
  }
  return true;
}

bool startI2C() {
  if (I2C_SDA == I2C_SCL) LOG_ALT("I2C pins not defined");
  else {
    Wire.begin(I2C_SDA, I2C_SCL); // join i2c bus as master 
    // list I2C devices available
    if (scanI2C()) checkI2C(); // start devices
    else return false;
    return true;
  }
  return false;
}

void checkI2C() {
  // regularly check if connected I2C devices available
  checkBMP();
  checkMPU6050();
  oledOK = oledInit();
}

bool getI2Cdata (uint8_t clientAddr, uint8_t controlByte, uint8_t numBytes) {
  // send command to I2C client and receive response
  // clientAddr is the I2C address
  // controlByte is the control instruction
  // numBytes is number of bytes to request
  Wire.beginTransmission(clientAddr); // select which client to use
  Wire.write(controlByte); // send device command
  if (sendTransmission(clientAddr, false)) {
    // get required number of bytes
    Wire.requestFrom (clientAddr, numBytes);
    for (int i=0; i<numBytes; i++) I2CDATA[i] = Wire.read();
    return sendTransmission(clientAddr, false);
  } 
  return false; 
}

bool sendI2Cdata(int clientAddr, uint8_t controlByte, uint8_t numBytes) {
  // send data to I2C device
  // clientAddr is the I2C address
  // controlByte is the control instruction
  // numBytes is number of bytes to send
  Wire.beginTransmission(clientAddr);
  if (controlByte) Wire.write(controlByte);
  for (int i=numBytes-1; i>=0; i--) Wire.write(I2CDATA[i]);
  return sendTransmission(clientAddr, false);
}
