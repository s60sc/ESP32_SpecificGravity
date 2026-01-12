
// Calculates fermenting wort Specific Gravity from tilt angle of floating sensor,
// where tilt angle varies with wort density.
// Tilt Angle is calculated assuming device only has rotational (not linear) motion
// Specific Gravity is calculated from the tilt angle with reference to a nth order polynomial 
// - for this implementation, only a first order is used with 2 data points
// See README.md for more info.
//
// Connections:
// MPU6050  ESP32
// INT      n/c // interrupt
// ADO      3V3 // address select
// SDA      GPIO 21 (green)
// SCL      GPIO 22 (yellow)
// GND      GND (brown)
// VCC      3V3 (white)
//
// Check accelerometer works properly (can be faulty), by checking that 
// gXYZ is close to 1g, and gX varies between to 0g (horizontal) to +/- 1g (vertical)
//
// The battery voltage to be measured needs to go thru a voltage divider comprising 
// two equal high value resistors to keep voltage below 3V3 at the ADC pin.
//
// s60sc 2018, 2020, 2023, 2025

#include "appGlobals.h"

static bool startedUp = false;

void setup() {
  logSetup();
  // prep storage & load saved user configuration
  if (startStorage()) loadConfig();

#ifdef DEV_ONLY
  devSetup();
#endif

  // connect wifi or start config AP if router details not available
  startNetwork();
  if (startWebServer()) {
    // start rest of services
    if (SGsetup()) startedUp = true;
    checkMemory();
  }
}

void loop() {
  startedUp ? SGloop() : delay(1000);
}
