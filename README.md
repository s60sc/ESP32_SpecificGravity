# ESP32-SpecificGravity

Device to calculate Specific Gravity during fermentation from tilt angle of floating sensor, where tilt angle varies with liquid density.

Uses the hardware design approach from http://www.ispindel.de/docs/README_en.html
but simplified to use an ESP32 module with an integral 18650 battery holder and battery management IC TPS5400 and obtain G force readings and temperature from a separate MCU6050 accelerometer on GY-521 module.
The code base has been independently developed.

The generated data is packaged into a json string, this data can be viewed:
* whilst the ESP32 is awake by using its own web server accessed from a browser using the `ipaddr` address in the code
* on a remote hub listening on the address given by `wsServerHost` in the code, which could be another ESP32.

On power up, ESP32 will remain awake for calibration purposes until user presses the __Start__ button on web page.
The ESP32 will then periodically wakeup to collect data and send this to the remote hub then return to deep sleep after `TIME_AWAKE` for the period of time in `TIME_TO_SLEEP`

## Setup and Use

The ESP32 and GY-521 modules fit snugly into a 33mm width x 120mm height PETling:

![image1](extras/device.png)


Open the device web page to view current data:
![image1](extras/webpage.png)

To calibrate device before first use:
* Open ESP32 web page to get readings with periodic refresh
* Balance device in PETling so reads c. 25 deg angle in plain water at 20C
* Use hydrometer to measure SG for different sugar solutions to above your maximum wort original gravity
* Record each tilt angle and associated hydrometer reading in `angle_gravity` in `SGdata.h`.
* More data points gives improved accuracy, order does not need to be sorted

For subsequent use, adjust position of device in PETling so that it reads the same tilt angle in plain water at 20C as originally calibrated.

See comments in `ESP32_SpecificGravity.ino` for further details.
