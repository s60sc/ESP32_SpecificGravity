# ESP32_SpecificGravity

Device to calculate Specific Gravity of wort during fermentation from tilt angle of floating sensor, where tilt angle varies with liquid density. 
Designed to provide a qualified rather than precise value for Specific Gravity for simplicity of calibration.

Uses the hardware design approach from http://www.ispindel.de/docs/README_en.html
but simplified to use an ESP32 module with an integral 18650 battery holder and battery management IC TPS5400, and obtain G force readings and temperature from a separate MCU6050 accelerometer on GY-521 module.
The code base has been independently developed.

The generated data is packaged into a json string, this data can be viewed:
* whilst the ESP32 is awake by using its own web server accessed from a browser using the `ipaddr` address in the code.
* on a remote host listening on the address given by `wsServerHost` in the code, which could be another ESP32, see `ESP32_SGhost.ino` in `ESP32_SGhost` folder as an example. 

On power up, ESP32 will remain awake for calibration purposes (see below) until user presses the __Start__ button on the web page.
The ESP32 will then periodically wakeup to collect data and send this to the remote host, then return to deep sleep after `TIME_AWAKE` for the period of time in `TIME_TO_SLEEP`

## Setup and Calibration

The ESP32 and GY-521 modules fit snugly into a 33mm width x 120mm height PETling:

![image1](extras/device.png)


To view current data and calibrate device before each use:
* Open ESP32 web page to get readings with periodic refresh.
* Balance device in PETling so reads c. 25 deg angle in plain water at 20C, then press __Save__ button under __Water Angle__ field.
* Use hydrometer to measure the original gravity of the wort prior to fermentation.
* Place device in wort and wait till settles, then press __Set__ button under __Original Gravity__ field.
* Enter hydrometer value into __Original Gravity__ field, e.g. 1.045, then press button under __Original Gravity__ field.
![image1](extras/webpage.png)

See comments in `ESP32_SpecificGravity.ino` for further details.
