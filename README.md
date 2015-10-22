# ADIS16448 Arduino Demo
### An example C++ library and Arduino project for the ADIS16448 Ten Degrees of Freedom Inertial Sensor

This example library was written to give engineers, students, and makers a starting point for using a high-performance, compact IMU. The code in this repository will provide the user with:
- A header file listing all of the unit's available registers
- Functions for reading output registers and writing control registers using **8-bit** frames
- Functions for performing common routines such as resetting the sensor
- Example Arduino sketches which synchronously read data from the sensor and write it to the serial port

### What do I need to get started?

In order to compile and execute the Arduino sketch, you'll need to download the "legacy" Arduino package (v1.0.6 as of this writing). You can download the IDE [here](http://arduino.cc/download.php?f=/arduino-1.0.6-windows.zip).

You'll also need an 8-bit arduino such as an [Arduino Uno](http://www.arduino.cc/en/Main/ArduinoBoardUno).

### How do I install the library?

Once you've installed the Arduino IDE, copy the ADIS16448 folder into `My Documents > Arduino > libraries`

### How do I connect the IMU to my Arduino?

**The ADIS16448 is a 3.3V part, so your Arduino must be modified before connecting the sensor to it! A guide to modifying an Arduino Uno can be found [here](https://learn.adafruit.com/arduino-tips-tricks-and-techniques/3-3v-conversion).**

After modifying the Arduino, you'll need to build a cable to interface the sensor with the [ADIS16448/PCBZ](http://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/eval-adis16448.html#eb-overview).

![ADIS16448-Arduino Cable Interface](https://raw.githubusercontent.com/juchong/ADIS16448-Arduino-Demo/master/setup_pictures/IMG_4569.JPG)

Pin assignments can be found in the Arduino sketch comments.
