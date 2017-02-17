# ADIS16448 Teensy RoboRIO MXP Breakout Board for FIRST Robotics
### A solution for accurately capturing time-sensitive data from high-performance sensors! 

This project is meant to simplify the process of capturing data from high-performance ADI sensors, processing it, and making the data available to any control system by means of Serial/UART, SPI, or I2C. Due to limitations with working within a CPU environment, a secondary processor is required to quickly service data capture ISRs and perform time-sensitive calculations in real-time navigation applications. This project will allow the user to interface **any** iSensor product to any device capable of receiving (at least) UART data! 

The Teensy platform was chosen due to its low cost, availability, ease of use, and horsepower. This code could theoretically be ported over to any other Arduino-compatible controller, but it will be up to the user to determine feasibility, cost, performance, etc. 

### Supported Features:

The Teensy-based software currently offers:
- Synchronous acquisition of IMU data at max rate (819.2 SPS for the ADIS16448)
- Integrated angle outputs (XDelta, YDelta, ZDelta)
- AHRS Madgwick outputs (Pitch, Roll, Yaw)
- Instantaneous sensor rate outputs (XGyro, YGyro, ZGyro, XAccel, YAccel, ZAccel, XMag, YMag, ZMag, Baro, Temp)
- Serial/UART output at ~30Hz - Note that this value can be changed in the main loop to suit the application
- Onboard Teensy EEPROM storage for pre-recorded gyro offset values and a gyro offset calibration routine
- Z+ / Z- orientation selection, delta angle offset reset via simple UART commands

The LabVIEW-based software currently offers:
- Receiving and decoding sensor data packets from the Teensy/sensor
- Inserting data into FGVs (Functional Global Variables) for easy, clean integration into robot projects
- Simple, reliable, easy-to-use data acquisition

Additional algorithms may be added to the Teensy in the future, so be on the lookout for changes to this repository! 

### Hardware:
For FIRST Robotics use, the ADIS16448 may only be acquired from [First Choice](http://firstchoicebyandymark.com/fc17-007). A Teensy 3.2 can be purchased directly from [PJRC](https://www.pjrc.com/store/teensy32.html). 
An interface PCB with space for an onboard Teensy can be purchased from [OSHPark](https://oshpark.com/shared_projects/kfl18WHg). Reference schematics can be found in the repository in the PCB folder. 

[Picture of PCB Here]

### Additional Libraries:
In addition to a properly installed/configured Teensyduino environment, this project requires the use of several libraries to properly compile. These are listed below:

- [ADIS16448 Arduino Teensy Interface Driver](https://github.com/juchong/ADIS16448-Arduino-Teensy) 
- [EEPROMEx](https://github.com/thijse/Arduino-EEPROMEx)
- [MadgwickAHRS](https://github.com/arduino-libraries/MadgwickAHRS) - **Note that a slightly modified version of this library is included with this project**
- SimpleGyroIntegrator - Included with this project

These libraries must be installed in your `\Documents\Arduino\libraries` folder. 
