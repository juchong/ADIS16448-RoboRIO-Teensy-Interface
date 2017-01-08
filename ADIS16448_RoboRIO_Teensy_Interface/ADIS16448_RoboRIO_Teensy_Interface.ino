////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448_RoboRIO_Teensy_Interface.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces an ADIS16448 IMU to a RoboRIO via UART (Serial). 
//  The Arduino software provides a means of extracting IMU data while precicely controlling 
//  software timing. In addition, this software greatly reduces the processing overhead
//  required to acquire and calculate heading using only the RoboRIO. Gyro offset calibration
//  and heading reset are currently supported using ASCII serial commands sent via serail. 
//
//  This library relies on the ADIS16448 Arduino Teensy library which may be found
//  here: https://github.com/juchong/ADIS16448-Arduino-Teensy
//  It also relies on the Madgwick AHRS implementation which may be found here:
//  https://github.com/arduino-libraries/MadgwickAHRS
//
//  This project has been tested on a PJRC 32-Bit Teensy 3.1/3.2 Development Board, 
//  but should be compatible with any other embedded platform with some modification.
//
//  Permission is hereby granted, free of charge, to any person obtaining
//  a copy of this software and associated documentation files (the
//  "Software"), to deal in the Software without restriction, including
//  without limitation the rights to use, copy, modify, merge, publish,
//  distribute, sublicense, and/or sell copies of the Software, and to
//  permit persons to whom the Software is furnished to do so, subject to
//  the following conditions:
//
//  The above copyright notice and this permission notice shall be
//  included in all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
//  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
//  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
//  LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//  Pinout for a Teensy 3.1/3.2 Development Board
//  RST = D6
//  SCK = D13/SCK
//  CS = D10/CS
//  DOUT(MISO) = D12/MISO
//  DIN(MOSI) = D11/MOSI
//  DR = D2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16448.h>
#include <MadgwickAHRS.h>
#include <SimpleGyroIntegrator.h>
#include <SPI.h>

// Uncomment to enable debug
//#define DEBUG

// Initialize data variables
int16_t *burstData;
float scaledData[11];
float pitch, roll, yaw, xdelta, ydelta, zdelta;

// Delay counter variable
int printCounter = 0;

// Program flags
int calFlag = 0;
int resetGyroFlag = 0;

// Delta time variables
unsigned long oldtime = 0;
unsigned long deltat = 0;

// Call ADIS16448 Class
ADIS16448 IMU(10,2,6); // Chip Select, Data Ready, Reset, Pin Assignments

// Call Madgwick Class
Madgwick AHRS;

// Call Gyro Integrator Class
SimpleGyroIntegrator SGI;

// Initial configuration and initialization
void setup() {
  Serial.begin(115200);
  
  IMU.configSPI();
  IMU.regWrite(MSC_CTRL, 0x06);  // Enable Data Ready, set polarity
  delay(20); 
  IMU.regWrite(SENS_AVG, 0x402); // Set digital filter
  delay(20);
  IMU.regWrite(SMPL_PRD, 0x01), // Disable decimation
  delay(20);

  attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge

}

// Read data from the sensor using burst mode, scale data, and update SGI/AHRS loops. Check for cal flag before reading next frame of data.
void grabData() {
  if (calFlag == 1) {
    calibrateIMU();
  }
    burstData = IMU.burstRead(); // Read data and insert into array
    scaleData(); // Scale IMU data
    deltat = micros() - oldtime; // Calculate delta time from last interrupt
    if (deltat > 2000) { // Check for micros() rollover and correct - dirty, but works.
      deltat = 1025;
    }
    oldtime = micros(); // Update delta time variable for next iteration
    SGI.update(scaledData[0], scaledData[1], scaledData[2], deltat, resetGyroFlag); // Integrate gyros over time
    AHRS.update(scaledData[0], scaledData[1], scaledData[2], scaledData[3], scaledData[4], scaledData[5], scaledData[6], scaledData[7], scaledData[8], deltat); // Calculage Madgwick AHRS
    if (resetGyroFlag == 1) {
      resetGyroFlag = 0;
    }
}

// Function used to scale all acquired data (scaling functions are included in ADIS16448.cpp)
void scaleData() {
    scaledData[0] = IMU.gyroScale(*(burstData + 1)); //Scale X Gyro
    scaledData[1] = IMU.gyroScale(*(burstData + 2)); //Scale Y Gyro
    scaledData[2] = IMU.gyroScale(*(burstData + 3)); //Scale Z Gyro
    scaledData[3] = IMU.accelScale(*(burstData + 4)); //Scale X Accel
    scaledData[4] = IMU.accelScale(*(burstData + 5)); //Scale Y Accel
    scaledData[5] = IMU.accelScale(*(burstData + 6)); //Scale Z Accel
    scaledData[6] = IMU.magnetometerScale(*(burstData + 7)); //Scale X Mag
    scaledData[7] = IMU.magnetometerScale(*(burstData + 8)); //Scale Y Mag
    scaledData[8] = IMU.magnetometerScale(*(burstData + 9)); //Scale Z Mag
    //scaledData[9] = IMU.pressureScale(*(burstData + 10)); //Scale Pressure Sensor
    //scaledData[10] = IMU.tempScale(*(burstData + 11)); //Scale Temp Sensor
}

// IMU calibration routine
void calibrateIMU() {
  detachInterrupt(2); //Detach interrupt to avoid overwriting data
  Serial.println("Make sure the sensor is in a stable position!");
  Serial.print("Starting in ");
  Serial.print("5... ");
  delay(1000);
  Serial.print("4... ");
  delay(1000);
  Serial.print("3... ");
  delay(1000);
  Serial.print("2... ");
  delay(1000);
  Serial.println("1... ");
  delay(1000);
  Serial.println("Recording data. This will take ~25 Seconds. Do not move the sensor!");
  IMU.regWrite(SENS_AVG, 0x102); // Set gyro range to 250dps
  delay(50);
  IMU.regWrite(SMPL_PRD, 0xE01); //Set averaging to ~25 seconds
  for(int i = 0; i < 25; i++) {
    Serial.print(".");
    delay(1000);
  }  
  IMU.regWrite(GLOB_CMD,0x01); // Write offset data to offset registers
  delay(50);
  IMU.regWrite(SMPL_PRD,0x01); // Set sample period to 819.2SPS
  delay(50);
  IMU.regWrite(GLOB_CMD,0x08); // Write configuration to IMU flash
  delay(500);
  IMU.regWrite(GLOB_CMD,0x80); // Software reset IMU
  delay(1000);
  IMU.regWrite(SENS_AVG,0x402); // Set range to 2000dps
  delay(50);
  IMU.regWrite(GLOB_CMD,0x08); // Write configuration to IMU flash
  delay(1000);
  Serial.println(" ");
  Serial.println("Offsets have been recorded! Here's what was written to the sensor:");
  int16_t xgoffset = IMU.regRead(XGYRO_OFF);
  int16_t ygoffset = IMU.regRead(YGYRO_OFF);
  int16_t zgoffset = IMU.regRead(ZGYRO_OFF);
  Serial.print("XGYRO_OFF: ");
  Serial.println(xgoffset);
  Serial.print("YGYRO_OFF: ");
  Serial.println(ygoffset);
  Serial.print("ZGYRO_OFF: ");
  Serial.println(zgoffset);
  delay(5000);
  calFlag = 0; // Clear cal flag
  attachInterrupt(2, grabData, RISING); // Re-attach interrupt to pin 2. Trigger on the rising edge
}

// Function to act upon serial data received from the host
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == 'c') {
      calFlag = 1;
    }
    if (inChar == 'r') {
      resetGyroFlag = 1;
    }
  }
}

void loop() {
  printCounter ++;
    if (printCounter >= 10000) // Delay for writing data to the serial port
    {
        //detachInterrupt(2); //Detach interrupt to avoid overwriting data

        //Clear the serial terminal and reset cursor
        //Only works on supported serial terminal programs (Putty)
        Serial.print("\033[2J");
        Serial.print("\033[H");
        
        // Print scaled gyro data
        Serial.print("Pitch: ");
        pitch = AHRS.getPitch();
        Serial.println(pitch);
        
        Serial.print("Roll: ");
        roll = AHRS.getRoll();
        Serial.println(roll);
        
        Serial.print("Yaw: ");
        yaw = AHRS.getYaw();
        Serial.println(yaw);

        Serial.println(" ");

        Serial.print("XDelta: ");
        xdelta = SGI.getX();
        Serial.println(xdelta);
        
        Serial.print("YDelta: ");
        ydelta = SGI.getY();
        Serial.println(ydelta);
        
        Serial.print("ZDelta: ");
        zdelta = SGI.getZ();
        Serial.println(zdelta);
        
        printCounter = 0;
    }

}
