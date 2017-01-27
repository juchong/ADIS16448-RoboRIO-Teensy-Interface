////////////////////////////////////////////////////////////////////////////////////////////////////////
//  January 2017
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448_RoboRIO_Teensy_Interface.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces an ADIS16448 IMU to a RoboRIO via UART (Serial1). 
//  The Arduino software provides a means of extracting IMU data while precicely controlling 
//  software timing. In addition, this software greatly reduces the processing overhead
//  required to acquire and calculate heading using only the RoboRIO. Gyro offset calibration
//  and heading reset are currently supported using ASCII Serial1 commands sent via serail. 
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
#include <EEPROMex.h>

//#define DEBUG
#define HWSERIAL Serial1

// Initialize data variables
int16_t *burstData;
float scaledData[11];
float pitch, roll, yaw, xdelta, ydelta, zdelta;
String datapacket = "";
String separator = ',';
float calDataX = 0;
float calDataY = 0;
float calDataZ = 0;
int calCounter = 0;

// EEPROM initialization
int calXloc = 0;
int calYloc = 5;
int calZloc = 10;

// Delay counter variable
int printCounter = 0;

// Z mounting orientation
bool initOrientation = true; //true = ZUP, false = ZDOWN

// Program flags
int internalCalFlag = 0;
int resetGyroFlag = 0;
bool teensyCalFlag = false;

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
  HWSERIAL.begin(115200);
  IMU.configSPI();
  IMU.regWrite(MSC_CTRL, 0x06);  // Enable Data Ready, set polarity
  delay(20); 
  IMU.regWrite(SENS_AVG, 0x402); // Set digital filter
  delay(20);
  IMU.regWrite(SMPL_PRD, 0x01), // Disable decimation
  delay(20);

  calDataX = EEPROM.readFloat(calXloc);
  calDataY = EEPROM.readFloat(calYloc);
  calDataZ = EEPROM.readFloat(calZloc);

  attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge

}

// Read data from the sensor using burst mode, scale data, and update SGI/AHRS loops. Check for cal flag before reading next frame of data.
void grabData() {
  if (internalCalFlag == 1) {
    calibrateOffsetRegisterIMU();
  }
  if (teensyCalFlag == true) {
    calibrateTeensy();
  }
  else {
    burstData = IMU.burstRead(); // Read data and insert into array
    scaleData(); // Scale IMU data
    deltat = micros() - oldtime; // Calculate delta time from last interrupt
    if (deltat > 2000) { // Check for micros() rollover and correct - dirty, but works.
      deltat = 1025;
    }
    oldtime = micros(); // Update delta time variable for next iteration
    
    SGI.update(scaledData[0], scaledData[1], scaledData[2], deltat, resetGyroFlag); // Integrate gyros over time
  
    if (initOrientation == true) {
      AHRS.update(scaledData[0], scaledData[1], scaledData[2], scaledData[3], scaledData[4], scaledData[5], scaledData[6], scaledData[7], scaledData[8], deltat); // Calculage Madgwick AHRS
    }
    else {
      AHRS.update(scaledData[0], (scaledData[1] * -1), (scaledData[2] * -1), scaledData[3], (scaledData[4] * -1), (scaledData[5] * -1), scaledData[6], (scaledData[7] * -1), (scaledData[8] * -1), deltat); // Calculage Madgwick AHRS
    }
    
    if (resetGyroFlag == 1) {
      resetGyroFlag = 0;
    }
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
    // Subtract offsets from gyro signal
    scaledData[0] = scaledData[0] - calDataX;
    scaledData[1] = scaledData[1] - calDataY;
    scaledData[2] = scaledData[2] - calDataZ;
    //scaledData[9] = IMU.pressureScale(*(burstData + 10)); //Scale Pressure Sensor
    //scaledData[10] = IMU.tempScale(*(burstData + 11)); //Scale Temp Sensor
}

void calibrateTeensy() {
  //HWSERIAL.println("We're in cal mode");
  burstData = IMU.burstRead(); // Read data and insert into array
  scaleData();
  calDataX = scaledData[0] + calDataX;
  calDataY = scaledData[1] + calDataY;
  calDataZ = scaledData[2] + calDataZ;
  calCounter = calCounter + 1;
  if (calCounter >= 8192) {
    //calDataX = calDataX * 0.0004070004;
    //calDataY = calDataY * 0.0004070004;
    //calDataZ = calDataZ * 0.0004070004;
    EEPROM.writeFloat(calXloc,calDataX);
    EEPROM.writeFloat(calYloc,calDataY);
    EEPROM.writeFloat(calZloc,calDataZ);
    teensyCalFlag = false;
    calCounter = 0;
  }
}

// IMU calibration routine
void calibrateOffsetRegisterIMU() {
  detachInterrupt(2); //Detach interrupt to avoid overwriting data
  HWSERIAL.println("Make sure the sensor is in a stable position!");
  HWSERIAL.print("Starting in ");
  HWSERIAL.print("5... ");
  delay(1000);
  HWSERIAL.print("4... ");
  delay(1000);
  HWSERIAL.print("3... ");
  delay(1000);
  HWSERIAL.print("2... ");
  delay(1000);
  HWSERIAL.println("1... ");
  delay(1000);
  HWSERIAL.println("Recording data. This will take ~25 Seconds. Do not move the sensor!");
  IMU.regWrite(SENS_AVG, 0x102); // Set gyro range to 250dps
  delay(50);
  IMU.regWrite(SMPL_PRD, 0xE01); //Set averaging to ~25 seconds
  for(int i = 0; i < 25; i++) {
    HWSERIAL.print(".");
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
  HWSERIAL.println(" ");
  HWSERIAL.println("Offsets have been recorded! Here's what was written to the sensor:");
  int16_t xgoffset = IMU.regRead(XGYRO_OFF);
  int16_t ygoffset = IMU.regRead(YGYRO_OFF);
  int16_t zgoffset = IMU.regRead(ZGYRO_OFF);
  HWSERIAL.print("XGYRO_OFF: ");
  HWSERIAL.println(xgoffset);
  HWSERIAL.print("YGYRO_OFF: ");
  HWSERIAL.println(ygoffset);
  HWSERIAL.print("ZGYRO_OFF: ");
  HWSERIAL.println(zgoffset);
  delay(5000);
  internalCalFlag = 0; // Clear cal flag
  attachInterrupt(2, grabData, RISING); // Re-attach interrupt to pin 2. Trigger on the rising edge
}

// Function to act upon Serial1 data received from the host
void serialEvent1() {
  while (HWSERIAL.available()) {
    char inChar = (char)HWSERIAL.read();
    if (inChar == 'c') {
      internalCalFlag = 1;
    }
    if (inChar == 'r') {
      resetGyroFlag = 1;
    }
    if (inChar == 'u') {
    	initOrientation = true;
    }
    if (inChar == 'd') {
    	initOrientation = false;
    }
    if (inChar == 'a') {
      teensyCalFlag = true;
    }
  }
}

void loop() {
  printCounter ++;
    if (printCounter >= 10000) // Delay for writing data to the Serial1 port
    {
        
        // Gather packet data...
        pitch = AHRS.getPitch();
        roll = AHRS.getRoll();
        yaw = AHRS.getYaw();
        xdelta = SGI.getX();
        ydelta = SGI.getY();
        zdelta = SGI.getZ();

        // Print data packet

        datapacket += pitch;
        datapacket += separator;
        datapacket += roll;
        datapacket += separator;
        datapacket += yaw;
        datapacket += separator;
        datapacket += xdelta;
        datapacket += separator;
        datapacket += ydelta;
        datapacket += separator;
        datapacket += zdelta;

        HWSERIAL.println(datapacket);
        #ifdef DEBUG
          HWSERIAL.println(calDataX);
          HWSERIAL.println(calDataY);
          HWSERIAL.println(calDataZ);
        #endif

        datapacket = "";
        
        printCounter = 0;
    }

}
