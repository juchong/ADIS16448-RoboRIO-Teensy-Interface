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
//  and heading reset are currently supported using ASCII serial commands sent via serial. 
//
//  This library relies on the ADIS16448 Arduino Teensy library which may be found
//  here: https://github.com/juchong/ADIS16448-Arduino-Teensy
//
//  It also relies on the Madgwick AHRS implementation which may be found here:
//  https://github.com/arduino-libraries/MadgwickAHRS
//	
//	Saving data to EEPROM relies on the EEPROMex library found here:
//	https://github.com/thijse/Arduino-EEPROMEx
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
//	UART RX = D0
//	UART TX = D1
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16448.h>
#include <MadgwickAHRS.h>
#include <SimpleGyroIntegrator.h>
#include <SPI.h>
#include <EEPROMex.h>

//#define DEBUG

// Uncomment to overwrite garbage data in EEPROM with 0's
//#define FIRSTRUN

// Change printing from Serial1 (pins 0,1) to Serial (USB)
#define HWSERIAL Serial1

// Initialize data variables
int16_t *burstData;
float scaledData[11];
float pitch, roll, yaw, xdelta, ydelta, zdelta;
String datapacket = "";
String separator = ',';

// Initialize calibration registers
float calDataX, calDataY, calDataZ = 0;
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
  IMU.regWrite(SMPL_PRD, 0x01), // Disable decimation, enable internal sample clock
  delay(20);

// Overwrite EEPROM locations with 0's if this is the first time this code has run on this Teensy
  #ifdef FIRSTRUN
  	EEPROM.writeFloat(calXloc,calDataX);
    EEPROM.writeFloat(calYloc,calDataY);
    EEPROM.writeFloat(calZloc,calDataZ);
  #else
	calDataX = EEPROM.readFloat(calXloc);
	calDataY = EEPROM.readFloat(calYloc);
	calDataZ = EEPROM.readFloat(calZloc);
  #endif

  attachInterrupt(2, grabData, RISING); // Attach interrupt to pin 2. Trigger on the rising edge

}

// Read data from the sensor using burst mode, scale data, and update SGI/AHRS loops. Check for cal flags before reading next frame of data.
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

	    // Calculate delta time from last interrupt
	    deltat = micros() - oldtime; 
	    if (deltat > 2000) { // Check for micros() rollover and correct - dirty, but works
	      deltat = 1025;
	    }
	    oldtime = micros(); // Update delta time variable for next iteration
	    
	    // Integrate gyros over time
	    SGI.update(scaledData[0], scaledData[1], scaledData[2], deltat, resetGyroFlag); 
	  	
	  	// Compensate for IMU orientation (IMU is mounted upside-down when on an MXP breakout board)
	    if (initOrientation == true) {
	      AHRS.update(scaledData[0], scaledData[1], scaledData[2], scaledData[3], scaledData[4], scaledData[5], scaledData[6], scaledData[7], scaledData[8], deltat); // Calculage Madgwick AHRS
	    }
	    else {
	      AHRS.update(scaledData[0], (scaledData[1] * -1), (scaledData[2] * -1), scaledData[3], (scaledData[4] * -1), (scaledData[5] * -1), scaledData[6], (scaledData[7] * -1), (scaledData[8] * -1), deltat); // Calculage Madgwick AHRS
	    }
	    
	    // Clear reset gyro flag after reset has been performed
	    if (resetGyroFlag == 1) {
	      resetGyroFlag = 0;
	    }

     // Insert data into datapacket in the ISR to avoid transmitting partial data
        // Gather packet data...
        pitch = AHRS.getPitch();
        roll = AHRS.getRoll();
        yaw = AHRS.getYaw();
        xdelta = SGI.getX();
        ydelta = SGI.getY();
        zdelta = SGI.getZ();

        // Build data packet for frame
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
        datapacket += separator;
        datapacket += scaledData[0];
        datapacket += separator;
        datapacket += scaledData[1];
        datapacket += separator;
        datapacket += scaledData[2];
        datapacket += separator;
        datapacket += scaledData[3];
        datapacket += separator;
        datapacket += scaledData[4];
        datapacket += separator;
        datapacket += scaledData[5];
        datapacket += separator;
        datapacket += scaledData[6];
        datapacket += separator;
        datapacket += scaledData[7];
        datapacket += separator;
        datapacket += scaledData[8];
        datapacket += separator;
        datapacket += scaledData[9];
        datapacket += separator;
        datapacket += scaledData[10];
	}
}

// Function used to scale all acquired data (scaling functions are included in ADIS16448.cpp)
// Note pressure and temp aren't scaled since they're not passed to the RoboRIO
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
    scaledData[9] = IMU.pressureScale(*(burstData + 10)); //Scale Pressure Sensor
    scaledData[10] = IMU.tempScale(*(burstData + 11)); //Scale Temp Sensor

    // Subtract offsets from gyro signals
    scaledData[0] = scaledData[0] - calDataX;
    scaledData[1] = scaledData[1] - calDataY;
    scaledData[2] = scaledData[2] - calDataZ;
}

void calibrateTeensy() {
  burstData = IMU.burstRead(); // Read data and insert into array
  scaleData(); // Scale IMU data
  // Accumulate offset data
  calDataX = scaledData[0] + calDataX;
  calDataY = scaledData[1] + calDataY;
  calDataZ = scaledData[2] + calDataZ;
  calCounter = calCounter + 1;
  // 8192 = 10 seconds of data capture
  if (calCounter >= 8192) {
  	// Write data to EEPROM, clear flags
    EEPROM.writeFloat(calXloc,calDataX);
    EEPROM.writeFloat(calYloc,calDataY);
    EEPROM.writeFloat(calZloc,calDataZ);
    teensyCalFlag = false;
    calCounter = 0;
  }
}

// IMU built-in calibration routine
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

// Function to act upon serial data received from the host
// Change to serialEvent() if using USB
// Change to serialEvent1() if using pins 0,1
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
    if (printCounter >= 10000) // Delay for writing data to the serial port
    {
        // Print data packet to serial port
        HWSERIAL.println(datapacket);

        // Debug routines for testing
        #ifdef DEBUG
          HWSERIAL.println(calDataX);
          HWSERIAL.println(calDataY);
          HWSERIAL.println(calDataZ);
        #endif

        // Clear data packet... just in case
        //datapacket = "";
        
        // Reset print counter
        printCounter = 0;
    }

}
