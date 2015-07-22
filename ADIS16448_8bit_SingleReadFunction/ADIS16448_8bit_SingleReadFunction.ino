////////////////////////////////////////////////////////////////////////////////////////////////////////
//  May 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16448 using SPI and the accompanying C++ libraries, 
//  reads IMU data in LSBs, scales the data, and outputs measurements to a serial debug terminal (putty) via
//  the onboard USB serial port.
//
//  This project has been tested on an Arduino Duemilanove and Uno, but should be compatible with any other
//  8-Bit Arduino embedded platform. 
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free Software
//  Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
//  FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for more details.
//
//  You should have received a copy of the GNU Lesser Public License along with 
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
//  Pinout for Arduino Uno/Diecimila/Duemilanove
//  Gray = RST = 4
//  Purple = SCLK = 13
//  Blue = CS = 7
//  Yellow = DOUT(MISO) = 11
//  Green = DIN(MOSI) = 12
//  Black = GND
//  Red = VCC [3.3V ONLY]
//  White = DR = 2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <ADIS16448.h>
#include <SPI.h>

// Initialize Variables
float scaledData[9];
int MSC, SENS, SMPL;

// Data Ready Flag
boolean validData = false;

// Call ADIS16448 Class
ADIS16448 IMU(7,2,4); //ChipSelect,DataReady,Reset Pin Assignments

void setup()
{
  Serial.begin(115200); // Initialize serial output via USB
  IMU.configSPI(); // Configure SPI communication
  
  #ifdef DEBUG
    Serial.println("**********DEBUG MODE**********");
  #endif
  
  delay(100); // Give the part time to start up
  IMU.regWrite(MSC_CTRL,0x06);  // Enable Data Ready on IMU
  delay(20); 
  IMU.regWrite(SENS_AVG,0x400); // Set Digital Filter on IMU
  delay(20);
  IMU.regWrite(SMPL_PRD,0x201), // Set Decimation on IMU
  delay(20);
  
  // Read the control registers once to print to screen
  MSC = IMU.regRead(MSC_CTRL);
  SENS = IMU.regRead(SENS_AVG);
  SMPL = IMU.regRead(SMPL_PRD);
  
  attachInterrupt(0, setDRFlag, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
}

// Function used to read register values via SPI and load them into variables in LSBs
void grabData()
{
    // Put all the Data Registers you want to read here
    int16_t * data; // Instantiate data array
    //IMU.configSPI(); // Configure SPI before the read
    data = IMU.sensorRead(); // Read predefined sensors
    
    // Scale sensor data
    for(int i = 0; i < 3; i++){
      scaledData[i] = IMU.gyroScale(*(data + i));
    }
    for(int i = 0; i < 3; i++){
      scaledData[i + 3] = IMU.accelScale(*(data + (i + 3)));
    }
    for(int i = 0; i < 3; i++){
      scaledData[i + 6] = IMU.magnetometerScale(*(data + (i + 6)));
    }    
}

// Data Ready Interrupt Routine
void setDRFlag()
{
  validData = !validData;
}

// Main loop. Scale and display registers read using the interrupt
void loop()
{
  if (validData) // If data present in the ADIS16448 registers is valid...
  {
    grabData(); // Grab data from the IMU
    
    //Print control registers to the serial port
    Serial.println("ADIS16448 Arduino Demo Program");
    Serial.println("Juan J Chong - 2015");
    Serial.println(" ");
    
    Serial.println("Control Registers");
    Serial.print("MSC_CTRL: 0x");
    Serial.println(MSC,HEX);
    Serial.print("SENS_AVG: 0x");
    Serial.println(SENS,HEX);
    Serial.print("SMPL_PRD: 0x");
    Serial.println(SMPL,HEX);
    Serial.println(" ");
    Serial.println("Data Registers");
    
    //Print scaled gyro data
    Serial.print("XGYRO: ");
    Serial.println(scaledData[0]);
    Serial.print("YGYRO: ");
    Serial.println(scaledData[1]);
    Serial.print("ZGYRO: ");
    Serial.println(scaledData[2]);
  
    //Print scaled accel data
    Serial.print("XACCL: ");
    Serial.println(scaledData[3]);
    Serial.print("YACCL: ");
    Serial.println(scaledData[4]);
    Serial.print("ZACCL: ");
    Serial.println(scaledData[5]);
    
    //Print scaled magnetometer data
    Serial.print("XMAG: ");
    Serial.println(scaledData[6]);
    Serial.print("YMAG: ");
    Serial.println(scaledData[7]);
    Serial.print("ZMAG: ");
    Serial.println(scaledData[8]);
   
    delay(150); // Give the user time to read the data
    
    //Clear the serial terminal and reset cursor
    //Only works on supported serial terminal programs (Putty)
    Serial.print("\033[2J");
    Serial.print("\033[H");
  }
}
