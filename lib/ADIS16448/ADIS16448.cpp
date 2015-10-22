////////////////////////////////////////////////////////////////////////////////////////////////////////
//  July 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16448.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16448 IMU with an 
//  8-Bit Atmel-based Arduino development board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS164XX family of devices 
//  with some modification.
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
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "ADIS16448.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16448::ADIS16448(int CS, int DR, int RST) {
  _CS = CS;
  _DR = DR;
  _RST = RST;

  SPI.begin(); // Initialize SPI bus
  configSPI(); // Configure SPI

//Set default pin states
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
  pinMode(_DR, INPUT); // Set DR pin to be an input
  pinMode(_RST, OUTPUT); // Set RST pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
  digitalWrite(_RST, HIGH); // Initialize RST pin to be high
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16448::~ADIS16448() {
  // Close SPI bus
  SPI.end();
}

////////////////////////////////////////////////////////////////////////////
// Performs a hardware reset by setting _RST pin low for delay (in ms).
////////////////////////////////////////////////////////////////////////////
int ADIS16448::resetDUT(uint8_t ms) {
  digitalWrite(_RST, LOW);
  delay(100);
  digitalWrite(_RST, HIGH);
  delay(ms);
  return(1);
}

////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
int ADIS16448::configSPI() {
  SPI.setBitOrder(MSBFIRST); // Per the datasheet
  SPI.setClockDivider(SPI_CLOCK_DIV8); // Config for 2MHz (ADIS16448 max 2MHz)
  SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  return(1);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
int16_t ADIS16448::regRead(uint8_t regAddr) {
//Read registers using SPI
  
  // Write register address to be read
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(regAddr); // Write address over SPI bus
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(_CS, LOW); // Set CS low to enable device
  uint8_t _msbData = SPI.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData = SPI.transfer(0x00); // Send (0x00) and place lower byte into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)
  
  int16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits.

  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads all gyro, accel, and magnetometer registers
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (pointer) array of signed 16 bit 2's complement numbers
////////////////////////////////////////////////////////////////////////////////////////////
int16_t * ADIS16448::sensorRead() {
//Read registers using SPI
  // Initialize sensor data arrays and stall time variable
  uint8_t sensorData[18];
  int16_t joinedData[9];
  int stall = 25;

  // Write each requested register address and read back it's data
  digitalWrite(_CS, LOW); // Set CS low to enable communication with the device
  SPI.transfer(XGYRO_OUT); // Initial SPI read. Returned data for this transfer is invalid
  SPI.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable communication with the device
  delayMicroseconds(stall); // Delay to not violate read rate (40us)
  digitalWrite(_CS, LOW);
  sensorData[0] = SPI.transfer(YGYRO_OUT); // Write next address to device and read upper byte
  sensorData[1] = SPI.transfer(0x00); // Read lower byte
  joinedData[0] = (sensorData[0] << 8) | (sensorData[1] & 0xFF); // Concatenate two bytes into word
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall);
  digitalWrite(_CS, LOW);
  sensorData[2] = SPI.transfer(ZGYRO_OUT); 
  sensorData[3] = SPI.transfer(0x00); 
  joinedData[1] = (sensorData[2] << 8) | (sensorData[3] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[4] = SPI.transfer(XACCL_OUT); 
  sensorData[5] = SPI.transfer(0x00); 
  joinedData[2] = (sensorData[4] << 8) | (sensorData[5] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[6] = SPI.transfer(YACCL_OUT); 
  sensorData[7] = SPI.transfer(0x00); 
  joinedData[3] = (sensorData[6] << 8) | (sensorData[7] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall);
  digitalWrite(_CS, LOW);
  sensorData[8] = SPI.transfer(ZACCL_OUT); 
  sensorData[9] = SPI.transfer(0x00); 
  joinedData[4] = (sensorData[8] << 8) | (sensorData[9] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[10] = SPI.transfer(XMAGN_OUT); 
  sensorData[11] = SPI.transfer(0x00); 
  joinedData[5] = (sensorData[10] << 8) | (sensorData[11] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[12] = SPI.transfer(YMAGN_OUT); 
  sensorData[13] = SPI.transfer(0x00); 
  joinedData[6] = (sensorData[12] << 8) | (sensorData[13] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[14] = SPI.transfer(ZMAGN_OUT); 
  sensorData[15] = SPI.transfer(0x00); 
  joinedData[7] = (sensorData[14] << 8) | (sensorData[15] & 0xFF);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(stall); 
  digitalWrite(_CS, LOW);
  sensorData[16] = SPI.transfer(FLASH_CNT); // Final transfer. Data after this invalid
  sensorData[17] = SPI.transfer(0x00);
  joinedData[8] = (sensorData[16] << 8) | (sensorData[17] & 0xFF);
  digitalWrite(_CS, HIGH); 

  return(joinedData); // Return pointer with data
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16448::regWrite(uint8_t regAddr, int16_t regData) {

  // Write register address and data
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI.transfer(lowBytehighWord); // Write low byte from high word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI.transfer(lowBytelowWord); // Write low byte from low word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  return(1);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in g's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in g's
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::accelScale(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData; // Else return the raw number
  float finalData = signedData * 0.000833; // Multiply by accel sensitivity (250 uG/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts gyro data output from the regRead() function and returns gyro rate in deg/sec
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled gyro in degrees/sec
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::gyroScale(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = signedData * 0.04; // Multiply by gyro sensitivity (0.005 dps/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns temperature 
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::tempScale(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = (signedData * 0.07386) + 31; // Multiply by temperature scale and add 31 to equal 0x0000
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts barometer data output from regRead() function and returns pressure in bar
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled pressure in mBar
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::pressureScale(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = (signedData * 0.02); // Multiply by barometer sensitivity (0.02 mBar/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts magnetometer output from regRead() function and returns magnetic field
// reading in Gauss
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled magnetometer data in mgauss
/////////////////////////////////////////////////////////////////////////////////////////
float ADIS16448::magnetometerScale(int16_t sensorData)
{
  int signedData = 0;
  int isNeg = sensorData & 0x8000;
  if (isNeg == 0x8000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0xFFFF;
  else
    signedData = sensorData;
  float finalData = (signedData * 0.0001429); // Multiply by sensor resolution (142.9 uGa/LSB)
  return finalData;
}