/*
 * derived from: https://github.com/sabas1080/FXAS21002C_Arduino_Library
 *
 * modified by Francesco Ferraro 01/2016
 * francesco.ferrarogm@gmail.com
 *
*/

#include <Wire.h>
#include <math.h>

#include <FXAS21002C.h>

// Public Methods //////////////////////////////////////////////////////////////


FXAS21002C::FXAS21002C(byte addr)
{
	address = addr;
	gyroODR = GODR_200HZ; // In hybrid mode, accel/mag data sample rates are half of this value
	gyroFSR = GFS_500DPS;
}

void FXAS21002C::writeReg(byte reg, byte value)
{
	Wire1.beginTransmission(address);
	Wire1.write(reg);
	Wire1.write(value);
	Wire1.endTransmission();
}

// Reads a register
byte FXAS21002C::readReg(byte reg)
{
	byte value;

	Wire1.beginTransmission(address);
	Wire1.write(reg);
	Wire1.endTransmission(false);

	Wire1.requestFrom(address, (uint8_t)1, (uint8_t) true);
	value = Wire1.read();
	Wire1.endTransmission();

	return value;
}

void FXAS21002C::readRegs(byte reg, uint8_t count, byte dest[])
{
	uint8_t i = 0;

	Wire1.beginTransmission(address);   // Initialize the Tx buffer
	Wire1.write(reg);            	   // Put slave register address in Tx buffer
	Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	Wire1.requestFrom(address, count);  // Read bytes from slave register address

	while (Wire1.available()) {
		dest[i++] = Wire1.read();   // Put read results in the Rx buffer
	}
	Wire1.endTransmission ();
}

// Read the temperature data
void FXAS21002C::readTempData()
{
	tempData = readReg(FXAS21002C_H_TEMP);
}

// Put the FXAS21002C into standby mode.
// It must be in standby for modifying most registers
void FXAS21002C::standby()
{
	byte c = readReg(FXAS21002C_H_CTRL_REG1);
	writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));// Clear bits 0 and 1; standby mode
}
// Sets the FXAS21000 to active mode.
// Needs to be in this mode to output data
void FXAS21002C::ready()
{
  byte c = readReg(FXAS21002C_H_CTRL_REG1);
  writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  writeReg(FXAS21002C_H_CTRL_REG1, c |   0x01);   // Set bit 0 to 1, ready mode; no data acquisition yet
}

// Put the FXAS21002C into active mode.
// Needs to be in this mode to output data.
void FXAS21002C::active()
{
	byte c = readReg(FXAS21002C_H_CTRL_REG1);
	writeReg(FXAS21002C_H_CTRL_REG1, c & ~(0x03));  // Clear bits 0 and 1; standby mode
  	writeReg(FXAS21002C_H_CTRL_REG1, c |   0x02);   // Set bit 1 to 1, active mode; data acquisition enabled
}

void FXAS21002C::init()
{
	standby();  // Must be in standby to change registers

	// Set up the full scale range to 250, 500, 1000, or 2000 deg/s.
	gyroFSR = GFS_500DPS;
	//writeReg (FXAS21002C_H_CTRL_REG0, GFS_500DPS); 
	 // Setup the 3 data rate bits, 4:2
	//if (gyroODR < 8) 
	//	writeReg(FXAS21002C_H_CTRL_REG1, gyroODR << 2);

	// Disable FIFO, route FIFO and rate threshold interrupts to INT2, enable data ready interrupt, route to INT1
  	// Active HIGH, push-pull output driver on interrupts
  	writeReg(FXAS21002C_H_CTRL_REG2, 0x0E);

  	 // Set up rate threshold detection; at max rate threshold = FSR; rate threshold = THS*FSR/128
  	writeReg(FXAS21002C_H_RT_CFG, 0x07);         // enable rate threshold detection on all axes
 	writeReg(FXAS21002C_H_RT_THS, 0x00 | 0x0D);  // unsigned 7-bit THS, set to one-tenth FSR; set clearing debounce counter
	writeReg(FXAS21002C_H_RT_COUNT, 0x04);       // set to 4 (can set up to 255)
	writeReg(FXAS21002C_H_CTRL_REG3, 0x00);

	//Serial.println (readReg (FXAS21002C_H_CTRL_REG3));
	//Serial.println (readReg (FXAS21002C_H_RT_COUNT));
	// Configure interrupts 1 and 2
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) & ~(0x02)); // clear bits 0, 1 
	//writeReg(CTRL_REG3, readReg(CTRL_REG3) |  (0x02)); // select ACTIVE HIGH, push-pull interrupts    
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) & ~(0x1D)); // clear bits 0, 3, and 4
	//writeReg(CTRL_REG4, readReg(CTRL_REG4) |  (0x1D)); // DRDY, Freefall/Motion, P/L and tap ints enabled  
	//writeReg(CTRL_REG5, 0x01);  // DRDY on INT1, P/L and taps on INT2

	//active();  // Set to active to start reading
}

// Read the gyroscope data
void FXAS21002C::readGyroData()
{
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readRegs(FXAS21002C_H_OUT_X_MSB, 6, &rawData[0]);  // Read the six raw data registers into data array
	
	gyroData.x = ((int16_t)( ((int16_t) rawData[0]) << 8 | ((int16_t) rawData[1])));
	gyroData.y = ((int16_t)( ((int16_t) rawData[2]) << 8 | ((int16_t) rawData[3])));
	gyroData.z = ((int16_t)( ((int16_t) rawData[4]) << 8 | ((int16_t) rawData[5])));
}

int FXAS21002C::setRateODR (GyroODR odr)
{
	standby ();
	if (odr < 7) {
		writeReg (FXAS21002C_H_CTRL_REG1, (readReg (FXAS21002C_H_CTRL_REG1) & 0xE3) | odr << 2);
	}
}

int FXAS21002C::setLowPassLPF (GyroLPF lpf)
{
	standby ();
	byte write = (readReg (FXAS21002C_H_CTRL_REG0) & 0x3f) | lpf << 6;
	Serial.println (write, HEX);
	writeReg (FXAS21002C_H_CTRL_REG0, (readReg (FXAS21002C_H_CTRL_REG0) & 0x3f) | lpf << 6);
}

int FXAS21002C::setRate (int rate)
{
	switch (rate) {
		case 25:
			setRateODR (GODR_25HZ);
			break;
		case 50:
			setRateODR (GODR_50HZ);
			break;
		case 100:
			setRateODR (GODR_100HZ);
			break;
		case 200:
			setRateODR (GODR_200HZ);
			break;
		case 400:
			setRateODR (GODR_400HZ);
			break;
		case 800:
			setRateODR (GODR_800HZ);
			break;
		default:
			return -1;
	}
	return 0;
}

int FXAS21002C::setRangeFSR (GyroFSR fsr)
{
	standby ();
	if (fsr <= 3) {
		byte test = (readReg (FXAS21002C_H_CTRL_REG0) & 0xFC) | fsr;
		Serial.print ("set ctrl0 register to: 0x");
		Serial.println (test, HEX);
		writeReg (FXAS21002C_H_CTRL_REG0, (readReg (FXAS21002C_H_CTRL_REG0) & 0xFC) | fsr);

		byte test2 = readReg (FXAS21002C_H_CTRL_REG0);
		Serial.print ("ctrl0 register: 0x");
		Serial.println (test2, HEX);

		gyroFSR = fsr;
		return 0;
	}
	return -1;

}
int FXAS21002C::setRange (int range)
{
	disableDoubleFSR ();
	switch (range) {
		case 250:
			setRangeFSR (GFS_250DPS);
			break;

		case 500:
			setRangeFSR (GFS_500DPS);
			break;

		case 1000:
			setRangeFSR (GFS_1000DPS);
			break;

		// Fall through is desired 
		case 4000:
			enableDoubleFSR ();
		case 2000:
			setRangeFSR (GFS_2000DPS);
			break;

		default:
			return -1;
	}
	return 0;
}

int FXAS21002C::enableDoubleFSR ()
{
	standby ();
	writeReg (FXAS21002C_H_CTRL_REG3, readReg (FXAS21002C_H_CTRL_REG0) | 0x1);
}
int FXAS21002C::disableDoubleFSR ()
{
	standby ();
	writeReg (FXAS21002C_H_CTRL_REG3, readReg (FXAS21002C_H_CTRL_REG0) & 0x0);
}



// Get accelerometer resolution
float FXAS21002C::getGres(void)
{
	switch (gyroFSR)
	{
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (11), 500 DPS (10), 1000 DPS (01), and 2000 DPS  (00). 
    case GFS_2000DPS:
		return 0.0625; //2000.0/32768.0;
    
	case GFS_1000DPS:
        return 0.03125; //1000.0/32768.0;
    
	case GFS_500DPS:
        return 0.015625;
    
	case GFS_250DPS:
        return 0.0078125; //250.0/32768.0;
	}
}

void FXAS21002C::calibrate(int samples, int hertz)
{
	Serial.println ("Calibrating Gyroscope...");
	int32_t gyro_bias[3] = {0, 0, 0};
	uint16_t ii;
	int16_t temp[3];
	
	// Clear all interrupts by reading the data output and STATUS registers
	//readGyroData(temp);
	readReg(FXAS21002C_H_STATUS);
	
	//init ();
	
	standby();  // Must be in standby to change registers
	
	//writeReg(FXAS21002C_H_CTRL_REG1, 0x08);   // select 50 Hz ODR
	//writeReg(FXAS21002C_H_CTRL_REG0, 0x03);   // select 200 deg/s full scale
	//float gyrosensitivity = 32000.0/250.0; //GFS_250DPS;
	
	active();  // Set to active to start collecting data
	unsigned long microsPerReading = 1000000 / hertz;
	unsigned long microsPrevious = micros ();
	
	uint8_t rawData[6];  // x/y/z FIFO accel data stored here
	for(ii = 0; ii < samples; ii++)   // construct count sums for each axis
	{
		unsigned long microsNow;

  		// check if it's time to read data and update the filter
  		microsNow = micros();

		while (microsNow - microsPrevious < microsPerReading) {
			microsNow = micros ();	
		}
  		

		readRegs(FXAS21002C_H_OUT_X_MSB, 6, &rawData[0]);  // Read the FIFO data registers into data array
		temp[0] = ((int16_t)( ((int16_t) rawData[0]) << 8 | ((int16_t) rawData[1])));
		temp[1] = ((int16_t)( ((int16_t) rawData[2]) << 8 | ((int16_t) rawData[3])));
		temp[2] = ((int16_t)( ((int16_t) rawData[4]) << 8 | ((int16_t) rawData[5])));
		
		gyro_bias[0] += (int32_t) temp[0];
		gyro_bias[1] += (int32_t) temp[1];
		gyro_bias[2] += (int32_t) temp[2];


		microsPrevious = microsPrevious + microsPerReading;
	}
	
	gyro_bias[0] /= (int32_t) samples; // get average values
	gyro_bias[1] /= (int32_t) samples;
	gyro_bias[2] /= (int32_t) samples;
	
	gBias[0] = (float)gyro_bias[0] * getGres (); ///gyrosensitivity; // get average values
	gBias[1] = (float)gyro_bias[1] * getGres ();///gyrosensitivity; // get average values
	gBias[2] = (float)gyro_bias[2] * getGres ();///gyrosensitivity; // get average values
	
	Serial.print ("gyro bias:");
	Serial.print (gBias[0]);
	Serial.print (",");
	Serial.print (gBias[1]);
	Serial.print (",");
	Serial.println (gBias[2]);
	
	
	ready();  // Set to ready
}

void FXAS21002C::reset() 
{
	writeReg(FXAS21002C_H_CTRL_REG1, 0x40); // set reset bit to 1 to assert software reset to zero at end of boot process
	delay(100);
	while(!(readReg(FXAS21002C_H_INT_SRC_FLAG) & 0x08))  { // wait for boot end flag to be set
	}

}
// Private Methods //////////////////////////////////////////////////////////////
