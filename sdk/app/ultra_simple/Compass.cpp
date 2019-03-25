#include "Compass.h"
#include <math.h>
#include <string>
#include <iostream>
#include <math.h>

#define MPU9250_ADDR_ACCELCONFIG  0x1C
#define MPU9250_ADDR_INT_PIN_CFG  0x37
#define MPU9250_ADDR_ACCEL_XOUT_H 0x3B
#define MPU9250_ADDR_GYRO_XOUT_H  0x43
#define MPU9250_ADDR_PWR_MGMT_1   0x6B
#define MPU9250_ADDR_WHOAMI       0x75

void Compass::i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
	if (Address == AK8963_ADDRESS){
		uint8_t index=0;
		while (index < Nbytes) {

			Data[index++]= wiringPiI2CReadReg8(magFile, Register); // Change to Address if not working.
			Register++;
		}
	}
	else if (Address == MPU9250_ADDRESS_AD0_LOW){
		uint8_t index=0;
		while (index < Nbytes) {

			Data[index++]= wiringPiI2CReadReg8(mainFile, Register); // Change to Address if not working.
			Register++;
		}
	}
}



void Compass::i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
	if (Address == AK8963_ADDRESS){
		wiringPiI2CWriteReg8(magFile, Register, Data);
	}
	else if (Address == MPU9250_ADDRESS_AD0_LOW){
		wiringPiI2CWriteReg8(mainFile, Register, Data);
	}
}



uint8_t Compass::readId() {
	uint8_t id;
	i2cRead(address, MPU9250_ADDR_WHOAMI, 1, &id);
	return id;
}


void Compass::magReadAdjustValues() {
	magSetMode(MAG_MODE_POWERDOWN);
	magSetMode(MAG_MODE_FUSEROM);
	uint8_t buff[3];
	i2cRead(AK8963_ADDRESS, AK8963_RA_ASAX, 3, buff);
	magXAdjust = buff[0];
	magYAdjust = buff[1];
	magZAdjust = buff[2];
}

void Compass::beginMag(uint8_t mode) {
	magWakeup();
	magEnableSlaveMode();

	magReadAdjustValues();
	magSetMode(MAG_MODE_POWERDOWN);
	magSetMode(mode);
	delay(10);
}

void Compass::magSetMode(uint8_t mode) {
	i2cWriteByte(AK8963_ADDRESS, AK8963_RA_CNTL1, mode);
	delay(10);
}

void Compass::magWakeup() {
	unsigned char bits;
	i2cRead(address, MPU9250_ADDR_PWR_MGMT_1, 1, &bits);
	bits &= ~0x70; // Turn off SLEEP, STANDBY, CYCLE
	i2cWriteByte(address, MPU9250_ADDR_PWR_MGMT_1, bits);
	delay(10);
}

void Compass::magEnableSlaveMode() {
	unsigned char bits;
	i2cRead(address, MPU9250_ADDR_INT_PIN_CFG, 1, &bits);
	bits |= 0x02; // Activate BYPASS_EN
	i2cWriteByte(address, MPU9250_ADDR_INT_PIN_CFG, bits);
	delay(10);
}

const float Pi = 3.14159265359;

float Compass::magHorizDirection() {
	double dir = atan2(magY(), magX()) * 180 / Pi;
	if (dir < 0){
		dir += 360;
	}
	return dir;
}


void Compass::magUpdate() {
	i2cRead(AK8963_ADDRESS, AK8963_RA_HXL, 7, magBuf);
}

int16_t Compass::magGet(uint8_t highIndex, uint8_t lowIndex) {
	return (((int16_t) magBuf[highIndex]) << 8) | magBuf[lowIndex];
}

float adjustMagValue(int16_t value, uint8_t adjust) {
	return ((float) value * (((((float) adjust - 128) * 0.5) / 128) + 1));
}

float Compass::magX() {
	return adjustMagValue(magGet(1, 0), magXAdjust) + magXOffset;
}

float Compass::magY() {
	return adjustMagValue(magGet(3, 2), magYAdjust) + magYOffset;
}

float Compass::magZ() {
	return adjustMagValue(magGet(5, 4), magZAdjust) + magZOffset;
}




