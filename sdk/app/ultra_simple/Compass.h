#ifndef COMPASS_H
#define COMPASS_H
#include <stdint.h>
#include <wiringPi.h>
#include <math.h>
#include <wiringPiI2C.h>

#define AK8963_ADDRESS  0x0C
#define AK8963_RA_HXL   0x03
#define AK8963_RA_CNTL1 0x0A
#define AK8963_RA_ASAX  0x10


#define MPU9250_ADDRESS_AD0_LOW  0x68
#define MPU9250_ADDRESS_AD0_HIGH 0x69

#define ACC_FULL_SCALE_2_G       0x00
#define ACC_FULL_SCALE_4_G       0x08
#define ACC_FULL_SCALE_8_G       0x10
#define ACC_FULL_SCALE_16_G      0x18

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define MAG_MODE_POWERDOWN        0x0
#define MAG_MODE_SINGLE           0x1
#define MAG_MODE_CONTINUOUS_8HZ   0x2
#define MAG_MODE_EXTERNAL         0x4
#define MAG_MODE_CONTINUOUS_100HZ 0x6
#define MAG_MODE_SELFTEST         0x8
#define MAG_MODE_FUSEROM          0xF

class Compass {
public:
	int16_t magXOffset, magYOffset, magZOffset;

	Compass(uint8_t address = MPU9250_ADDRESS_AD0_LOW):
		address(address),
		magXOffset(0),
		magYOffset(0),
		magZOffset(0) {
			mainFile = wiringPiI2CSetup(MPU9250_ADDRESS_AD0_LOW);
			magFile = wiringPiI2CSetup(AK8963_ADDRESS);
			beginMag();
			sensorId = readId();
			}
		uint8_t readId();


	void beginMag(uint8_t mode = MAG_MODE_CONTINUOUS_8HZ);
	void magSetMode(uint8_t mode);
	void magUpdate();
	float magX();
	float magY();
	float magZ();
	float magHorizDirection();
	float getDirection(){ this->magUpdate(); return magHorizDirection();}
	uint8_t getSensorId(){return sensorId;}

private:
	uint8_t address;
	int magFile;
	int mainFile;
	
	uint8_t sensorId;
	
	uint8_t magBuf[7];
	uint8_t magXAdjust, magYAdjust, magZAdjust;
	float accelGet(uint8_t highIndex, uint8_t lowIndex);
	float gyroGet(uint8_t highIndex, uint8_t lowIndex);
	int16_t magGet(uint8_t highIndex, uint8_t lowIndex);
	void magEnableSlaveMode();
	void magReadAdjustValues();
	void magWakeup();
	void i2cRead(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
	void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data);
};

#endif
