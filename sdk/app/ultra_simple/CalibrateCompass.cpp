#include "Compass.h"
#include <string>
#include <time.h>
#include <iostream>


void setMagMinMaxAndSetOffset(Compass* sensor, int seconds);

#define CALIB_SEC 20

Compass mySensor;

uint8_t sensorId;
float mDirection, mX, mY, mZ;

void setupCal() {
	mySensor.beginMag();
	sensorId = mySensor.readId();

	float magXMin, magXMax, magYMin, magYMax, magZ, magZMin, magZMax;

	printf("Start scanning values of magnetometer to get offset values.\n");
	printf("Rotate your device for %i seconds.\n", CALIB_SEC);
	setMagMinMaxAndSetOffset(&mySensor, CALIB_SEC);
	printf("Finished setting offset values.\n");
}

void setMagMinMaxAndSetOffset(Compass* sensor, int seconds) {
	unsigned long calibStartAt = millis();
	float magX, magXMin, magXMax, magY, magYMin, magYMax, magZ, magZMin, magZMax;

	sensor->magUpdate();
	magXMin = magXMax = sensor->magX();
	magYMin = magYMax = sensor->magY();
	magZMin = magZMax = sensor->magZ();

	while (millis() - calibStartAt < (unsigned long) seconds * 1000) {
		delay(100);
		sensor->magUpdate();
		magX = sensor->magX();
		magY = sensor->magY();
		magZ = sensor->magZ();
		if (magX > magXMax) magXMax = magX;
		if (magY > magYMax) magYMax = magY;
		if (magZ > magZMax) magZMax = magZ;
		if (magX < magXMin) magXMin = magX;
		if (magY < magYMin) magYMin = magY;
		if (magZ < magZMin) magZMin = magZ;
	}

	sensor->magXOffset = - (magXMax - magXMin) / 2;
	sensor->magYOffset = - (magYMax - magYMin) / 2;
	sensor->magZOffset = - (magZMax - magZMin) / 2;
}

int main() {
	setupCal();
	std::cout << "sensorId: " << sensorId << std::endl;

	mySensor.magUpdate();
	mX = mySensor.magX();
	mY = mySensor.magY();
	mZ = mySensor.magZ();
	mDirection = mySensor.magHorizDirection();
	std::cout << "mySensor.magXOffset = " << mySensor.magXOffset << std::endl;
	std::cout << "mySensor.magYOffset = " << mySensor.magYOffset << std::endl;
	std::cout << "mySensor.magZOffset = " << mySensor.magZOffset << std::endl;

	std::cout << "magX: " << mX << std::endl;
	std::cout << "magY: " << mY << std::endl;
	std::cout << "magZ: " << mZ << std::endl;
	std::cout << "horizontal direction: " << mDirection << std::endl;
	

	delay(500);
	return 0;
}
