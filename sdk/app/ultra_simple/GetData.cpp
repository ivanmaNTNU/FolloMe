#include "Compass.h"
#include <string>
#include <iostream>





uint8_t sensorId;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;



int main() {
	Compass mySensor;
	// You can set your own offset for mag values
	// mySensor.magXOffset = -50;
	// mySensor.magYOffset = -55;
	// mySensor.magZOffset = -10;

	std::cout << "sensorId: " << mySensor.getSensorId() << std::endl;
	
	while(1==1){
		mySensor.magUpdate();
		mX = mySensor.magX();
		mY = mySensor.magY();
		mZ = mySensor.magZ();
		mDirection = mySensor.magHorizDirection();
		std::cout << "magX: " << mX << std::endl;
		std::cout << "magY: " << mY << std::endl;
		std::cout << "magZ: " << mZ << std::endl;
		std::cout << "horizontal direction: " << mDirection << std::endl;
		std::cout << "at " << millis() << "ms" << std::endl << std::endl;
		delay(50);
	}
	return 0;
}
