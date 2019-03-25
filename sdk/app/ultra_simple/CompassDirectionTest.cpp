#include "Compass.h"
#include <string>
#include <iostream>




int main() {

	Compass mySensor;
	
	while(1==1){
		std::cout << "horizontal direction: " << mySensor.getDirection() << std::endl;
		delay(50);
	}
	return 0;
}
