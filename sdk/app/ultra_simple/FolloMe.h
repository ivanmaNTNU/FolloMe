#include <Compass.h>
#include <Drive.h>
#include <Map.h>
#include <UltiGPS.h>
#include <PID.h>




class FolloMe{
public:
	FolloMe(): mode(0), pid(PID(0,1,40, 40, 40) ){
		std::cout << std::fixed;
		std::cout << std::setprecision(8);
		gps = UltiGPS();
		while (true){
        std::cin.get();
        gps.update_BlynkGPS();
        std::cout << "--------------------------------------------------------------------------------------------\n";
        std::cout << "Lat: " << gps.getBlat() << std::endl;

        std::cout << "Lon: " << gps.getBlon() << std::endl;
        std::cout << "--------------------------------------------------------------------------------------------\n";
        std::cout << std::endl;
		}
	}


	void screen_run();

	
	void manual();

private:
	UltiGPS gps;
	Compass compass;
	PathFinder pathFinder;
	Driver motors;
	PID pid;
	int mode;
};

