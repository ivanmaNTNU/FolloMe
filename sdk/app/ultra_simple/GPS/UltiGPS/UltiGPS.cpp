#include "UltiGPS.h"
#include <iostream>
#include <iomanip>
#include "BlynkGPS.h"









#define RE 6731000
#define PI 3.14159265


UltiGPS::UltiGPS(){
	gps_init();
	//gps_location(&loc);
	blynkLat = 0;
	blynkLon = 0;
}


void UltiGPS::update_GPS(){
	gps_location(&loc);
}


void UltiGPS::update_target(){
	update_BlynkGPS();
	double lat = loc.latitude; double lon = loc.longitude;
	double tx = RE*sin(blynkLat)*cos(lat) - RE*cos(blynkLat)*cos(blynkLon)*sin(lat)*cos(lon) - RE*cos(blynkLat)*sin(blynkLon)*sin(lat)*sin(lon);
	double ty = RE*cos(blynkLat)*cos(blynkLon)*sin(lon) - RE*cos(blynkLat)*sin(blynkLon)*cos(lon);
	target.x = tx;	target.y = ty;
	target.angle = atan2(ty,tx)*180/PI;
	target.distance = sqrt(pow(tx,2) + pow(ty,2));

}

Pos UltiGPS::getTarget(){
	update_GPS();
	update_target();
	return target;
}


void UltiGPS::update_BlynkGPS(){
	BlynkGps.BlynkUpdate();
	blynkLat = BlynkGps.getBlynkLat();
	blynkLon = BlynkGps.getBlynkLon();
}
	


