#pragma once 

#ifdef RASPBERRY
 #include <BlynkApiWiringPi.h>
#else
#include <BlynkApiLinux.h>
#endif
#include <BlynkSocket.h>
#include <BlynkOptionsParser.h>

static BlynkTransportSocket _blynkTransport;
BlynkSocket Blynk(_blynkTransport);

#include <BlynkWidgets.h>
#include <wiringPi.h>


#include <iostream>
#include <iomanip>


double LAT;
double LON;

BLYNK_WRITE(V1) {
    GpsParam gps(param);
    LAT = gps.getLat();
    LON = gps.getLon();
    std::cout << "Updated..." << std::endl;
}

class Blynk_GPS {
public:

    Blynk_GPS() : Lat(0), Lon(0) {this->start();}
    void BlynkUpdate(){ 
        Blynk.run();
        Lat = LAT;
        Lon = LON;
        } 
    double getBlynkLat(){return Lat;}
    double getBlynkLon(){return Lon;}
    
    void start(){
    const char auth[40] = "c91cd62590e74daf803ff0339b256f13";
    const char serv[20] = "blynk-cloud.com";
    uint16_t port = 80;
    Blynk.begin(auth, serv, port);

    
    while(!Blynk.connected()) {
        Blynk.run();
        } 
        std::cout << "Connected Sucessfully!" << std::endl;
    }

private:
    double Lat;
    double Lon;
    
};





    
    
    
    
        


